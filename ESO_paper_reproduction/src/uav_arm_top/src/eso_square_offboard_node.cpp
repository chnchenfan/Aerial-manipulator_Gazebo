/**
 * @file eso_square_offboard_node.cpp
 * @brief Offboard square trajectory publisher for PX4 (MAVROS).
 */

#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

struct Waypoint {
    double x;
    double y;
    double z;
};

static mavros_msgs::State g_current_state;
static geometry_msgs::PoseStamped g_current_pose;
static bool g_have_pose = false;

static void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    g_current_state = *msg;
}

static void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    g_current_pose = *msg;
    g_have_pose = true;
}

static double distance_xy(const geometry_msgs::PoseStamped& pose, const Waypoint& wp) {
    const double dx = pose.pose.position.x - wp.x;
    const double dy = pose.pose.position.y - wp.y;
    const double dz = pose.pose.position.z - wp.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "eso_square_offboard_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");

    double side_length = 2.0;
    double altitude = 2.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double corner_hold_s = 3.0;
    double reach_tol_m = 0.2;

    pnh.param("side_length", side_length, side_length);
    pnh.param("altitude", altitude, altitude);
    pnh.param("origin_x", origin_x, origin_x);
    pnh.param("origin_y", origin_y, origin_y);
    pnh.param("corner_hold_s", corner_hold_s, corner_hold_s);
    pnh.param("reach_tol_m", reach_tol_m, reach_tol_m);

    const double rate_hz = 20.0;
    ros::Rate rate(rate_hz);

    while (ros::ok() && !g_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection...");
    }

    std::vector<Waypoint> waypoints = {
        {origin_x, origin_y, altitude},
        {origin_x + side_length, origin_y, altitude},
        {origin_x + side_length, origin_y + side_length, altitude},
        {origin_x, origin_y + side_length, altitude},
        {origin_x, origin_y, altitude},
    };

    std::size_t wp_idx = 0;
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = waypoints[wp_idx].x;
    target_pose.pose.position.y = waypoints[wp_idx].y;
    target_pose.pose.position.z = waypoints[wp_idx].z;
    target_pose.pose.orientation.w = 1.0;

    for (int i = 100; ros::ok() && i > 0; --i) {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_wp_switch = ros::Time::now();
    ros::Time within_since;
    bool within = false;

    while (ros::ok()) {
        if (g_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!g_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        const Waypoint& wp = waypoints[wp_idx];
        if (g_have_pose) {
            const double dist = distance_xy(g_current_pose, wp);
            if (dist <= reach_tol_m) {
                if (!within) {
                    within = true;
                    within_since = ros::Time::now();
                }
                if (ros::Time::now() - within_since >= ros::Duration(corner_hold_s)) {
                    wp_idx = (wp_idx + 1) % waypoints.size();
                    within = false;
                }
            } else {
                within = false;
            }
        } else {
            if (ros::Time::now() - last_wp_switch > ros::Duration(corner_hold_s)) {
                wp_idx = (wp_idx + 1) % waypoints.size();
                last_wp_switch = ros::Time::now();
            }
        }

        const Waypoint& active_wp = waypoints[wp_idx];
        target_pose.pose.position.x = active_wp.x;
        target_pose.pose.position.y = active_wp.y;
        target_pose.pose.position.z = active_wp.z;
        target_pose.pose.orientation.w = 1.0;

        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
