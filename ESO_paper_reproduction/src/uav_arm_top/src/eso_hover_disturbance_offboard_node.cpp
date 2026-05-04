/**
 * @file eso_hover_disturbance_offboard_node.cpp
 * @brief Hover-at-2m offboard node for the disturbance rejection experiment.
 */

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace {

struct HoverTarget {
    double x;
    double y;
    double z;
};

mavros_msgs::State g_current_state;
geometry_msgs::PoseStamped g_current_pose;
bool g_have_pose = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    g_current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    g_current_pose = *msg;
    g_have_pose = true;
}

double distance_to_target(const geometry_msgs::PoseStamped& pose, const HoverTarget& target) {
    const double dx = pose.pose.position.x - target.x;
    const double dy = pose.pose.position.y - target.y;
    const double dz = pose.pose.position.z - target.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "eso_hover_disturbance_offboard_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    ros::Publisher arm_enable_pub = nh.advertise<std_msgs::Bool>(
        "experiment/arm_motion_enabled", 1, true);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");

    HoverTarget target{0.0, 0.0, 2.0};
    double reach_tol_m = 0.20;
    double activation_altitude_m = 1.90;
    double activation_hold_s = 1.00;
    double startup_delay_s = 0.00;
    double static_validation_delay_s = -1.00;
    double static_validation_reach_m = -1.00;

    pnh.param("target_x", target.x, target.x);
    pnh.param("target_y", target.y, target.y);
    pnh.param("target_z", target.z, target.z);
    pnh.param("reach_tol_m", reach_tol_m, reach_tol_m);
    pnh.param("activation_altitude_m", activation_altitude_m, activation_altitude_m);
    pnh.param("activation_hold_s", activation_hold_s, activation_hold_s);
    pnh.param("startup_delay_s", startup_delay_s, startup_delay_s);
    pnh.param("static_validation_delay_s", static_validation_delay_s, static_validation_delay_s);
    pnh.param("static_validation_reach_m", static_validation_reach_m, static_validation_reach_m);

    ros::Rate rate(20.0);

    while (ros::ok() && !g_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection...");
    }

    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = target.x;
    target_pose.pose.position.y = target.y;
    target_pose.pose.position.z = target.z;
    target_pose.pose.orientation.w = 1.0;

    for (int i = 100; ros::ok() && i > 0; --i) {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    if (startup_delay_s > 0.0) {
        ROS_INFO("Delaying offboard arming by %.2f s", startup_delay_s);
        const ros::Time delay_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - delay_start) < ros::Duration(startup_delay_s)) {
            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);
            ros::spinOnce();
            rate.sleep();
        }
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time activation_since;
    ros::Time validation_since;
    bool activation_window_started = false;
    bool validation_window_started = false;
    bool arm_motion_enabled = false;
    std_msgs::Bool arm_enable_msg;
    arm_enable_msg.data = false;
    arm_enable_pub.publish(arm_enable_msg);

    ROS_INFO("Hover disturbance experiment target: (%.2f, %.2f, %.2f)",
             target.x, target.y, target.z);

    while (ros::ok()) {
        if (g_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if (!g_current_state.armed &&
                   (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        if (!arm_motion_enabled && static_validation_delay_s >= 0.0) {
            const bool validation_ready =
                g_have_pose && g_current_state.armed &&
                g_current_pose.pose.position.z >= activation_altitude_m &&
                (static_validation_reach_m < 0.0 ||
                 distance_to_target(g_current_pose, target) <= static_validation_reach_m);
            if (!validation_window_started && validation_ready) {
                validation_window_started = true;
                validation_since = ros::Time::now();
                ROS_INFO("Static validation altitude reached, waiting %.2f s before marker",
                         static_validation_delay_s);
            }
            if (validation_window_started &&
                ros::Time::now() - validation_since >= ros::Duration(static_validation_delay_s)) {
                arm_motion_enabled = true;
                arm_enable_msg.data = true;
                arm_enable_pub.publish(arm_enable_msg);
                ROS_INFO("Static validation marker published");
            } else if (validation_window_started && !validation_ready) {
                validation_window_started = false;
                ROS_INFO("Static validation readiness lost, restarting marker timer");
            }
        } else if (!arm_motion_enabled && static_validation_delay_s < 0.0 &&
                   g_have_pose &&
                   g_current_pose.pose.position.z >= activation_altitude_m &&
                   distance_to_target(g_current_pose, target) <= reach_tol_m) {
            if (!activation_window_started) {
                activation_window_started = true;
                activation_since = ros::Time::now();
                ROS_INFO("Hover target reached, waiting %.2f s before enabling arm motion",
                         activation_hold_s);
            } else if (ros::Time::now() - activation_since >= ros::Duration(activation_hold_s)) {
                arm_motion_enabled = true;
                arm_enable_msg.data = true;
                arm_enable_pub.publish(arm_enable_msg);
                ROS_INFO("Arm motion enabled for disturbance rejection experiment");
            }
        } else if (!arm_motion_enabled) {
            activation_window_started = false;
        }

        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
