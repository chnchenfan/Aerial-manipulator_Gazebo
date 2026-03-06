/**
 * @file eso_circle_offboard_node.cpp
 * @brief Offboard circular trajectory publisher for PX4 (MAVROS).
 * @details This node publishes waypoints along a circular path with specified radius.
 *          The UAV will follow a circular trajectory around a center point.
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

/**
 * Generate circular trajectory waypoints
 * @param center_x X coordinate of circle center
 * @param center_y Y coordinate of circle center
 * @param altitude Z coordinate (constant for 2D circle in horizontal plane)
 * @param radius Radius of the circle in meters
 * @param num_waypoints Number of waypoints to generate around the circle
 * @return Vector of waypoints forming a circular path
 */
std::vector<Waypoint> generate_circular_path(
    double center_x, double center_y, double altitude,
    double radius, int num_waypoints) {

    std::vector<Waypoint> waypoints;
    const double PI = 3.14159265359;

    for (int i = 0; i < num_waypoints; ++i) {
        const double angle = 2.0 * PI * i / num_waypoints;
        const double x = center_x + radius * std::cos(angle);
        const double y = center_y + radius * std::sin(angle);
        waypoints.push_back({x, y, altitude});
    }

    // Close the loop by returning to the starting point
    waypoints.push_back(waypoints[0]);

    return waypoints;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "eso_circle_offboard_node");
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

    // Circle parameters
    double circle_radius = 2.0;        // 2 meters radius circle
    double altitude = 2.0;             // 2 meters altitude
    double center_x = 0.0;             // Circle center X
    double center_y = 0.0;             // Circle center Y
    double reach_tol_m = 0.2;          // Waypoint reach tolerance
    int num_waypoints = 20;            // Number of waypoints around the circle
    double waypoint_delay_s = 1.0;     // Delay at each waypoint in seconds
    double stability_margin = 0.05;    // Additional margin to stay in waypoint (m)

    pnh.param("circle_radius", circle_radius, circle_radius);
    pnh.param("altitude", altitude, altitude);
    pnh.param("center_x", center_x, center_x);
    pnh.param("center_y", center_y, center_y);
    pnh.param("reach_tol_m", reach_tol_m, reach_tol_m);
    pnh.param("num_waypoints", num_waypoints, num_waypoints);
    pnh.param("waypoint_delay_s", waypoint_delay_s, waypoint_delay_s);
    pnh.param("stability_margin", stability_margin, stability_margin);

    const double rate_hz = 20.0;
    ros::Rate rate(rate_hz);

    ROS_INFO("Circle Offboard Node initialized with parameters:");
    ROS_INFO("  Radius: %.2f m", circle_radius);
    ROS_INFO("  Altitude: %.2f m", altitude);
    ROS_INFO("  Center: (%.2f, %.2f)", center_x, center_y);
    ROS_INFO("  Waypoints: %d", num_waypoints);
    ROS_INFO("  Reach Tolerance: %.3f m", reach_tol_m);
    ROS_INFO("  Waypoint Delay: %.2f s", waypoint_delay_s);
    ROS_INFO("  Stability Margin: %.3f m", stability_margin);

    while (ros::ok() && !g_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection...");
    }

    // Generate circular waypoints
    std::vector<Waypoint> waypoints = generate_circular_path(
        center_x, center_y, altitude, circle_radius, num_waypoints);

    ROS_INFO("Generated %zu waypoints for circular path", waypoints.size());

    std::size_t wp_idx = 0;
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = waypoints[wp_idx].x;
    target_pose.pose.position.y = waypoints[wp_idx].y;
    target_pose.pose.position.z = waypoints[wp_idx].z;
    target_pose.pose.orientation.w = 1.0;

    // Pre-send setpoints to stabilize the drone before arming
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
    bool within = false;
    ros::Time within_since;

    ROS_INFO("Starting circular trajectory...");

    while (ros::ok()) {
        // Switch to Offboard mode
        if (g_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            // Arm the vehicle
            if (!g_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Check if current waypoint is reached
        const Waypoint& wp = waypoints[wp_idx];
        if (g_have_pose) {
            const double dist = distance_xy(g_current_pose, wp);
            const double effective_tolerance = reach_tol_m + stability_margin;

            // Waypoint reached when distance is less than tolerance
            if (dist <= reach_tol_m) {
                if (!within) {
                    within = true;
                    within_since = ros::Time::now();
                    ROS_INFO("Reached waypoint %zu (dist: %.3f m), stabilizing...", wp_idx, dist);
                }
                // Wait for the specified delay before switching to next waypoint
                double wait_time = (ros::Time::now() - within_since).toSec();
                if (wait_time >= waypoint_delay_s) {
                    ROS_INFO("Waypoint %zu stable for %.2f s, switching to next", wp_idx, wait_time);
                    wp_idx = (wp_idx + 1) % waypoints.size();
                    within = false;
                }
            } else {
                // Lost waypoint lock (distance increased beyond tolerance + margin)
                if (within && dist > effective_tolerance) {
                    ROS_WARN("Lost lock on waypoint %zu (dist: %.3f m, tolerance: %.3f m)",
                             wp_idx, dist, effective_tolerance);
                    within = false;
                }
            }
        }

        // Publish target pose
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
