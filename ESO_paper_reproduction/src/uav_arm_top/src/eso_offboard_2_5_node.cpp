/**
 * @file eso_offboard_2_5_node.cpp
 * @brief ROS node for testing eso_pos_control module in PX4 (2m hold -> 5m)
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// Global state for FCU connection
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eso_offboard_2_5_node");
    ros::NodeHandle nh;

    // 1. Subscribe to flight state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
            "mavros/state", 10, state_cb);

    // 2. Publish position setpoints (Local NED)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 10);

    // 3. Service clients: arm and set mode
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
            "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
            "mavros/set_mode");

    ros::Rate rate(20.0);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection...");
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 2.0;

    // Send initial setpoints before Offboard
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool hold_started = false;
    ros::Time hold_start;

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (current_state.mode == "OFFBOARD" && current_state.armed) {
            if (!hold_started) {
                hold_start = ros::Time::now();
                hold_started = true;
            }

            if (ros::Time::now() - hold_start >= ros::Duration(10.0)) {
                pose.pose.position.z = 5.0;
            } else {
                pose.pose.position.z = 2.0;
            }
        } else {
            pose.pose.position.z = 2.0;
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
