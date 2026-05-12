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

struct TdState {
    HoverTarget position;
    HoverTarget velocity;
    bool initialized;
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

void update_td_axis(double target, double dt, double bandwidth_rad_s, double acc_limit,
                    double vel_limit, double& position, double& velocity) {
    const double error = target - position;
    double accel = bandwidth_rad_s * bandwidth_rad_s * error - 2.0 * bandwidth_rad_s * velocity;
    accel = std::max(-acc_limit, std::min(acc_limit, accel));
    velocity += accel * dt;
    velocity = std::max(-vel_limit, std::min(vel_limit, velocity));
    position += velocity * dt;
}

void update_td(const HoverTarget& target, double dt, double bandwidth_rad_s,
               double acc_limit, double vel_limit, TdState& td) {
    update_td_axis(target.x, dt, bandwidth_rad_s, acc_limit, vel_limit,
                   td.position.x, td.velocity.x);
    update_td_axis(target.y, dt, bandwidth_rad_s, acc_limit, vel_limit,
                   td.position.y, td.velocity.y);
    update_td_axis(target.z, dt, bandwidth_rad_s, acc_limit, vel_limit,
                   td.position.z, td.velocity.z);
}

geometry_msgs::PoseStamped make_target_pose(const HoverTarget& target) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = target.x;
    pose.pose.position.y = target.y;
    pose.pose.position.z = target.z;
    pose.pose.orientation.w = 1.0;
    return pose;
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
    bool use_td_setpoint = false;
    double td_bandwidth_hz = 0.35;
    double td_accel_limit_mps2 = 0.35;
    double td_vel_limit_mps = 0.25;

    pnh.param("target_x", target.x, target.x);
    pnh.param("target_y", target.y, target.y);
    pnh.param("target_z", target.z, target.z);
    pnh.param("reach_tol_m", reach_tol_m, reach_tol_m);
    pnh.param("activation_altitude_m", activation_altitude_m, activation_altitude_m);
    pnh.param("activation_hold_s", activation_hold_s, activation_hold_s);
    pnh.param("startup_delay_s", startup_delay_s, startup_delay_s);
    pnh.param("static_validation_delay_s", static_validation_delay_s, static_validation_delay_s);
    pnh.param("static_validation_reach_m", static_validation_reach_m, static_validation_reach_m);
    pnh.param("use_td_setpoint", use_td_setpoint, use_td_setpoint);
    pnh.param("td_bandwidth_hz", td_bandwidth_hz, td_bandwidth_hz);
    pnh.param("td_accel_limit_mps2", td_accel_limit_mps2, td_accel_limit_mps2);
    pnh.param("td_vel_limit_mps", td_vel_limit_mps, td_vel_limit_mps);
    td_bandwidth_hz = std::max(0.01, td_bandwidth_hz);
    td_accel_limit_mps2 = std::max(0.01, td_accel_limit_mps2);
    td_vel_limit_mps = std::max(0.05, td_vel_limit_mps);
    const double td_bandwidth_rad_s = 2.0 * M_PI * td_bandwidth_hz;

    ros::Rate rate(20.0);

    while (ros::ok() && !g_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection...");
    }

    TdState td{{target.x, target.y, target.z}, {0.0, 0.0, 0.0}, false};
    ros::Time last_td_update = ros::Time::now();
    geometry_msgs::PoseStamped target_pose = make_target_pose(target);

    for (int i = 100; ros::ok() && i > 0; --i) {
        if (use_td_setpoint && !td.initialized && g_have_pose) {
            td.position = {
                g_current_pose.pose.position.x,
                g_current_pose.pose.position.y,
                g_current_pose.pose.position.z,
            };
            td.velocity = {0.0, 0.0, 0.0};
            td.initialized = true;
            last_td_update = ros::Time::now();
        }
        if (use_td_setpoint && td.initialized) {
            const ros::Time now = ros::Time::now();
            const double dt = std::max(0.001, (now - last_td_update).toSec());
            last_td_update = now;
            update_td(target, dt, td_bandwidth_rad_s, td_accel_limit_mps2, td_vel_limit_mps, td);
            target_pose = make_target_pose(td.position);
        } else {
            target_pose = make_target_pose(target);
        }
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    if (startup_delay_s > 0.0) {
        ROS_INFO("Delaying offboard arming by %.2f s", startup_delay_s);
        const ros::Time delay_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - delay_start) < ros::Duration(startup_delay_s)) {
            if (use_td_setpoint && !td.initialized && g_have_pose) {
                td.position = {
                    g_current_pose.pose.position.x,
                    g_current_pose.pose.position.y,
                    g_current_pose.pose.position.z,
                };
                td.velocity = {0.0, 0.0, 0.0};
                td.initialized = true;
                last_td_update = ros::Time::now();
            }
            if (use_td_setpoint && td.initialized) {
                const ros::Time now = ros::Time::now();
                const double dt = std::max(0.001, (now - last_td_update).toSec());
                last_td_update = now;
                update_td(target, dt, td_bandwidth_rad_s, td_accel_limit_mps2, td_vel_limit_mps, td);
                target_pose = make_target_pose(td.position);
            } else {
                target_pose = make_target_pose(target);
            }
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

    ROS_INFO("Hover disturbance experiment target: (%.2f, %.2f, %.2f), TD %s, TD bw %.2f Hz",
             target.x, target.y, target.z, use_td_setpoint ? "enabled" : "disabled",
             td_bandwidth_hz);

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

        if (use_td_setpoint && !td.initialized && g_have_pose) {
            td.position = {
                g_current_pose.pose.position.x,
                g_current_pose.pose.position.y,
                g_current_pose.pose.position.z,
            };
            td.velocity = {0.0, 0.0, 0.0};
            td.initialized = true;
            last_td_update = ros::Time::now();
        }
        if (use_td_setpoint && td.initialized) {
            const ros::Time now = ros::Time::now();
            const double dt = std::max(0.001, (now - last_td_update).toSec());
            last_td_update = now;
            update_td(target, dt, td_bandwidth_rad_s, td_accel_limit_mps2, td_vel_limit_mps, td);
            target_pose = make_target_pose(td.position);
        } else {
            target_pose = make_target_pose(target);
        }
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
