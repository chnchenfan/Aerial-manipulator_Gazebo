/**
 * @file eso_square_arm_experiment_node.cpp
 * @brief Square trajectory offboard node with altitude-triggered arm excitation.
 */

#include <algorithm>
#include <cmath>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace {

struct Waypoint {
    double x;
    double y;
    double z;
};

struct TrackingDifferentiator {
    Waypoint position;
    Waypoint velocity;
    bool initialized{false};
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

double distance_to_waypoint(const geometry_msgs::PoseStamped& pose, const Waypoint& wp) {
    const double dx = pose.pose.position.x - wp.x;
    const double dy = pose.pose.position.y - wp.y;
    const double dz = pose.pose.position.z - wp.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double waypoint_distance(const Waypoint& a, const Waypoint& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Waypoint interpolate_waypoint(const Waypoint& a, const Waypoint& b, double ratio) {
    ratio = std::max(0.0, std::min(1.0, ratio));
    return {
        a.x + (b.x - a.x) * ratio,
        a.y + (b.y - a.y) * ratio,
        a.z + (b.z - a.z) * ratio,
    };
}

double smoothstep(double ratio) {
    ratio = std::max(0.0, std::min(1.0, ratio));
    return ratio * ratio * (3.0 - 2.0 * ratio);
}

double clamp_value(double value, double limit) {
    if (limit <= 0.0) {
        return value;
    }
    return std::max(-limit, std::min(limit, value));
}

void reset_td(TrackingDifferentiator& td, const Waypoint& position) {
    td.position = position;
    td.velocity = {0.0, 0.0, 0.0};
    td.initialized = true;
}

void update_td_axis(double target, double dt, double bandwidth_rad_s, double acc_limit,
                    double vel_limit, double& position, double& velocity) {
    const double error = target - position;
    double acceleration = bandwidth_rad_s * bandwidth_rad_s * error -
                          2.0 * bandwidth_rad_s * velocity;
    acceleration = clamp_value(acceleration, acc_limit);
    velocity = clamp_value(velocity + acceleration * dt, vel_limit);
    position += velocity * dt;
}

void update_td(TrackingDifferentiator& td, const Waypoint& target, double dt,
               double bandwidth_rad_s, double acc_limit, double vel_limit) {
    if (!td.initialized || dt <= 0.0) {
        reset_td(td, target);
        return;
    }

    update_td_axis(target.x, dt, bandwidth_rad_s, acc_limit, vel_limit, td.position.x, td.velocity.x);
    update_td_axis(target.y, dt, bandwidth_rad_s, acc_limit, vel_limit, td.position.y, td.velocity.y);
    update_td_axis(target.z, dt, bandwidth_rad_s, acc_limit, vel_limit, td.position.z, td.velocity.z);
}

void fill_pose_setpoint(geometry_msgs::PoseStamped& msg, const Waypoint& position) {
    msg.pose.position.x = position.x;
    msg.pose.position.y = position.y;
    msg.pose.position.z = position.z;
    msg.pose.orientation.w = 1.0;
}

void fill_raw_setpoint(mavros_msgs::PositionTarget& msg, const Waypoint& position, const Waypoint& velocity) {
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask =
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    msg.position.x = position.x;
    msg.position.y = position.y;
    msg.position.z = position.z;
    msg.velocity.x = velocity.x;
    msg.velocity.y = velocity.y;
    msg.velocity.z = velocity.z;
}

}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "eso_square_arm_experiment_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    ros::Publisher raw_setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>(
        "mavros/setpoint_raw/local", 10);
    ros::Publisher arm_enable_pub = nh.advertise<std_msgs::Bool>(
        "experiment/arm_motion_enabled", 1, true);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");

    double side_length = 2.0;
    double altitude = 2.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double corner_hold_s = 3.0;
    double path_speed_mps = 0.09621358687658534;
    double reach_tol_m = 0.2;
    double activation_altitude_m = 1.9;
    double activation_hold_s = 1.0;
    double startup_delay_s = 0.0;
    double static_validation_delay_s = -1.0;
    double static_validation_reach_m = -1.0;
    bool smooth_segments = false;
    bool use_raw_setpoint = true;
    bool use_td_setpoint = true;
    double velocity_ff_scale = 0.008279872451230363;
    double td_bandwidth_hz = 0.32880281660571076;
    double td_accel_limit_mps2 = 0.19318573839061587;
    double td_vel_limit_mps = 0.22063069013967385;

    pnh.param("side_length", side_length, side_length);
    pnh.param("altitude", altitude, altitude);
    pnh.param("origin_x", origin_x, origin_x);
    pnh.param("origin_y", origin_y, origin_y);
    pnh.param("corner_hold_s", corner_hold_s, corner_hold_s);
    pnh.param("path_speed_mps", path_speed_mps, path_speed_mps);
    pnh.param("reach_tol_m", reach_tol_m, reach_tol_m);
    pnh.param("activation_altitude_m", activation_altitude_m, activation_altitude_m);
    pnh.param("activation_hold_s", activation_hold_s, activation_hold_s);
    pnh.param("startup_delay_s", startup_delay_s, startup_delay_s);
    pnh.param("static_validation_delay_s", static_validation_delay_s, static_validation_delay_s);
    pnh.param("static_validation_reach_m", static_validation_reach_m, static_validation_reach_m);
    pnh.param("smooth_segments", smooth_segments, smooth_segments);
    pnh.param("use_raw_setpoint", use_raw_setpoint, use_raw_setpoint);
    pnh.param("use_td_setpoint", use_td_setpoint, use_td_setpoint);
    pnh.param("velocity_ff_scale", velocity_ff_scale, velocity_ff_scale);
    pnh.param("td_bandwidth_hz", td_bandwidth_hz, td_bandwidth_hz);
    pnh.param("td_accel_limit_mps2", td_accel_limit_mps2, td_accel_limit_mps2);
    pnh.param("td_vel_limit_mps", td_vel_limit_mps, td_vel_limit_mps);
    path_speed_mps = std::max(0.05, path_speed_mps);
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

    const std::vector<Waypoint> waypoints = {
        {origin_x, origin_y, altitude},
        {origin_x + side_length, origin_y, altitude},
        {origin_x + side_length, origin_y + side_length, altitude},
        {origin_x, origin_y + side_length, altitude},
        {origin_x, origin_y, altitude},
    };

    std::size_t wp_idx = 0;
    geometry_msgs::PoseStamped target_pose;
    mavros_msgs::PositionTarget target_raw;
    TrackingDifferentiator td;
    reset_td(td, waypoints[wp_idx]);
    fill_pose_setpoint(target_pose, waypoints[wp_idx]);
    fill_raw_setpoint(target_raw, waypoints[wp_idx], td.velocity);
    ros::Time last_setpoint_time = ros::Time::now();

    for (int i = 100; ros::ok() && i > 0; --i) {
        const ros::Time stamp = ros::Time::now();
        if (use_raw_setpoint) {
            target_raw.header.stamp = stamp;
            raw_setpoint_pub.publish(target_raw);
        } else {
            target_pose.header.stamp = stamp;
            local_pos_pub.publish(target_pose);
        }
        ros::spinOnce();
        rate.sleep();
    }

    if (startup_delay_s > 0.0) {
        ROS_INFO("Delaying offboard arming by %.2f s", startup_delay_s);
        const ros::Time delay_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - delay_start) < ros::Duration(startup_delay_s)) {
            const ros::Time stamp = ros::Time::now();
            if (use_raw_setpoint) {
                target_raw.header.stamp = stamp;
                raw_setpoint_pub.publish(target_raw);
            } else {
                target_pose.header.stamp = stamp;
                local_pos_pub.publish(target_pose);
            }
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
    ros::Time phase_start;
    bool activation_window_started = false;
    bool validation_window_started = false;
    bool arm_motion_enabled = false;
    bool moving_segment = false;
    bool square_complete = false;
    std_msgs::Bool arm_enable_msg;
    arm_enable_msg.data = false;
    arm_enable_pub.publish(arm_enable_msg);

    ROS_INFO("Square tracking experiment initialized at altitude %.2f m, path speed %.2f m/s, smooth segments %s, raw setpoint %s, TD %s, velocity FF scale %.2f, TD bw %.2f Hz",
             altitude, path_speed_mps, smooth_segments ? "enabled" : "disabled",
             use_raw_setpoint ? "enabled" : "disabled",
             use_td_setpoint ? "enabled" : "disabled",
             velocity_ff_scale, td_bandwidth_hz);

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
                 distance_to_waypoint(g_current_pose, waypoints.front()) <= static_validation_reach_m);
            if (!validation_window_started && validation_ready) {
                validation_window_started = true;
                validation_since = ros::Time::now();
                ROS_INFO("Static validation altitude reached, waiting %.2f s before marker",
                         static_validation_delay_s);
            }
            if (validation_window_started &&
                ros::Time::now() - validation_since >= ros::Duration(static_validation_delay_s)) {
                arm_motion_enabled = true;
                phase_start = ros::Time::now();
                reset_td(td, waypoints.front());
                last_setpoint_time = phase_start;
                arm_enable_msg.data = true;
                arm_enable_pub.publish(arm_enable_msg);
                ROS_INFO("Static validation marker published, square tracking begins");
            } else if (validation_window_started && !validation_ready) {
                validation_window_started = false;
                ROS_INFO("Static validation readiness lost, restarting marker timer");
            }
        } else if (!arm_motion_enabled && g_have_pose && static_validation_delay_s < 0.0) {
            const Waypoint& takeoff_wp = waypoints.front();
            const bool reached_altitude =
                g_current_pose.pose.position.z >= activation_altitude_m &&
                distance_to_waypoint(g_current_pose, takeoff_wp) <= reach_tol_m;

            if (reached_altitude) {
                if (!activation_window_started) {
                    activation_window_started = true;
                    activation_since = ros::Time::now();
                    ROS_INFO("Reached takeoff corner, waiting %.2f s before enabling arm motion",
                             activation_hold_s);
                } else if (ros::Time::now() - activation_since >= ros::Duration(activation_hold_s)) {
                    arm_motion_enabled = true;
                    phase_start = ros::Time::now();
                    reset_td(td, waypoints.front());
                    last_setpoint_time = phase_start;
                    arm_enable_msg.data = true;
                    arm_enable_pub.publish(arm_enable_msg);
                    ROS_INFO("Arm motion enabled, square tracking begins");
                }
            } else {
                activation_window_started = false;
            }
        } else if (!arm_motion_enabled) {
            activation_window_started = false;
        }

        Waypoint active_wp = waypoints[wp_idx];

        if (arm_motion_enabled && !square_complete) {
            const ros::Duration phase_elapsed = ros::Time::now() - phase_start;

            if (!moving_segment) {
                active_wp = waypoints[wp_idx];

                if (phase_elapsed >= ros::Duration(corner_hold_s)) {
                    if (wp_idx + 1 >= waypoints.size()) {
                        square_complete = true;
                    } else {
                        moving_segment = true;
                        phase_start = ros::Time::now();
                    }
                }
            } else {
                const Waypoint& start_wp = waypoints[wp_idx];
                const Waypoint& end_wp = waypoints[wp_idx + 1];
                const double segment_time_s = waypoint_distance(start_wp, end_wp) / path_speed_mps;
                const double ratio = phase_elapsed.toSec() / segment_time_s;
                const double interp_ratio = smooth_segments ? smoothstep(ratio) : ratio;
                active_wp = interpolate_waypoint(start_wp, end_wp, interp_ratio);

                if (ratio >= 1.0) {
                    ++wp_idx;
                    moving_segment = false;
                    phase_start = ros::Time::now();
                    active_wp = waypoints[wp_idx];
                }
            }
        }

        const ros::Time stamp = ros::Time::now();
        const double dt = (stamp - last_setpoint_time).toSec();
        last_setpoint_time = stamp;
        if (use_td_setpoint) {
            update_td(td, active_wp, dt, td_bandwidth_rad_s, td_accel_limit_mps2, td_vel_limit_mps);
        } else {
            reset_td(td, active_wp);
        }

        const Waypoint published_velocity = {
            td.velocity.x * velocity_ff_scale,
            td.velocity.y * velocity_ff_scale,
            td.velocity.z * velocity_ff_scale,
        };

        if (use_raw_setpoint) {
            fill_raw_setpoint(target_raw, td.position, published_velocity);
            target_raw.header.stamp = stamp;
            raw_setpoint_pub.publish(target_raw);
        } else {
            fill_pose_setpoint(target_pose, td.position);
            target_pose.header.stamp = stamp;
            local_pos_pub.publish(target_pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
