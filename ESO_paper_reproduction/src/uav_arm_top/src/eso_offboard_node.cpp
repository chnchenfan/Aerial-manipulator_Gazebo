/**
 * @file eso_offboard_node.cpp
 * @brief ROS node for testing eso_pos_control module in PX4
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// 全局变量存储当前飞控状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eso_offboard_node");
    ros::NodeHandle nh;

    // 1. 订阅连接状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
/*
消息类型区别：

geometry_msgs::PoseStamped（用于 /mavros/setpoint_position/local）：
    只包含位置（x, y, z）和姿态（四元数，表示 yaw 等方向）。
    适合简单的位置控制 + yaw 控制。
    MAVROS 内部会将其转换为 MAVLink 的 SET_POSITION_TARGET_LOCAL_NED 消息，但固定处理方式：总是发送位置和 yaw，忽略速度、加速度等字段（相当于 type_mask 隐式设置，只关注位置和 yaw）。

mavros_msgs::PositionTarget（用于 /mavros/setpoint_raw/local）：
    更完整，直接对应 MAVLink 的 SET_POSITION_TARGET_LOCAL_NED（消息 ID 84）。
    包含：位置（position）、速度（velocity）、加速度/力（acceleration_or_force）、yaw、yaw_rate。
    有 type_mask 字段（uint16 位掩码），可以显式忽略某些字段，例如：
    IGNORE_YAW = 1024：忽略 yaw（不控制偏航角）。
    IGNORE_YAW_RATE = 2048：忽略 yaw_rate。
    其他如 IGNORE_VX/VY/VZ 等，用于只控制部分量（位置 + 速度混合等）。
    支持更多控制模式：纯位置、纯速度、位置+速度、加速度前馈等。


话题区别：

/mavros/setpoint_position/local：
    简单易用，主要用于Offboard 模式下的基本位置控制（带 yaw）。
    yaw 总是被设置（通过 Pose 的四元数提取），无法显式忽略 yaw。
    如果你的控制中 yaw 设置不当，可能导致轨迹发散或电机差速异常（因为 FCU 会试图跟踪指定的 yaw）。

/mavros/setpoint_raw/local：
    更灵活的“raw”版本，直接暴露 SET_POSITION_TARGET_LOCAL_NED 的全部功能。
    通过 type_mask 可以精确控制哪些字段有效，例如设置 type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW 来完全忽略 yaw（让 FCU 保持当前 yaw 或自由）。
    正如你的代码注释所述，适合验证“是否因为 yaw 设定导致发散/对角电机差速”的场景。
    支持速度控制、混合控制等高级用法。

*/
    // // 2. 发布位置期望值 (Local NED)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // // 2. 发布位置期望值（Raw Local，能够直接设置 type_mask）
    // // 目的：把 yaw 明确设为“忽略”，验证是否因为 yaw 设定导致发散/对角电机差速。
    // // 对应 MAVLink: SET_POSITION_TARGET_LOCAL_NED (MSG_ID=84)
    // ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(
    //         "mavros/setpoint_raw/local", 10);

    // 3. 服务客户端：用于解锁和切换模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // 设置发送频率 (必须 > 2Hz，否则 PX4 会自动切出 Offboard 模式)
    ros::Rate rate(20.0);

    // 等待 MAVROS 连接到飞控
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection...");
    }

    // 设置目标点：起飞到 2米高度
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.5;

    // // 设置目标点：起飞到 2 米高度
    // mavros_msgs::PositionTarget sp{};
    // sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // sp.position.x = 0.f;
    // sp.position.y = 0.f;
    // sp.position.z = 2.f;

    // type_mask=3576：只控制 position(x,y,z)，忽略 velocity/acceleration/yaw/yaw_rate
    // 2552 = IGNORE_VX|VY|VZ|AFX|AFY|AFZ|YAW_RATE
    // 3576 = 2552 + IGNORE_YAW
    // sp.type_mask =
    //         mavros_msgs::PositionTarget::IGNORE_VX |
    //         mavros_msgs::PositionTarget::IGNORE_VY |
    //         mavros_msgs::PositionTarget::IGNORE_VZ |
    //         mavros_msgs::PositionTarget::IGNORE_AFX |
    //         mavros_msgs::PositionTarget::IGNORE_AFY |
    //         mavros_msgs::PositionTarget::IGNORE_AFZ |
    //         mavros_msgs::PositionTarget::IGNORE_YAW |
    //         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // 在切换到 Offboard 模式之前，必须先发送一些设定点
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        // sp.header.stamp = ros::Time::now();
        // local_pos_pub.publish(sp);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // 逻辑：如果没切到 Offboard，就请求切；如果没解锁，就请求解锁
        // 为了安全，间隔 5秒 请求一次
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 持续发布目标点给 eso_pos_control
        local_pos_pub.publish(pose);
        // sp.header.stamp = ros::Time::now();
        // local_pos_pub.publish(sp);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
