#ifndef CONTROL_UTILS_HPP
#define CONTROL_UTILS_HPP

#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <utility> // for std::pair

namespace ControlUtils {

    // =========================================================================
    // 函数 1: 核心控制映射 (Force -> Attitude & Thrust)
    // =========================================================================
    /**
     * @brief [核心算法] 将期望的力向量映射为 PX4 可执行的姿态四元数和油门
     * * 原理: 基于微分平坦特性，利用叉乘构建 SO(3) 旋转矩阵。
     * 对应论文: Eq. (34)
     * @param force_vec_enu     期望控制力向量 (牛顿, ENU系, 需包含重力补偿)
     * @param target_yaw_rad    期望偏航角 (弧度)
     * @param max_thrust_newton 飞机最大物理推力 (牛顿, 用于归一化)
     * @return std::pair<四元数, 油门0-1>
     * * // --- 使用示例 ---
     * Eigen::Vector3d force_cmd(1.0, 2.0, 15.0); // 计算出的力
     * double max_thrust = 30.0; // 假设最大推力30N
     * auto result = ControlUtils::forceToAttitude(force_cmd, 0.0, max_thrust);
     * mavros_msgs::AttitudeTarget msg;
     * msg.orientation = result.first;  // 赋值四元数
     * msg.thrust = result.second;      // 赋值油门
     */
    std::pair<geometry_msgs::Quaternion, double> forceToAttitudeThrust(
        const Eigen::Vector3d& force_vec_enu,
        double target_yaw_rad,
        double max_thrust_newton
    );


    // =========================================================================
    // 函数 2: 调试辅助 (Quaternion -> Euler)
    // =========================================================================
    /**
     * @brief [调试用] 将四元数转换为欧拉角 (Roll, Pitch, Yaw)
     * * 用途: ESO 控制器里全是四元数，人眼看不懂。用这个转成角度，
     * 方便在终端打印或用 rqt_plot 画图，检查飞机是不是飞歪了。
     * @param q_msg ROS 格式的四元数消息
     * @return Eigen::Vector3d (x=Roll, y=Pitch, z=Yaw) 单位: 弧度
     * * // --- 使用示例 ---
     * // 在回调函数里收到 current_pose
     * Eigen::Vector3d rpy = ControlUtils::quaternionToEuler(current_pose.pose.orientation);
     * ROS_INFO("Current Pitch: %.2f deg", rpy.y() * 180.0 / M_PI);
     */
    Eigen::Vector3d quaternionToEuler(const geometry_msgs::Quaternion& q_msg);


    // =========================================================================
    // 函数 3: 安全限幅 (Safety Saturation)
    // =========================================================================
    /**
     * @brief [安全保护] 对计算出的控制力进行球形限幅
     * * 用途: 防止 PID 或 ESO 瞬间算出 1000N 的力，导致数值爆炸。
     * 它会保持力的方向不变，只缩短长度。
     * @param force_vec    原始力向量
     * @param limit_newton 允许的最大力 (牛顿)
     * @return Eigen::Vector3d 限幅后的力向量
     * * // --- 使用示例 ---
     * Eigen::Vector3d u_unsafe = pid_out + eso_out; // 可能很大
     * // 限制最大只能输出 40N
     * Eigen::Vector3d u_safe = ControlUtils::saturateForce(u_unsafe, 40.0);
     */
    Eigen::Vector3d saturateForce(const Eigen::Vector3d& force_vec, double limit_newton);


    // =========================================================================
    // 函数 4: 系统质心计算 (Center of Mass)
    // =========================================================================
    /**
     * @brief 根据各部件质量和位置，计算系统总质心
     * 公式: p_com = sum(m_i * p_i) / sum(m_i)
     * @param masses     各部件质量列表 (kg)
     * @param positions  各部件位置列表 (机身坐标系下的向量)
     * @return Eigen::Vector3d 系统总质心位置
     */
    Eigen::Vector3d calculateCoM(
        const std::vector<double>& masses,
        const std::vector<Eigen::Vector3d>& positions
    );

}

#endif // CONTROL_UTILS_HPP


/*
 * ======================================================================================
 * 【开发指南】如何确定最大推力参数 (max_thrust_newton) ？
 * ======================================================================================
 * * 作用：
 * 控制器计算输出的是物理力（单位：牛顿 N），而 PX4 接收的是归一化油门（0.0 ~ 1.0）。
 * 我们需要这个参数来建立映射关系： 油门 = 期望力 / 最大推力^2
 * * 设定不准的后果：
 * - 设太大（如真实20N，设成60N）：计算出需要10N悬停，映射油门仅为 0.16，飞机飞不起来（掉高）。
 * - 设太小（如真实20N，设成10N）：计算出需要10N悬停，映射油门变成 1.0，飞机直接冲顶（飞飞）。
 * * --------------------------------------------------------------------------------------
 * 方法 1：基于 SDF 仿真参数计算 (最精准的仿真值)
 * --------------------------------------------------------------------------------------
 * 原理：直接读取 gazebo 模型文件 (.sdf) 中的电机参数进行计算。
 * 公式：F_total = 电机数量 * (motorConstant * maxRotVelocity^2)
 * * [案例 - 当前 uav_arm_v4.sdf 参数]
 * - motorConstant = 5.84e-06
 * - maxRotVelocity = 1100 (转/秒)
 * - 单电机推力 = 5.84e-06 * 1100^2 ≈ 7.06 N
 * - 整机最大推力 = 7.06 * 4 ≈ 28.24 N
 * * 建议值：28.0 ~ 30.0
 * * --------------------------------------------------------------------------------------
 * 方法 2：悬停油门反推法 (最实用的工程调参法)
 * --------------------------------------------------------------------------------------
 * 原理：先飞起来，看悬停时用了多少油门，反推总推力。
 * 公式：Max_Thrust = (飞机总重 * 9.8) / 悬停油门(0~1)
 * * [案例 - 轻量级配置]
 * - 飞机总重 ≈ 1.25 kg (重力 ≈ 12.25 N)
 * - 实测悬停油门 ≈ 0.5 (50%)
 * - 反推最大推力 = 12.25 / 0.5 = 24.5 N
 * * 建议值：根据实际悬停表现动态调整。如果发现悬停油门偏大(>0.6)，说明Max_Thrust设大了，需调小。
 * * --------------------------------------------------------------------------------------
 * 方法 3：基于论文/实物参数估算 (理论设计值)
 * --------------------------------------------------------------------------------------
 * 原理：根据设计指标中的“推重比”来估算。通常推重比设计为 1.5 ~ 2.0 倍。
 * 公式：Max_Thrust = 飞机总重 * 9.8 * 推重比
 * * [案例 - 论文原始配置]
 * - 飞机总重 = 4.6 kg (3.6kg 机身 + 1.0kg 机械臂)
 * - 重力 G = 45 N
 * - 假设推重比 1.5 -> Max_Thrust ≈ 67.5 N
 * * 注意：此方法仅适用于完全复刻论文物理参数（修改了SDF质量）的情况。
 * 如果使用默认 SDF 模型（1.25kg），用这个值会导致严重掉高。
 * ======================================================================================
 */
