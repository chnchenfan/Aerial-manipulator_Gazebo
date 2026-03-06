#include "uav_control/TrajectoryGenerator.hpp"
#include <iostream>

TrajectoryGenerator::TrajectoryGenerator(const TrajectoryParameters& params)
    : params_(params) {}

// --- 辅助函数: S-Curve (Sin^2) 生成 ---
Eigen::Vector3d TrajectoryGenerator::computeSmoothScalar(double t, double total_time, double target_val) {
    Eigen::Vector3d res = Eigen::Vector3d::Zero();

    if (t <= 0) return res;
    if (t >= total_time) {
        res[0] = target_val;
        return res;
    }

    double omega = M_PI / (2.0 * total_time);
    double angle = omega * t;
    double s = sin(angle);
    double c = cos(angle);

    res[0] = target_val * s * s;                                // Pos
    res[1] = target_val * 2.0 * s * c * omega;                  // Vel
    res[2] = target_val * 2.0 * omega * omega * (c*c - s*s);    // Acc
    return res;
}

// =================================================================
// 接口 1: 位置/速度/加速度 参考生成
// =================================================================
void TrajectoryGenerator::update(double t, TrajectoryMode mode,
                                 Eigen::Vector3d& pos,
                                 Eigen::Vector3d& vel,
                                 Eigen::Vector3d& acc)
{
    pos.setZero(); vel.setZero(); acc.setZero();

    // --- Mode 1: 悬停 ---
    if (mode == TrajectoryMode::MODE1_HOVER) {
        Eigen::Vector3d z_state = computeSmoothScalar(t, params_.duration, params_.target_height);
        pos.z() = z_state[0]; vel.z() = z_state[1]; acc.z() = z_state[2];
    }
    // --- Mode 2: 画圆 ---
    else if (mode == TrajectoryMode::MODE2_CIRCLE) {
        double T1 = params_.duration;
        double T2 = 3.0;

        if (t < T1) {
            Eigen::Vector3d z_state = computeSmoothScalar(t, T1, params_.target_height);
            pos.z() = z_state[0]; vel.z() = z_state[1]; acc.z() = z_state[2];
        } else {
            pos.z() = params_.target_height;
        }

        if (t < T1) {
            pos.x() = 0; pos.y() = 0;
        } else if (t < T1 + T2) {
            Eigen::Vector3d x_state = computeSmoothScalar(t - T1, T2, params_.circle_radius);
            pos.x() = x_state[0]; vel.x() = x_state[1]; acc.x() = x_state[2];
            pos.y() = 0;
        } else {
            double t_circle = t - (T1 + T2);
            double w = params_.circle_speed / params_.circle_radius;
            pos.x() = params_.circle_radius * cos(w * t_circle);
            pos.y() = params_.circle_radius * sin(w * t_circle);
            vel.x() = -params_.circle_radius * w * sin(w * t_circle);
            vel.y() =  params_.circle_radius * w * cos(w * t_circle);
            acc.x() = -params_.circle_radius * w * w * cos(w * t_circle);
            acc.y() = -params_.circle_radius * w * w * sin(w * t_circle);
        }
    }
    // --- Mode 3: 螺旋 ---
    else if (mode == TrajectoryMode::MODE3_SPIRAL) {
        double T_total = params_.duration;
        if (t < T_total) {
            Eigen::Vector3d z_state = computeSmoothScalar(t, T_total, params_.target_height);
            pos.z() = z_state[0]; vel.z() = z_state[1]; acc.z() = z_state[2];

            double target_angle = 2.0 * M_PI * params_.spiral_turns;
            Eigen::Vector3d ang_state = computeSmoothScalar(t, T_total, target_angle);
            double phi = ang_state[0];
            double phi_dot = ang_state[1];
            double phi_ddot = ang_state[2];
            double R = params_.circle_radius;

            pos.x() = R * cos(phi);
            pos.y() = R * sin(phi);
            vel.x() = -R * sin(phi) * phi_dot;
            vel.y() =  R * cos(phi) * phi_dot;
            acc.x() = -R * (cos(phi)*phi_dot*phi_dot + sin(phi)*phi_ddot);
            acc.y() =  R * (-sin(phi)*phi_dot*phi_dot + cos(phi)*phi_ddot);
        } else {
            pos.z() = params_.target_height;
            pos.x() = params_.circle_radius * cos(2.0 * M_PI * params_.spiral_turns);
            pos.y() = params_.circle_radius * sin(2.0 * M_PI * params_.spiral_turns);
        }
    }
    // --- Mode 4: 姿态测试 (位置保持悬停) ---
    else if (mode == TrajectoryMode::MODE4_ATTITUDE_SINE) {
        // 在做姿态测试时，位置环只要给一个固定的悬停点即可，防止飞机因为没有位置设定而乱飘
        pos.z() = params_.target_height;
        pos.x() = 0.0;
        pos.y() = 0.0;
        // 速度加速度设为0
        vel.setZero();
        acc.setZero();
    }
}

// =================================================================
// 接口 2: 姿态/角速度/角加速度 参考生成 (新增)
// =================================================================
void TrajectoryGenerator::updateAttitude(double t, TrajectoryMode mode,
                                         Eigen::Quaterniond& q_ref,
                                         Eigen::Vector3d& omega_ref,
                                         Eigen::Vector3d& alpha_ref)
{
    // 初始化为平飞静止状态
    q_ref = Eigen::Quaterniond(1, 0, 0, 0); // w, x, y, z
    omega_ref.setZero();
    alpha_ref.setZero();

    // 只有在 Mode 4 下生成正弦姿态，其他模式默认平飞 (或由位置控制器决定姿态)
    if (mode == TrajectoryMode::MODE4_ATTITUDE_SINE) {
        double A = params_.att_test_amp;
        double f = params_.att_test_freq;
        double w = 2.0 * M_PI * f;

        // 生成 Roll 轴正弦波: phi(t) = A * sin(w * t)
        double angle      = A * std::sin(w * t);
        double angle_dot  = A * w * std::cos(w * t);
        double angle_ddot = -A * w * w * std::sin(w * t);

        // 1. 期望四元数 (绕 X 轴旋转)
        q_ref = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());

        // 2. 期望角速度 (Body Frame)
        // 对于纯 Roll 转动，Body Frame 下的角速度就是 [phi_dot, 0, 0]
        omega_ref << angle_dot, 0.0, 0.0;

        // 3. 期望角加速度 (Body Frame)
        alpha_ref << angle_ddot, 0.0, 0.0;
    }
}
