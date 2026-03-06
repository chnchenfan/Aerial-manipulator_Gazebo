#include "uav_control/ControlUtils.hpp"
#include <cmath>
#include <tf/transform_datatypes.h> // 需要在 CMakeLists.txt 里加 tf 依赖

namespace ControlUtils {

    // =========================================================================
    // 函数 1 实现: 力 -> 姿态映射
    // =========================================================================
    std::pair<geometry_msgs::Quaternion, double> forceToAttitudeThrust(
        const Eigen::Vector3d& force_vec_enu,
        double target_yaw_rad,
        double max_thrust_newton)
    {
        // 1. 计算推力模长
        double thrust_force = force_vec_enu.norm();

        // 2. 计算归一化油门 (0.0 ~ 1.0)
        if (max_thrust_newton < 0.1) max_thrust_newton = 1.0; // 防止除零

        double thrust_norm = std::sqrt(thrust_force / max_thrust_newton);

        // 饱和限制
        if (thrust_norm > 1.0) thrust_norm = 1.0;
        if (thrust_norm < 0.0) thrust_norm = 0.0;

        // 3. 构建旋转矩阵 (SO3)
        // b3: 机身 Z 轴方向 (推力方向)
        Eigen::Vector3d z_body;
        if (thrust_force < 0.001) {
            z_body << 0, 0, 1; // 无力时默认向上
        } else {
            z_body = force_vec_enu.normalized();
        }

        // 构造辅助向量 a (只含 Yaw 信息)
        Eigen::Vector3d x_ref(cos(target_yaw_rad), sin(target_yaw_rad), 0.0);

        // b2: 机身 Y 轴 = z_body X x_ref
        Eigen::Vector3d y_body = z_body.cross(x_ref);
        // 奇异点保护
        if (y_body.norm() < 0.001) {
            y_body << -sin(target_yaw_rad), cos(target_yaw_rad), 0;
        } else {
            y_body.normalize();
        }

        // b1: 机身 X 轴 = y_body X z_body
        Eigen::Vector3d x_body = y_body.cross(z_body);

        // 填入旋转矩阵
        Eigen::Matrix3d R;
        R.col(0) = x_body;
        R.col(1) = y_body;
        R.col(2) = z_body;

        // 4. 转为四元数
        Eigen::Quaterniond q(R);
        geometry_msgs::Quaternion q_msg;
        q_msg.w = q.w();
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();

        return {q_msg, thrust_norm};
    }


    // =========================================================================
    // 函数 2 实现: 四元数 -> 欧拉角
    // =========================================================================
    Eigen::Vector3d quaternionToEuler(const geometry_msgs::Quaternion& q_msg) {
        tf::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return Eigen::Vector3d(roll, pitch, yaw);
    }


    // =========================================================================
    // 函数 3 实现: 力向量限幅
    // =========================================================================
    Eigen::Vector3d saturateForce(const Eigen::Vector3d& force_vec, double limit_newton) {
        double current_norm = force_vec.norm();
        if (current_norm > limit_newton) {
            return force_vec.normalized() * limit_newton;
        }
        return force_vec;
    }


    // =========================================================================
    // 函数 4 实现: 质心计算
    // =========================================================================
    Eigen::Vector3d calculateCoM(
        const std::vector<double>& masses,
        const std::vector<Eigen::Vector3d>& positions)
    {
        if (masses.size() != positions.size() || masses.empty()) {
            // 错误保护：如果数据对不上，返回 0
            return Eigen::Vector3d::Zero();
        }

        Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
        double total_mass = 0.0;

        for (size_t i = 0; i < masses.size(); ++i) {
            weighted_sum += masses[i] * positions[i];
            total_mass += masses[i];
        }

        if (total_mass < 0.001) return Eigen::Vector3d::Zero(); // 防止除零

        return weighted_sum / total_mass;
    }






}
