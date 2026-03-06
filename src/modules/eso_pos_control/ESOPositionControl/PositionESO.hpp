/**
 * PositionESO.hpp
 * * 修正版说明：
 * 1. 升级为 Vector3f 向量化实现，单次 Update 处理 XYZ 三轴。
 * 2. 支持三轴独立设置带宽 (Bandwidth)，以适应不同的轴向动态。
 */

#pragma once

#include <lib/matrix/matrix/math.hpp>

class PositionESO {
public:
    PositionESO();
    ~PositionESO() = default;

    /**
     * @brief 设置ESO带宽
     * @param bw 三轴独立的带宽向量 (rad/s), 建议格式: Vector3f(xy_bw, xy_bw, z_bw)
     */
    void setBandwidth(const matrix::Vector3f& bw);

    /**
     * @brief 重置内部状态 (在切入 Offboard 模式或起飞前调用)
     */
    void reset();

    /**
     * @brief 重置并用测量初始化内部状态（避免启用瞬间导数尖峰）
     *
     * 用途：当 ESO 在起飞后才“启用”时，如果仍从 0 开始，会在短时间内产生较大的估计误差 err，
     * 进而把 z2/z3 推得很大（相当于导数尖峰/扰动尖峰），导致位置环输出突变、姿态打满。
     *
     * @param pos_meas 当前位置测量 [m]
     * @param vel_meas 当前速度测量 [m/s]
     */
    void reset(const matrix::Vector3f &pos_meas, const matrix::Vector3f &vel_meas);

    /**
     * @brief 核心更新函数
     * @param pos_meas  位置测量值 (Measurement), [m]
     * @param u_control 当前施加的加速度控制量 (Input), [m/s^2]
     * @param dt        控制周期 [s]
     * @return matrix::Vector3f 估计出的总扰动 (Disturbance Estimate)
     */
    matrix::Vector3f update(const matrix::Vector3f& pos_meas,
                            const matrix::Vector3f& u_control,
                            float dt);

    // --- Getters (获取状态) ---
    matrix::Vector3f getEstimatedPosition() const { return _z1; }
    matrix::Vector3f getEstimatedVelocity() const { return _z2; }
    matrix::Vector3f getEstimatedDisturbance() const { return _z3; }

private:
    // --- 增益参数 (Gains) ---
    matrix::Vector3f _bw;    // 带宽
    matrix::Vector3f _beta1; // 观测器增益 1
    matrix::Vector3f _beta2; // 观测器增益 2
    matrix::Vector3f _beta3; // 观测器增益 3

    // --- 内部状态 (States) ---
    matrix::Vector3f _z1;    // 估计位置 (Position Estimate)
    matrix::Vector3f _z2;    // 估计速度 (Velocity Estimate)
    matrix::Vector3f _z3;    // 估计干扰 (Lumped Disturbance Estimate)
};
