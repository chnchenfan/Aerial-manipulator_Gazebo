/**
 * PositionESO.cpp
 */

#include "PositionESO.hpp"

PositionESO::PositionESO() {
    // 初始化状态为 0
    reset();

    // 设置默认带宽 (防止未设置参数时计算出错)
    // 默认值: XY轴 8.0, Z轴 4.0 (经验值，请根据实际参数覆盖)
    setBandwidth(matrix::Vector3f(8.0f, 8.0f, 4.0f));
}

void PositionESO::setBandwidth(const matrix::Vector3f& bw) {
    _bw = bw;

    // 参数化 LESO (Linear ESO) 增益
    // 对应特征多项式 (s + w)^3
    // 使用 .emult() 进行元素对元素的乘法 (Element-wise multiplication)

    // Beta1 = 3 * w
    _beta1 = _bw * 3.0f;

    // Beta2 = 3 * w^2
    _beta2 = _bw.emult(_bw) * 3.0f;

    // Beta3 = w^3
    _beta3 = _bw.emult(_bw).emult(_bw);
}

void PositionESO::reset() {
    _z1.setZero();
    _z2.setZero();
    _z3.setZero();
}

void PositionESO::reset(const matrix::Vector3f &pos_meas, const matrix::Vector3f &vel_meas)
{
	_z1 = pos_meas;
	_z2 = vel_meas;
	_z3.setZero();
}

matrix::Vector3f PositionESO::update(const matrix::Vector3f& pos_meas,
                                     const matrix::Vector3f& u_control,
                                     float dt) {
    // 1. 计算观测误差 (Estimation Error)
    // error = Measurement - Estimate
    matrix::Vector3f err = pos_meas - _z1;

    // 2. 状态微分方程 (State Derivatives)
    // 根据 Luenberger 观测器形式: dot_x = Ax + Bu + L(y - Cx)
    // 对应 Offboard 代码逻辑

    // dot_z1 = z2 + beta1 * err
    matrix::Vector3f z1_dot = _z2 + _beta1.emult(err);

    // dot_z2 = z3 + u + beta2 * err
    // u_control 是控制器的输出加速度
    matrix::Vector3f z2_dot = _z3 + u_control + _beta2.emult(err);

    // dot_z3 = beta3 * err
    // 干扰项的变化率由观测误差驱动
    matrix::Vector3f z3_dot = _beta3.emult(err);

    // 3. 欧拉积分更新 (Euler Integration)
    _z1 += z1_dot * dt;
    _z2 += z2_dot * dt;
    _z3 += z3_dot * dt;

    // 返回估计出的干扰
    return _z3;
}
