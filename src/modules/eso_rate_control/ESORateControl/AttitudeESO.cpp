#include "AttitudeESO.hpp"

AttitudeESO::AttitudeESO(float wp_x, float wp_y, float wp_z) {
    bandwidth_(0) = wp_x;
    bandwidth_(1) = wp_y;
    bandwidth_(2) = wp_z;

    // beta1 = 2 * w
    beta1_ = bandwidth_ * 2.0f;

    // beta2 = w^2 (使用 emult 进行元素对元素乘法)
    beta2_ = bandwidth_.emult(bandwidth_);

    z1_est_omega_.setZero();
    z2_est_dist_.setZero();
}

void AttitudeESO::reset() {
    z1_est_omega_.setZero();
    z2_est_dist_.setZero();
}

matrix::Vector3f AttitudeESO::update(const matrix::Vector3f& omega_meas,
                                     const matrix::Vector3f& u_input,
                                     float dt) {
    // 1. Error
    matrix::Vector3f err = omega_meas - z1_est_omega_;

    // 2. State Derivatives
    // dot_hat_omega = u + hat_Delta + beta1 .* err
    matrix::Vector3f z1_dot = u_input + z2_est_dist_ + beta1_.emult(err);

    // dot_hat_Delta = beta2 .* err
    matrix::Vector3f z2_dot = beta2_.emult(err);

    // 3. Euler Integration
    z1_est_omega_ += z1_dot * dt;
    z2_est_dist_  += z2_dot * dt;

    return z2_est_dist_;
}

matrix::Vector3f AttitudeESO::getEstimatedAngularVelocity() const {
    return z1_est_omega_;
}

matrix::Vector3f AttitudeESO::getEstimatedDisturbance() const {
    return z2_est_dist_;
}
