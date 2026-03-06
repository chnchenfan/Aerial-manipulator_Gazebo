#ifndef ATTITUDE_ESO_HPP
#define ATTITUDE_ESO_HPP

#include <lib/matrix/matrix/math.hpp>

class AttitudeESO {
public:
    AttitudeESO(float wp_x, float wp_y, float wp_z);

    matrix::Vector3f update(const matrix::Vector3f& omega_meas,
                            const matrix::Vector3f& u_input,
                            float dt);

    void reset();

    matrix::Vector3f getEstimatedAngularVelocity() const;
    matrix::Vector3f getEstimatedDisturbance() const;

private:
    matrix::Vector3f bandwidth_;
    matrix::Vector3f beta1_;
    matrix::Vector3f beta2_;

    matrix::Vector3f z1_est_omega_;
    matrix::Vector3f z2_est_dist_;
};

#endif // ATTITUDE_ESO_HPP
