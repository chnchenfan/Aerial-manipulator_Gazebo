/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ESOAttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>


class ESOAttitudeControl
{
public:
	ESOAttitudeControl() = default;
	~ESOAttitudeControl() = default;

	/** 设置姿态外环比例增益和偏航权重，把外部的增益值传递进来，命名为内部的变量名
	 * Set proportional attitude control gain
	 * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
	 * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and pitch
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/** 设置姿态外环角速度(控制器输出)限制，把外部的增益值传递进来，命名为内部的变量名
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/** 设置期望值，把外部的增益值传递进来，命名为内部的变量名
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();// 归一化，防止数值漂移导致的误差
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/** 设置期望值自适应，防止因为坐标系漂移导致的突变，把外部的增益值传递进来，命名为内部的变量名
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		// 1. 把“世界变化的量”(q_delta) 叠加到“当前目标”(_attitude_setpoint_q) 上
		// 数学上：四元数相乘 = 旋转叠加
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;

		// 2. 再次归一化，防止数据计算误差（老规矩，保命操作）
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @param omega current body angular velocity [rad/s] (used to compute tau_s)
	 * @param dt control loop time step [s] (used to计算 omega_r 数值导数)
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	matrix::Vector3f update(const matrix::Quatf &q, const matrix::Vector3f &omega, float dt);

	/**
	 * 获取上一次调用 update 计算得到的 omega_r 导数
	 */
	const matrix::Vector3f &omegaRDerivative() const { return _omega_r_dot; }

	/**
	 * 重置 omega_r 数值求导状态（避免解锁/模式切换/首次运行时的导数尖峰）
	 * - 清零导数输出
	 * - 标记上一周期 omega_r 无效，下一次 update 会自动重新对齐 omega_r_prev
	 */
	void resetOmegaRDerivative()
	{
		_omega_r_dot.zero();
		_omega_r_prev.zero();
		_omega_r_prev_valid = false;
	}

	/**
	 * 获取最近一次计算的 beta_v（姿态误差向量部分）
	 */
	const matrix::Vector3f &betaVector() const { return _beta_v; }

	/**
	 * 设置 (31) 所需的补偿参数：
	 * @param mass_total (m_B + m_M) 总质量 [kg]
	 * @param com_offset p_C^B 质心偏移（机体系）[m]
	 * @param arm_inertia M_M^B 机械臂/载荷惯性矩阵（机体系）[kg*m^2]
	 */
	void setTauSParameters(float mass_total, const matrix::Vector3f &com_offset, const matrix::Matrix3f &arm_inertia)
	{
		_mass_total = mass_total;
		_com_offset = com_offset;
		_arm_inertia = arm_inertia;
	}

	/**
	 * 获取最近一次计算的 tau_s（补偿力矩）
	 */
	const matrix::Vector3f &tauS() const { return _tau_s; }

private:
	matrix::Vector3f _proportional_gain;
	matrix::Vector3f _rate_limit;
	float _yaw_w{0.f}; ///< yaw weight [0,1] to deprioritize caompared to roll and pitch

	matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
	float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint
	// float _rollspeed_setpoint{0.f}; /// < latest known rollspeed feed-forward setpoint
	// float _pitchspeed_setpoint{0.f}; /// < latest known pitchspeed feed-forward setpoint

	matrix::Vector3f _omega_r_prev{}; ///< 上一周期的参考角速度
	matrix::Vector3f _omega_r_dot{};  ///< 数值导数
	bool _omega_r_prev_valid{false};  ///< omega_r_prev 是否已经初始化
	matrix::Vector3f _beta_v{}; ///< 最近一次的 beta_v

	// (31) tau_s 补偿参数与缓存
	float _mass_total{0.f};           ///< (m_B + m_M)
	matrix::Vector3f _com_offset{};   ///< p_C^B
	matrix::Matrix3f _arm_inertia{};  ///< M_M^B
	matrix::Vector3f _tau_s{};        ///< 最近一次计算的 tau_s
};
