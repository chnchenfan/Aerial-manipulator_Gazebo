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
 * @file ESORateControl.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>
#include <ESORateControl/AttitudeESO.hpp>

class ESORateControl
{
public:
	ESORateControl() = default;
	~ESORateControl() = default;

	/** 设置PID参数，把外部的增益值传递进来，命名为内部的变量名
	 * Set the rate control gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/** 设置积分限幅，把外部的增益值传递进来，命名为内部的变量名
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/** 设置前馈增益，把外部的增益值传递进来，命名为内部的变量名
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/** 抗饱和
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @param landed：飞机是不是落地了（落地了就要清空积分，防止怠速时疯转）。
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed);

	/**
	 * 设置来自姿态环的附加量
	 * @param omega_r_dot 姿态环计算的参考角速度导数 [rad/s^2]
	 * @param beta_v      姿态误差向量部分（几何误差）[-]
	 * @param tau_s       (31) 中的补偿力矩 [N*m]
	 */
	void setAttitudeAux(const matrix::Vector3f &omega_r_dot, const matrix::Vector3f &beta_v, const matrix::Vector3f &tau_s)
	{
		_omega_r_dot = omega_r_dot;
		_beta_v = beta_v;
		_tau_s = tau_s;
	}

	/**
	 * 设置刚体惯性对角（M），会同步更新逆矩阵。
	 * @param inertia_diag [Ixx, Iyy, Izz] 单位 kg*m^2
	 */
	void setInertiaDiagonal(const matrix::Vector3f &inertia_diag);

	// [NEW] 动态设置全惯量矩阵 (用于 VIPM)
	void setInertiaMatrix(const matrix::Matrix3f &inertia);

	void setESOBandwidth(const matrix::Vector3f &eso_bw);

	/**
	 * 设置 beta_v 反馈系数 k_beta（公式(45)）。
	 */
	void setKBeta(float k_beta);


	/**
	 * 设置物理力矩归一化系数 max_torque。
	 */
	void setMaxTorque(float max_torque);

	/**
	 * 设置积分通道输出缩放系数（小积分比例）。
	 * 实际加入力矩的积分项为：integral_term = _rate_int * integral_scale
	 */
	void setIntegralScale(float integral_scale);

	/**
	 * 设置 tau_s 软启用系数（[0,1]）。
	 */
	void setTauSScale(float tau_s_scale);

		/**
		 * 设置 tau_s 分轴缩放/符号。最终注入为：
		 * tau_s_base = constrain(tau_s, +/- lim) * ESO_TAUS_K * axis_scale
		 */
		void setTauSAxisScale(const matrix::Vector3f &tau_s_axis_scale);

		/**
		 * 设置 tau_s 给 ESO 名义模型的分轴比例/符号。
		 */
		void setTauSObserverAxisScale(const matrix::Vector3f &tau_s_obs_axis_scale);

		/**
		 * 设置 tau_s 给控制律直接补偿的分轴比例/符号。
		 */
		void setTauSControlAxisScale(const matrix::Vector3f &tau_s_ctrl_axis_scale);

	/**
	 * 设置 tau_s 单轴绝对限幅（N*m）。
	 */
	void setTauSLimitNm(float tau_s_limit_nm);

	/**
	 * 设置 tau_s 注入低通时间常数（s）。
	 */
	void setTauSFilterTimeConstant(float tau_s_filter_tau);

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral() { _rate_int.zero(); } // 重置积分（比如刚解锁时清零）

	/**
	 * 重置 ESO 观测器与物理力矩缓存
	 * 用于“未解锁/刚解锁/落地”等阶段，避免观测器积累和历史力矩导致起飞瞬间突变。
	 */
	void resetESO()
	{
		_eso.reset();
		_last_torque.zero();
		_omega_r_dot.zero();
		_beta_v.zero();
		_tau_s.zero();
	}

	/** 获取ESO估计的角速度 */
	matrix::Vector3f getESOEstimatedAngularVelocity() const { return _eso.getEstimatedAngularVelocity(); }

	/** 获取ESO估计的扰动力矩 */
	matrix::Vector3f getESOEstimatedDisturbance() const { return _eso.getEstimatedDisturbance(); }

	/**
	 * 获取上一周期力矩分量（用于调试各项贡献）
	 */
	void getLastTorqueTerms(matrix::Vector3f &inertia_term,
				matrix::Vector3f &feedback_term,
				matrix::Vector3f &integral_term,
				matrix::Vector3f &beta_term) const
	{
		inertia_term = _last_inertia_term;
		feedback_term = _last_feedback_term;
		integral_term = _last_integral_term;
		beta_term = _last_beta_term;
	}

	/**
	 * 获取 tau_s 与观测器相关调试量（上一周期）
	 */
	void getLastTauSDebug(matrix::Vector3f &tau_s_raw,
			      matrix::Vector3f &tau_s_used,
			      matrix::Vector3f &omega_r_dot,
			      matrix::Vector3f &disturbance_hat) const
	{
		tau_s_raw = _last_tau_s_raw;
		tau_s_used = _last_tau_s_used;
		omega_r_dot = _last_omega_r_dot;
		disturbance_hat = _last_disturbance_hat;
	}

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getESORateControlStatus(rate_ctrl_status_s &rate_ctrl_status); // 把内部数据倒出来做黑匣子记录

private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_int; ///< integral term of the rate controller

	// Aux inputs from attitude controller (cached)
	matrix::Vector3f _omega_r_dot{}; ///< d/dt omega_r
	matrix::Vector3f _beta_v{};      ///< geometric attitude error vector part
	matrix::Vector3f _tau_s{};       ///< tau_s compensation torque

	// System model and ESO states (parameterized defaults matching eso_att_control1)
	matrix::Matrix3f _inertia{matrix::diag(matrix::Vector3f(0.045f, 0.045f, 0.08f))}; ///< base inertia M
	matrix::Matrix3f _inertia_inv{_inertia.I()};
	AttitudeESO _eso{30.f, 30.f, 30.f}; ///< ESO bandwidths (wp_x, wp_y, wp_z)
	float _k_beta{0.5f}; ///< k_beta feedback gain
	float _max_torque{1.5f}; ///< max torque for normalization [N*m]
	float _integral_scale{0.2f}; ///< 积分输出缩放（小积分比例）
		float _tau_s_scale{0.f}; ///< tau_s 软启用比例 [0,1]
		matrix::Vector3f _tau_s_axis_scale{1.f, 1.f, 1.f}; ///< tau_s 共同分轴缩放/符号
		matrix::Vector3f _tau_s_obs_axis_scale{1.f, 1.f, 1.f}; ///< tau_s 给 ESO 名义模型的分轴比例/符号
		matrix::Vector3f _tau_s_ctrl_axis_scale{1.f, 1.f, 1.f}; ///< tau_s 给控制律直接补偿的分轴比例/符号
		float _tau_s_limit_nm{0.3f}; ///< tau_s 单轴绝对限幅 [N*m]
	float _tau_s_filter_tau{0.1f}; ///< tau_s 注入低通时间常数 [s]
	bool _tau_s_filter_valid{false}; ///< tau_s 低通状态是否已初始化
	matrix::Vector3f _tau_s_filtered{}; ///< 低通后的 tau_s 实际注入量
	matrix::Vector3f _last_torque{}; ///< last physical torque command [N*m]
	matrix::Vector3f _last_rate{}; ///< last measured body angular rate [rad/s]
	matrix::Vector3f _last_rate_sp{}; ///< last reference body angular rate [rad/s]
	matrix::Vector3f _last_inertia_term{};  ///< 上一周期惯性项
	matrix::Vector3f _last_feedback_term{}; ///< 上一周期速率误差反馈项
	matrix::Vector3f _last_integral_term{}; ///< 上一周期积分项
	matrix::Vector3f _last_beta_term{};     ///< 上一周期 beta 耦合项
	matrix::Vector3f _last_tau_s_raw{}; ///< 上一周期 tau_s 原始输入
	matrix::Vector3f _last_tau_s_used{}; ///< 上一周期 tau_s 实际使用值
	matrix::Vector3f _last_omega_r_dot{}; ///< 上一周期 omega_r 导数
	matrix::Vector3f _last_disturbance_hat{}; ///< 上一周期 ESO 扰动估计

	// Feedback (记录电机有没有满负荷)
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;
};
