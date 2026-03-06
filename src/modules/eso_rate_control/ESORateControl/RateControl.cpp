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
 * @file ESORateControl.cpp
 */

#include <ESORateControl/RateControl.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

void ESORateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void ESORateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				      const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void ESORateControl::setInertiaDiagonal(const Vector3f &inertia_diag)
{
	_inertia.setZero();
	_inertia(0, 0) = inertia_diag(0);
	_inertia(1, 1) = inertia_diag(1);
	_inertia(2, 2) = inertia_diag(2);

	_inertia_inv.setZero();
	if (fabsf(_inertia(0,0)) > 1e-6f && fabsf(_inertia(1,1)) > 1e-6f && fabsf(_inertia(2,2)) > 1e-6f) {
		_inertia_inv(0,0) = 1.0f / _inertia(0,0);
		_inertia_inv(1,1) = 1.0f / _inertia(1,1);
		_inertia_inv(2,2) = 1.0f / _inertia(2,2);
	}
}

// [NEW] 动态惯量更新实现
void ESORateControl::setInertiaMatrix(const Matrix3f &inertia)
{
	_inertia = inertia;

	// 求逆 (用于 ESO 名义模型输入计算)
	// 如果矩阵接近奇异，保持旧值或者使用 backup 对角逆
	// 这里使用 I() 计算逆矩阵
	Matrix3f inv{};
	if (_inertia.I(inv)) {
		_inertia_inv = inv;

	} else {
		const float ixx = _inertia(0, 0);
		const float iyy = _inertia(1, 1);
		const float izz = _inertia(2, 2);

		if (fabsf(ixx) > 1e-6f && fabsf(iyy) > 1e-6f && fabsf(izz) > 1e-6f) {
			Matrix3f diag_inv{};
			diag_inv.setZero();
			diag_inv(0, 0) = 1.0f / ixx;
			diag_inv(1, 1) = 1.0f / iyy;
			diag_inv(2, 2) = 1.0f / izz;
			_inertia_inv = diag_inv;
		}
	}
}

void ESORateControl::setESOBandwidth(const Vector3f &bandwidth)
{
	Vector3f bw_safe{};

	for (int i = 0; i < 3; i++) {
		const float val = bandwidth(i);
		bw_safe(i) = (PX4_ISFINITE(val) && val >= 0.f) ? val : 0.f;
	}

	// 重建 ESO 以更新 beta1/beta2，并清零内部状态
	_eso = AttitudeESO(bw_safe(0), bw_safe(1), bw_safe(2));
}

void ESORateControl::setKBeta(float k_beta)
{
	if (PX4_ISFINITE(k_beta) && k_beta >= 0.f) {
		_k_beta = k_beta;
	}
}

void ESORateControl::setMaxTorque(float max_torque)
{
	if (PX4_ISFINITE(max_torque) && max_torque > 1e-3f) {
		_max_torque = max_torque;
	}
}

void ESORateControl::setIntegralScale(float integral_scale)
{
	if (PX4_ISFINITE(integral_scale) && integral_scale >= 0.f) {
		_integral_scale = integral_scale;
	}
}

void ESORateControl::setTauSScale(float tau_s_scale)
{
	if (PX4_ISFINITE(tau_s_scale) && tau_s_scale >= 0.f) {
		_tau_s_scale = tau_s_scale;
	}
}

void ESORateControl::setTauSLimitNm(float tau_s_limit_nm)
{
	if (PX4_ISFINITE(tau_s_limit_nm) && tau_s_limit_nm >= 0.f) {
		_tau_s_limit_nm = tau_s_limit_nm;
	}
}

Vector3f ESORateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
                             const float dt, const bool landed)
{
	// ---------------------------------------------------------------------
	// 速率环 ESO 控制（严格对应论文公式(45)与 eso_att_control1）
	//
	// 论文(45)：
	//   τ = M( ω̇_r - Δ̂ ) + ω × (M ω) - K_ω r_ω - k_β β̃_v - τ_s
	//
	// 其中：
	// - ω      : 当前机体角速度（rate）
	// - ω_r    : 姿态环给出的参考角速度（rate_sp）
	// - ω̇_r   : 参考角速度导数（由姿态环数值求导下传）
	// - r_ω    : 速率跟踪误差 = ω - ω_r
	// - β̃_v   : 姿态几何误差向量部分（由姿态环下传）
	// - τ_s    : (31) 的补偿力矩（由姿态环计算下传）
	// - M      : 机体惯性矩阵（这里用 eso_att_control1 的对角惯性）
	// - Δ̂     : ESO 估计的扰动
	//
	// 注意：这里先按物理单位计算 τ[N*m]，最后再归一化到 [-1, 1] 输出，
	// 归一化系数 max_torque 可通过参数 ESO_MAX_TORQUE 配置，默认沿用 eso_att_control1 的 1.5 N*m。
	// ---------------------------------------------------------------------

	const float k_beta = _k_beta;
	const float max_torque = math::max(_max_torque, 1e-3f);

	const Vector3f omega = rate;
	const Vector3f omega_r = rate_sp;

	// aux 输入做有限性保护，避免上游没发/发 NaN 时污染控制
	Vector3f omega_r_dot = _omega_r_dot;
	Vector3f beta_v = _beta_v;
	Vector3f tau_s = _tau_s;
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(omega_r_dot(i))) { omega_r_dot(i) = 0.f; }
		if (!PX4_ISFINITE(beta_v(i))) { beta_v(i) = 0.f; }
		if (!PX4_ISFINITE(tau_s(i))) { tau_s(i) = 0.f; }
	}
	Vector3f tau_s_used = tau_s;
	const float tau_s_scale = math::constrain(_tau_s_scale, 0.f, 1.f);
	const float tau_s_limit_nm = math::max(_tau_s_limit_nm, 0.f);
	for (int i = 0; i < 3; i++) {
		tau_s_used(i) = math::constrain(tau_s_used(i), -tau_s_limit_nm, tau_s_limit_nm);
	}
	tau_s_used *= tau_s_scale;

		// 速率误差统一定义为 e_omega = ω_r - ω（与积分通道符号一致）
		const Vector3f e_omega = omega_r - omega;

	// ω × (M ω)  —— 陀螺/科氏项
	const Vector3f coriolis_term = omega.cross(_inertia * omega);

	// landed 时清 ESO/上一力矩，避免地面抖动累积
	if (landed) {
		_eso.reset();
		_last_torque.zero();
	}

	// ESO 名义输入：u = M^{-1}(τ_last - ω×Mω + tau_s_used)
	const Vector3f u_nominal = _inertia_inv * (_last_torque - coriolis_term + tau_s_used);
	const Vector3f disturbance_hat = _eso.update(omega, u_nominal, dt);
	_last_tau_s_raw = tau_s;
	_last_tau_s_used = tau_s_used;
	_last_omega_r_dot = omega_r_dot;
	_last_disturbance_hat = disturbance_hat;

	// Debug: print actual vs estimated rate (and disturbance if valid), throttled
	static hrt_abstime last_print{0};
	if (hrt_elapsed_time(&last_print) > 2000000) {//2秒一次
		last_print = hrt_absolute_time();

		const Vector3f est_rate = _eso.getEstimatedAngularVelocity();
		const bool dist_valid = PX4_ISFINITE(disturbance_hat(0)) && PX4_ISFINITE(disturbance_hat(1)) && PX4_ISFINITE(disturbance_hat(2));

		if (dist_valid) {
			PX4_INFO("ESO rate: w[%.2f %.2f %.2f] w_hat[%.2f %.2f %.2f]",
				 (double)omega(0), (double)omega(1), (double)omega(2),
				 (double)est_rate(0), (double)est_rate(1), (double)est_rate(2));
		} else {
			PX4_INFO("ESO rate: w[%.2f %.2f %.2f] w_hat[%.2f %.2f %.2f]",
				 (double)omega(0), (double)omega(1), (double)omega(2),
				 (double)est_rate(0), (double)est_rate(1), (double)est_rate(2));
		}
	}

		// 小积分通道：仅在飞行阶段积分，避免地面/异常 dt 下积累
		if (!landed && PX4_ISFINITE(dt) && dt > 1e-6f) {
			Vector3f e_omega_for_int = e_omega;
			updateIntegral(e_omega_for_int, dt); // 内部已做抗饱和与限幅
		}

		// 物理力矩 τ（N*m）
		const Vector3f inertia_term = _inertia * (omega_r_dot - disturbance_hat);
		const Vector3f feedback_term = _gain_p.emult(e_omega);
		// 采用“小积分”策略：比例由参数 ESO_RATE_I_SC 配置
		const Vector3f integral_term = _rate_int * _integral_scale;
		const Vector3f beta_term = -beta_v * k_beta;
		const Vector3f torque_physical = inertia_term + coriolis_term + feedback_term + beta_term + integral_term - tau_s_used;
		// 缓存各力矩分量，供外层低频调试打印分析“谁在主导”
		_last_inertia_term = inertia_term;
		_last_feedback_term = feedback_term;
		_last_integral_term = integral_term;
		_last_beta_term = beta_term;
		_last_torque = torque_physical;

	// 归一化输出到 [-1, 1]（与 PX4 混控接口一致）
	Vector3f torque_norm{};
	for (int i = 0; i < 3; i++) {
		torque_norm(i) = math::constrain(torque_physical(i) / max_torque, -1.f, 1.f);
	}

	// 目前不再使用 legacy PID 的 I/D/FF 通道，保持 updateIntegral 供后续可选启用。
	return torque_norm;
}

void ESORateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
        // 遍历 Roll, Pitch, Yaw 三个轴
        for (int i = 0; i < 3; i++) {

                // --- 抗积分饱和 (Anti-Windup) 开始 ---

                // 情况 A：如果在正方向饱和了（电机推满了）
                if (_control_allocator_saturation_positive(i)) {
                        // 强行把误差限制在 <= 0。
                        // 意思是：如果还要往正方向推，我不积了；但如果是往回推（负误差），我允许积分减小。
                        rate_error(i) = math::min(rate_error(i), 0.f);
                }

                // 情况 B：如果在负方向饱和了
                if (_control_allocator_saturation_negative(i)) {
                        // 强行把误差限制在 >= 0。同理，只允许积分退回来。
                        rate_error(i) = math::max(rate_error(i), 0.f);
                }

                // --- 动态积分增益 (Gain Scheduling) ---

                // 这是一个很高明的技巧！防止大幅度机动（如翻滚）后的“反弹”。
                // 逻辑：如果误差很大（比如 > 200度/秒），我就把 I 的作用关小一点。

                // 1. 计算归一化误差：当前误差 / 400度(弧度制)
                float i_factor = rate_error(i) / math::radians(400.f);

                // 2. 计算衰减系数：是一个抛物线形状。
                // 误差越小，i_factor 越接近 1（正常积分）。
                // 误差越大，i_factor 越接近 0（暂停积分）。
                i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

                // --- 执行积分 ---
                // 新积分 = 旧积分 + (衰减系数 * I增益 * 误差 * 时间dt)
                float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

                // --- 安全性检查 ---
                // 1. 确保算出来的数不是无穷大(NaN/Inf)
                if (PX4_ISFINITE(rate_i)) {
                        // 2. 积分限幅：不管积了多少，不能超过 _lim_int
                        _rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
                }
        }
}

void ESORateControl::getESORateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
