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
 * @file AttitudeControl.cpp
 */

#include <ESOAttitudeControl/AttitudeControl.hpp>
#include <px4_platform_common/defines.h>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

void ESOAttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	// 第一步：把用户设置的P值先存下来
	_proportional_gain = proportional_gain;

	// 第二步：把偏航权重限制在 0 到 1 之间
	// 防止用户输入负数或者超过1的奇怪数值
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// 第三步：【关键点】除以权重，对偏航（Yaw）的P值进行修改
	if (_yaw_w > 1e-4f) { // 如果权重不是0（防止除以0导致死机）
		_proportional_gain(2) /= _yaw_w; // 这里的(2)代表Z轴，也就是偏航轴
	}
	/*权重存在的意义：
		当飞机在飞行过程中，如果同时出现 Roll（翻滚）和 Yaw（转头）的误差时，
		我们希望控制器优先把精力放在 Roll 上，保证飞机的水平平衡，而不是一味地去纠正 Yaw。

		为什么要这样做呢？因为在多旋翼飞行器中，Roll 和 Pitch 的稳定性对飞行安全性更为关键，
		而 Yaw 的误差通常不会直接导致失控。通过降低 Yaw 的优先级，可以让控制器更有效地
		分配资源，确保飞机在复杂环境下的稳定飞行。
	“乘权重”作用在“姿态解算”阶段：
		这是一个规划层。当飞机同时需要翻滚（Roll）和转头（Yaw）时，因为 Yaw 的误差被缩小了（假象），算法会决定：“优先把能量分配给 Roll（因为它误差看起来更大），Yaw 可以先放一放。”
		目的达成：成功降低了 Yaw 的优先级，保住了飞机的水平平衡。

	“除权重”作用在“控制输出”阶段：
		这是一个执行层。虽然规划层把 Yaw 排到了后面，但一旦轮到 Yaw 执行时，我们需要它按原本设定的力度去执行，不能软绵绵的。
		目的达成：保住了用户的 P 值手感，不会因为权重设置而改变灵敏度。
	*/
}

matrix::Vector3f ESOAttitudeControl::update(const Quatf &q, const Vector3f &omega, float dt)
{
	// 对输入角速度做有限性保护，避免上游异常导致 tau_s 产生 NaN/Inf
	Vector3f omega_body = omega;
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(omega_body(i))) {
			omega_body(i) = 0.f;
		}
	}

	// 1. [健壮性] 数据清洗：归一化四元数
	// 虽然上游通常会给归一化数据，但在做数学运算前再次确保是好习惯
	matrix::Quatf q_curr = q;
	if (q_curr.norm_squared() > 0.f) {
		q_curr.normalize();
	} else {
		// 异常处理：如果当前姿态无效，保持原样或给个单位阵，防止除0崩溃
		q_curr = matrix::Quatf();
	}

	matrix::Quatf q_target = _attitude_setpoint_q;
	if (q_target.norm_squared() > 0.f) {
		q_target.normalize();
	}

	// 2. [前馈] 构造目标角速度向量 (World Frame)
	// 检查数值有效性 (ISFINITE)，如果上游没发数据，默认为0
	matrix::Vector3f target_omega_world(0.f, 0.f, 0.f);
	if (PX4_ISFINITE(_yawspeed_setpoint)) {
		target_omega_world(2) = _yawspeed_setpoint;
	}

	// 如果未来上层支持 Roll/Pitch 前馈，可以在这里扩展：
	// if (PX4_ISFINITE(_rollspeed_setpoint)) target_omega_world(0) = _rollspeed_setpoint;

	// 3. [核心算法] 几何控制误差计算
	// 对应论文：R_tilde = R_d^T * R
	matrix::Dcmf R(q_curr);
	matrix::Dcmf R_d(q_target);
	matrix::Dcmf R_tilde = R_d.transpose() * R;

	// 4. [转换] 提取误差向量 beta_v
	matrix::Quatf q_tilde(R_tilde);

	// [符号修正] 保证 w > 0，处理四元数双倍覆盖特性 (Double Cover)
	// 这一步至关重要，否则姿态差180度时会反转控制方向
	if (q_tilde(0) < 0.f) {
		q_tilde = q_tilde * -1.0f;
	}

	_beta_v = Vector3f(q_tilde(1), q_tilde(2), q_tilde(3));

	// 将 Yaw 轴的几何误差缩小。
	// 这改变了误差向量的方向，使其更偏向 Roll/Pitch 平面。
	// 在大角度耦合运动中，这会让控制器优先消除 Roll/Pitch 误差。
	if (_yaw_w > 1e-4f) {
		_beta_v(2) *= _yaw_w;
	}

	// 5. [投影] 计算前馈项在机体系的投影
	// 公式：R_tilde^T * (R_d^T * w_world)
	// 数学简化：(R^T R_d) * (R_d^T * w_world) = R^T * w_world
	// 物理含义：把世界坐标系下的期望角速度，投影到当前机体坐标系上
	// 这一步代替了原版代码中复杂的 if (is_finite...) 逻辑
	matrix::Vector3f R_tilde_T_omega_d = R.transpose() * target_omega_world;

	// 5.1 [补偿力矩 tau_s] 优化版：仅重力力矩补偿
	/*
		原论文(31)包含陀螺项：tau_s = gravity_torque - ω×(M_arm*ω)

		但由于速率环 RateControl 的 coriolis_term = ω×(M_full*ω) 已使用全系统惯量，
		其中已包含机械臂惯量的贡献。若 tau_s 再包含 ω×(M_arm*ω)，会导致双重补偿。

		因此这里只保留重力力矩补偿，陀螺效应由速率环统一处理。

		坐标系约定：PX4 使用 NED 世界系，重力指向 +Z（向下），g = [0, 0, 9.81]。
	*/
	const Vector3f gravity_world{0.f, 0.f, 9.81f};
	const Vector3f g_body = R.transpose() * gravity_world; // R^{-1} g
	const Vector3f gravity_torque = _com_offset.cross(g_body) * _mass_total;
	_tau_s = gravity_torque;  // 只保留重力力矩，移除陀螺项
	// 再做一次有限性保护：如果任一分量非有限，整项置零，保证下游力矩律不被污染
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(_tau_s(i))) {
			_tau_s(i) = 0.f;
		}
	}

	// 6. [控制律] 计算参考角速度 (omega_r)
	// 公式：omega_r = FeedForward - 2 * Gain * Error
	matrix::Vector3f omega_r = R_tilde_T_omega_d - _proportional_gain.emult(_beta_v) * 2.0f;

	// 7. [保护] 输出限幅
	// 防止计算出的角速度超过飞机的物理极限（或者参数设定的极限）
	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(omega_r(i))) {
		omega_r(i) = math::constrain(omega_r(i), -_rate_limit(i), _rate_limit(i));
		} else {
		omega_r(i) = 0.f; // 如果算出 NaN，强制置零，保护电机
		}
	}

	// 8. 数值求导（供速率环使用），避免 dt 异常
	if (!_omega_r_prev_valid) {
		// 首次运行/刚复位：对齐上一周期 omega_r，导数输出置零，避免导数尖峰
		_omega_r_dot.zero();
		_omega_r_prev = omega_r;
		_omega_r_prev_valid = true;

	} else if (PX4_ISFINITE(dt) && dt > 1e-4f) {
		constexpr float omega_r_dot_lpf_tau = 0.02f;
		constexpr float omega_r_slew_time = 0.1f;
		const float alpha = math::constrain(dt / (omega_r_dot_lpf_tau + dt), 0.f, 1.f);
		const Vector3f omega_r_dot_raw = (omega_r - _omega_r_prev) / dt;

		for (int i = 0; i < 3; i++) {
			const float max_omega_r_dot = math::max(_rate_limit(i) / omega_r_slew_time, 1.f);
			const float omega_r_dot_limited = PX4_ISFINITE(omega_r_dot_raw(i))
							 ? math::constrain(omega_r_dot_raw(i), -max_omega_r_dot, max_omega_r_dot)
							 : 0.f;
			_omega_r_dot(i) += alpha * (omega_r_dot_limited - _omega_r_dot(i));
		}

		_omega_r_prev = omega_r;

	} else {
		// dt 异常：直接复位求导状态，下一次重新对齐
		resetOmegaRDerivative();
		_omega_r_prev = omega_r;
		_omega_r_prev_valid = true;
	}

	return omega_r;
}
