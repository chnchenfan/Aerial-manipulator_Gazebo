/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "ESOMulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/eso_rate_aux.h>

#define MODULE_NAME "eso_rate_control"

using namespace matrix;
using namespace time_literals;
using math::radians;

ESOMulticopterRateControl::ESOMulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

ESOMulticopterRateControl::~ESOMulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
ESOMulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
ESOMulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_eso_rollrate_k.get(), _param_eso_pitchrate_k.get(), _param_eso_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_eso_rollrate_p.get(), _param_eso_pitchrate_p.get(), _param_eso_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_eso_rollrate_i.get(), _param_eso_pitchrate_i.get(), _param_eso_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_eso_rollrate_d.get(), _param_eso_pitchrate_d.get(), _param_eso_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_eso_rr_int_lim.get(), _param_eso_pr_int_lim.get(), _param_eso_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_eso_rollrate_ff.get(), _param_eso_pitchrate_ff.get(), _param_eso_yawrate_ff.get()));

	_active_profile = eso_common::resolveModelProfile(_param_eso_arm_model.get());
	_arm_kinematics.setModelProfile(_active_profile);

	if (_active_profile == eso_common::ModelProfile::UamV5) {
		const eso_common::DynamicsProfile &profile = eso_common::getDynamicsProfile(_active_profile);
		_static_inertia = profile.staticSystemInertiaMatrix();
		_arm_kinematics.setParameters(profile.mass_total, profile.comVector(), profile.bodyInertiaMatrix());

	} else {
		const Vector3f inertia_diag(_param_eso_inertia_xx.get(), _param_eso_inertia_yy.get(), _param_eso_inertia_zz.get());
		_static_inertia = matrix::diag(inertia_diag);
		_arm_kinematics.setParameters(eso_common::kUavArmV4Profile.mass_total,
					      eso_common::kUavArmV4Profile.comVector(),
					      _static_inertia);
	}

	_rate_control.setInertiaDiagonal(
		Vector3f(_static_inertia(0, 0), _static_inertia(1, 1), _static_inertia(2, 2)));

	_rate_control.setESOBandwidth(
		Vector3f(_param_eso_rate_bw_r.get(), _param_eso_rate_bw_p.get(), _param_eso_rate_bw_y.get()));

	_dyn_ff_enabled = _param_eso_dyn_ff_en.get();
	_rate_control.setKBeta(_param_eso_k_beta.get());
	_rate_control.setMaxTorque(_param_eso_max_torque.get());
	_rate_control.setIntegralScale(_param_eso_rate_i_scale.get());
	_rate_control.setTauSScale(_param_eso_taus_k.get());
	_rate_control.setTauSAxisScale(Vector3f(_param_eso_taus_k_r.get(), _param_eso_taus_k_p.get(), _param_eso_taus_k_y.get()));
	_rate_control.setTauSObserverAxisScale(Vector3f(_param_eso_taus_obs_r.get(), _param_eso_taus_obs_p.get(), _param_eso_taus_obs_y.get()));
	_rate_control.setTauSControlAxisScale(Vector3f(_param_eso_taus_ctl_r.get(), _param_eso_taus_ctl_p.get(), _param_eso_taus_ctl_y.get()));
	_rate_control.setTauSLimitNm(_param_eso_taus_lim.get());
	_rate_control.setTauSFilterTimeConstant(_param_eso_taus_tau.get());

	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_eso_acro_r_max.get()), radians(_param_eso_acro_p_max.get()),
				  radians(_param_eso_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}


void ESOMulticopterRateControl::update_dynamic_parameters()
{
	arm_joint_states_s joints;
	if (_arm_joint_states_sub.update(&joints)) {
		if (!joints.valid) {
			_rate_control.setInertiaMatrix(_static_inertia);
			return;
		}

		// 计算实时惯量
		Matrix3f dynamic_inertia = _arm_kinematics.computeSystemInertia(joints.q);

		// 更新速率控制器内部惯量 (影响 ESO 和 P 阵)
		_rate_control.setInertiaMatrix(dynamic_inertia);

		// Debug
		// PX4_INFO_LIMIT(2, "Dyn Inertia Ixx: %.4f Iyy: %.4f", (double)dynamic_inertia(0,0), (double)dynamic_inertia(1,1));
	}
}


void
ESOMulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// [NEW] 尝试更新动态惯量
		update_dynamic_parameters();

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		if (_landing_gear_sub.updated()) {
			landing_gear_s landing_gear;

			if (_landing_gear_sub.copy(&landing_gear)) {
				if (landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
					if (landing_gear.landing_gear == landing_gear_s::GEAR_UP && (_landed || _maybe_landed)) {
						mavlink_log_critical(&_mavlink_log_pub, "Landed, unable to retract landing gear\t");
						events::send(events::ID("eso_rate_control_not_retract_landing_gear_landed"),
						{events::Log::Error, events::LogInternal::Info},
						"Landed, unable to retract landing gear");

					} else {
						_landing_gear = landing_gear.landing_gear;
					}
				}
			}
		}

		if (_v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.y, _param_eso_acro_expo.get(), _param_eso_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.x, _param_eso_acro_expo.get(), _param_eso_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.r, _param_eso_acro_expo_y.get(), _param_eso_acro_supexpoy.get())};

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = math::constrain(manual_control_setpoint.z, 0.0f, 1.0f);

				// publish rate setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = PX4_ISFINITE(v_rates_sp.roll)  ? v_rates_sp.roll  : rates(0);
				_rates_sp(1) = PX4_ISFINITE(v_rates_sp.pitch) ? v_rates_sp.pitch : rates(1);
				_rates_sp(2) = PX4_ISFINITE(v_rates_sp.yaw)   ? v_rates_sp.yaw   : rates(2);
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}
		}

		// 订阅姿态环附加输出（omega_r_dot, beta_v, tau_s）
		// 注意：姿态环在 Acro/纯速率模式可能不发布该话题，因此需要做新鲜度检查，
		// 否则速率环会一直使用旧的 beta_v/omega_r_dot/tau_s 造成模式切换异常。
		static constexpr hrt_abstime ESO_AUX_TIMEOUT{100_ms};
		eso_rate_aux_s eso_aux{};
		if (_eso_rate_aux_sub.update(&eso_aux)) {
			_last_eso_aux_timestamp = eso_aux.timestamp;

			if ((now - _last_eso_aux_timestamp) <= ESO_AUX_TIMEOUT) {
				// Keep raw tau_s visible in low-rate diagnostics even when dynamic feed-forward
				// injection is disabled. setTauSScale() still gates whether it affects control.
				const Vector3f tau_s = Vector3f(eso_aux.tau_s);
				_rate_control.setAttitudeAux(Vector3f(eso_aux.omega_r_dot), Vector3f(eso_aux.beta_v), tau_s);
			}
		}

		// 如果 aux 长时间未更新，则清零附加量（等效为纯速率 ESO 控制）
		if (_last_eso_aux_timestamp == 0 || (now - _last_eso_aux_timestamp) > ESO_AUX_TIMEOUT) {
			_rate_control.setAttitudeAux(Vector3f{}, Vector3f{}, Vector3f{});
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
				// 未解锁时也重置 ESO 观测器，确保起飞前从零开始估计
				_rate_control.resetESO();
			}

			// 从控制分配器读取“分配是否达成”的反馈：
			// - unallocated_torque: 期望力矩与实际可分配力矩的差（剩余未分配部分）
			// - saturation_positive/negative: 按轴标记正/负方向是否“给不出来”
			// 这些标志会传给速率环积分器做 anti-windup，避免在执行器打满时继续积累积分。
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive{};
				Vector<bool, 3> saturation_negative{};
				// 每次先清零缓存：若本周期完全可分配，则 unallocated_torque 维持 0
				_alloc_unallocated_torque.zero();

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						const float unallocated = control_allocator_status.unallocated_torque[i];
						// 缓存每个轴的未分配力矩，供低频调试打印观察
						_alloc_unallocated_torque(i) = unallocated;

						if (unallocated > FLT_EPSILON) {
							// 该轴“正方向力矩”需求超出可分配能力 -> 正饱和
							saturation_positive(i) = true;

						} else if (unallocated < -FLT_EPSILON) {
							// 该轴“负方向力矩”需求超出可分配能力 -> 负饱和
							saturation_negative(i) = true;
						}
					}
				}

				// 缓存饱和标志，供调试打印与积分抗饱和逻辑共同使用
				_alloc_sat_pos = saturation_positive;
				_alloc_sat_neg = saturation_negative;

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(_alloc_sat_pos, _alloc_sat_neg);
			}

			// run rate controller
			const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);

			/*// -------------------------------------------------------------------------
			// - rates           : 当前机体角速度 ω（实际值）
			// - rates_sp        : 期望角速度 ω_r（来自姿态环/手动）
			// - rate_error      : r_ω = ω - ω_r（跟踪误差）
			// - att_control     : 速率环输出（归一化力矩/控制量，[-1,1]），接近 ±1 代表轴向饱和。
			//                    “两个对角电机快、两个慢”通常对应某个轴（尤其 yaw）力矩长期打满。
			// - thrust_sp       : 推力指令，确认是否在起飞阶段。
			// - aux_fresh       : eso_rate_aux 是否新鲜（omega_r_dot/beta_v/tau_s），避免模式切换时吃到旧值。
			// -------------------------------------------------------------------------
			const Vector3f rate_error = rates - _rates_sp;
			const bool saturating =
				(fabsf(att_control(0)) > 0.8f) ||
				(fabsf(att_control(1)) > 0.8f) ||
				(fabsf(att_control(2)) > 0.8f);

			const bool aux_fresh = (_last_eso_aux_timestamp != 0) && ((now - _last_eso_aux_timestamp) <= 100_ms);

			static hrt_abstime last_dbg_print{0};
			if (_v_control_mode.flag_armed && saturating && (now - last_dbg_print) > 50_ms) {
				last_dbg_print = now;
				PX4_WARN("ESO dbg: w[%.2f %.2f %.2f] sp[%.2f %.2f %.2f] e(w-sp)[%.2f %.2f %.2f] u[%.2f %.2f %.2f] thr=%.2f aux=%d",
					 (double)rates(0), (double)rates(1), (double)rates(2),
					 (double)_rates_sp(0), (double)_rates_sp(1), (double)_rates_sp(2),
					 (double)rate_error(0), (double)rate_error(1), (double)rate_error(2),
					 (double)att_control(0), (double)att_control(1), (double)att_control(2),
					 (double)_thrust_sp, (int)aux_fresh);
			}*/
				// Debug hook kept disabled during automated tuning to avoid perturbing SITL timing with high-volume logs.
			static constexpr bool kRateDebugPrintEnabled = false;
			static hrt_abstime last_print1{0};
				if (kRateDebugPrintEnabled && hrt_elapsed_time(&last_print1) > 500000) {//0.5秒一次
					last_print1 = hrt_absolute_time();
					const Vector3f rate_error = rates - _rates_sp;
					PX4_INFO("rate: e(w-sp)[%.2f %.2f %.2f] w[%.2f %.2f %.2f] w_d[%.2f %.2f %.2f]",
						(double)rate_error(0), (double)rate_error(1), (double)rate_error(2),
						(double)rates(0), (double)rates(1), (double)rates(2),
						(double)_rates_sp(0), (double)_rates_sp(1), (double)_rates_sp(2));

					// sat+ / sat- 说明：
					// - sat+[i]=1: 第 i 轴正方向力矩需求超出可分配能力（正饱和）
					// - sat-[i]=1: 第 i 轴负方向力矩需求超出可分配能力（负饱和）
					PX4_INFO("rate alloc: u[%.2f %.2f %.2f] unalloc[%.3f %.3f %.3f] sat+[%d %d %d] sat-[%d %d %d]",
						(double)att_control(0), (double)att_control(1), (double)att_control(2),
						(double)_alloc_unallocated_torque(0), (double)_alloc_unallocated_torque(1), (double)_alloc_unallocated_torque(2),
						(int)_alloc_sat_pos(0), (int)_alloc_sat_pos(1), (int)_alloc_sat_pos(2),
						(int)_alloc_sat_neg(0), (int)_alloc_sat_neg(1), (int)_alloc_sat_neg(2));

					// 速率环力矩分量打印（0.5s）：用于判断各项贡献大小与主导项
					Vector3f inertia_term{};// 前馈加速度+ESO+M
					Vector3f feedback_term{};// 速率误差项
					Vector3f integral_term{};// 积分项
					Vector3f beta_term{};// 姿态误差项
					Vector3f tau_s_raw{};
					Vector3f tau_s_used{};
					Vector3f omega_r_dot_dbg{};// 参考角速度导数
					Vector3f dist_hat_dbg{};// 总扰动
					_rate_control.getLastTorqueTerms(inertia_term, feedback_term, integral_term, beta_term);
					_rate_control.getLastTauSDebug(tau_s_raw, tau_s_used, omega_r_dot_dbg, dist_hat_dbg);
					// 拆成两行打印，避免单行过长被控制台截断导致看不到 beta 项
					PX4_INFO("rate term1: iner[%.3f %.3f %.3f] int[%.3f %.3f %.3f]",
						(double)inertia_term(0), (double)inertia_term(1), (double)inertia_term(2),
						(double)integral_term(0), (double)integral_term(1), (double)integral_term(2));
					PX4_INFO("rate term2: fb[%.3f %.3f %.3f] beta[%.3f %.3f %.3f]",
						(double)feedback_term(0), (double)feedback_term(1), (double)feedback_term(2),
						(double)beta_term(0), (double)beta_term(1), (double)beta_term(2));
					PX4_INFO("rate tau: raw[%.3f %.3f %.3f] use[%.3f %.3f %.3f]",
						(double)tau_s_raw(0), (double)tau_s_raw(1), (double)tau_s_raw(2),
						(double)tau_s_used(0), (double)tau_s_used(1), (double)tau_s_used(2));
					PX4_INFO("rate obs: wdot_r[%.3f %.3f %.3f] dhat[%.3f %.3f %.3f]",
						(double)omega_r_dot_dbg(0), (double)omega_r_dot_dbg(1), (double)omega_r_dot_dbg(2),
						(double)dist_hat_dbg(0), (double)dist_hat_dbg(1), (double)dist_hat_dbg(2));
				}

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getESORateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// Keep high-volume named-value debug publishing disabled during autotune.
			static constexpr bool kPublishRateDebugKeyValues = false;
			if (kPublishRateDebugKeyValues) {
				/* Removed internal uORB topic
				eso_rate_debug_s eso_dbg{};
				eso_dbg.timestamp = hrt_absolute_time();
				*/

				// ESO估计值
				const matrix::Vector3f est_rate = _rate_control.getESOEstimatedAngularVelocity();
				const matrix::Vector3f est_dist = _rate_control.getESOEstimatedDisturbance();

				/*
				est_rate.copyTo(eso_dbg.estimated_angular_velocity);
				est_dist.copyTo(eso_dbg.estimated_disturbance);

				// 实际值
				rates.copyTo(eso_dbg.actual_angular_velocity);

				// 目标值
				_rates_sp.copyTo(eso_dbg.target_angular_velocity);

				_eso_rate_debug_pub.publish(eso_dbg);
				*/

				// Bridge to ROS via named_value_float
				auto publish_key_value = [&](const char* key, float value) {
					debug_key_value_s debug_msg{};
					debug_msg.timestamp = hrt_absolute_time();
					strncpy(debug_msg.key, key, sizeof(debug_msg.key));
					debug_msg.key[sizeof(debug_msg.key) - 1] = '\0';
					debug_msg.value = value;
					_debug_key_value_pub.publish(debug_msg);
				};

				publish_key_value("ESO_WX", est_rate(0));
				publish_key_value("ESO_WY", est_rate(1));
				publish_key_value("ESO_WZ", est_rate(2));
				publish_key_value("ESO_TX", est_dist(0));
				publish_key_value("ESO_TY", est_dist(1));
				publish_key_value("ESO_TZ", est_dist(2));
			}

			// publish actuator controls
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			if (!_vehicle_status.is_vtol) {
				publishTorqueSetpoint(att_control, angular_velocity.timestamp_sample);
				publishThrustSetpoint(angular_velocity.timestamp_sample);
			}

			// scale effort by battery status if enabled
			if (_param_eso_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						actuators.control[i] *= _battery_status_scale;
					}
				}
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

			updateActuatorControlsStatus(actuators, dt);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

	perf_end(_loop_perf);
}

void ESOMulticopterRateControl::publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;
	v_torque_sp.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	v_torque_sp.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	v_torque_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void ESOMulticopterRateControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;
	v_thrust_sp.xyz[0] = 0.0f;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = PX4_ISFINITE(_thrust_sp) ? -_thrust_sp : 0.0f; // Z is Down

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

void ESOMulticopterRateControl::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int ESOMulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	ESOMulticopterRateControl *instance = new ESOMulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ESOMulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ESOMulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("eso_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int eso_rate_control_main(int argc, char *argv[])
{
	return ESOMulticopterRateControl::main(argc, argv);
}
