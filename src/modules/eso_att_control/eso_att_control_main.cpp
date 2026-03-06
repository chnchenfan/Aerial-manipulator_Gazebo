/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file eso_att_control_main.cpp
 * ESO-based multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "eso_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

ESOMulticopterAttitudeControl::ESOMulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
	if (_vtol) {
		int32_t vt_type = -1;

		if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
			_vtol_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
		}
	}

	parameters_updated();
}

ESOMulticopterAttitudeControl::~ESOMulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
ESOMulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
ESOMulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_eso_roll_p.get(), _param_eso_pitch_p.get(), _param_eso_yaw_p.get()),
					      _param_eso_yaw_weight.get());

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_eso_rollrate_max.get()), radians(_param_eso_pitchrate_max.get()),
						radians(_param_eso_yawrate_max.get())));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());

	_active_profile = eso_common::resolveModelProfile(_param_eso_arm_model.get());
	_arm_kinematics.setModelProfile(_active_profile);

	if (_active_profile == eso_common::ModelProfile::UamV5) {
		const eso_common::DynamicsProfile &profile = eso_common::getDynamicsProfile(_active_profile);
		_mass_total = profile.mass_total;
		_com_offset = profile.comVector();
		_arm_inertia = profile.staticSystemInertiaMatrix();

	} else {
		_mass_total = _param_eso_mass_total.get();
		_com_offset = Vector3f(_param_eso_com_x.get(), _param_eso_com_y.get(), _param_eso_com_z.get());
		_arm_inertia = diag(Vector3f(_param_eso_arm_inertia_xx.get(), _param_eso_arm_inertia_yy.get(), _param_eso_arm_inertia_zz.get()));
	}

	_attitude_control.setTauSParameters(_mass_total, _com_offset, _arm_inertia);

	// [NEW] 初始化时同步更新参数到 ArmKinematics，作为 fallback
	_arm_kinematics.setParameters(_mass_total, _com_offset, _arm_inertia);
}

void ESOMulticopterAttitudeControl::update_dynamic_parameters()
{
	// 尝试获取最新的机械臂关节状态
	if (_arm_joint_states_sub.update(&_arm_joints)) {
		if (!_arm_joints.valid) {
			_attitude_control.setTauSParameters(_mass_total, _com_offset, _arm_inertia);
			return;
		}

		// 1. 调用运动学库，实时计算重心和惯量
		// 这里的 q 数组直接从 uORB 消息中获取
		Vector3f dynamic_com = _arm_kinematics.computeSystemCoM(_arm_joints.q);
		Matrix3f dynamic_inertia = _arm_kinematics.computeSystemInertia(_arm_joints.q);

		// 2. 更新到控制器中
		// 注意：质量 _mass_total 暂时认为是恒定的（除非有抓取检测）
		// 如果有负载抓取，这里还需要根据 _arm_joints.valid (或另加 grip_flag) 来增加负载质量
		_attitude_control.setTauSParameters(_mass_total, dynamic_com, dynamic_inertia);

		// 3. (可选) 可在此处发布调试消息，观察计算出的 CoM 是否符合预期
		// PX4_INFO_LIMIT(2, "Dyn CoM: %.2f %.2f %.2f", (double)dynamic_com(0), (double)dynamic_com(1), (double)dynamic_com(2));
	}

}

float
ESOMulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
    // 1. 获取最小油门（防止电机停转）
    const float throttle_min = _landed ? 0.0f : _param_mpc_manthr_min.get();

    // 2. 根据参数 MPC_THR_CURVE 选择映射方式
    switch (_param_mpc_thr_curve.get()) {
    case 1: // 线性模式：杆量多少就是多少（适合特技飞行）
        return throttle_min + throttle_stick_input * (_param_mpc_thr_max.get() - throttle_min);

    default: // 默认模式：把 50% 杆量映射为悬停油门 (MPC_THR_HOVER)
        // math::gradual3 是一个分段函数，保证在 0.5 处平滑过渡到悬停油门
        return math::gradual3(throttle_stick_input,
                              0.f, .5f, 1.f,  // 输入节点：0, 0.5, 1
                              throttle_min, _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()); // 输出节点
    }
}

void
ESOMulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(q).psi();

	/* reset yaw setpoint to current position if needed */
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f) > 0.05f
		   || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

		const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(x*x + y*y)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_x_input_filter.setParameters(dt, _param_eso_man_tilt_tau.get());
	_man_y_input_filter.setParameters(dt, _param_eso_man_tilt_tau.get());
	_man_x_input_filter.update(_manual_control_setpoint.x * _man_tilt_max);
	_man_y_input_filter.update(_manual_control_setpoint.y * _man_tilt_max);
	const float x = _man_x_input_filter.getState();
	const float y = _man_y_input_filter.getState();

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

	/* modify roll/pitch only if we're a VTOL */
	if (_vtol) {
		// Construct attitude setpoint rotation matrix. Modify the setpoints for roll
		// and pitch such that they reflect the user's intention even if a large yaw error
		// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
		// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
		// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
		// heading of the vehicle.
		// However there's also a coupling effect that causes oscillations for fast roll/pitch changes
		// at higher tilt angles, so we want to avoid using this on multicopters.
		// The effect of that can be seen with:
		// - roll/pitch into one direction, keep it fixed (at high angle)
		// - apply a fast yaw rotation
		// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

		// calculate our current yaw error
		float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

		// compute the vector obtained by rotating a z unit vector by the rotation
		// given by the roll and pitch commands of the user
		Vector3f zB = {0.0f, 0.0f, 1.0f};
		Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
		Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

		// transform the vector into a new frame which is rotated around the z axis
		// by the current yaw error. this vector defines the desired tilt when we look
		// into the direction of the desired heading
		Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
		z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

		// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
		// R_tilt is computed from_euler; only true if cos(roll) not equal zero
		// -> valid if roll is not +-pi/2;
		attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
		attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
	}

	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(math::constrain(_manual_control_setpoint.z, 0.f, 1.f));
	attitude_setpoint.timestamp = hrt_absolute_time();

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
	// 缓存最近一次 setpoint（用于姿态环调试快照打印，定位“期望姿态是否在跳变”）
	_attitude_setpoint_last = attitude_setpoint;
	_attitude_setpoint_last_valid = true;

	// update attitude controller setpoint immediately
	_attitude_control.setAttitudeSetpoint(q_sp, attitude_setpoint.yaw_sp_move_rate);
	_thrust_setpoint_body = Vector3f(attitude_setpoint.thrust_body);
	_last_attitude_setpoint = attitude_setpoint.timestamp;
}

void
ESOMulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
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

	// run controller on attitude updates
	vehicle_attitude_s v_att;

	// [NEW] 在每次姿态环运行前，检查并更新动态参数
	update_dynamic_parameters();

	if (_vehicle_attitude_sub.update(&v_att)) {


		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
			const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
			_last_run = v_att.timestamp_sample;

			const Quatf q{v_att.q};
			// 读取当前角速度（用于 tau_s 与后续速率环）
			vehicle_angular_velocity_s ang_vel{};
			_vehicle_angular_velocity_sub.copy(&ang_vel);
			const Vector3f omega_curr{ang_vel.xyz};

		// Check for new attitude setpoint
		if (_vehicle_attitude_setpoint_sub.updated()) {
			vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
				    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

					_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
					// 缓存最近一次 setpoint（用于姿态环调试快照打印）
					_attitude_setpoint_last = vehicle_attitude_setpoint;
					_attitude_setpoint_last_valid = true;
				}
			}

		// Check for a heading reset
		if (_quat_reset_counter != v_att.quat_reset_counter) {
			const Quatf delta_q_reset(v_att.delta_q_reset);

			// for stabilized attitude generation only extract the heading change from the delta quaternion
			_man_yaw_sp = wrap_pi(_man_yaw_sp + Eulerf(delta_q_reset).psi());

			if (v_att.timestamp > _last_attitude_setpoint) {
				// adapt existing attitude setpoint unless it was generated after the current attitude estimate
				_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
			}

			_quat_reset_counter = v_att.quat_reset_counter;
		}

		/* check for updates in other topics */
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = vehicle_status.is_vtol;
				_vtol_in_transition_mode = vehicle_status.in_transition_mode;
			}
		}

		bool attitude_setpoint_generated = false;

		const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);

		// vehicle is a tailsitter in transition mode
		const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

		bool run_att_ctrl = _v_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);

		// 差分状态管理：omega_r_dot 是由 omega_r 数值差分得到的，存在“首次进入/切换/解锁瞬间”的导数尖峰风险。
		// 因此在以下边界条件下复位差分状态：
		// 1) 未解锁：持续复位，保证起飞前导数为 0
		// 2) 刚解锁：复位一次，避免继承地面阶段的历史
		// 3) 刚进入姿态控制：复位一次，避免模式切换造成的差分尖峰
		const bool armed = _v_control_mode.flag_armed;
		if (!armed || (!_was_armed && armed) || (!_was_run_att_ctrl && run_att_ctrl) || (_was_run_att_ctrl && !run_att_ctrl)) {
			_attitude_control.resetOmegaRDerivative();
		}

		_was_armed = armed;
		_was_run_att_ctrl = run_att_ctrl;

		if (run_att_ctrl) {

			// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
			if (_v_control_mode.flag_control_manual_enabled &&
			    !_v_control_mode.flag_control_altitude_enabled &&
			    !_v_control_mode.flag_control_velocity_enabled &&
			    !_v_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(q, dt, _reset_yaw_sp);
				attitude_setpoint_generated = true;

			} else {
				_man_x_input_filter.reset(0.f);
				_man_y_input_filter.reset(0.f);
			}

				Vector3f rates_sp = _attitude_control.update(q, omega_curr, dt);

			const hrt_abstime now = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((now - pid_autotune.timestamp) < 1_s)) {
					rates_sp += Vector3f(pid_autotune.rate_sp);
				}
			}

			// publish rate setpoint
			vehicle_rates_setpoint_s v_rates_sp{};
			v_rates_sp.roll = rates_sp(0);
			v_rates_sp.pitch = rates_sp(1);
			v_rates_sp.yaw = rates_sp(2);
				_thrust_setpoint_body.copyTo(v_rates_sp.thrust_body);
				v_rates_sp.timestamp = hrt_absolute_time();

					_v_rates_sp_pub.publish(v_rates_sp);

					// Bridge to ROS via named_value_float (Target Attitude)
					if (_attitude_setpoint_last_valid) {
						const Eulerf euler_sp(Quatf(_attitude_setpoint_last.q_d));

						auto publish_key_value = [&](const char* key, float value) {
							debug_key_value_s debug_msg{};
							debug_msg.timestamp = hrt_absolute_time();
							strncpy(debug_msg.key, key, sizeof(debug_msg.key));
							debug_msg.key[sizeof(debug_msg.key) - 1] = '\0';
							debug_msg.value = value;
							_debug_key_value_pub.publish(debug_msg);
						};

						publish_key_value("AT_R", euler_sp.phi());
						publish_key_value("AT_P", euler_sp.theta());
						publish_key_value("AT_Y", euler_sp.psi());
					}

					// -------------------------------------------------------------------------
					// [调试快照] 炸机往往发生在瞬间，来不及手动 listener。
					// 这里在“姿态环输出 rates_sp 接近限幅”时自动打印关键量，快速判断：
					// - 是“期望姿态在跳变”（上游 setpoint/位置环/Offboard 问题），还是
					// - “姿态环算出来的 rates_sp 自己在乱跳”（姿态环逻辑/符号/权重问题）。
					//
					// 打印字段含义：
					// - att[r p y]      : 当前姿态欧拉角（度），看飞机实际朝向
					// - sp[r p y]       : 期望姿态欧拉角（度，来自 vehicle_attitude_setpoint.q_d）
					// - rates_sp[r p y] : 姿态环输出的机体系角速度指令 ω_r（rad/s），这是速率环的直接输入
					// -------------------------------------------------------------------------
					using math::radians;
					const Vector3f rate_limit(
						radians(_param_eso_rollrate_max.get()),
						radians(_param_eso_pitchrate_max.get()),
						radians(_param_eso_yawrate_max.get()));

					const bool rates_sp_near_limit =
						(fabsf(rates_sp(0)) > 0.8f * rate_limit(0)) ||
						(fabsf(rates_sp(1)) > 0.8f * rate_limit(1)) ||
						(fabsf(rates_sp(2)) > 0.8f * rate_limit(2));

					if (armed && rates_sp_near_limit && (now - _last_att_dbg_print) > 50_ms) {
						_last_att_dbg_print = now;

						const Eulerf euler_curr(q);
						const float roll_deg = math::degrees(euler_curr.phi());
						const float pitch_deg = math::degrees(euler_curr.theta());
						const float yaw_deg = math::degrees(euler_curr.psi());

						bool sp_valid = _attitude_setpoint_last_valid;
						float sp_roll_deg = 0.f;
						float sp_pitch_deg = 0.f;
						float sp_yaw_deg = 0.f;

						if (sp_valid) {
							const Eulerf euler_sp(Quatf(_attitude_setpoint_last.q_d));
							sp_roll_deg = math::degrees(euler_sp.phi());
							sp_pitch_deg = math::degrees(euler_sp.theta());
							sp_yaw_deg = math::degrees(euler_sp.psi());
						}

						PX4_WARN("ESO att dbg: att[%.1f %.1f %.1f] sp[%.1f %.1f %.1f] rates_sp[%.2f %.2f %.2f] spv=%d",
							 (double)roll_deg, (double)pitch_deg, (double)yaw_deg,
							 (double)sp_roll_deg, (double)sp_pitch_deg, (double)sp_yaw_deg,
							 (double)rates_sp(0), (double)rates_sp(1), (double)rates_sp(2),
							 (int)sp_valid);
					}

					// 发布附加话题：omega_r 导数、beta_v、tau_s
					eso_rate_aux_s aux{};
					aux.timestamp = v_rates_sp.timestamp;
					_attitude_control.omegaRDerivative().copyTo(aux.omega_r_dot);
				rates_sp.copyTo(aux.omega_r);
				_attitude_control.betaVector().copyTo(aux.beta_v);
				_attitude_control.tauS().copyTo(aux.tau_s);
				_eso_rate_aux_pub.publish(aux);
			}

		// reset yaw setpoint during transitions, tailsitter.cpp generates
		// attitude setpoint for the transition
		_reset_yaw_sp = !attitude_setpoint_generated || _landed || (_vtol && _vtol_in_transition_mode);
	}

	perf_end(_loop_perf);
}

int ESOMulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	ESOMulticopterAttitudeControl *instance = new ESOMulticopterAttitudeControl(vtol);

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

int ESOMulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ESOMulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the ESO-based multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("eso_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int eso_att_control_main(int argc, char *argv[])
{
	return ESOMulticopterAttitudeControl::main(argc, argv);
}
