/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "ESOMulticopterPositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "ESOPositionControl/ControlMath.hpp"

using namespace matrix;

// 初始化
ESOMulticopterPositionControl::ESOMulticopterPositionControl(bool vtol) : // 成员初始化列表
	SuperBlock(nullptr, "ESO"),// 初始化父类 SuperBlock
	ModuleParams(nullptr),// 初始化父类 ModuleParams, 参数处理模块，用于管理和更新参数
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),// 初始化调度任务父类 ScheduledWorkItem，指定任务名称和工作队列配置
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),// 初始化 uORB 发布者,如果是 普通多旋翼，就发布到 vehicle_attitude_setpoint 话题（直接给姿态控制）
	_vel_x_deriv(this, "VELD"),// 初始化三个 微分器对象
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD")
{// 构造函数体
	parameters_update(true);// 立即读取一遍所有参数。确保控制器刚启动时，用的参数（如 PID 增益、最大速度）都是最新的。
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);// 设置“失效保护降落”的迟滞时间。
	_tilt_limit_slew_rate.setSlewRate(.2f);// 设置“倾斜角限制”的变化率。让最大倾斜角的限制变化得平滑一点，不要突变，防止飞机姿态控制过激。
	reset_setpoint_to_nan(_setpoint);// 把目标位置（Setpoint）全部设为 NAN（无效值）。设为 NAN，控制器就会知道“现在没目标”，保持待命。
	_takeoff_status_pub.advertise();// 正式广播“起飞状态”话题。
}

ESOMulticopterPositionControl::~ESOMulticopterPositionControl()
{
	perf_free(_cycle_perf);
}

bool ESOMulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

// [NEW] 动态耦合参数更新：实时更新 CoM
void ESOMulticopterPositionControl::update_dynamic_parameters()
{
	if (!_use_dynamic_arm_model) {
		_control.setSystemCoM(_fallback_com);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	bool use_dynamic_com = false;

	arm_joint_states_s joints{};
	if (_arm_joint_states_sub.update(&joints)) {
		if (!joints.valid) {
			_dynamic_com_valid = false;
			_control.setSystemCoM(_fallback_com);
			return;
		}

		// 1. 计算实时质心
		const matrix::Vector3f dynamic_com = _arm_kinematics.computeSystemCoM(joints.q);
		const bool dynamic_com_valid = PX4_ISFINITE(dynamic_com(0))
						&& PX4_ISFINITE(dynamic_com(1))
						&& PX4_ISFINITE(dynamic_com(2));

		if (dynamic_com_valid) {
			// 2. 动态 CoM 有效：写入控制器并更新时间戳
			_control.setSystemCoM(dynamic_com);
			_last_dynamic_com_timestamp = now;
			_dynamic_com_valid = true;
			use_dynamic_com = true;
		}
	}

	// 3. 动态 CoM 不可用或超时：回退到硬编码 CoM
	const bool dynamic_com_fresh = _dynamic_com_valid && ((now - _last_dynamic_com_timestamp) <= DYNAMIC_COM_TIMEOUT);

	if (!use_dynamic_com && !dynamic_com_fresh) {
		_control.setSystemCoM(_fallback_com);
	}
}

// 参数更新函数.它的作用是 “读取用户设置的参数”。比如你在地面站设置“最大上升速度 ESO_Z_VEL_MAX_UP”，代码就是在这里读进去的。
void ESOMulticopterPositionControl::parameters_update(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		_active_profile = eso_common::resolveModelProfile(_param_eso_arm_model.get());
		_arm_kinematics.setModelProfile(_active_profile);

		if (_active_profile == eso_common::ModelProfile::UamV5) {
			const eso_common::DynamicsProfile &profile = eso_common::getDynamicsProfile(_active_profile);
			_fallback_com = profile.comVector();
			_use_dynamic_arm_model = profile.use_dynamic_arm_model;
			_arm_kinematics.setParameters(profile.mass_total, profile.comVector(), profile.bodyInertiaMatrix());

		} else {
			_fallback_com = Vector3f(-0.015f, 0.0f, -0.164f);
			_use_dynamic_arm_model = true;
			_arm_kinematics.setParameters(eso_common::kUavArmV4Profile.mass_total,
					      eso_common::kUavArmV4Profile.comVector(),
					      eso_common::kUavArmV4Profile.bodyInertiaMatrix());
		}

		_control.setSystemCoM(_fallback_com);

		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// make it less sensitive at the lower end
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			num_changed += _param_ESO_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_ESO_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_ESO_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_ESO_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_ESO_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_ESO_tiltmax_air.commit_no_notification(45.f);

			} else {
				num_changed += _param_ESO_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_ESO_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_ESO_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_ESO_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_ESO_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_ESO_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_ESO_xy_vel_all.get();
			num_changed += _param_ESO_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_ESO_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_ESO_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_ESO_z_vel_all.get() >= 0.f) {
			float z_vel = _param_ESO_z_vel_all.get();
			num_changed += _param_ESO_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_ESO_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_ESO_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_ESO_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_ESO_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_ESO_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_ESO_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_ESO_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_ESO_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>ESO_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("eso_pos_ctrl_tilt_set"), events::Log::Warning,
					    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		if (_param_ESO_tiltmax_lnd.get() > _param_ESO_tiltmax_air.get()) {
			_param_ESO_tiltmax_lnd.set(_param_ESO_tiltmax_air.get());
			_param_ESO_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>ESO_TILTMAX_LND</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("eso_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_ESO_tiltmax_air.get());
		}
		// ------------------------------------------------------------------
		// 1. 设置位置环 PID (加入新增的积分项 I)
		// ------------------------------------------------------------------
		_control.setPositionGains(
			Vector3f(_param_ESO_x_p.get(), _param_ESO_y_p.get(), _param_ESO_z_p.get()),
			Vector3f(_param_ESO_x_i.get(), _param_ESO_y_i.get(), _param_ESO_z_i.get()));

		// ------------------------------------------------------------------
		// 2. 设置速度环 PID (ESO模式下通常只需 P，I和D由观测器替代，或根据需要保留)
		// ------------------------------------------------------------------
		// 【修改为】：注意这里 I 和 D 项暂时传 0 或者传入对应的参数(如果保留了的话)
		// 根据您的 .hpp，因为移除了 VEL_I 和 VEL_D 的参数定义，所以这里直接传 0
		_control.setVelocityGains(
			Vector3f(_param_ESO_x_vel_p_acc.get(), _param_ESO_y_vel_p_acc.get(), _param_ESO_z_vel_p_acc.get()),
			Vector3f(0.f, 0.f, 0.f),  // 速度环 I (由 ESO 扰动估计替代)
			Vector3f(0.f, 0.f, 0.f));   // 速度环 D (由 ESO 状态估计替代));
		_control.setHorizontalThrustMargin(_param_ESO_thr_xy_marg.get());

		// ------------------------------------------------------------------
		// 3. 【新增】传递 ESO 特有参数
		// ------------------------------------------------------------------
		// 设置位置环积分限幅
		_control.setPositionIntegralLimit(_param_ESO_pos_int_lim.get());
		_control.setDynamicsFeedforwardEnabled(_param_ESO_dyn_ff_en.get());

		// 设置 ESO 带宽
		_control.setESOBandwidth(Vector3f(
			_param_ESO_x_bw.get(),
			_param_ESO_y_bw.get(),
			_param_ESO_z_bw.get()
		));

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_ESO_xy_cruise.get() > _param_ESO_xy_vel_max.get()) {
			_param_ESO_xy_cruise.set(_param_ESO_xy_vel_max.get());
			_param_ESO_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>ESO_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("eso_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_ESO_xy_vel_max.get());
		}

		if (_param_ESO_vel_manual.get() > _param_ESO_xy_vel_max.get()) {
			_param_ESO_vel_manual.set(_param_ESO_xy_vel_max.get());
			_param_ESO_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>ESO_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("eso_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_ESO_xy_vel_max.get());
		}

		if (_param_ESO_z_v_auto_up.get() > _param_ESO_z_vel_max_up.get()) {
			_param_ESO_z_v_auto_up.set(_param_ESO_z_vel_max_up.get());
			_param_ESO_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>ESO_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("eso_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_ESO_z_vel_max_up.get());
		}

		if (_param_ESO_z_v_auto_dn.get() > _param_ESO_z_vel_max_dn.get()) {
			_param_ESO_z_v_auto_dn.set(_param_ESO_z_vel_max_dn.get());
			_param_ESO_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>ESO_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("eso_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_ESO_z_vel_max_dn.get());
		}

		if (_param_ESO_thr_hover.get() > _param_ESO_thr_max.get() ||
		    _param_ESO_thr_hover.get() < _param_ESO_thr_min.get()) {
			_param_ESO_thr_hover.set(math::constrain(_param_ESO_thr_hover.get(), _param_ESO_thr_min.get(),
						 _param_ESO_thr_max.get()));
			_param_ESO_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>ESO_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("eso_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_ESO_thr_hover.get());
		}

		if (!_param_ESO_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_ESO_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_ESO_tko_speed.set(math::min(_param_ESO_tko_speed.get(), _param_ESO_z_vel_max_up.get()));
		_param_ESO_land_speed.set(math::min(_param_ESO_land_speed.get(), _param_ESO_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_ESO_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_ESO_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_ESO_z_vel_p_acc.get());
	}

	update_dynamic_parameters();
}

PositionControlStates ESOMulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s &local_pos)
{
	PositionControlStates states;

	// only set position states if valid and finite
	if (PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && local_pos.xy_valid) {
		states.position(0) = local_pos.x;
		states.position(1) = local_pos.y;

	} else {
		states.position(0) = NAN;
		states.position(1) = NAN;
	}

	if (PX4_ISFINITE(local_pos.z) && local_pos.z_valid) {
		states.position(2) = local_pos.z;

	} else {
		states.position(2) = NAN;
	}

	if (PX4_ISFINITE(local_pos.vx) && PX4_ISFINITE(local_pos.vy) && local_pos.v_xy_valid) {
		states.velocity(0) = local_pos.vx;
		states.velocity(1) = local_pos.vy;
		states.acceleration(0) = _vel_x_deriv.update(local_pos.vx);
		states.acceleration(1) = _vel_y_deriv.update(local_pos.vy);

	} else {
		states.velocity(0) = NAN;
		states.velocity(1) = NAN;
		states.acceleration(0) = NAN;
		states.acceleration(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		states.velocity(2) = local_pos.vz;
		states.acceleration(2) = _vel_z_deriv.update(states.velocity(2));

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	states.yaw = local_pos.heading;

	return states;
}

void ESOMulticopterPositionControl::Run()
{
	// 1. 检查系统是否请求关掉这个模块 (例如在控制台输入了 stop),则进入清理和退出流程
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();// 取消订阅回调
		exit_and_cleanup();// 清理内存并退出
		return;
	}

	// 2. 重新调度备份任务 (Watchdog)
    	// 如果长时间没有传感器数据回调，这个延时任务会保证 Run 至少 100ms 跑一次，防止死锁
	ScheduleDelayed(100_ms);

	// 3. 检查参数更新。false：仅在参数实际发生变化时才更新参数。
	parameters_update(false);

	// 4. 开始性能计数 (用于系统负载分析)
	perf_begin(_cycle_perf);

	// 5. 定义一个局部变量，用来接收最新的位置数据
	vehicle_local_position_s local_pos;

	// 6. 【关键触发】尝试获取最新的位置数据
  	// 只有当有新数据时，update 返回 true，才会进入下面的控制逻辑
	if (_local_pos_sub.update(&local_pos)) {

		// 7. 计算时间间隔 dt
		const hrt_abstime time_stamp_now = local_pos.timestamp_sample;// 获取数据产生的时间戳
		// 计算距离上次循环过了多久，并限制在 0.002s 到 0.04s 之间 (防止 dt 异常导致积分爆炸)
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;// 更新上次时间

		// 8. 把 dt 传给父类 (微分器需要用到这个 dt)
		setDt(dt);
		_in_failsafe = false; // 先假设当前不在失效保护状态

		// 9. 更新其他订阅数据 (非阻塞更新)
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);// 更新控制模式 (是否Offboard? 是否解锁?)
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);// 更新落地状态

		// ------------------------------------------------------------------
		// 【新增】更新姿态和角速度数据 (ESO 需要)
		// ------------------------------------------------------------------
		vehicle_attitude_s att;
		if (_vehicle_attitude_sub.update(&att)) {
		_control.setAttitude(Quatf(att.q)); // 获取当前姿态，喂给 ESO 控制器
		}

		vehicle_angular_velocity_s ang_vel;
		if (_vehicle_angular_velocity_sub.update(&ang_vel)) {
		_control.setBodyRate(Vector3f(ang_vel.xyz));// 将角速度喂给 ESO 控制器
		}

		// 10. 更新悬停油门估计 (如果启用了)
		if (_param_ESO_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);// 动态调整悬停油门
				}
			}
		}

		// 11. 【关键数据清洗】将原始 EKF 数据转换为控制器可用的状态
        	// 这里会处理 NAN，并将速度微分得到加速度
		PositionControlStates states{set_vehicle_states(local_pos)};

		// 12. 只有在 "多旋翼位置控制模式" 启用时才工作 (比如 Loiter, Offboard, Hold)
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {

			// 13. 获取期望轨迹 (Setpoint) - 来自 MAVROS 或 Navigator
				const bool is_trajectory_setpoint_updated = _trajectory_setpoint_sub.update(&_setpoint);
				/*// ==================== 【开始：新增 Yaw 打印代码】 ====================
				// 仅在接收到新的 Setpoint 更新时打印，或者你可以去掉 if 限制按频率打印
				if (is_trajectory_setpoint_updated) {
					static hrt_abstime last_yaw_debug_time = 0;
					// 限制打印频率为 5Hz (每1000ms一次)，防止刷屏卡顿
					if (hrt_elapsed_time(&last_yaw_debug_time) > 1000_ms) {
						last_yaw_debug_time = hrt_absolute_time();// 更新上次打印时间

						// 打印原始弧度和转换后的角度
						// _setpoint.yaw 是弧度制
						PX4_INFO(">>> RECV_offboard Setpoint Yaw: %.2f",
							(double)math::degrees(_setpoint.yaw));
					}
				}
				// ==================== 【结束：新增 Yaw 打印代码】 ====================*/
				/*// ==================== 【开始：新增调试打印代码】 ====================
				// 限制打印频率：默认每 500ms 打印一次；起飞后提高到 200ms，方便捕捉“刚离地那一瞬间”的突变
				//
				// 目的：快速判断“问题出在上游 setpoint/坐标系/Offboard”，还是“控制器内部算错”。
				// - CurrentState：来自 EKF 的实际状态（飞机真实在什么位置/速度/航向）
				// - Setpoint    ：来自 TrajectorySetpoint 的期望（上游到底让飞机怎么飞）
				// - Takeoff     ：起飞状态机是否进入 flight（未进入 flight 时很多量会被清零/压制）
				//
				// 建议重点盯：
				// - Setpoint yaw / yawspeed 是否为 NaN（表示忽略 yaw）还是固定常数
				// - Setpoint age 是否持续 < 0.5s（否则 Offboard 可能掉线）
				static hrt_abstime last_debug_print = 0;
				const bool dbg_flying = (_takeoff.getTakeoffState() >= ESOTakeoffState::flight);
				const hrt_abstime dbg_interval = dbg_flying ? 200_ms : 500_ms;

				if (hrt_elapsed_time(&last_debug_print) > dbg_interval) {
					last_debug_print = hrt_absolute_time();

					// 保留原来的注释打印模板（便于你后续按需改格式/加字段）
					// PX4_INFO("--- ESO Controller Data Link Check ---");
					// PX4_INFO("[CurrentState] Pos:[%.2f %.2f %.2f] Vel:[%.2f %.2f %.2f] Valid(XY:%d Z:%d V_XY:%d V_Z:%d)", ...);
					// PX4_INFO("[Setpoint] Pos:[%.2f %.2f %.2f] Vel:[%.2f %.2f %.2f] Acc:[%.2f %.2f %.2f]", ...);
					// PX4_INFO("[Takeoff] State: %d (Want: %d)", ...);
					// PX4_INFO("--------------------------------------");

					PX4_INFO("--- ESO pos dbg ---");

					// 1) 实际状态（来自 EKF / vehicle_local_position）
					PX4_INFO("[Current] pos[%.2f %.2f %.2f] vel[%.2f %.2f %.2f] yaw=%.1fdeg valid(xy:%d z:%d vxy:%d vz:%d)",
						 (double)local_pos.x, (double)local_pos.y, (double)local_pos.z,
						 (double)local_pos.vx, (double)local_pos.vy, (double)local_pos.vz,
						 (double)math::degrees(local_pos.heading),
						 (int)local_pos.xy_valid, (int)local_pos.z_valid, (int)local_pos.v_xy_valid, (int)local_pos.v_z_valid);

					// 2) 期望状态（来自 TrajectorySetpoint / 上游 Offboard 或导航）
					const float sp_age_s = hrt_elapsed_time(&_setpoint.timestamp) * 1e-6f;
					PX4_INFO("[Setpoint] pos[%.2f %.2f %.2f] vel[%.2f %.2f %.2f] acc[%.2f %.2f %.2f] yaw=%.1fdeg yawspeed=%.2f age=%.2fs upd=%d",
						 (double)_setpoint.x, (double)_setpoint.y, (double)_setpoint.z,
						 (double)_setpoint.vx, (double)_setpoint.vy, (double)_setpoint.vz,
						 (double)_setpoint.acceleration[0], (double)_setpoint.acceleration[1], (double)_setpoint.acceleration[2],
						 (double)math::degrees(_setpoint.yaw), (double)math::degrees(_setpoint.yawspeed),
						 (double)sp_age_s, (int)is_trajectory_setpoint_updated);

					// 3) 起飞状态机
					PX4_INFO("[Takeoff] state=%d want=%d armed=%d offboard=%d",
						 (int)_takeoff.getTakeoffState(),
						 (int)_vehicle_constraints.want_takeoff,
						 (int)_vehicle_control_mode.flag_armed,
						 (int)_vehicle_control_mode.flag_control_offboard_enabled);

					PX4_INFO("---------------");
				}
				// ==================== 【结束：新增调试打印代码】 ====================*/
			// 14. EKF 重置处理 (高级逻辑)
			// 如果 GPS 信号跳变导致 EKF 坐标系发生平移，这里会自动把目标点也平移
			// 确保飞机相对于地面的运动是平滑的，不会因为坐标系变了就突然猛冲
			if (_setpoint.timestamp < local_pos.timestamp) {
				if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
					_setpoint.vx += local_pos.delta_vxy[0];// 补偿XY速度重置
					_setpoint.vy += local_pos.delta_vxy[1];
				}

				if (local_pos.vz_reset_counter != _vz_reset_counter) {
					_setpoint.vz += local_pos.delta_vz;// 补偿Z速度重置
				}

				if (local_pos.xy_reset_counter != _xy_reset_counter) {
					_setpoint.x += local_pos.delta_xy[0];// 补偿XY位置重置
					_setpoint.y += local_pos.delta_xy[1];
				}

				if (local_pos.z_reset_counter != _z_reset_counter) {
					_setpoint.z += local_pos.delta_z;// 补偿Z位置重置
				}

				if (local_pos.heading_reset_counter != _heading_reset_counter) {
					_setpoint.yaw += local_pos.delta_heading;// 补偿yaw角重置
				}
			}

			// 15. 更新飞行限制 (最大速度、最大倾角)
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// 16. 处理 Offboard 模式下的起飞意图
			// 这是一个特殊的逻辑：如果我们在 Offboard 给了一个很高的 Z 轴速度，或者是上升的位置
			// 并且当前处于落地状态，那么系统判断为 "想要起飞" (want_takeoff = true)
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_ESO_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_ESO_z_vel_max_up.get();
			}

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				bool want_takeoff = _vehicle_control_mode.flag_armed && _vehicle_land_detected.landed
						    && hrt_elapsed_time(&_setpoint.timestamp) < 1_s;

				if (want_takeoff && PX4_ISFINITE(_setpoint.z)
				    && (_setpoint.z < states.position(2))) {

					_vehicle_constraints.want_takeoff = true;// 目标高度比当前高 -> 想起飞

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.vz)
					   && (_setpoint.vz < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_ESO_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_ESO_z_vel_max_dn.get();
			}

			// 17. 更新起飞状态机 (Takeoff State Machine)
            		// 负责处理电机怠速旋转 (Spoolup) -> 爬坡 (Ramp up) -> 正常飞行 (Flight) 的状态切换
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, false, time_stamp_now);

			// 18. 起飞过程中的特殊处理
				/*
					Disarmed (未解锁)：清零。
					Armed (已解锁) 但在 Spoolup (怠速旋转)：依然清零。这时候电机虽然在转（怠速），但还没有起飞意图，控制器依然被按死在 0。
					Rampup (开始推油门起飞)：停止清零，ESO 和积分器正式接管控制，开始累积数值。
				*/
				const bool flying = (_takeoff.getTakeoffState() >= ESOTakeoffState::flight);
				// -------------------------------------------------------------------------
				// [ESO 起飞门控开关] 起飞前不更新位置ESO，进入 flight 后才启用，并在启用瞬间用测量初始化 z1/z2
				//
				// 目的：避免怠速/爬坡阶段由于模型不一致或输入饱和导致 ESO 产生“导数尖峰/扰动尖峰”，把位置环输出推爆。
				// 参数：ESO_POS_ESOGATE
				// - 1：flight 前禁用ESO更新（d_hat=0），flight 后启用（并初始化）
				// - 0：始终更新ESO（旧行为）
				// -------------------------------------------------------------------------
				_control.setESOUpdateEnabled(!_param_ESO_pos_esogate.get() || flying);

				if (is_trajectory_setpoint_updated) {
					// make sure takeoff ramp is not amended by acceleration feed-forward
					if (!flying) {
						_setpoint.acceleration[2] = NAN;
					// hover_thrust maybe reset on takeoff
					_control.setHoverThrust(_param_ESO_thr_hover.get());
				}

				const bool not_taken_off             = (_takeoff.getTakeoffState() < ESOTakeoffState::rampup);
				const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

				// 如果还没起飞，强行把目标加速度设为向下 100m/s^2，确保电机不输出推力 (压在地上)
				if (not_taken_off || flying_but_ground_contact) {
					// we are not flying yet and need to avoid any corrections
					reset_setpoint_to_nan(_setpoint);// 清空目标
					Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

					// prevent any integrator windup
					_control.resetIntegral();// 清空积分器，防止地面积分饱和
				}
			}

			// limit tilt during takeoff ramupup
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < ESOTakeoffState::flight)
						     ? _param_ESO_tiltmax_lnd.get() : _param_ESO_tiltmax_air.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			// 19. 计算起飞时的速度限制斜坡 (Ramp)
            		// 让最大允许速度从 0 慢慢增加到最大值，实现平滑起飞,避免阶跃
			const float speed_up = _takeoff.updateRamp(dt,
					       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_ESO_z_vel_max_up.get());
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_ESO_z_vel_max_dn.get();

			// 20. 设置控制器的物理限制
            		// 包括最小推力 (怠速推力) 和 最大推力
			const float minimum_thrust = flying ? _param_ESO_thr_min.get() : 0.f;

			_control.setThrustLimits(minimum_thrust, _param_ESO_thr_max.get());

			// 设置速度限制 (包括刚才算出来的平滑起飞限制 speed_up)
			_control.setVelocityLimits(
				_param_ESO_xy_vel_max.get(),
				math::min(speed_up, _param_ESO_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
				math::max(speed_down, 0.f));

			// 21. 【数据清洗】Sanitize Setpoint (修改的部分在这里生效)
			// 确保喂给控制器的 Setpoint 没有 NAN，如果有，替换为 0 或忽略
			// (虽然原始代码是在 _inputValid 里检查，但建议在 setInputSetpoint 前手动处理)
			vehicle_local_position_setpoint_s sanitized_sp = _setpoint;
			// --- 【位置保护逻辑】 ---
                        // 如果目标位置是 NAN (通常是被 Failsafe 重置了)，
                        // 就暂时把“当前位置”当作目标，保持悬停，防止控制器崩溃。
                        if (!PX4_ISFINITE(sanitized_sp.x)) sanitized_sp.x = states.position(0);
                        if (!PX4_ISFINITE(sanitized_sp.y)) sanitized_sp.y = states.position(1);
                        if (!PX4_ISFINITE(sanitized_sp.z)) sanitized_sp.z = states.position(2);
				// --- 速度/加速度清洗逻辑 ---
				if (!PX4_ISFINITE(sanitized_sp.vx)) sanitized_sp.vx = 0.f;
				if (!PX4_ISFINITE(sanitized_sp.vy)) sanitized_sp.vy = 0.f;
				if (!PX4_ISFINITE(sanitized_sp.vz)) sanitized_sp.vz = 0.f;
				if (!PX4_ISFINITE(sanitized_sp.acceleration[0])) sanitized_sp.acceleration[0] = 0.f;
				if (!PX4_ISFINITE(sanitized_sp.acceleration[1])) sanitized_sp.acceleration[1] = 0.f;
				if (!PX4_ISFINITE(sanitized_sp.acceleration[2])) sanitized_sp.acceleration[2] = 0.f;
				// --- 航向清洗逻辑（重要） ---
				// 在 PX4 语义里，yaw / yawspeed 允许为 NaN，表示“该维不控制/忽略”（例如 Offboard 的 type_mask 忽略 yaw）。
				// 不能把 NaN 粗暴改成 0，否则会把“忽略 yaw”变成“强制追 0°航向”，起飞阶段极易导致偏航打满 -> 对角电机快慢明显 -> 翻滚炸机。
				// 正确做法：保持 NaN，让 PositionControl 内部在 update() 里用“当前航向 _yaw”替代 yaw_sp（相当于保持当前航向）。
				if (!PX4_ISFINITE(sanitized_sp.yaw)) {
				sanitized_sp.yaw = states.yaw; // 显式设置为当前航向，确保不转头
				/*// ==================== 【开始：新增 Yaw 打印代码】 ====================
					static hrt_abstime last_yaw_debug_time1 = 0;
					// 限制打印频率为 5Hz (每1000ms一次)，防止刷屏卡顿
					if (hrt_elapsed_time(&last_yaw_debug_time1) > 1000_ms) {
						last_yaw_debug_time1 = hrt_absolute_time();// 更新上次打印时间

						// 打印原始弧度和转换后的角度
						// _setpoint.yaw 是弧度制
						PX4_INFO(">>> 清洗之后 为现在的yaw值 Setpoint Yaw: %.2f",
							(double)math::degrees(sanitized_sp.yaw));
					}
				// ==================== 【结束：新增 Yaw 打印代码】 ====================*/

				}

				if (!PX4_ISFINITE(sanitized_sp.yawspeed)) sanitized_sp.yawspeed = 0.f; // 角速度设为0是安全的

				_control.setInputSetpoint(sanitized_sp); // 喂目标值

			// 垂直速度状态融合：解决 “降落时 EKF 速度不准导致砸地” 的问题。它实现了一个动态切换机制：飞得快时信“微分”（准但不稳），飞得慢时信“融合”（稳但不准）。
			if (!PX4_ISFINITE(_setpoint.z)
			    && PX4_ISFINITE(_setpoint.vz) && (fabsf(_setpoint.vz) > FLT_EPSILON)
			    && PX4_ISFINITE(local_pos.z_deriv) && local_pos.z_valid && local_pos.v_z_valid) {
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  ESO_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= ESO_LAND_SPEED: use altitude derivative
				float weighting = fminf(fabsf(_setpoint.vz) / _param_ESO_land_speed.get(), 1.f);
				states.velocity(2) = local_pos.z_deriv * weighting + local_pos.vz * (1.f - weighting);
			}

			// 22. 喂状态值
			_control.setState(states);// 喂当前状态

			// 23. 【核心计算】执行控制器算法
            		// 如果计算成功，返回 true；如果数据无效或算出了 NAN，返回 false
			if (_control.update(dt)) {
				// 成功：重置失效保护迟滞计数器
				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);

			} else {
				// 失败 (Failsafe)：进入失效保护逻辑
                		// 打印 "invalid setpoints" 就在这里
				const bool warn_failsafe = ((time_stamp_now - _last_warn) > 2_s) && _vehicle_control_mode.flag_armed;

				/*if (warn_failsafe) {
					PX4_WARN("invalid setpoints");
					PX4_INFO("sp: pos[%.2f %.2f %.2f] vel[%.2f %.2f %.2f] acc[%.2f %.2f %.2f] yaw[%.2f] valid xy:%d z:%d v_xy:%d v_z:%d",
						 (double)_setpoint.x, (double)_setpoint.y, (double)_setpoint.z,
						 (double)_setpoint.vx, (double)_setpoint.vy, (double)_setpoint.vz,
						 (double)_setpoint.acceleration[0], (double)_setpoint.acceleration[1], (double)_setpoint.acceleration[2],
						 (double)_setpoint.yaw,
						 local_pos.xy_valid, local_pos.z_valid, local_pos.v_xy_valid, local_pos.v_z_valid);
					_last_warn = time_stamp_now;
				}*/

				// 生成一个急救目标点 (原地降落或悬停)
				vehicle_local_position_setpoint_s failsafe_setpoint{};

				failsafe(time_stamp_now, failsafe_setpoint, states, warn_failsafe);

				// reset constraints
				_vehicle_constraints = {0, NAN, NAN, false, {}};

				// 尝试用急救目标点再算一次，保证飞机不要失控摔机
				_control.setInputSetpoint(failsafe_setpoint);
				_control.setVelocityLimits(_param_ESO_xy_vel_max.get(), _param_ESO_z_vel_max_up.get(), _param_ESO_z_vel_max_dn.get());
				_control.update(dt);
			}

			// 24. 发布控制结果 (Local Position Setpoint)
            		// 这不是给电机用的，是发出来记录日志，或者给 LandDetector 判断是否达到目标用的
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			if ((_last_pos_ctrl_status_pub == 0) || hrt_elapsed_time(&_last_pos_ctrl_status_pub) >= 200_ms) {
				_last_pos_ctrl_status_pub = hrt_absolute_time();

				const matrix::Vector3f pos = _control.getCurrentPosition();
				const matrix::Vector3f vel = _control.getCurrentVelocity();
				const matrix::Vector3f pos_hat = _control.getESOEstimatedPosition();
				const matrix::Vector3f vel_hat = _control.getESOEstimatedVelocity();
				const matrix::Vector3f dist_hat = _control.getESODisturbance();
				const matrix::Vector3f eso_input = _control.getESOInput();

				position_controller_status_s pos_status{};
				pos_status.timestamp = _last_pos_ctrl_status_pub;
				pos_status.nav_bearing = NAN;
				pos_status.target_bearing = NAN;
				pos_status.xtrack_error = NAN;
				pos_status.yaw_acceptance = NAN;

				for (int i = 0; i < 3; i++) {
					pos_status.eso_pos[i] = pos(i);
					pos_status.eso_pos_hat[i] = pos_hat(i);
					pos_status.eso_pos_error[i] = pos(i) - pos_hat(i);
					pos_status.eso_vel[i] = vel(i);
					pos_status.eso_vel_hat[i] = vel_hat(i);
					pos_status.eso_vel_error[i] = vel(i) - vel_hat(i);
					pos_status.eso_disturbance_hat[i] = dist_hat(i);
					pos_status.eso_input[i] = eso_input(i);
				}

				_pos_ctrl_status_pub.publish(pos_status);
			}

			// Keep high-volume named-value debug publishing disabled during autotune.
			static constexpr bool kPublishPosDebugKeyValues = false;
			if (kPublishPosDebugKeyValues) {
				// ESO估计值
				const matrix::Vector3f est_pos = _control.getESOEstimatedPosition();
				// const matrix::Vector3f est_vel = _control.getESOEstimatedVelocity();
				const matrix::Vector3f est_dist = _control.getESODisturbance();

				/* Removed internal uORB topic
				eso_pos_debug_s eso_debug{};
				eso_debug.timestamp = hrt_absolute_time();
				est_pos.copyTo(eso_debug.estimated_position);
				est_vel.copyTo(eso_debug.estimated_velocity);
				est_dist.copyTo(eso_debug.estimated_disturbance);

				// 实际值
				const matrix::Vector3f cur_pos = _control.getCurrentPosition();
				const matrix::Vector3f cur_vel = _control.getCurrentVelocity();
				const matrix::Vector3f eso_in = _control.getESOInput();

				cur_pos.copyTo(eso_debug.actual_position);
				cur_vel.copyTo(eso_debug.actual_velocity);
				eso_in.copyTo(eso_debug.eso_input);

				_eso_pos_debug_pub.publish(eso_debug);
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

				publish_key_value("ESO_PX", est_pos(0));
				publish_key_value("ESO_PY", est_pos(1));
				publish_key_value("ESO_PZ", est_pos(2));
				publish_key_value("ESO_DX", est_dist(0));
				publish_key_value("ESO_DY", est_dist(1));
				publish_key_value("ESO_DZ", est_dist(2));
			}

			// 25. 【最终输出】发布姿态设定点 (Attitude Setpoint)
            		// 这包含了期望的 滚转、俯仰、偏航角 以及 油门大小
            		// 这个消息会被 mc_att_control (姿态控制器) 接收，转化为角速度和电机转速
				vehicle_attitude_setpoint_s attitude_setpoint{};
				_control.getAttitudeSetpoint(attitude_setpoint);
				attitude_setpoint.timestamp = hrt_absolute_time();
				_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

				// -------------------------------------------------------------------------
				// [调试打印-输出侧] 用于判断是否因为“推力/倾角饱和”导致位置环发散
				//
				// 现象：起飞后姿态逐渐变成大倾角（甚至翻滚），常见原因是：
				// - 推力不足（接近 thr_max），导致垂直优先后水平控制失效，位置误差越滚越大
				// - 倾角限制被顶满（tilt_limit），水平加速度不够，位置误差越积越大
				// - hover_thrust/推力映射不匹配，导致控制器认为“该加推力”但实际升力不够
				//
				// 打印字段说明：
				// - thrust_body     : 最终发布给姿态环的机体系推力向量（FRD），注意 z 通常为负（向上推力）
				// - thrust_ned      : 位置控制内部的 NED 推力向量（用于生成姿态 setpoint），看水平/垂直分量
				// - tilt_sp/limit   : 期望倾角（由 q_d 解析）与当前倾角限制
				// - thr_min/max/hover: 推力上下限与悬停推力参数（排查推力映射）
				// -------------------------------------------------------------------------
					// static hrt_abstime last_out_dbg_print{0};
					const bool dbg_flying_out = (_takeoff.getTakeoffState() >= ESOTakeoffState::flight);
					if (dbg_flying_out) {
						// const Vector3f d_hat = _control.getESODisturbance();
						// const Vector3f u_in = _control.getESOInput();

						const Vector3f thr_body(attitude_setpoint.thrust_body);
						const Vector3f thr_ned(local_pos_sp.thrust);

						// const float thr_body_mag = thr_body.norm();
						// const float thr_ned_xy = Vector2f(thr_ned).norm();
						// const float thr_ned_z = thr_ned(2);

					const Dcmf R_sp(Quatf(attitude_setpoint.q_d));
					// const float cos_tilt = math::constrain(R_sp(2, 2), -1.f, 1.f);
					// const float tilt_sp_deg = math::degrees(acosf(cos_tilt));
					// const float tilt_lim_deg = math::degrees(_tilt_limit_slew_rate.getState());

					// const float thr_min = _param_ESO_thr_min.get();
					// const float thr_max = _param_ESO_thr_max.get();
						// const float thr_hover = _param_ESO_thr_hover.get();

						// const bool near_tilt_limit = (tilt_lim_deg > 1e-3f) && (tilt_sp_deg > 0.9f * tilt_lim_deg);
						// const bool near_thrust_max = (fabsf(thr_body(2)) > 0.9f * thr_max);

						// 只在“接近倾角/推力饱和”时打印快照（炸机通常发生在这一瞬间）
						// const bool want_snapshot = (near_tilt_limit || near_thrust_max);

						/*if (want_snapshot && (hrt_elapsed_time(&last_out_dbg_print) > 100_ms)) {
							last_out_dbg_print = hrt_absolute_time();

							const Vector3f pos_sp(local_pos_sp.x, local_pos_sp.y, local_pos_sp.z);
							const Vector3f vel_sp(local_pos_sp.vx, local_pos_sp.vy, local_pos_sp.vz);
							const Vector3f acc_sp(local_pos_sp.acceleration);
							const Vector3f pos_err = pos_sp - states.position;
							const Vector3f vel_err = vel_sp - states.velocity;

							PX4_INFO("ESO pos out: thr_body[%.2f %.2f %.2f]|%.2f thr_ned_xy=%.2f thr_ned_z=%.2f tilt=%.1f/%.1f thr(min/max/hover)=%.2f/%.2f/%.2f sat(tilt/thr)=%d/%d",
								 (double)thr_body(0), (double)thr_body(1), (double)thr_body(2), (double)thr_body_mag,
								 (double)thr_ned_xy, (double)thr_ned_z,
								 (double)tilt_sp_deg, (double)tilt_lim_deg,
								 (double)thr_min, (double)thr_max, (double)thr_hover,
								 (int)near_tilt_limit, (int)near_thrust_max);

							// -----------------------------------------------------------------
							// [调试打印-对齐快照] 用同一触发条件把“输入/误差/ESO/输出”对齐到同一时刻
							//
							// 为什么要看这些量：
							// - pos_err/vel_err：判断是不是位置误差驱动了大倾角（正常：误差大→倾角大）
							// - acc_sp/thr_ned：判断位置环是否已经把输出打到极限（饱和会导致进一步发散）
							// - d_hat/u_in：判断 ESO 是否在给出异常大的扰动估计（符号/带宽/输入建模错误会导致 d_hat 爆炸）
							//
							// 注：local_pos_sp 是“喂给控制器的输入 setpoint”（pos/yaw） + “控制器内部执行的 setpoint”（vel/acc/thrust）。
							// -----------------------------------------------------------------
							PX4_INFO("ESO pos snap: dt=%.3f state=%u pos[%.2f %.2f %.2f] sp[%.2f %.2f %.2f] e_p[%.2f %.2f %.2f] vel[%.2f %.2f %.2f] sp_v[%.2f %.2f %.2f] e_v[%.2f %.2f %.2f] acc_sp[%.2f %.2f %.2f] d_hat[%.2f %.2f %.2f] u_in[%.2f %.2f %.2f]",
								 (double)dt,
								 (unsigned)_takeoff.getTakeoffState(),
								 (double)states.position(0), (double)states.position(1), (double)states.position(2),
								 (double)pos_sp(0), (double)pos_sp(1), (double)pos_sp(2),
								 (double)pos_err(0), (double)pos_err(1), (double)pos_err(2),
								 (double)states.velocity(0), (double)states.velocity(1), (double)states.velocity(2),
								 (double)vel_sp(0), (double)vel_sp(1), (double)vel_sp(2),
								 (double)vel_err(0), (double)vel_err(1), (double)vel_err(2),
								 (double)acc_sp(0), (double)acc_sp(1), (double)acc_sp(2),
								 (double)d_hat(0), (double)d_hat(1), (double)d_hat(2),
								 (double)u_in(0), (double)u_in(1), (double)u_in(2));

						}*/
					}

				// -------------------------------------------------------------------------
				// [调试打印] 验证 yaw setpoint 是否是问题根因
				//
				// 需要盯住两件事：
				// 1) `vehicle_attitude_setpoint` 里的 yaw / yaw_sp_move_rate（yawspeed）是否被固定/突变（尤其起飞瞬间）
				// 2) 下游姿态环把它转换成 `vehicle_rates_setpoint.yaw` 后是否长期贴着 ±rate_limit（会导致对角电机快慢明显）
				//
				// 这里先在位置环侧把“发出去的 vehicle_attitude_setpoint”打印出来，方便确认上游是否一直在喂一个固定航向。
				// 触发条件：已解锁且 yaw 误差很大（>30deg）或 yawspeed 非零；5Hz 节流。
				// -------------------------------------------------------------------------
				static hrt_abstime last_attsp_dbg_print{0};
				const float curr_yaw = local_pos.heading;
				const Eulerf euler_sp(Quatf(attitude_setpoint.q_d));
				const float sp_yaw = euler_sp.psi();
				const float yaw_err = wrap_pi(sp_yaw - curr_yaw);
				const bool want_print =
					_vehicle_control_mode.flag_armed &&
					((fabsf(yaw_err) > math::radians(30.f)) || (fabsf(attitude_setpoint.yaw_sp_move_rate) > math::radians(5.f)));

				if (want_print && (hrt_elapsed_time(&last_attsp_dbg_print) > 200_ms)) {
					last_attsp_dbg_print = hrt_absolute_time();
					PX4_INFO("ESO pos attsp: yaw_sp=%.1fdeg yawspeed=%.1fdeg/s yaw=%.1fdeg err=%.1fdeg traj_yaw=%.1fdeg traj_yawspeed=%.1fdeg/s",
						 (double)math::degrees(sp_yaw),
						 (double)math::degrees(attitude_setpoint.yaw_sp_move_rate),
						 (double)math::degrees(curr_yaw),
						 (double)math::degrees(yaw_err),
						 (double)math::degrees(_setpoint.yaw),
						 (double)math::degrees(_setpoint.yawspeed));
				}

			} else {
				// 如果不在位置控制模式 (比如切到了 Stabilized)，也要维持起飞状态机的更新
	           		// 防止切回来的时候状态机逻辑错乱
			// an update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    time_stamp_now);
		}

		// 26. 发布起飞状态 (用于状态显示)
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}

		// save latest reset counters
		_vxy_reset_counter = local_pos.vxy_reset_counter;
		_vz_reset_counter = local_pos.vz_reset_counter;
		_xy_reset_counter = local_pos.xy_reset_counter;
		_z_reset_counter = local_pos.z_reset_counter;
		_heading_reset_counter = local_pos.heading_reset_counter;
	}

	// 27. 结束性能计数
	perf_end(_cycle_perf);
}

void ESOMulticopterPositionControl::failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint,
		const PositionControlStates &states, bool warn)
{
	// Only react after a short delay
	_failsafe_land_hysteresis.set_state_and_update(true, now);

	if (_failsafe_land_hysteresis.get_state()) {
		reset_setpoint_to_nan(setpoint);

		if (PX4_ISFINITE(states.velocity(0)) && PX4_ISFINITE(states.velocity(1))) {
			// don't move along xy
			setpoint.vx = setpoint.vy = 0.f;

			if (warn) {
				PX4_WARN("Failsafe: stop and wait");
			}

		} else {
			// descend with land speed since we can't stop
			setpoint.acceleration[0] = setpoint.acceleration[1] = 0.f;
			setpoint.vz = _param_ESO_land_speed.get();

			if (warn) {
				PX4_WARN("Failsafe: blind land");
			}
		}

		if (PX4_ISFINITE(states.velocity(2))) {
			// don't move along z if we can stop in all dimensions
			if (!PX4_ISFINITE(setpoint.vz)) {
				setpoint.vz = 0.f;
			}

		} else {
			// emergency descend with a bit below hover thrust
			setpoint.vz = NAN;
			setpoint.acceleration[2] = .3f;

			if (warn) {
				PX4_WARN("Failsafe: blind descend");
			}
		}

		_in_failsafe = true;
	}
}

void ESOMulticopterPositionControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

int ESOMulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	ESOMulticopterPositionControl *instance = new ESOMulticopterPositionControl(vtol);

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

int ESOMulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ESOMulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("eso_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int eso_pos_control_main (int argc, char *argv[])
{
	return ESOMulticopterPositionControl::main(argc, argv);
}
