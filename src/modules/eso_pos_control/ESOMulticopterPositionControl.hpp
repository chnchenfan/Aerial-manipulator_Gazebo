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

/**
 * Multicopter position controller.
 */

#pragma once

#include "ESOPositionControl/PositionControl.hpp"
#include "ESOTakeoff/Takeoff.hpp"

#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/arm_joint_states.h>    // [NEW]
#include <eso_att_control/ArmKinematics.hpp> // [NEW] 位置环也需要算 CoM
#include <eso_common/ESOModelProfile.hpp>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>          // <--- 必须补上
#include <uORB/topics/vehicle_angular_velocity.h>  // <--- 必须补上
#include <uORB/topics/debug_key_value.h>           // <--- Bridge to ROS named_value_float

using namespace time_literals;

// 定义主类，继承自：
// ModuleBase: 提供模块入口、启动停止、状态查询等基础功能
// SuperBlock: 提供参数组管理功能
// ModuleParams: 处理参数更新
// ScheduledWorkItem: 说明这是一个在工作队列中按时间调度运行的任务
class ESOMulticopterPositionControl : public ModuleBase<ESOMulticopterPositionControl>, public control::SuperBlock,
	public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ESOMulticopterPositionControl(bool vtol = false);// 构造函数，vtol 标志是否为垂起机型
	~ESOMulticopterPositionControl() override;       // 析构函数

	/** 标准模块入口函数 */
	static int task_spawn(int argc, char *argv[]);

	/** 处理自定义命令行指令 */
	static int custom_command(int argc, char *argv[]);

	/** 打印模块使用帮助 */
	static int print_usage(const char *reason = nullptr);

	bool init(); // 初始化函数，订阅话题等

private:
	void Run() override; // 【核心】主循环函数，会被系统定时调用，控制逻辑都在这里

	ESOTakeoff _takeoff; /**< 起飞状态机，处理平滑起飞逻辑 */

	orb_advert_t _mavlink_log_pub{nullptr}; // 用于向地面站发送文本消息

	// 发布者 (Publications)：将控制结果发给其他模块
	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub {ORB_ID(takeoff_status)};// 发布起飞状态
	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub {ORB_ID(vehicle_attitude_setpoint)}; // 发布期望姿态（这是位置控制的最终输出）
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub {ORB_ID(vehicle_local_position_setpoint)};	/**< 发布当前实际执行的设定点（包含平滑后的结果） */
	// uORB::Publication<eso_pos_debug_s>                   _eso_pos_debug_pub {ORB_ID(eso_pos_debug)};  /**< Removed: ESO位置调试话题 */
	uORB::Publication<debug_key_value_s>                 _debug_key_value_pub {ORB_ID(debug_key_value)}; /**< Bridge to ROS */

	// 订阅者 (Subscriptions)：获取传感器数据和其他模块的状态
	uORB::SubscriptionCallbackWorkItem _local_pos_sub {this, ORB_ID(vehicle_local_position)};	/**< 订阅当前位置/速度估计值 (EKF输出) */

	uORB::SubscriptionInterval _parameter_update_sub {ORB_ID(parameter_update), 1_s}; // 订阅参数更新通知

	uORB::Subscription _hover_thrust_estimate_sub {ORB_ID(hover_thrust_estimate)}; // 订阅悬停油门估计
	uORB::Subscription _trajectory_setpoint_sub {ORB_ID(trajectory_setpoint)};     // 订阅期望轨迹（来自 Navigator 或 Offboard）
	uORB::Subscription _vehicle_constraints_sub {ORB_ID(vehicle_constraints)};     // 订阅飞行限制
	uORB::Subscription _vehicle_control_mode_sub {ORB_ID(vehicle_control_mode)};   // 订阅控制模式（是否解锁、是否位置模式等）
	uORB::Subscription _vehicle_land_detected_sub {ORB_ID(vehicle_land_detected)}; // 订阅着陆检测状态

	// [NEW]
	uORB::Subscription _arm_joint_states_sub{ORB_ID(arm_joint_states)};
	ArmKinematics _arm_kinematics;
	eso_common::ModelProfile _active_profile{eso_common::ModelProfile::UavArmV4};
	bool _use_dynamic_arm_model{true};

	// [NEW] 动态耦合参数更新
	void update_dynamic_parameters();

	// 【新增】ESO 需要姿态和角速度来计算离心力补偿
	uORB::Subscription _vehicle_attitude_sub {ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub {ORB_ID(vehicle_angular_velocity)};

	// CoM 处理策略：优先使用动态 CoM；若动态数据不可用/超时，则回退到硬编码 CoM
	static constexpr hrt_abstime DYNAMIC_COM_TIMEOUT{500_ms};
	matrix::Vector3f _fallback_com{-0.015f, 0.0f, -0.164f};
	hrt_abstime _last_dynamic_com_timestamp{0};
	bool _dynamic_com_valid{false};

	hrt_abstime	_time_stamp_last_loop{0};		/**< 上次循环的时间戳，用于计算 dt */

	// 缓存的数据结构
	vehicle_local_position_setpoint_s _setpoint {};
	vehicle_control_mode_s _vehicle_control_mode {};

	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	// 【参数定义宏】这里将 C语言定义的参数 (如 ESO_XY_P) 映射为 C++ 成员变量 (如 _param_ESO_xy_p)
	DEFINE_PARAMETERS(
		// Position Control
		(ParamFloat<px4::params::ESO_XY_P>)         _param_ESO_xy_p,
		(ParamFloat<px4::params::ESO_XY_I>)         _param_ESO_xy_i,
		(ParamFloat<px4::params::ESO_XY_VEL_P_ACC>) _param_ESO_xy_vel_p_acc,
		(ParamFloat<px4::params::ESO_XY_BW>)        _param_ESO_xy_bw,

		(ParamFloat<px4::params::ESO_Z_P>)          _param_ESO_z_p,
		(ParamFloat<px4::params::ESO_Z_I>)          _param_ESO_z_i,
		(ParamFloat<px4::params::ESO_Z_VEL_P_ACC>)  _param_ESO_z_vel_p_acc,
		(ParamFloat<px4::params::ESO_Z_BW>)         _param_ESO_z_bw,

		// 起飞前ESO门控开关：flight 前不更新ESO，启用瞬间用测量初始化，避免尖峰
		(ParamBool<px4::params::ESO_POS_ESOGATE>)   _param_ESO_pos_esogate,

		(ParamFloat<px4::params::ESO_POS_INT_LIM>) _param_ESO_pos_int_lim,


		(ParamFloat<px4::params::ESO_XY_VEL_MAX>)   _param_ESO_xy_vel_max,
		(ParamFloat<px4::params::ESO_Z_V_AUTO_UP>)  _param_ESO_z_v_auto_up,
		(ParamFloat<px4::params::ESO_Z_VEL_MAX_UP>) _param_ESO_z_vel_max_up,
		(ParamFloat<px4::params::ESO_Z_V_AUTO_DN>)  _param_ESO_z_v_auto_dn,
		(ParamFloat<px4::params::ESO_Z_VEL_MAX_DN>) _param_ESO_z_vel_max_dn,
		(ParamFloat<px4::params::ESO_TILTMAX_AIR>)  _param_ESO_tiltmax_air,
		(ParamFloat<px4::params::ESO_THR_HOVER>)    _param_ESO_thr_hover,
		(ParamBool<px4::params::ESO_USE_HTE>)       _param_ESO_use_hte,

		// Takeoff / Land
		(ParamFloat<px4::params::ESO_SPOOLUP_TIME>) _param_ESO_spoolup_time, /**< time to let motors spool up after arming */
		(ParamFloat<px4::params::ESO_TKO_RAMP_T>)   _param_ESO_tko_ramp_t,   /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::ESO_TKO_SPEED>)    _param_ESO_tko_speed,
		(ParamFloat<px4::params::ESO_LAND_SPEED>)   _param_ESO_land_speed,

		(ParamFloat<px4::params::ESO_VEL_MANUAL>)   _param_ESO_vel_manual,
		(ParamFloat<px4::params::ESO_XY_CRUISE>)    _param_ESO_xy_cruise,
		(ParamFloat<px4::params::ESO_LAND_ALT2>)    _param_ESO_land_alt2,    /**< downwards speed limited below this altitude */
		(ParamInt<px4::params::ESO_POS_MODE>)       _param_ESO_pos_mode,
		(ParamInt<px4::params::ESO_ALT_MODE>)       _param_ESO_alt_mode,
		(ParamFloat<px4::params::ESO_TILTMAX_LND>)  _param_ESO_tiltmax_lnd,  /**< maximum tilt for landing and smooth takeoff */
		(ParamFloat<px4::params::ESO_THR_MIN>)      _param_ESO_thr_min,
		(ParamFloat<px4::params::ESO_THR_MAX>)      _param_ESO_thr_max,
		(ParamFloat<px4::params::ESO_THR_XY_MARG>)  _param_ESO_thr_xy_marg,

		(ParamFloat<px4::params::SYS_VEHICLE_RESP>) _param_sys_vehicle_resp,
		(ParamFloat<px4::params::ESO_ACC_HOR>)      _param_ESO_acc_hor,
		(ParamFloat<px4::params::ESO_ACC_DOWN_MAX>) _param_ESO_acc_down_max,
		(ParamFloat<px4::params::ESO_ACC_UP_MAX>)   _param_ESO_acc_up_max,
		(ParamFloat<px4::params::ESO_ACC_HOR_MAX>)  _param_ESO_acc_hor_max,
		(ParamFloat<px4::params::ESO_JERK_AUTO>)    _param_ESO_jerk_auto,
		(ParamFloat<px4::params::ESO_JERK_MAX>)     _param_ESO_jerk_max,
		(ParamFloat<px4::params::ESO_MAN_Y_MAX>)    _param_ESO_man_y_max,
		(ParamFloat<px4::params::ESO_MAN_Y_TAU>)    _param_ESO_man_y_tau,

		(ParamFloat<px4::params::ESO_XY_VEL_ALL>)   _param_ESO_xy_vel_all,
		(ParamFloat<px4::params::ESO_Z_VEL_ALL>)    _param_ESO_z_vel_all,
		(ParamInt<px4::params::ESO_ARM_MODEL>)      _param_eso_arm_model
	);

	// 微分模块：用来计算速度的导数（加速度）。如果你在底层直接用了 ESO 的估计值，这些可能后续可以移除。
	control::BlockDerivative _vel_x_deriv; /**< velocity derivative in x */
	control::BlockDerivative _vel_y_deriv; /**< velocity derivative in y */
	control::BlockDerivative _vel_z_deriv; /**< velocity derivative in z */

	ESOPositionControl _control;  /**< 【核心】底层的 PID+ESO 控制器实例 */

	hrt_abstime _last_warn{0}; /**< 防止日志报警刷屏的计时器 */

	bool _in_failsafe{false};  /**< 标记当前是否进入了失效保护状态 */

	bool _hover_thrust_initialized{false};

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	/** If Flighttask fails, keep 0.2 seconds the current setpoint before going into failsafe land */
	static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200_ms;

	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off and tilt is limited */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	static constexpr float MAX_SAFE_TILT_DEG = 89.f; // Numerical issues above this value due to tanf

	systemlib::Hysteresis _failsafe_land_hysteresis{false}; /**< 失效保护降落的迟滞处理 */
	SlewRate<float> _tilt_limit_slew_rate; // 倾角限制的斜率限制（防止倾角变化太剧烈）

	// 重置计数器，用于检测 EKF 是否发生了重置（如 GPS 跳变）
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")}; // 性能计数器

	/**
         * 参数更新函数。
         * 在这里，你需要把上面 DEFINE_PARAMETERS 里读取到的 _param_... 的值
         * 塞到 _control (ESOPositionControl) 对象里去。
         */
	void parameters_update(bool force);

	/** 状态检查辅助函数
	 * Check for validity of positon/velocity states.
	 */
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos);

	/** 失效保护处理逻辑
	 * Failsafe.
	 * If flighttask fails for whatever reason, then do failsafe. This could
	 * occur if the commander fails to switch to a mode in case of invalid states or
	 * setpoints. The failsafe will occur after LOITER_TIME_BEFORE_DESCEND. If force is set
	 * to true, the failsafe will be initiated immediately.
	 */
	void failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states,
		      bool warn);

	/** 辅助函数：将设定点重置为 NAN */
	void reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint);
};
