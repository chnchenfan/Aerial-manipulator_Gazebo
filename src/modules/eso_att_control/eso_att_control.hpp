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

#pragma once

#include <lib/mixer/MixerBase/Mixer.hpp> // Airmode
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/eso_rate_aux.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <vtol_att_control/vtol_type.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/topics/debug_key_value.h>   // <--- Bridge to ROS named_value_float

#include <ESOAttitudeControl/AttitudeControl.hpp>
#include <eso_att_control/ArmKinematics.hpp> // [NEW] 引入运动学解算库
#include <eso_common/ESOModelProfile.hpp>
#include <uORB/topics/arm_joint_states.h>    // [NEW] 引入关节状态消息

using namespace time_literals;

/**
 * 继承自 ModuleBase：说明它是一个标准的 PX4 模块，可以在控制台用 eso_att_control start 启动。
 * 继承自 WorkItem：说明它是一个**“打工人”。它不是一直死循环空转，而是“有活儿才干”**。
 * 通常是当陀螺仪或姿态传感器传来新数据时，它会被叫醒工作一次
 */
extern "C" __EXPORT int eso_att_control_main(int argc, char *argv[]);

class ESOMulticopterAttitudeControl : public ModuleBase<ESOMulticopterAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	ESOMulticopterAttitudeControl(bool vtol = false);
	~ESOMulticopterAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]); // 创建并启动任务

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]); // 特殊指令：处理像 "status" 这种命令

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr); // 打印帮助信息

	bool init(); // 初始化话题订阅和发布

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	/**
	 * [NEW] 动态耦合参数更新逻辑
	 * 根据接收到的关节角 q，利用运动学模型实时更新 _com_offset 和 _arm_inertia
	 */
	void 		update_dynamic_parameters();


	/** 油门曲线函数，只处理飞手手中的遥控器（RC）摇杆信号
	 * 在位置模式下，推力是直通的（位置环 -> 姿态环 -> 速率环），中间没有修改。
	 * 在手动模式下，推力不是直通的，它经过了 throttle_curve 的“美颜”（为了手感好）。
	 */
	float		throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void		generate_attitude_setpoint(const matrix::Quatf &q, float dt, bool reset_yaw_sp);

	ESOAttitudeControl _attitude_control; ///< class for attitude control calculations
	ArmKinematics _arm_kinematics;        ///< [NEW] 机械臂运动学解算器


	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};// 监听参数修改

	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)}; // 订阅目标姿态
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)}; // 订阅当前角速度（用于 tau_s）
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription 定高定点自稳 */
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	// 【核心触发源】监听当前姿态。注意它是 CallbackWorkItem，
	// 意思是：只要这封信一到，马上触发 Run() 函数开始工作！
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	// 【核心输出】发布角速度指令。这是给内环（角速度控制器）看的命令。
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};			/**< rate setpoint publication */
	uORB::Publication<eso_rate_aux_s>           _eso_rate_aux_pub{ORB_ID(eso_rate_aux)}; /**< 附加输出：omega_r_dot 与 beta_v */
	// 发布处理后的姿态设定点（用于记录日志或显示在地面站上）
	uORB::Publication<vehicle_attitude_setpoint_s>	_vehicle_attitude_setpoint_pub;
	uORB::Publication<debug_key_value_s>		_debug_key_value_pub{ORB_ID(debug_key_value)}; /**< Bridge to ROS */

	struct manual_control_setpoint_s	_manual_control_setpoint {};	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode {};	/**< vehicle control mode */

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	matrix::Vector3f _thrust_setpoint_body; ///< body frame 3D thrust vector
	float _mass_total{0.f};
	matrix::Vector3f _com_offset{};
	matrix::Matrix3f _arm_inertia{};
	eso_common::ModelProfile _active_profile{eso_common::ModelProfile::UavArmV4};

	// [NEW] 机械臂关节状态订阅与数据缓存
	uORB::Subscription _arm_joint_states_sub{ORB_ID(arm_joint_states)};
	arm_joint_states_s _arm_joints{};


	float _man_yaw_sp{0.f};				/**< current yaw setpoint in manual mode */
	float _man_tilt_max;			/**< maximum tilt allowed for manual flight [rad] */
	AlphaFilter<float> _man_x_input_filter;
	AlphaFilter<float> _man_y_input_filter;

	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};
	hrt_abstime _last_att_dbg_print{0}; ///< 姿态环调试快照节流（避免刷屏）
	vehicle_attitude_setpoint_s _attitude_setpoint_last{}; ///< 最近一次有效的 vehicle_attitude_setpoint（用于调试打印）
	bool _attitude_setpoint_last_valid{false};

	bool _landed{true};
	bool _reset_yaw_sp{true};
	bool _vehicle_type_rotary_wing{true};
	bool _vtol{false};
	bool _vtol_tailsitter{false};
	bool _vtol_in_transition_mode{false};

	uint8_t _quat_reset_counter{0};
	bool _was_armed{false};
	bool _was_run_att_ctrl{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ESO_ROLL_P>) _param_eso_roll_p,
		(ParamFloat<px4::params::ESO_PITCH_P>) _param_eso_pitch_p,
		(ParamFloat<px4::params::ESO_YAW_P>) _param_eso_yaw_p,
		(ParamFloat<px4::params::ESO_YAW_WEIGHT>) _param_eso_yaw_weight,

		(ParamFloat<px4::params::ESO_ROLLRATE_MAX>) _param_eso_rollrate_max,
		(ParamFloat<px4::params::ESO_PITCHRT_MAX>) _param_eso_pitchrate_max,
		(ParamFloat<px4::params::ESO_YAWRATE_MAX>) _param_eso_yawrate_max,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		/* Stabilized mode params */
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,			/**< maximum tilt allowed for manual flight */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,			/**< minimum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,				/**< maximum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_HOVER>)
		_param_mpc_thr_hover,			/**< throttle at which vehicle is at hover equilibrium */
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,				/**< throttle curve behavior */

		(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode,
		(ParamFloat<px4::params::ESO_MAN_TILT_TAU>) _param_eso_man_tilt_tau,

		// tau_s 补偿相关参数
		(ParamFloat<px4::params::ESO_MASS_TOTAL>) _param_eso_mass_total,
		(ParamFloat<px4::params::ESO_COM_X>) _param_eso_com_x,
		(ParamFloat<px4::params::ESO_COM_Y>) _param_eso_com_y,
		(ParamFloat<px4::params::ESO_COM_Z>) _param_eso_com_z,
		(ParamFloat<px4::params::ESO_ARM_IXX>) _param_eso_arm_inertia_xx,
		(ParamFloat<px4::params::ESO_ARM_IYY>) _param_eso_arm_inertia_yy,
		(ParamFloat<px4::params::ESO_ARM_IZZ>) _param_eso_arm_inertia_zz,
		(ParamInt<px4::params::ESO_ARM_MODEL>) _param_eso_arm_model
	)
};
