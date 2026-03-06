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

#include <ESORateControl/RateControl.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/eso_rate_aux.h>
#include <uORB/topics/debug_key_value.h>   // <--- Bridge to ROS named_value_float
#include <uORB/topics/arm_joint_states.h>    // [NEW]
#include <eso_att_control/ArmKinematics.hpp> // [NEW] 用于速率环惯量更新
#include <eso_common/ESOModelProfile.hpp>


using namespace time_literals;

class ESOMulticopterRateControl : public ModuleBase<ESOMulticopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	ESOMulticopterRateControl(bool vtol = false);
	~ESOMulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	// [NEW] 动态惯量参数更新
	void 		update_dynamic_parameters();


	void updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt);

	void publishTorqueSetpoint(const matrix::Vector3f &torque_sp, const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);

	ESORateControl _rate_control; ///< class for rate control calculations

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _eso_rate_aux_sub{ORB_ID(eso_rate_aux)}; /**< 附加 ESO 姿态输出订阅 */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// [NEW]
	uORB::Subscription _arm_joint_states_sub{ORB_ID(arm_joint_states)};
	ArmKinematics _arm_kinematics;
	eso_common::ModelProfile _active_profile{eso_common::ModelProfile::UavArmV4};
	matrix::Matrix3f _static_inertia{matrix::diag(matrix::Vector3f(0.045f, 0.045f, 0.08f))};


	uORB::Publication<actuator_controls_s>		_actuators_0_pub;
	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_0_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	// uORB::Publication<eso_rate_debug_s>		_eso_rate_debug_pub{ORB_ID(eso_rate_debug)};  /**< Removed: 发布ESO角速度调试信息 */
	uORB::Publication<debug_key_value_s>		_debug_key_value_pub{ORB_ID(debug_key_value)}; /**< Bridge to ROS */

	orb_advert_t _mavlink_log_pub{nullptr};

	vehicle_control_mode_s		_v_control_mode{};
	vehicle_status_s		_vehicle_status{};

	bool _actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */
	bool _landed{true};
	bool _maybe_landed{true};

	float _battery_status_scale{0.0f};
	hrt_abstime _last_eso_aux_timestamp{0}; /**< 最近一次收到 eso_rate_aux 的时间戳 */
	matrix::Vector3f _alloc_unallocated_torque{}; /**< 控制分配器未分配力矩缓存（x/y/z） */
	matrix::Vector<bool, 3> _alloc_sat_pos{}; /**< 轴向正饱和标志缓存 */
	matrix::Vector<bool, 3> _alloc_sat_neg{}; /**< 轴向负饱和标志缓存 */

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */

	float		_thrust_sp{0.0f};		/**< thrust setpoint */

	hrt_abstime _last_run{0};

	int8_t _landing_gear{landing_gear_s::GEAR_DOWN};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ESO_ROLLRATE_P>) _param_eso_rollrate_p,
		(ParamFloat<px4::params::ESO_ROLLRATE_I>) _param_eso_rollrate_i,
		(ParamFloat<px4::params::ESO_RR_INT_LIM>) _param_eso_rr_int_lim,
		(ParamFloat<px4::params::ESO_ROLLRATE_D>) _param_eso_rollrate_d,
		(ParamFloat<px4::params::ESO_ROLLRATE_FF>) _param_eso_rollrate_ff,
		(ParamFloat<px4::params::ESO_ROLLRATE_K>) _param_eso_rollrate_k,

		(ParamFloat<px4::params::ESO_PITCHRATE_P>) _param_eso_pitchrate_p,
		(ParamFloat<px4::params::ESO_PITCHRATE_I>) _param_eso_pitchrate_i,
		(ParamFloat<px4::params::ESO_PR_INT_LIM>) _param_eso_pr_int_lim,
		(ParamFloat<px4::params::ESO_PITCHRATE_D>) _param_eso_pitchrate_d,
		(ParamFloat<px4::params::ESO_PITCHRATE_FF>) _param_eso_pitchrate_ff,
		(ParamFloat<px4::params::ESO_PITCHRATE_K>) _param_eso_pitchrate_k,

		(ParamFloat<px4::params::ESO_YAWRATE_P>) _param_eso_yawrate_p,
		(ParamFloat<px4::params::ESO_YAWRATE_I>) _param_eso_yawrate_i,
		(ParamFloat<px4::params::ESO_YR_INT_LIM>) _param_eso_yr_int_lim,
		(ParamFloat<px4::params::ESO_YAWRATE_D>) _param_eso_yawrate_d,
		(ParamFloat<px4::params::ESO_YAWRATE_FF>) _param_eso_yawrate_ff,
		(ParamFloat<px4::params::ESO_YAWRATE_K>) _param_eso_yawrate_k,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		// ESO rate control model/observer parameters
		(ParamFloat<px4::params::ESO_INERTIA_XX>) _param_eso_inertia_xx,
		(ParamFloat<px4::params::ESO_INERTIA_YY>) _param_eso_inertia_yy,
		(ParamFloat<px4::params::ESO_INERTIA_ZZ>) _param_eso_inertia_zz,

		(ParamFloat<px4::params::ESO_RATE_BW_R>) _param_eso_rate_bw_r,
		(ParamFloat<px4::params::ESO_RATE_BW_P>) _param_eso_rate_bw_p,
		(ParamFloat<px4::params::ESO_RATE_BW_Y>) _param_eso_rate_bw_y,

			(ParamFloat<px4::params::ESO_K_BETA>) _param_eso_k_beta,
			(ParamFloat<px4::params::ESO_MAX_TORQUE>) _param_eso_max_torque,
			(ParamFloat<px4::params::ESO_RATE_I_SC>) _param_eso_rate_i_scale,
			(ParamFloat<px4::params::ESO_TAUS_K>) _param_eso_taus_k,
			(ParamFloat<px4::params::ESO_TAUS_LIM>) _param_eso_taus_lim,

		(ParamFloat<px4::params::ESO_ACRO_R_MAX>) _param_eso_acro_r_max,
		(ParamFloat<px4::params::ESO_ACRO_P_MAX>) _param_eso_acro_p_max,
		(ParamFloat<px4::params::ESO_ACRO_Y_MAX>) _param_eso_acro_y_max,
		(ParamFloat<px4::params::ESO_ACRO_EXPO>) _param_eso_acro_expo,				/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::ESO_ACRO_EXPO_Y>) _param_eso_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::ESO_ACRO_SUPEXPO>) _param_eso_acro_supexpo,			/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::ESO_ACRO_SUPEX_Y>) _param_eso_acro_supexpoy,			/**< superexpo stick curve shape (yaw) */

		(ParamBool<px4::params::ESO_BAT_SCALE_EN>) _param_eso_bat_scale_en,

		(ParamInt<px4::params::ESO_ARM_MODEL>) _param_eso_arm_model,
		(ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl
	)

	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */

};
