/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include "PositionESO.hpp"
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

struct PositionControlStates { // 飞行器当前状态
	matrix::Vector3f position;      // 当前位置 (x, y, z)
	matrix::Vector3f velocity;      // 当前速度 (vx, vy, vz)
	matrix::Vector3f acceleration;  // 当前加速度 (ax, ay, az)
	float yaw;                      // 当前偏航角
};

class ESOPositionControl
{
public:

	ESOPositionControl() = default;
	~ESOPositionControl() = default;
//=================================---- 控制参数设置接口 ----=============================================//
	/** 设置位置外环比例增益，把外部的增益值传递进来，命名为内部的变量名
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P, const matrix::Vector3f &I) { _gain_pos_p = P; _gain_pos_i = I; }

	/** 设置位置内环PID增益，把外部的增益值传递进来，命名为内部的变量名
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/** 设置ESO带宽，把外部的值传递进来，命名为内部的变量名
	 * Set the Extended State Observer bandwidth
	 * @param bw 3D vector of bandwidth for x,y,z axis
	 */
	void setESOBandwidth(const matrix::Vector3f &bw) { _eso.setBandwidth(bw); }

	/** 控制是否更新位置ESO（起飞前可禁用，避免观测器尖峰）
	 *
	 * 说明：禁用时，位置控制仍运行，但扰动估计 d_hat 置 0，不更新 ESO 内部状态。
	 * 启用瞬间会用当前测量初始化 ESO（z1=pos, z2=vel, z3=0），以避免导数/扰动尖峰。
	 */
	void setESOUpdateEnabled(const bool enabled) { _eso_update_enabled = enabled; }

	/** 获取ESO估计的扰动 d_hat（用于调试对齐快照打印） */
	matrix::Vector3f getESODisturbance() const { return _eso_disturbance; }

	/** 获取给ESO的名义输入 u_in（用于调试对齐快照打印） */
	matrix::Vector3f getESOInput() const { return _eso_input; }

	/** 获取ESO估计的位置 z1 */
	matrix::Vector3f getESOEstimatedPosition() const { return _eso.getEstimatedPosition(); }

	/** 获取ESO估计的速度 z2 */
	matrix::Vector3f getESOEstimatedVelocity() const { return _eso.getEstimatedVelocity(); }

	/** 获取当前状态（位置、速度） */
	matrix::Vector3f getCurrentPosition() const { return _pos; }
	matrix::Vector3f getCurrentVelocity() const { return _vel; }

	/** 设置机体系速度，把外部的值传递进来，命名为内部的变量名
	 * Set the body frame angular rates
	 * @param rate 3D vector of angular rates in body frame
	 */
	void setBodyRate(const matrix::Vector3f &rate) { _body_rate = rate; }

	/** 设置系统质心位置，把外部的值传递进来，命名为内部的变量名
	 * Set the position of the system center of mass in body frame
	 * @param com 3D vector of CoM position in body frame
	 */
	void setSystemCoM(const matrix::Vector3f &com) { _sys_com_pos = com; }

	/** 设置当前姿态，把外部的值传递进来，命名为内部的变量名
	 * Set the current attitude
	 * @param att current attitude quaternion
	 */
	void setAttitude(const matrix::Quatf &att) { _attitude = att; }

	void setDynamicsFeedforwardEnabled(bool enabled) { _dyn_ff_enabled = enabled; }

//=================================---- 限制接口 ----=============================================//
	/** 设置速度限制，把外部的值传递进来，命名为内部的变量名
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit 水平速度限制
	 * @param vel_up upwards velocity limit 上升速度限制
	 * @param vel_down downwards velocity limit 下降速度限制
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	// 【新增】设置位置环积分限幅
    	void setPositionIntegralLimit(const float limit) { _lim_pos_int = limit; }

	/** 设置垂直推力限制，把外部的值传递进来，命名为内部的变量名
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/** 设置水平推力限制，把外部的值传递进来，命名为内部的变量名。存在优先级，因为总推力是有限的，水平推力要让位于垂直推力
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * @param margin of normalized thrust that is kept for horizontal control
	 * e.g. 0.3【0,1】意思是水平推力最多只占据总推力的30%，剩下的70%留给垂直推力使用。
	 */
	void setHorizontalThrustMargin(const float margin);

	/** 设置最大倾斜角度（弧度）限制，把外部的值传递进来，命名为内部的变量名。
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

//=================================---- 物理参数接口 ----=============================================//
	/** 设置悬停推力的标定值（悬停时需要的推力大小映射），把外部的值传递进来，命名为内部的变量名。这个标定值的范围是0.1到0.9。
	 * Set the normalized hover thrust
	 * @param thrust [0.1, 0.9] with which the vehicle hovers not acelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, 0.1f, 0.9f); }

	/** 悬停推力更新的平滑过渡。在更新悬停推力参数时，防止无人机的推力输出发生突变，从而避免高度跳变。
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

//=================================---- 状态输入接口 ----=============================================//
	/** 设置当前飞行器状态，把外部的值传递进来，命名为内部的变量名
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure  当前位置、速度、加速度、偏航角
	 */
	void setState(const PositionControlStates &states);

	/** 设置目标飞行器状态，把外部的值传递进来，命名为内部的变量名
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 */
	void setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint);

//=================================---- 控制计算 ----=============================================//
	/** 主控制循环，计算输出。返回：true表示更新成功，输出设定点可执行；false表示更新失败，输入无效
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral() {
	        _vel_int.setZero();
	        _pos_int.setZero();
	        _eso_input.setZero();
	        _acc_cmd_prev.setZero();
	_eso.reset();
		// 让下一次 ESO 启用时走“测量初始化”，避免 reset() 后从 0 开始导致尖峰
		_eso_update_enabled_prev = false;
	   	}

	/** 获取控制器的输出位置/速度/加速度指令，把内部的值传递到外部的结构体变量中
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/** 获取从加速度转换的期望姿态（四元数）和推力指令，把内部的值传递到外部的结构体变量中
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

private:
	bool _inputValid();//确定目标值和当前状态有效，不是NAN

	void _positionControl(const float dt); ///< Position proportional control
	void _velocityControl(const float dt); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_pos_i; ///< 【新增】Position control integral gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain

	// Limits
	float _lim_pos_int{};     ///< 【新增】Position integral limit
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [0.1, 0.9] with which the vehicle hovers not accelerating down or up with level orientation

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */
	matrix::Vector3f _pos_int;    ///< 【新增】integral term of the position controller
	matrix::Vector3f _body_rate{};   // 机体角速度
   	matrix::Vector3f _sys_com_pos{}; // 质心偏移
    	matrix::Quatf    _attitude{1.f, 0.f, 0.f, 0.f};    // 当前姿态四元数
	bool _dyn_ff_enabled{false}; ///< Enable model-based dynamic feed-forward terms.

	// 外部类
	// Extended State Observer
	PositionESO _eso;
	matrix::Vector3f _eso_disturbance{}; ///< 从ESO估计的扰动
	bool _eso_update_enabled{true}; ///< 是否更新ESO（由上层根据起飞状态控制）
	bool _eso_update_enabled_prev{true}; ///< 上一周期是否更新ESO（用于检测“启用边沿”）
	matrix::Vector3f _acc_cmd_prev{}; ///< 上一时刻的加速度命令
	matrix::Vector3f _eso_input{};

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */
};
