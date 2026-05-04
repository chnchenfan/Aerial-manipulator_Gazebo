/****************************************************************************
 *
 * Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * @file eso_pos_control_params.c
 * Multicopter position controller parameters.
 *
 * @author Anton Babushkin <anton@px4.io>
 */


/* =================================================================
 * 1. 推力 (Thrust) 与 悬停 (Hover) 相关配置
 * ================================================================= */
/**
 * Minimum collective thrust in auto thrust control
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_THR_MIN, 0.12f);

/**
 * Hover thrust
 *
 * @unit norm
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_THR_HOVER, 0.5f);

/**
 * Hover thrust source selector
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(ESO_USE_HTE, 1);

/**
 * Horizontal thrust margin
 *
 * @unit norm
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_THR_XY_MARG, 0.3f);

/**
 * Maximum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_THR_MAX, 1.0f);


/* =================================================================
 * 2. PID 参数
 * ================================================================= */
/**
 * 垂直位置误差的 P 增益 (位置环)
 */
PARAM_DEFINE_FLOAT(ESO_Z_P, 1.05f);

/**
 * 垂直位置误差的 I 增益 (位置环)
 */
PARAM_DEFINE_FLOAT(ESO_Z_I, 0.11f);

/**
 * 垂直速度误差的 P 增益 (速度环)
 *
 * @min 0.1
 * @max 15.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_Z_VEL_P_ACC, 1.05f);

/**
 * ESO Z轴带宽参数
 */
PARAM_DEFINE_FLOAT(ESO_Z_BW, 0.95f);

/* =================================================================
 * 3. 垂直速度限制 (Z Axis Constraints)
 * ================================================================= */

/**
 * Automatic ascent velocity
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_Z_V_AUTO_UP, 3.f);

/**
 * Maximum ascent velocity
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_Z_VEL_MAX_UP, 3.f);

/**
 * Automatic descent velocity
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_Z_V_AUTO_DN, 1.f);

/**
 * Maximum descent velocity
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_Z_VEL_MAX_DN, 1.f);

/* =================================================================
 * 4. 水平位置控制 (XY轴) - PID 参数
 * ================================================================= */

/**
 * 水平位置误差的 P 增益 (位置环)
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_XY_P, 1.10f);

/**
 * 水平位置误差的 i 增益 (位置环)
 */
PARAM_DEFINE_FLOAT(ESO_XY_I, 0.42f);

/**
 * 水平速度误差的 P 增益 (速度环)
 *
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_XY_VEL_P_ACC, 1.80f);

/**
 * ESO XY轴带宽
 */
PARAM_DEFINE_FLOAT(ESO_XY_BW, 1.20f);

/**
 * 位置 ESO 起飞门控（开关）
 *
 * 作用：
 * - 1：起飞前不更新位置ESO（只用常规位置/速度环），进入 flight 后才启用，并在启用瞬间用测量初始化 z1/z2，避免导数/扰动尖峰
 * - 0：始终更新位置ESO（旧行为，可能在怠速/爬坡阶段更容易出现 ESO 估计尖峰）
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(ESO_POS_ESOGATE, 1);

/* =================================================================
 * 5. 导航与轨迹参数 (Navigation & Trajectory)
 * ================================================================= */

/**
 * Default horizontal velocity in mission
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_XY_CRUISE, 5.0f);

/* =================================================================
 * 6. 速度与姿态限制 (Limits)
 * ================================================================= */

/**
 * Maximum horizontal velocity setpoint for manual controlled mode
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_VEL_MANUAL, 10.0f);

// 【新增】积分限幅参数
PARAM_DEFINE_FLOAT(ESO_POS_INT_LIM, 0.85f);

/**
 * Maximum horizontal velocity
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_XY_VEL_MAX, 2.60f);

/**
 * Maximum tilt angle in air
 *
 * @unit deg
 * @min 20.0
 * @max 89.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_TILTMAX_AIR, 45.0f);

/**
 * Maximum tilt during landing
 *
 * @unit deg
 * @min 10.0
 * @max 89.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_TILTMAX_LND, 12.0f);

/* =================================================================
 * 7. 起飞与降落逻辑 (Takeoff & Land)
 * ================================================================= */

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.6
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_LAND_SPEED, 0.7f);

/**
 * Takeoff climb rate
 *
 * @unit m/s
 * @min 1
 * @max 5
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_TKO_SPEED, 1.5f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_MAN_Y_MAX, 150.0f);

/**
 * Manual yaw rate input filter time constant
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_MAN_Y_TAU, 0.08f);

/**
 * Maximum horizontal acceleration for auto mode and for manual mode
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_ACC_HOR_MAX, 5.50f);

/**
 * Acceleration for auto and for manual
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_ACC_HOR, 4.50f);

/**
 * Maximum vertical acceleration in velocity controlled modes upward
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_ACC_UP_MAX, 4.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes down
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_ACC_DOWN_MAX, 3.0f);

/**
 * Maximum jerk limit
 *
 * @unit m/s^3
 * @min 0.5
 * @max 500.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_JERK_MAX, 6.0f);

/**
 * Jerk limit in auto mode
 *
 * @unit m/s^3
 * @min 1.0
 * @max 80.0
 * @increment 1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_JERK_AUTO, 6.0f);

/**
 * Altitude control mode.
 *
 * @min 0
 * @max 2
 * @value 0 Altitude following
 * @value 1 Terrain following
 * @value 2 Terrain hold
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(ESO_ALT_MODE, 0);

/**
 * Altitude for 2. step of slow landing (landing)
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_LAND_ALT2, 5.0f);

/**
 * Position control smooth takeoff ramp time constant
 *
 * @min 0
 * @max 5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_TKO_RAMP_T, 3.0f);

/**
 * Manual-Position control sub-mode
 *
 * @value 0 Simple position control
 * @value 3 Smooth position control (Jerk optimized)
 * @value 4 Acceleration based input
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(ESO_POS_MODE, 4);

/**
 * Enforced delay between arming and takeoff
 *
 * @min 0
 * @max 10
 * @unit s
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_SPOOLUP_TIME, 1.0f);

/**
 * Overall Horizonal Velocity Limit
 *
 * @min -20
 * @max 20
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_XY_VEL_ALL, -10.0f);

/**
 * Overall Vertical Velocity Limit
 *
 * @min -3
 * @max 8
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_Z_VEL_ALL, -3.0f);

/**
 * Low pass filter cut freq. for numerical velocity derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_VELD_LP, 5.0f);
