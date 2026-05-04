/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file eso_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * 横滚 P 增益
 *
 * 横滚比例增益：当姿态横滚误差为 1 rad 时，控制器期望输出的角速度指令约为多少 rad/s。
 * 数值越大，Roll 响应越快、刚度越高，但过大可能导致振荡。
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_ROLL_P, 1.30f);

/**
 * 俯仰 P 增益
 *
 * 俯仰比例增益：当姿态俯仰误差为 1 rad 时，控制器期望输出的角速度指令约为多少 rad/s。
 * 数值越大，Pitch 响应越快、刚度越高，但过大可能导致振荡。
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_PITCH_P, 1.30f);

/**
 * 偏航 P 增益
 *
 * 偏航比例增益：当姿态偏航误差为 1 rad 时，控制器期望输出的角速度指令约为多少 rad/s。
 * 一般偏航控制余度比 Roll/Pitch 小，因此该增益通常设得更小以避免偏航震荡。
 *
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_YAW_P, 0.8f);

/**
 * 偏航权重
 *
 * 用于在非线性/几何姿态控制中降低偏航相对 Roll/Pitch 的优先级，取值范围 [0,1]。
 * 多旋翼的偏航力矩余度通常较小，而且偏航对悬停稳定性和位置导航不如 Roll/Pitch 关键，
 * 因此会用该权重让控制器在耦合大角度机动时优先修正 Roll/Pitch。
 *
 * 偏航“刚度/速度”主要由 ESO_YAW_P 调整；该权重只改变优先级，不改变偏航增益的物理大小
 * （控制律内部会做等效缩放以保持力矩幅值一致）。
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_YAW_WEIGHT, 0.35f);

/**
 * 最大横滚角速度
 *
 * 手动与自动模式（非 Acro）下的 Roll 角速度上限。
 * 在自主大角度转动时限制输出，避免控制量过大导致混控饱和。
 * 上限受机体能力与陀螺仪最大测量范围共同限制。
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_ROLLRATE_MAX, 220.0f);

/**
 * 最大俯仰角速度
 *
 * 手动与自动模式（非 Acro）下的 Pitch 角速度上限。
 * 在自主大角度转动时限制输出，避免控制量过大导致混控饱和。
 * 上限受机体能力与陀螺仪最大测量范围共同限制。
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_PITCHRT_MAX, 220.0f);

/**
 * 最大偏航角速度
 *
 * 手动与自动模式（非 Acro）下的 Yaw 角速度上限。
 * 用于限制偏航指令，避免偏航饱和或过快旋转。
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_YAWRATE_MAX, 200.0f);

/**
 * 手动倾斜输入滤波时间常数
 *
 * 对手动姿态（倾斜角）输入做一阶低通滤波的时间常数。
 * 设为 0 表示关闭滤波；数值越大，滤波越强、手感更柔和但响应更慢。
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(ESO_MAN_TILT_TAU, 0.0f);

/**
 * Active arm dynamics model profile
 *
 * 0: uav_arm_v4 legacy profile
 * 1: uam_v5 runtime profile
 *
 * @min 0
 * @max 1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(ESO_ARM_MODEL, 1);

/**
 * Total mass for tau_s compensation
 *
 * Total mass (m_B + m_M) used in (31) to compute gravity torque.
 *
 * @unit kg
 * @min 0.1
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_MASS_TOTAL, 1.289f);

/**
 * Center of mass offset X (body frame)
 *
 * p_C^B x component used in (31) for gravity torque compensation.
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_COM_X, -0.01f);

/**
 * Center of mass offset Y (body frame)
 *
 * p_C^B y component used in (31) for gravity torque compensation.
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_COM_Y, 0.0f);

/**
 * Center of mass offset Z (body frame)
 *
 * p_C^B z component used in (31) for gravity torque compensation.
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_COM_Z, 0.161f);

/**
 * Arm inertia XX (body frame)
 *
 * Diagonal inertia of manipulator/arm M_M^B used in (31).
 *
 * @unit kg m^2
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.0001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_ARM_IXX, 0.016f);

/**
 * Arm inertia YY (body frame)
 *
 * @unit kg m^2
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.0001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_ARM_IYY, 0.016f);

/**
 * Arm inertia ZZ (body frame)
 *
 * @unit kg m^2
 * @min 0.0
 * @max 1.0
 * @decimal 6
 * @increment 0.0001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(ESO_ARM_IZZ, 0.016f);
