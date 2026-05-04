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

/**
 * @file eso_rate_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * 横滚角速度 P 增益
 *
 * 横滚速率环比例增益：角速度误差为 1 rad/s 时的控制输出大小。
 * 数值越大，Roll 速率跟踪越硬、响应越快，但过大可能引起高频振荡。
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ROLLRATE_P, 0.14f);

/**
 * 横滚角速度 I 增益
 *
 * 横滚速率环积分增益：用于消除稳态误差，补偿静态推力不平衡或重心偏置等慢变扰动。
 * 数值过大可能导致低频摆动或积分累积过快。
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ROLLRATE_I, 0.12f);

/**
 * 横滚角速度积分限幅
 *
 * 横滚速率积分项的绝对值上限。增大可提高抗恒定扰动能力，
 * 减小可降低大动作后积分残留并改善回稳时间。
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_RR_INT_LIM, 0.18f);

/**
 * Roll rate D gain (预留)
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward (预留)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ROLLRATE_FF, 0.0f);

/**
 * 横滚速率控制器总增益
 *
 * 控制器全局缩放系数，会同时缩放 P/I/D：
 * output = ESO_ROLLRATE_K * (ESO_ROLLRATE_P * error
 * 			     + ESO_ROLLRATE_I * error_integral
 * 			     + ESO_ROLLRATE_D * error_derivative)
 *
 * 设 ESO_ROLLRATE_P=1 可得到理想形式 PID；设 ESO_ROLLRATE_K=1 可得到并联形式 PID。
 * 当前 ESO 速率律主要对 P 有效，I/D 为预留。
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ROLLRATE_K, 1.0f);

/**
 * 俯仰角速度 P 增益
 *
 * 俯仰速率环比例增益：角速度误差为 1 rad/s 时的控制输出大小。
 * 数值越大，Pitch 速率跟踪越硬、响应越快，但过大可能引起高频振荡。
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_PITCHRATE_P, 0.14f);

/**
 * 俯仰角速度 I 增益
 *
 * 俯仰速率环积分增益：用于消除稳态误差，补偿静态推力不平衡或重心偏置等慢变扰动。
 * 数值过大可能导致低频摆动或积分累积过快。
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_PITCHRATE_I, 0.12f);

/**
 * 俯仰角速度积分限幅
 *
 * 俯仰速率积分项的绝对值上限。增大可提高抗恒定扰动能力，
 * 减小可降低大动作后积分残留并改善回稳时间。
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_PR_INT_LIM, 0.18f);

/**
 * Pitch rate D gain (预留)
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward (预留)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_PITCHRATE_FF, 0.0f);

/**
 * 俯仰速率控制器总增益
 *
 * 控制器全局缩放系数，会同时缩放 P/I/D：
 * output = ESO_PITCHRATE_K * (ESO_PITCHRATE_P * error
 * 			     + ESO_PITCHRATE_I * error_integral
 * 			     + ESO_PITCHRATE_D * error_derivative)
 *
 * 设 ESO_PITCHRATE_P=1 可得到理想形式 PID；设 ESO_PITCHRATE_K=1 可得到并联形式 PID。
 * 当前 ESO 速率律主要对 P 有效，I/D 为预留。
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_PITCHRATE_K, 1.0f);

/**
 * 偏航角速度 P 增益
 *
 * 偏航速率环比例增益：角速度误差为 1 rad/s 时的控制输出大小。
 * 偏航力矩余度通常更小，P 过大更容易引起偏航振荡/发抖。
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_YAWRATE_P, 0.16f);

/**
 * 偏航角速度 I 增益
 *
 * 偏航速率环积分增益：用于消除稳态误差，补偿静态推力不平衡或重心偏置等慢变扰动。
 * 数值过大可能导致低频摆动或积分累积过快。
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_YAWRATE_I, 0.05f);

/**
 * 偏航角速度积分限幅
 *
 * 偏航速率积分项的绝对值上限。增大可提高抗恒定扰动能力，
 * 减小可降低大动作后积分残留并改善回稳时间。
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_YR_INT_LIM, 0.12f);

/**
 * Yaw rate D gain (预留)
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward (预留)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_YAWRATE_FF, 0.0f);

/**
 * 偏航速率控制器总增益
 *
 * 控制器全局缩放系数，会同时缩放 P/I/D：
 * output = ESO_YAWRATE_K * (ESO_YAWRATE_P * error
 * 			     + ESO_YAWRATE_I * error_integral
 * 			     + ESO_YAWRATE_D * error_derivative)
 *
 * 设 ESO_YAWRATE_P=1 可得到理想形式 PID；设 ESO_YAWRATE_K=1 可得到并联形式 PID。
 * 当前 ESO 速率律主要对 P 有效，I/D 为预留。
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_YAWRATE_K, 1.0f);

/**
 * 名义机体惯性 XX
 *
 * 机体系 X 轴（Roll）方向的名义转动惯量 Ixx，用作 ESO 速率控制中的惯性矩阵 M 对角元。
 * 建议根据机体/挂载实际惯性设置，模型越准，ESO 估计与力矩律效果越好。
 *
 * @unit kg m^2
 * @min 0.0001
 * @decimal 5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_INERTIA_XX, 0.045f);

/**
 * 名义机体惯性 YY
 *
 * 机体系 Y 轴（Pitch）方向的名义转动惯量 Iyy，用作惯性矩阵 M 对角元。
 *
 * @unit kg m^2
 * @min 0.0001
 * @decimal 5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_INERTIA_YY, 0.045f);

/**
 * 名义机体惯性 ZZ
 *
 * 机体系 Z 轴（Yaw）方向的名义转动惯量 Izz，用作惯性矩阵 M 对角元。
 *
 * @unit kg m^2
 * @min 0.0001
 * @decimal 5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_INERTIA_ZZ, 0.08f);

/**
 * ESO 带宽（Roll 轴）
 *
 * 扩张状态观测器带宽 wp_x。值越大，扰动估计越快，但对噪声更敏感。
 * 一般先从较小值开始，逐步增大到不引起噪声放大/抖振为止。
 *
 * @unit rad/s
 * @min 0.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_RATE_BW_R, 2.0f);

/**
 * ESO 带宽（Pitch 轴）
 *
 * 扩张状态观测器带宽 wp_y。值越大，扰动估计越快，但对噪声更敏感。
 *
 * @unit rad/s
 * @min 0.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_RATE_BW_P, 2.0f);

/**
 * ESO 带宽（Yaw 轴）
 *
 * 扩张状态观测器带宽 wp_z。值越大，扰动估计越快，但对噪声更敏感。
 * 偏航轴通常带宽可以比 Roll/Pitch 略小，以避免偏航噪声/震荡。
 *
 * @unit rad/s
 * @min 0.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_RATE_BW_Y, 0.8f);

/**
 * 姿态误差反馈系数 k_beta
 *
 * 论文公式(45)中 (-k_beta * beta_v) 的系数，用于将姿态几何误差直接反馈到力矩。
 * 数值越大，姿态误差被更快抑制，但过大可能与速率 P/ESO 带宽耦合引起抖振。
 *
 * @min 0.0
 * @decimal 3
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_K_BETA, 0.85f);

/**
 * Enable model-based dynamic feed-forward terms.
 *
 * 0 disables the position centrifugal feed-forward. tau_s torque injection is
 * controlled separately by ESO_TAUS_K so that it can be diagnosed/tuned without
 * also enabling the position feed-forward path.
 * The rate controller still consumes attitude error/reference derivative aux
 * terms because they are part of the nominal feedback structure.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(ESO_DYN_FF_EN, 0);

/**
 * 最大物理力矩（归一化用）
 *
 * 当输出力矩为 ESO_MAX_TORQUE [N*m] 时，对应混控输入为 1.0。
 * 用于将物理力矩 tau 归一化到 [-1, 1] 再送入混控。
 * 建议设为机体单轴可提供的近似最大力矩，过小会提前饱和，过大则输出偏软。
 * 当前使用单一标量对三轴同时归一化（与 eso_att_control1 一致）；若以后需要按轴单独限幅/归一化，可再拆分参数。
 *
 * @unit Nm
 * @min 0.1
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_MAX_TORQUE, 2.20f);

/**
 * 速率环积分输出缩放系数（小积分比例）
 *
 * 在 ESO 速率力矩律中，积分项按如下方式注入：
 * integral_term = rate_int * ESO_RATE_I_SC
 *
 * 该参数用于“开小积分”做残差收敛：
 * - 设小一点：积分更保守，不易引入低频摆动
 * - 设大一点：稳态误差收敛更快，但更容易与噪声/饱和耦合
 *
 * 注意：积分状态本身仍由 ESO_ROLLRATE_I / ESO_PITCHRATE_I / ESO_YAWRATE_I
 * 与 ESO_RR_INT_LIM / ESO_PR_INT_LIM / ESO_YR_INT_LIM 共同决定。
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_RATE_I_SC, 0.10f);

/**
 * tau_s 软启用系数
 *
 * 对姿态环下传的补偿力矩 tau_s 做比例缩放：
 * tau_s_used = constrain(tau_s, +/- ESO_TAUS_LIM) * ESO_TAUS_K
 *
 * 0 表示关闭 tau_s 注入，1 表示全量使用（受限幅约束）。
 * 推荐从 0.1 开始逐步上调，观察稳态误差与抖振情况。
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_K, 0.0f);

/**
 * tau_s roll-axis scale/sign
 *
 * Per-axis multiplier applied after ESO_TAUS_K and ESO_TAUS_LIM. Keep at 1.0
 * for legacy scalar behavior. Set to 0.0 to disable roll-only injection, or
 * negative to flip the roll-axis tau_s sign during offline-identified tests.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_K_R, 1.0f);

/**
 * tau_s pitch-axis scale/sign
 *
 * Per-axis multiplier applied after ESO_TAUS_K and ESO_TAUS_LIM. Keep at 1.0
 * for legacy scalar behavior. Set to 0.0 to disable pitch-only injection, or
 * negative to flip the pitch-axis tau_s sign during offline-identified tests.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_K_P, 1.0f);

/**
 * tau_s yaw-axis scale/sign
 *
 * Per-axis multiplier applied after ESO_TAUS_K and ESO_TAUS_LIM. Keep at 1.0
 * for legacy scalar behavior. Set to 0.0 to disable yaw-only injection, or
 * negative to flip the yaw-axis tau_s sign during offline-identified tests.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_K_Y, 1.0f);

/**
 * tau_s observer roll scale/sign
 *
 * Additional per-axis multiplier for the tau_s path that enters the ESO nominal
 * input. Keep at 1.0 for legacy coupled behavior. Tune separately from the
 * control path to make ESO estimate only the residual disturbance.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_OBS_R, 1.0f);

/**
 * tau_s observer pitch scale/sign
 *
 * Additional per-axis multiplier for the tau_s path that enters the ESO nominal
 * input. Keep at 1.0 for legacy coupled behavior. Pitch is the first axis to
 * tune because current offline evidence is strongest there.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_OBS_P, 1.0f);

/**
 * tau_s observer yaw scale/sign
 *
 * Additional per-axis multiplier for the tau_s path that enters the ESO nominal
 * input. Keep yaw disabled in experiments unless yaw torque reconstruction is
 * physically validated.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_OBS_Y, 1.0f);

/**
 * tau_s control roll scale/sign
 *
 * Additional per-axis multiplier for the tau_s path that is subtracted from the
 * final physical torque command. Keep at 1.0 for legacy coupled behavior.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_CTL_R, 1.0f);

/**
 * tau_s control pitch scale/sign
 *
 * Additional per-axis multiplier for the tau_s path that is subtracted from the
 * final physical torque command. This can be kept smaller than the observer
 * pitch scale so model compensation informs ESO without over-driving motors.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_CTL_P, 1.0f);

/**
 * tau_s control yaw scale/sign
 *
 * Additional per-axis multiplier for the tau_s path that is subtracted from the
 * final physical torque command. Keep yaw disabled in experiments unless yaw
 * torque reconstruction is physically validated.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_CTL_Y, 1.0f);

/**
 * tau_s 单轴绝对限幅
 *
 * 对每个轴的 tau_s 分量分别限幅，防止姿态环补偿瞬时过大导致速率环突变。
 * 单位为 N*m，按轴独立应用。
 *
 * @unit Nm
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_LIM, 0.30f);

/**
 * tau_s injection low-pass time constant.
 *
 * Filters the limited and scaled tau_s before it is injected into the rate
 * torque law and the ESO nominal input. This keeps model feed-forward usable
 * at small nonzero ESO_TAUS_K without coupling joint/attitude update steps
 * directly into motor torque.
 *
 * @unit s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_TAUS_TAU, 0.10f);

/**
 * Max acro roll rate
 *
 * default: 2 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ACRO_R_MAX, 720.0f);

/**
 * Max acro pitch rate
 *
 * default: 2 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ACRO_P_MAX, 720.0f);

/**
 * Max acro yaw rate
 *
 * default 1.5 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ACRO_Y_MAX, 540.0f);

/**
 * Acro mode Expo factor for Roll and Pitch.
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ACRO_EXPO, 0.69f);

/**
 * Acro mode Expo factor for Yaw.
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ACRO_EXPO_Y, 0.69f);

/**
 * Acro mode SuperExpo factor for Roll and Pitch.
 *
 * SuperExpo factor for refining the input curve shape tuned using ESO_ACRO_EXPO.
 *
 * 0 Pure Expo function
 * 0.7 resonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ACRO_SUPEXPO, 0.7f);

/**
 * Acro mode SuperExpo factor for Yaw.
 *
 * SuperExpo factor for refining the input curve shape tuned using ESO_ACRO_EXPO_Y.
 *
 * 0 Pure Expo function
 * 0.7 resonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(ESO_ACRO_SUPEX_Y, 0.7f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(ESO_BAT_SCALE_EN, 0);
