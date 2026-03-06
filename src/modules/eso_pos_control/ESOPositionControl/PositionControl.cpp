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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <geo/geo.h>

using namespace matrix;
using namespace time_literals;

void ESOPositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void ESOPositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void ESOPositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void ESOPositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void ESOPositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	if (hover_thrust_new > FLT_EPSILON) {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		setHoverThrust(hover_thrust_new);
	}
}

void ESOPositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void ESOPositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool ESOPositionControl::update(const float dt)
{
	bool valid = _inputValid();
	static hrt_abstime last_warn_update{0};

			if (valid) {

				// -------------------------------------------------------------------------
				// [ESO 启停逻辑] 起飞前可以禁用 ESO 更新，并在启用瞬间用测量初始化 z1/z2
				//
				// 背景：如果 ESO 在起飞后才切入，而内部状态仍从 0 开始，会导致 err=pos_meas-z1 很大，
				// 观测器会在很短时间内把 z2/z3 推到很大（等价于“导数尖峰/扰动尖峰”），从而引起推力/倾角突变。
				// 解决：启用边沿时做一次 reset(pos, vel)，并且在禁用期间不更新 ESO（d_hat 置 0）。
				// -------------------------------------------------------------------------
				if (_eso_update_enabled && !_eso_update_enabled_prev) {
					_eso.reset(_pos, _vel);
				}

				if (_eso_update_enabled) {
					_eso_disturbance = _eso.update(_pos, _eso_input, dt);

				} else {
					_eso_disturbance.setZero();
				}
				_positionControl(dt);
				_velocityControl(dt);

				_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
				_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
				/*// ==================== 【开始：新增 Yaw 打印代码】 ====================
					static hrt_abstime last_yaw_debug_time1 = 0;
					// 限制打印频率为 5Hz (每1000ms一次)，防止刷屏卡顿
					if (hrt_elapsed_time(&last_yaw_debug_time1) > 1000_ms) {
						last_yaw_debug_time1 = hrt_absolute_time();// 更新上次打印时间

						// 打印原始弧度和转换后的角度
						// _setpoint.yaw 是弧度制
						PX4_INFO(">>> Controller after clear现在的yaw值 Setpoint Yaw: %.2f",
							(double)math::degrees(_yaw_sp));
					}
				// ==================== 【结束：新增 Yaw 打印代码】 ====================*/
			// -------------------------------------------------------------------------
			// [调试打印-ESO 输出] 用于排查“ESO 估计是否在发散/符号是否反了”
			//
			// 说明：
			// - d_hat (z3)      : ESO 估计的总扰动（包含未建模动力学/外扰/参数误差等），数值异常大通常意味着观测器带宽过高或输入建模不一致
			// - u_eso_input     : 给 ESO 的名义输入（这里使用上一周期“可实现加速度”+离心补偿），用于让 ESO 感知饱和后的真实输入
			// - z1/z2           : ESO 内部估计的位置/速度，可对照 EKF 的 pos/vel 判断是否跟得上
			//
			// 频率：5Hz（200ms），避免刷屏影响实时性。
			// -------------------------------------------------------------------------
			static hrt_abstime last_eso_dbg_print{0};
				if (hrt_elapsed_time(&last_eso_dbg_print) > 200_ms) {
				// 	last_eso_dbg_print = hrt_absolute_time();

				// const Vector3f z1 = _eso.getEstimatedPosition();
				// const Vector3f z2 = _eso.getEstimatedVelocity();
				// const Vector3f z3 = _eso.getEstimatedDisturbance();

				// PX4_INFO("ESO est: pos[%.2f %.2f %.2f] z1[%.2f %.2f %.2f] z2[%.2f %.2f %.2f] d_hat[%.2f %.2f %.2f] u_in[%.2f %.2f %.2f]",
				// 	 (double)_pos(0), (double)_pos(1), (double)_pos(2),
				// 	 (double)z1(0), (double)z1(1), (double)z1(2),
				// 	 (double)z2(0), (double)z2(1), (double)z2(2),
				// 	 (double)z3(0), (double)z3(1), (double)z3(2),
				// 	 (double)_eso_input(0), (double)_eso_input(1), (double)_eso_input(2));
				// PX4_WARN("state thr_sp_2:[%.2f] yaw_sp: [%.2f]",
			 	// 	(double)_thr_sp(2), (double)_yaw_sp);
				}
			}
			// 记录“上一周期是否更新ESO”，用于检测启用边沿（即使当前输入无效也需要更新该标记）
			_eso_update_enabled_prev = _eso_update_enabled;

		// There has to be a valid output accleration and thrust setpoint otherwise something went wrong
		valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));

	if (!valid && (hrt_elapsed_time(&last_warn_update) > 1_s)) {
		last_warn_update = hrt_absolute_time();
		// PX4_WARN("update() invalid output. pos_sp:[%.2f %.2f %.2f] vel_sp:[%.2f %.2f %.2f] acc_sp:[%.2f %.2f %.2f]",
		// 	 (double)_pos_sp(0), (double)_pos_sp(1), (double)_pos_sp(2),
		// 	 (double)_vel_sp(0), (double)_vel_sp(1), (double)_vel_sp(2),
		// 	 (double)_acc_sp(0), (double)_acc_sp(1), (double)_acc_sp(2));
		// PX4_WARN("state pos:[%.2f %.2f %.2f] vel:[%.2f %.2f %.2f] vel_dot:[%.2f %.2f %.2f] thr_sp:[%.2f %.2f %.2f]",
		// 	 (double)_pos(0), (double)_pos(1), (double)_pos(2),
		// 	 (double)_vel(0), (double)_vel(1), (double)_vel(2),
		// 	 (double)_vel_dot(0), (double)_vel_dot(1), (double)_vel_dot(2),
		// 	 (double)_thr_sp(0), (double)_thr_sp(1), (double)_thr_sp(2));

	}

	return valid;
}

void ESOPositionControl::_positionControl(const float dt)
{
    // 1. 计算误差
    Vector3f pos_err = _pos_sp - _pos;

    // 2. PID 各项计算
    Vector3f vel_sp_p = pos_err.emult(_gain_pos_p);

    // 3. 构造 "未限幅的总期望" (Unconstrained)
    // 包括：P项 + I项 + 前馈(_vel_sp)
    Vector3f vel_sp_unc = vel_sp_p + _pos_int.emult(_gain_pos_i);
    ControlMath::addIfNotNanVector3f(vel_sp_unc, _vel_sp); // 这里的 _vel_sp 是纯前馈

    // 4. 执行限幅 (Constraints)
    // 参数1 (vel_sp_unc): 我们希望达到的总速度
    // 参数2 (vel_sp_unc - vel_sp_p - I): 这其实就是提取出的"前馈分量"。
    // 逻辑：限幅器会优先保留 vel_sp_p + I (稳控)，挤占前馈空间。

    Vector3f vel_sp_limited = vel_sp_unc;
    ControlMath::setZeroIfNanVector3f(vel_sp_limited);

    // 水平限幅
    vel_sp_limited.xy() = ControlMath::constrainXY(vel_sp_limited.xy(),
                                                  (vel_sp_limited - vel_sp_p - _pos_int.emult(_gain_pos_i)).xy(),
                                                  _lim_vel_horizontal);
    // 垂直限幅
    vel_sp_limited(2) = math::constrain(vel_sp_limited(2), -_lim_vel_up, _lim_vel_down);

    // =======================================================
    // 5. 抗饱和 (Tracking Anti-Windup)
    // =======================================================

    // A. 算出被切了多少 (Saturation Delta)
    Vector3f saturation_diff = vel_sp_unc - vel_sp_limited;

    // B. 计算增益 (2/Kp)
    Vector3f arw_gain;
    arw_gain(0) = (_gain_pos_p(0) > FLT_EPSILON) ? (2.0f / _gain_pos_p(0)) : 0.0f;
    arw_gain(1) = (_gain_pos_p(1) > FLT_EPSILON) ? (2.0f / _gain_pos_p(1)) : 0.0f;
    arw_gain(2) = (_gain_pos_p(2) > FLT_EPSILON) ? (2.0f / _gain_pos_p(2)) : 0.0f;

    // C. 修正误差
    Vector3f pos_err_for_int = pos_err - saturation_diff.emult(arw_gain);

    // 6. 积分更新
    if (PX4_ISFINITE(pos_err(0)) && PX4_ISFINITE(pos_err(1)) && PX4_ISFINITE(pos_err(2))) {
        _pos_int += pos_err_for_int * dt;

        // 积分硬限幅
        _pos_int(0) = math::constrain(_pos_int(0), -_lim_pos_int, _lim_pos_int);
        _pos_int(1) = math::constrain(_pos_int(1), -_lim_pos_int, _lim_pos_int);
        _pos_int(2) = math::constrain(_pos_int(2), -_lim_pos_int, _lim_pos_int);
    }

    // 7. 输出赋值
    _vel_sp = vel_sp_limited;
}

void ESOPositionControl::_velocityControl(const float dt)
{
	// --- 1. 计算速度误差 (Velocity Error) ---
	// PX4 习惯: error = setpoint - current 这里从论文的current-setpoint改为了PX4格式，后续计算时符号也做出了相应调整
	Vector3f vel_error = _vel_sp - _vel;

	// --- 2. 计算离心力补偿 (Centrifugal Force Compensation) ---
	// 公式: a_f = - R * (w x (w x p_com))
	Vector3f w_cross_p = _body_rate.cross(_sys_com_pos);
	Vector3f centrifugal_body = _body_rate.cross(w_cross_p);

	// 将机体坐标系下的离心力转到 NED 系
	// 您的代码逻辑是：a_f = R * centrifugal * (-1.0)
	Vector3f a_f_ned = _attitude.rotateVector(centrifugal_body) * (-1.0f);

	// --- 3. 计算期望加速度 (Backstepping Control Law) ---
	// 原公式: u = acc_ref - Kv * vel_err - dist - a_f - g

	// 3.1 清洗前馈中的 NaN (acc_ref)
	ControlMath::setZeroIfNanVector3f(_acc_sp);

	// 3.2 计算控制量
	// 【修改】这里使用 减号 (-)，配合上面定义的 (Current - Reference) 误差
	// _acc_sp (前馈)
	// - Kv * vel_error (P项)
	// - _eso_disturbance (ESO干扰)
	// - a_f_ned (离心力)
	// (注：重力 -g 由后续的 _accelerationControl 隐式处理，这里算出的是运动学加速度)
	Vector3f acc_cmd = _acc_sp
			+ vel_error.emult(_gain_vel_p)
			- _eso_disturbance
			- a_f_ned;
	// Vector3f acc_cmd = _acc_sp
	// 		+ vel_error.emult(_gain_vel_p);

	// 更新加速度设定点，供后续转换为推力
	_acc_sp = acc_cmd;

	// --- 4. 加速度 -> 推力 & 姿态限制 ---
	// PX4 原生函数：负责处理重力补偿、倾角限制、推力映射
	_accelerationControl();

	// 5. 垂直方向的推力限幅
	/*
		如果推力已经最小了（怠速），且你还想往下掉（vel_error > 0，NED系下误差正意味着目标在下面），那就别积分了。
		如果推力已经最大了（满油门），且你还想往上飞，也别积分了。
	*/
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// 6. 推力优先级逻辑（最复杂的几何限制）:保高度 > 保水平位置。如果推力不够，先保证不掉下来，哪怕偏离航线。
	// Prioritize vertical control while keeping a horizontal margin

	// 函数拷贝：如果 _thr_sp 是 Vector3f，构造函数会取其 x,y 分量来得到一个仅含 XY 的向量；如果本身就是 Vector2f，则是一次拷贝构造。
	const Vector2f thrust_sp_xy(_thr_sp);
	// 算出水平推力的大小 和 最大推力的平方
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// A. 给水平推力预留一点点“生存空间” (_lim_thr_xy_margin)，不能全给垂直
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// B. 限制垂直推力：垂直推力最大不能超过 (总推力 - 预留水平推力)
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// C. 既然垂直推力确定了，剩下的全给水平推力
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// 如果水平推力超限，就按比例缩放（保持方向不变，只缩短长度）
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// 高级水平抗饱和 ---> PX4的抗饱和方法,针对积分项，这里不需要，因为会影响ESO的判断
	// --- 7. 反算实际值 ---
	if (_hover_thrust > FLT_EPSILON) {
		// 这里计算出的是实际上飞机能执行的加速度
		_acc_cmd_prev = _thr_sp * (CONSTANTS_ONE_G / _hover_thrust);
	} else {
		_acc_cmd_prev = _thr_sp * CONSTANTS_ONE_G;
	}

		// 注意：_acc_cmd_prev 是由推力反算得到的“可实现推力加速度”(仅推力项，不含重力)。
		// 但 ESO 的被控对象是位置二阶系统 p̈（惯性系下的“净加速度”），其中天然包含重力项 g。
		// 如果这里不把 +g 加回去，ESO 会把重力当成“扰动 d_hat”去估计，导致：
		// - z 轴 d_hat 接近 +9.81（量级很大）
		// - 速度环再去减 d_hat 时相当于重复做重力补偿，最终推力容易被顶到饱和（thr_body.z≈-1）
		//
		// 因此：给 ESO 的名义输入应当是“净加速度” ≈ (推力加速度 + 重力) + 离心补偿项。
		_eso_input = _acc_cmd_prev + Vector3f(0.f, 0.f, CONSTANTS_ONE_G) + a_f_ned;

	// 只要你的 ESO 下一帧使用的是这个 _acc_cmd_prev 作为输入 (b*u)，
	// 那么 ESO 就会自动具备抗饱和能力，因为它知道实际输入的 u 变小了。
}

void ESOPositionControl::_accelerationControl()
{
	// 1. 构建目标机体Z轴 (body_z)
	// 期望加速度 = (ax, ay, az)。我们希望机体产生的推力去抵消重力并产生这个加速度。
	// 所以推力方向应该是 (-ax, -ay, g)。注意 Z 轴是 g，因为我们要抵抗重力。
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();

	// 2. 限制最大倾角 (Tilt Limit)
	// 如果算出来的 body_z 太倾斜（比如 60度），就强行把它掰回到最大倾角（比如 45度）。
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

	// 3. 计算推力大小 (Collective Thrust)
	// 原始公式：推力 = (垂直加速度期望 - 重力) * 悬停油门系数
	// 注意：PX4 的推力是归一化的 [0, 1]，加速度是 [m/s^2]。
	// _hover_thrust / CONSTANTS_ONE_G 是一个转换系数：把加速度映射到油门。
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

	// 4. 投影修正 (Project Thrust)
	// 因为我们刚才 limitTilt 改变了 body_z 的方向，导致原本垂直方向的推力分量变小了。
	// 为了保证垂直方向依然有足够的力（不掉高），需要把总推力放大一点。
	// divide by cos(tilt_angle)
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));

	// 5. 再次限制推力最小值 (防止停转)
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);

	// 6. 最终输出
	_thr_sp = body_z * collective_thrust;
}

bool ESOPositionControl::_inputValid()
{
	bool valid = true;
	static hrt_abstime last_debug_print{0};

	// Every axis x, y, z needs to have some setpoint 每个轴必须至少有一个控制目标
	/*
	逻辑：如果你想控制 Z 轴，你必须告诉我 Z 轴的目标位置、目标速度或目标加速度中的至少一个。
	     如果三个都是 NaN（无效值），控制器就不知道该干嘛。
	*/
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs 水平轴（X/Y）必须成对出现
	/*
		不能只控制 X 轴的位置而不控制 Y 轴的位置。如果 X 轴有位置设定点，Y 轴也必须有。
		速度和加速度同理。这是因为多旋翼的水平运动是耦合的。
	*/
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid 确定当前状态有效
	/*
		如果给定了位置目标，就得知道当前位置才能控制位置。
		如果给定了速度目标，就得知道当前速度才能控制速度。
		如果给定了加速度目标，就得知道当前加速度才能控制加速度
	*/
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	if (!valid && (hrt_elapsed_time(&last_debug_print) > 1_s)) {
		last_debug_print = hrt_absolute_time();

		// PX4_WARN("setpoint invalid. pos_sp:[%.2f %.2f %.2f] vel_sp:[%.2f %.2f %.2f] acc_sp:[%.2f %.2f %.2f]",
		// 	 (double)_pos_sp(0), (double)_pos_sp(1), (double)_pos_sp(2),
		// 	 (double)_vel_sp(0), (double)_vel_sp(1), (double)_vel_sp(2),
		// 	 (double)_acc_sp(0), (double)_acc_sp(1), (double)_acc_sp(2));

		// PX4_WARN("state check pos[%d %d %d] vel[%d %d %d] vel_dot[%d %d %d]",
		// 	 PX4_ISFINITE(_pos(0)), PX4_ISFINITE(_pos(1)), PX4_ISFINITE(_pos(2)),
		// 	 PX4_ISFINITE(_vel(0)), PX4_ISFINITE(_vel(1)), PX4_ISFINITE(_vel(2)),
		// 	 PX4_ISFINITE(_vel_dot(0)), PX4_ISFINITE(_vel_dot(1)), PX4_ISFINITE(_vel_dot(2)));
	}

	return valid;
}

void ESOPositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void ESOPositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
