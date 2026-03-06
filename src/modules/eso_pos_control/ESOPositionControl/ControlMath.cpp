/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.cpp
 */

#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace ControlMath
{
void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// 1. 方向取反：外部输入为世界坐标的力矩，内部计算时采用的机体系的力矩。
	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
	// 2. 模长取负：PX4 定义 thrust_body[2] 为负值时表示产生向上推力。得到的是机体系的 Z 轴方向。
	att_sp.thrust_body[2] = -thr_sp.length();// 计算推力的模长。length() 只返回标量长度，没有方向。
}

/*
	输入 body_unit: 期望的方向
		↓
	计算与 world_unit 的夹角
		↓
	如果夹角 > max_angle?
	├─ 是 → 调整 body_unit 到刚好 max_angle 处
	└─ 否 → body_unit 保持不变
		↓
	输出 body_unit: 经过限制的方向
*/
void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
	// determine tilt
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);
	// limit tilt
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	// corner case exactly parallel vectors
	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}
/*
		输入: body_z (推力方向), yaw_sp (偏航角)
			↓
		┌──────────────────────────┐
		│ 步骤1: 处理零向量        │
		│ 如果 body_z 是(0,0,0)    │
		│ → 设为 (0,0,1) [向上]   │
		└──────────────────────────┘
			↓
		┌──────────────────────────┐
		│ 步骤2: 正规化 body_z      │
		│ 变成长度为1的方向向量    │
		└──────────────────────────┘
			↓
		┌──────────────────────────┐
		│ 步骤3: 计算 body_x       │
		│ body_x = yaw方向 × body_z│
		│ (叉积)                   │
		└──────────────────────────┘
			↓
		┌──────────────────────────┐
		│ 步骤4: 计算 body_y       │
		│ body_y = body_z × body_x │
		│ (叉积)                   │
		└──────────────────────────┘
			↓
		┌──────────────────────────┐
		│ 步骤5: 构造旋转矩阵      │
		│ 3个轴向量组成一个矩阵   │
		└──────────────────────────┘
			↓
		┌──────────────────────────┐
		│ 步骤6: 转为四元数        │
		│ 填入 att_sp.q_d         │
		└──────────────────────────┘
			↓
		输出: att_sp (完整姿态四元数 + 欧拉角)
*/
void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// zero vector, no direction, set safe level value
	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	body_z.normalize();

	// vector of desired yaw direction in XY plane, rotated by PI/2
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

	// desired body_x axis, orthogonal to body_z
	Vector3f body_x = y_C % body_z;

	// keep nose to front while inverted upside down
	if (body_z(2) < 0.0f) {
		body_x = -body_x;
	}

	if (fabsf(body_z(2)) < 0.000001f) {
		// desired thrust is in XY plane, set X downside to construct correct matrix,
		// but yaw component will not be used actually
		body_x.zero();
		body_x(2) = 1.0f;
	}

	body_x.normalize();

	// desired body_y axis
	const Vector3f body_y = body_z % body_x;

	Dcmf R_sp;

	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// calculate euler angles, for logging only, must not be used for control
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();
}

/*
步骤：
	输入：v0 = [3, 0]  v1 = [2, 0]  max = 4
		↓
	步骤1：尝试直接相加 v0 + v1 = [5, 0]，大小 = 5
	步骤2：检查 5 <= 4？ NO → 需要缩放
		↓
	步骤3：用二次方程求解最大的v1缩放系数，使得 ||v0 + s·v1|| = 4
	步骤4：返回 [4, 0]（经过缩放的合成向量）
实际应用：
   位置控制系统中：
	v0 = 期望的水平加速度指令    [例如：向东5m/s²]
	v1 = 风力补偿向量            [例如：向西2m/s²]
	max = 多轴飞行器的最大倾角对应的加速度 [例如：4m/s²]
	输出 = 合成后的总加速度指令，保证不超过物理限制 [返回：向东4m/s²]
*/
Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	if (Vector2f(v0 + v1).norm() <= max) {
		// vector does not exceed maximum magnitude
		return v0 + v1;

	} else if (v0.length() >= max) {
		// the magnitude along v0, which has priority, already exceeds maximum.
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		// the two vectors are equal
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		// the first vector is 0.
		return v1.normalized() * max;

	} else {
		// vf = final vector with ||vf|| <= max
		// s = scaling factor
		// u1 = unit of v1
		// vf = v0 + v1 = v0 + s * u1
		// constraint: ||vf|| <= max
		//
		// solve for s: ||vf|| = ||v0 + s * u1|| <= max
		//
		// Derivation:
		// For simplicity, replace v0 -> v, u1 -> u
		// 				   		   v0(0/1/2) -> v0/1/2
		// 				   		   u1(0/1/2) -> u0/1/2
		//
		// ||v + s * u||^2 = (v0+s*u0)^2+(v1+s*u1)^2+(v2+s*u2)^2 = max^2
		// v0^2+2*s*u0*v0+s^2*u0^2 + v1^2+2*s*u1*v1+s^2*u1^2 + v2^2+2*s*u2*v2+s^2*u2^2 = max^2
		// s^2*(u0^2+u1^2+u2^2) + s*2*(u0*v0+u1*v1+u2*v2) + (v0^2+v1^2+v2^2-max^2) = 0
		//
		// quadratic equation:
		// -> s^2*a + s*b + c = 0 with solution: s1/2 = (-b +- sqrt(b^2 - 4*a*c))/(2*a)
		//
		// b = 2 * u.dot(v)
		// a = 1 (because u is normalized)
		// c = (v0^2+v1^2+v2^2-max^2) = -max^2 + ||v||^2
		//
		// sqrt(b^2 - 4*a*c) =
		// 		sqrt(4*u.dot(v)^2 - 4*(||v||^2 - max^2)) = 2*sqrt(u.dot(v)^2 +- (||v||^2 -max^2))
		//
		// s1/2 = ( -2*u.dot(v) +- 2*sqrt(u.dot(v)^2 - (||v||^2 -max^2)) / 2
		//      =  -u.dot(v) +- sqrt(u.dot(v)^2 - (||v||^2 -max^2))
		// m = u.dot(v)
		// s = -m + sqrt(m^2 - c)
		//
		//
		//
		// notes:
		// 	- s (=scaling factor) needs to be positive
		// 	- (max - ||v||) always larger than zero, otherwise it never entered this if-statement
		Vector2f u1 = v1.normalized();
		float m = u1.dot(v0);
		float c = v0.dot(v0) - max * max;
		float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

bool cross_sphere_line(const Vector3f &sphere_c, const float sphere_r,
		       const Vector3f &line_a, const Vector3f &line_b, Vector3f &res)
{
	// project center of sphere on line  normalized AB
	Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		// we have triangle CDX with known CD and CX = R, find DX
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			// target waypoint is already behind us
			res = line_b;

		} else {
			// target is in front of us
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {

		// have no roots, return D
		res = d; // go directly to line

		// previous waypoint is still in front of us
		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}

		// target waypoint is already behind us
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}

		return false;
	}
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {// 如果两个值都不是 NAN，则相加
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {// 如果 setpoint 是 NAN，则直接取 addition
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do 如果 addition 是 NAN 或两个都是 NAN ，则保持 setpoint 不变
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	addIfNotNanVector3f(vector, Vector3f()); // Vector3f() 表示零向量
}

} // ControlMath
