/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file ArmKinematics.hpp
 *
 * Complete forward kinematics and dynamics for uav_arm_v4 serial manipulator.
 * Parameters extracted from: G:\PX4_Firmware\Tools\sitl_gazebo\models\uav_arm_v4\uav_arm_v4.sdf
 *
 * === 机械臂结构 (4-DOF) ===
 * q[0]: Joint1 - 基座旋转 (Yaw around Z)
 * q[1]: Joint2 - 肩部俯仰 (Pitch around Y)
 * q[2]: Joint3 - 肘部俯仰 (Pitch around Y)
 * q[3]: Joint4 - 腕部俯仰 (Pitch around Y)
 *
 * === 惯量计算理论 ===
 * 完整平行轴定理: I_sys = I_base + sum( R_i * I_local_i * R_i^T + m_i * S(r_i)^T * S(r_i) )
 */

#pragma once

#include <eso_common/ESOModelProfile.hpp>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

using namespace matrix;

class ArmKinematics
{
public:
	ArmKinematics() = default;

	void setModelProfile(eso_common::ModelProfile profile)
	{
		_model_profile = profile;
	}

	void setParameters(float mass_total, const Vector3f &base_com_offset, const Matrix3f &base_inertia)
	{
		_mass_total_system = mass_total;
		_base_com_offset = base_com_offset;
		_base_inertia_val = base_inertia;
	}

	/**
	 * 计算叉乘矩阵 (Skew-symmetric matrix)
	 * S(v) * u = v × u
	 */
	static Matrix3f skew(const Vector3f &v)
	{
		Matrix3f S;
		S(0, 0) = 0.f;     S(0, 1) = -v(2);  S(0, 2) = v(1);
		S(1, 0) = v(2);    S(1, 1) = 0.f;    S(1, 2) = -v(0);
		S(2, 0) = -v(1);   S(2, 1) = v(0);   S(2, 2) = 0.f;
		return S;
	}

	/**
	 * 计算绕 Z 轴旋转矩阵 (Yaw)
	 */
	static Matrix3f rotZ(float angle)
	{
		const float c = cosf(angle);
		const float s = sinf(angle);
		Matrix3f R;
		R(0, 0) = c;   R(0, 1) = -s;  R(0, 2) = 0.f;
		R(1, 0) = s;   R(1, 1) = c;   R(1, 2) = 0.f;
		R(2, 0) = 0.f; R(2, 1) = 0.f; R(2, 2) = 1.f;
		return R;
	}

	static Matrix3f rotX(float angle)
	{
		const float c = cosf(angle);
		const float s = sinf(angle);
		Matrix3f R;
		R(0, 0) = 1.f; R(0, 1) = 0.f; R(0, 2) = 0.f;
		R(1, 0) = 0.f; R(1, 1) = c;   R(1, 2) = -s;
		R(2, 0) = 0.f; R(2, 1) = s;   R(2, 2) = c;
		return R;
	}

	/**
	 * 计算绕 Y 轴旋转矩阵 (Pitch)
	 */
	static Matrix3f rotY(float angle)
	{
		const float c = cosf(angle);
		const float s = sinf(angle);
		Matrix3f R;
		R(0, 0) = c;   R(0, 1) = 0.f; R(0, 2) = s;
		R(1, 0) = 0.f; R(1, 1) = 1.f; R(1, 2) = 0.f;
		R(2, 0) = -s;  R(2, 1) = 0.f; R(2, 2) = c;
		return R;
	}

	/**
	 * 计算系统总质心 (Center of Mass) 在 PX4 机体坐标系下的位置
	 */
	Vector3f computeSystemCoM(const float q[4])
	{
		if (_model_profile == eso_common::ModelProfile::UamV5) {
			return computeSystemCoM_UamV5(q);
		}

		return computeSystemCoM_UavArmV4(q);
	}

	/**
	 * 计算系统总转动惯量 (完整版 - 平行轴定理)
	 *
	 * I_sys = I_base + sum( R_i * I_local_i * R_i^T + m_i * S(r_i)^T * S(r_i) )
	 *
	 * 其中 r_i = p_i - p_com 是连杆重心到系统重心的向量
	 */
	Matrix3f computeSystemInertia(const float q[4])
	{
		if (_model_profile == eso_common::ModelProfile::UamV5) {
			return computeSystemInertia_UamV5(q);
		}

		return computeSystemInertia_UavArmV4(q);
	}

private:
	Vector3f computeSystemCoM_UavArmV4(const float q[4])
	{
		// ========== 1. 物理参数 (来自 SDF) ==========
		constexpr float m_base = 0.844401f + 0.015f;
		constexpr float m1 = 0.250494f;
		constexpr float m2 = 0.020149f;
		constexpr float m3 = 0.0397105f;
		constexpr float m4 = 0.0631624f;
		constexpr float m_grip = 0.016f;
		constexpr float m_total = m_base + m1 + m2 + m3 + m4 + m_grip;

		// ========== 2. 连杆几何参数 ==========
		constexpr float z_base_to_j1 = 0.1745f;
		constexpr float L1 = 0.132f;
		constexpr float L2 = 0.0885f;
		constexpr float L3 = 0.0885f;
		constexpr float com1_z = 0.043715f;
		constexpr float com2_x = 0.04063f;
		constexpr float com3_x = 0.025422f;
		constexpr float com4_x = 0.030336f;

		// ========== 3. 正向运动学计算 ==========
		const float c0 = cosf(q[0]), s0 = sinf(q[0]);
		const float c1 = cosf(q[1]), s1 = sinf(q[1]);
		const float c12 = cosf(q[1] + q[2]), s12 = sinf(q[1] + q[2]);
		const float c123 = cosf(q[1] + q[2] + q[3]), s123 = sinf(q[1] + q[2] + q[3]);

		const Vector3f p_base_com(0.000642f, 0.f, -0.193246f);
		const Vector3f p_j1(0.f, 0.f, z_base_to_j1);
		const Vector3f p1_com = p_j1 + Vector3f(0.f, 0.f, com1_z);
		const Vector3f p_j2 = p_j1 + Vector3f(c0 * L1 * s1, s0 * L1 * s1, L1 * c1);
		const Vector3f p2_com = p_j2 + Vector3f(c0 * com2_x * c1, s0 * com2_x * c1, -com2_x * s1);
		const Vector3f p_j3 = p_j2 + Vector3f(c0 * L2 * s12, s0 * L2 * s12, L2 * c12);
		const Vector3f p3_com = p_j3 + Vector3f(c0 * com3_x * c12, s0 * com3_x * c12, -com3_x * s12);
		const Vector3f p_j4 = p_j3 + Vector3f(c0 * L3 * s123, s0 * L3 * s123, L3 * c123);
		const Vector3f p4_com = p_j4 + Vector3f(c0 * com4_x * c123, s0 * com4_x * c123, -com4_x * s123);

		// ========== 4. 加权求和 ==========
		Vector3f p_sys_com = (p_base_com * m_base + p1_com * m1 + p2_com * m2
				      + p3_com * m3 + p4_com * (m4 + m_grip)) / m_total;

		// 缓存用于惯量计算
		_cached_p1 = p1_com;
		_cached_p2 = p2_com;
		_cached_p3 = p3_com;
		_cached_p4 = p4_com;
		_cached_com = p_sys_com;
		_cached_q[0] = q[0]; _cached_q[1] = q[1]; _cached_q[2] = q[2]; _cached_q[3] = q[3];

		return p_sys_com;
	}

	Matrix3f computeSystemInertia_UavArmV4(const float q[4])
	{
		// 确保 CoM 已计算 (如果 q 变了，重新算)
		constexpr float q_eps = 1e-6f;
		if (fabsf(q[0] - _cached_q[0]) > q_eps || fabsf(q[1] - _cached_q[1]) > q_eps ||
		    fabsf(q[2] - _cached_q[2]) > q_eps || fabsf(q[3] - _cached_q[3]) > q_eps) {
			computeSystemCoM(q);
		}

		// ========== 1. 连杆质量和局部惯量 (来自 SDF) ==========
		constexpr float m1 = 0.250494f;
		constexpr float m2 = 0.020149f;
		constexpr float m3 = 0.0397105f;
		constexpr float m4 = 0.0631624f + 0.016f; // link4 + grippers

		// 局部惯量 (对角阵, 来自 SDF)
		const Matrix3f I1_local = diag(Vector3f(0.005f, 0.005f, 0.005f));
		const Matrix3f I2_local = diag(Vector3f(0.005f, 0.005f, 0.005f));
		const Matrix3f I3_local = diag(Vector3f(0.003f, 0.003f, 0.003f));
		const Matrix3f I4_local = diag(Vector3f(0.003f, 0.003f, 0.003f));

		// ========== 2. 计算各连杆的旋转矩阵 ==========
		// R1: 绕 Z 旋转 q[0]
		const Matrix3f R1 = rotZ(q[0]);

		// R2: 绕 Z 旋转 q[0], 再绕 Y 旋转 q[1]
		const Matrix3f R2 = R1 * rotY(q[1]);

		// R3: 累积旋转
		const Matrix3f R3 = R2 * rotY(q[2]);

		// R4: 累积旋转
		const Matrix3f R4 = R3 * rotY(q[3]);

		// ========== 3. 计算相对于系统重心的位置向量 ==========
		const Vector3f r1 = _cached_p1 - _cached_com;
		const Vector3f r2 = _cached_p2 - _cached_com;
		const Vector3f r3 = _cached_p3 - _cached_com;
		const Vector3f r4 = _cached_p4 - _cached_com;

		// ========== 4. 应用平行轴定理 ==========
		// I_i_body = R_i * I_i_local * R_i^T + m_i * S(r_i)^T * S(r_i)

		Matrix3f I_total = _base_inertia_val;

		// Link 1
		Matrix3f S1 = skew(r1);
		I_total += R1 * I1_local * R1.transpose() + S1.transpose() * S1 * m1;

		// Link 2
		Matrix3f S2 = skew(r2);
		I_total += R2 * I2_local * R2.transpose() + S2.transpose() * S2 * m2;

		// Link 3
		Matrix3f S3 = skew(r3);
		I_total += R3 * I3_local * R3.transpose() + S3.transpose() * S3 * m3;

		// Link 4 + Grippers
		Matrix3f S4 = skew(r4);
		I_total += R4 * I4_local * R4.transpose() + S4.transpose() * S4 * m4;

		return I_total;
	}

	Vector3f computeSystemCoM_UamV5(const float q[4])
	{
		constexpr float m_base = 2.0f;
		constexpr float m1 = 0.2f;
		constexpr float m2 = 0.35f;
		constexpr float m_left = 0.01f;
		constexpr float m_right = 0.01f;
		constexpr float m_total = m_base + m1 + m2 + m_left + m_right;

		const Vector3f p_base_com(0.01027894f, -0.00024771f, 0.16606907f);

		const Vector3f t_j1(0.104f, 0.0f, 0.098434f);
		const Matrix3f R_j1 = rotX(-M_PI_F) * rotZ(q[0]);
		const Vector3f p1_com = t_j1 + R_j1 * Vector3f(0.0f, 0.013f, 0.043f);

		const Vector3f t_j2 = t_j1 + R_j1 * Vector3f(0.0f, 0.02f, 0.056f);
		const Matrix3f R_j2 = R_j1 * rotX(-M_PI_2_F) * rotZ(q[1]);
		const Vector3f p2_com = t_j2 + R_j2 * Vector3f(0.0913f, 0.0001f, 0.0f);

		const Vector3f p_left_joint = t_j2 + R_j2 * Vector3f(0.19f, 0.001f, -0.0075f + q[2]);
		const Vector3f p_left_com = p_left_joint + R_j2 * Vector3f(0.02582393f, -0.001f, -0.00633685f);

		const Vector3f p_right_joint = t_j2 + R_j2 * Vector3f(0.19f, 0.001f, 0.0075f);
		const Vector3f p_right_com = p_right_joint + R_j2 * Vector3f(0.02582393f, -0.001f, 0.00633685f);

		const Vector3f p_sys_com = (p_base_com * m_base + p1_com * m1 + p2_com * m2
				      + p_left_com * m_left + p_right_com * m_right) / m_total;

		_cached_p1 = p1_com;
		_cached_p2 = p2_com;
		_cached_p3 = p_left_com;
		_cached_p4 = p_right_com;
		_cached_com = p_sys_com;
		_cached_q[0] = q[0];
		_cached_q[1] = q[1];
		_cached_q[2] = q[2];
		_cached_q[3] = q[3];

		return p_sys_com;
	}

	Matrix3f computeSystemInertia_UamV5(const float q[4])
	{
		constexpr float q_eps = 1e-6f;
		if (fabsf(q[0] - _cached_q[0]) > q_eps || fabsf(q[1] - _cached_q[1]) > q_eps ||
		    fabsf(q[2] - _cached_q[2]) > q_eps || fabsf(q[3] - _cached_q[3]) > q_eps) {
			computeSystemCoM_UamV5(q);
		}

		constexpr float m_base = 2.0f;
		constexpr float m1 = 0.2f;
		constexpr float m2 = 0.35f;
		constexpr float m_left = 0.01f;
		constexpr float m_right = 0.01f;

		Matrix3f I_base_local;
		I_base_local(0, 0) = 0.02601f;
		I_base_local(0, 1) = 8.015e-05f;
		I_base_local(0, 2) = -0.00070008f;
		I_base_local(1, 0) = 8.015e-05f;
		I_base_local(1, 1) = 0.02942848f;
		I_base_local(1, 2) = -8.915e-05f;
		I_base_local(2, 0) = -0.00070008f;
		I_base_local(2, 1) = -8.915e-05f;
		I_base_local(2, 2) = 0.04177777f;
		const Matrix3f I1_local = diag(Vector3f(0.00030f, 0.00030f, 0.00015f));
		const Matrix3f I2_local = diag(Vector3f(0.00080f, 0.00120f, 0.00090f));
		const Matrix3f I_hand_local = diag(Vector3f(1.0e-05f, 1.0e-05f, 1.0e-05f));

		const Vector3f p_base_com(0.01027894f, -0.00024771f, 0.16606907f);
		const Vector3f t_j1(0.104f, 0.0f, 0.098434f);
		const Matrix3f R_j1 = rotX(-M_PI_F) * rotZ(q[0]);
		const Matrix3f R_j2 = R_j1 * rotX(-M_PI_2_F) * rotZ(q[1]);

		const Vector3f r_base = p_base_com - _cached_com;
		const Vector3f r1 = _cached_p1 - _cached_com;
		const Vector3f r2 = _cached_p2 - _cached_com;
		const Vector3f r3 = _cached_p3 - _cached_com;
		const Vector3f r4 = _cached_p4 - _cached_com;

		Matrix3f I_total = I_base_local;

		Matrix3f S_base = skew(r_base);
		I_total += S_base.transpose() * S_base * m_base;

		Matrix3f S1 = skew(r1);
		I_total += R_j1 * I1_local * R_j1.transpose() + S1.transpose() * S1 * m1;

		Matrix3f S2 = skew(r2);
		I_total += R_j2 * I2_local * R_j2.transpose() + S2.transpose() * S2 * m2;

		Matrix3f S3 = skew(r3);
		I_total += R_j2 * I_hand_local * R_j2.transpose() + S3.transpose() * S3 * m_left;

		Matrix3f S4 = skew(r4);
		I_total += R_j2 * I_hand_local * R_j2.transpose() + S4.transpose() * S4 * m_right;

		return I_total;
	}

	float _mass_total_system{1.5f};
	Vector3f _base_com_offset{};
	Matrix3f _base_inertia_val{};
	eso_common::ModelProfile _model_profile{eso_common::ModelProfile::UavArmV4};

	// 缓存值 (避免重复计算)
	Vector3f _cached_p1{}, _cached_p2{}, _cached_p3{}, _cached_p4{};
	Vector3f _cached_com{};
	float _cached_q[4]{0.f, 0.f, 0.f, 0.f};
};
