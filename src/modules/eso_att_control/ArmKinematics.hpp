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
		_uam_v5_cache_valid = false;
	}

	void setParameters(float mass_total, const Vector3f &base_com_offset, const Matrix3f &base_inertia)
	{
		_mass_total_system = mass_total;
		_base_com_offset = base_com_offset;
		_base_inertia_val = base_inertia;
		_uam_v5_cache_valid = false;
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
	// Pose3f 表示一个刚体位姿（rigid-body pose）：
	// - R: 子坐标系相对父坐标系的旋转矩阵
	// - t: 子坐标系原点在父坐标系下的位置
	//
	// 数学上可写成齐次变换：
	//     T = [ R  t ]
	//         [ 0  1 ]
	//
	// 后面的 UamV5 主链不再手写 sin/cos 展开，而是统一通过 Pose3f 传递
	// “旋转 + 平移”这两个信息，便于和 MDH/动捕辨识参数对应。
	struct Pose3f {
		Matrix3f R{};
		Vector3f t{};
	};

	// makePose 的作用：
	// 把“旋转矩阵 R”和“平移向量 t”打包成一个位姿对象 Pose3f。
	//
	// 它本身不做前向运动学推导，只做“位姿表示”的封装。
	// 物理意义上，你可以把它理解成：
	// “已知子坐标系相对父坐标系怎么安装，把这个安装关系记成一个刚体位姿。”
	//
	// 典型用途：
	// 1. 表示固定安装外参，例如 base_link -> joint1 的安装位姿
	// 2. 给 composePose() / transformPoint() 提供统一输入
	static Pose3f makePose(const Matrix3f &R, const Vector3f &t)
	{
		Pose3f pose{};
		pose.R = R;
		pose.t = t;
		return pose;
	}

	// composePose 的作用：
	// 计算两级位姿链的复合结果，也就是把 child 接到 parent 后面。
	//
	// 若：
	// - parent 表示 T_A_B（B 相对 A 的位姿）
	// - child  表示 T_B_C（C 相对 B 的位姿）
	// 则 composePose(parent, child) 得到：
	// - T_A_C = T_A_B * T_B_C
	//
	// 对应的刚体位姿复合公式：
	// - R_A_C = R_A_B * R_B_C
	// - t_A_C = t_A_B + R_A_B * t_B_C
	//
	// 物理意义：
	// 先经过 parent 的旋转和平移，再叠加 child 的局部安装关系，
	// 最终得到更远一级子坐标系在全局中的位姿。
	static Pose3f composePose(const Pose3f &parent, const Pose3f &child)
	{
		Pose3f composed{};
		composed.R = parent.R * child.R;
		composed.t = parent.t + parent.R * child.t;
		return composed;
	}

	// transformPoint 的作用：
	// 把“局部坐标系中的点”变换到“父/全局坐标系”中。
	//
	// 数学公式：
	//     p_global = t + R * p_local
	//
	// 在本文件里，它最常用于：
	// - 已知某个模块局部质心偏移 r_com_local
	// - 已知该模块当前位姿 Pose3f
	// - 求该模块质心在 PX4 机体系下的位置 p_com_global
	//
	// 这一步是“局部 CoM -> 全局 CoM”的核心。
	static Vector3f transformPoint(const Pose3f &pose, const Vector3f &local_point)
	{
		return pose.t + pose.R * local_point;
	}

	// Gazebo SDF link frames are FLU/Z-up, while PX4 control code uses body FRD.
	// Keep the UAM V5 forward kinematics in the SDF frame, then convert model
	// outputs at the controller boundary.
	static Vector3f fluToFrdVector(const Vector3f &v_flu)
	{
		return Vector3f(v_flu(0), -v_flu(1), -v_flu(2));
	}

	static Matrix3f fluToFrdMatrix(const Matrix3f &m_flu)
	{
		Matrix3f C{};
		C(0, 0) = 1.f;
		C(1, 1) = -1.f;
		C(2, 2) = -1.f;
		return C * m_flu * C.transpose();
	}

	// makeMdhPose 的作用：
	// 根据一组 Modified DH 参数，构造相邻两节坐标系之间的位姿变换。
	//
	// 这里采用的 MDH 形式为：
	//     T_{i-1}^{i} = RotX(alpha_{i-1}) * TransX(a_{i-1}) * RotZ(theta_i) * TransZ(d_i)
	//
	// 对应到 Pose3f 后可拆成：
	// - 旋转部分：R = RotX(alpha_{i-1}) * RotZ(theta_i)
	// - 平移部分：t = RotX(alpha_{i-1}) * [a_{i-1}, 0, d_i]^T
	//
	// 物理意义：
	// - alpha_prev: 前一坐标轴到当前坐标轴之间的固定扭转角
	// - a_prev    : 前一坐标轴到当前坐标轴之间沿 x 的固定连杆长度
	// - theta_i   : 当前转动关节的变量角（通常来自编码器）
	// - d_i       : 当前关节沿 z 的固定/变量偏移
	//
	// 在 UamV5 当前实现里，我们只用它表达“变量关节转角”这一部分，
	// 这样能把固定安装外参和变量关节分开写，便于后续把动捕辨识出的
	// MDH/等价几何参数直接回填。
	static Pose3f makeMdhPose(float alpha_prev, float a_prev, float theta_i, float d_i)
	{
		// Modified DH: T_{i-1}^{i} = RotX(alpha_{i-1}) * TransX(a_{i-1}) * RotZ(theta_i) * TransZ(d_i)
		Pose3f pose{};
		pose.R = rotX(alpha_prev) * rotZ(theta_i);
		pose.t = rotX(alpha_prev) * Vector3f(a_prev, 0.f, d_i);
		return pose;
	}

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
		constexpr float z_base_to_j1 = 0.1745f;//机体原点到第一个关节的固定高度，假设第一个关节只沿机体 z 方向抬高
		constexpr float L1 = 0.132f;
		constexpr float L2 = 0.0885f;
		constexpr float L3 = 0.0885f;
		constexpr float com1_z = 0.043715f; //各 link 自身质心相对其关节原点的局部偏移
		constexpr float com2_x = 0.04063f;
		constexpr float com3_x = 0.025422f;
		constexpr float com4_x = 0.030336f;

		// ========== 3. 正向运动学计算 ==========
		const float c0 = cosf(q[0]), s0 = sinf(q[0]);
		const float c1 = cosf(q[1]), s1 = sinf(q[1]);
		const float c12 = cosf(q[1] + q[2]), s12 = sinf(q[1] + q[2]);
		const float c123 = cosf(q[1] + q[2] + q[3]), s123 = sinf(q[1] + q[2] + q[3]);

		const Vector3f p_base_com(0.000642f, 0.f, -0.193246f); // 机体系下，base_link距离机体坐标系的位置偏移
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

	/**
	 * UamV5 版本的系统总质心计算。
	 *
	 * 注意：UAM V5 的几何/惯量常量来自 Gazebo SDF，坐标系是 FLU/Z-up。
	 * 本函数内部按 SDF 坐标做运动学，返回值转换为 PX4 body FRD，供论文
	 * (31) 中的 p_C^B 与控制器重力补偿使用。
	 *
	 * 设计目标：
	 * 1. 主链仅保留 2 个连续自由度：q[0] = arm_joint1，q[1] = arm_joint2。
	 * 2. 左右夹爪在当前阶段始终视为“固定合并状态”，并入 link2+gripper 模块。
	 * 3. q[2]/q[3] 继续保留接口兼容，但不进入连续 CoM/I(q) 补偿。
	 *
	 * 为什么这里采用 MDH/等价齐次变换，而不是像 UavArmV4 那样手写三角展开：
	 * - UamV5 主链含有固定安装旋转和固定安装平移，更适合用“位姿链”表达；
	 * - 这种写法能直接对齐后续动捕系统辨识出的几何参数，便于把实测值回填到代码；
	 * - 对当前实验阶段更重要的是参数语义清晰，而不是把旋转矩阵手工展开成 sin/cos。
	 *
	 * 本函数最终使用的物理公式只有两类：
	 * 1. 点坐标变换：p_global = p_origin + R * p_local
	 * 2. 系统总质心：p_sys = sum(m_i * p_i) / sum(m_i)
	 */
	Vector3f computeSystemCoM_UamV5(const float q[4])
	{
		// ------------------------------------------------------------------
		// 1. 模块质量参数
		// ------------------------------------------------------------------
		// 【实测参数】以下质量参数应由电子秤/模块拆分称重得到。
		// 本版仅建模 3 个刚体：base、link1、link2+gripper。
		constexpr float m_base = 3.6678f;
		constexpr float m1 = 0.593f;
		constexpr float m2g = 0.594f; // link2 already includes gripper mass in measured data.
		constexpr float m_total = m_base + m1 + m2g;

		// ------------------------------------------------------------------
		// 2. 几何/运动学参数（主链 MDH/等价齐次变换）
		// ------------------------------------------------------------------
		// 【编码器/标定参数】joint1 / joint2 的零位偏置和符号。
		// 这些参数需要通过动捕 + 编码器 least-squares 标定得到；
		// “编码器时延”也是需要实测的参数，但它属于实时链路问题，不在纯几何函数内使用。
		constexpr float joint1_zero_offset = 0.f;
		constexpr float joint2_zero_offset = 0.f;
		constexpr float joint1_sign = 1.f;
		constexpr float joint2_sign = 1.f;

		// 【计算量】当前真正参与 UamV5 动力学补偿的只有 q[0] / q[1]。
		// q[2] / q[3] 继续保留接口兼容，但不再影响 CoM(q) / I(q)。
		const float theta1 = joint1_sign * (q[0] + joint1_zero_offset);
		const float theta2 = joint2_sign * (q[1] + joint2_zero_offset);

		// 【实测参数】base 自身 CoM，相对 base_link 原点，在 SDF FLU/Z-up 坐标系下表达。
		const Vector3f r_base_com_local(-0.044f, 0.0f, -0.046f);
		const Vector3f p_base_com = r_base_com_local;

		// 【实测/标定参数】base_link -> joint1 的固定外参。
		// 物理意义：机械臂根部安装在机体上的固定位置和固定朝向。
		const Pose3f T_B_J1_mount = makePose(rotX(-M_PI_F), Vector3f(0.115f, 0.0f, -0.09f));

		// 【编码器/标定参数】joint1 的转动变换。这里采用等价 MDH 变量关节表达：
		// 仅保留 theta1 的局部绕 z 轴旋转，方便后续直接对接由动捕辨识出的主链参数。
		const Pose3f T_J1_var = makeMdhPose(0.f, 0.f, theta1, 0.f);

		// 【计算量】base_link -> link1 坐标系的全局位姿。
		// 公式：T_B_1 = T_B_J1_mount * T_J1_var
		const Pose3f T_B_1 = composePose(T_B_J1_mount, T_J1_var);

		// 【实测/标定参数】joint1 -> joint2 的固定安装外参。
		// 物理意义：link1 末端到 joint2 根部的刚性安装偏移与固定安装角。
		const Pose3f T_1_2_mount = makePose(rotX(-M_PI_2_F), Vector3f(0.0f, 0.0f, 0.072f));

		// 【编码器/标定参数】joint2 的变量转角。
		const Pose3f T_2_var = makeMdhPose(0.f, 0.f, theta2, 0.f);

		// 【计算量】base_link -> link2+gripper 主坐标系的全局位姿。
		// 公式：T_B_2 = T_B_1 * T_1_2_mount * T_2_var
		const Pose3f T_B_2 = composePose(composePose(T_B_1, T_1_2_mount), T_2_var);

		// ------------------------------------------------------------------
		// 3. 各模块局部 CoM 偏移
		// ------------------------------------------------------------------
		// 【实测参数】link1 模块自身质心相对 link1/joint1 局部原点的偏移。
		const Vector3f r1_com_local(0.0f, 0.0f, 0.0f);

		// 【实测参数】link2+gripper 合并模块的局部 CoM 偏移。
		const Vector3f r2g_com_local(0.075f, 0.0f, 0.0f);

		// ------------------------------------------------------------------
		// 4. 局部 CoM -> 全局 CoM
		// ------------------------------------------------------------------
		// 【计算量】使用点坐标变换公式 p_global = p_origin + R * p_local，
		// 将各模块局部 CoM 投影到 SDF base_link 坐标系。
		const Vector3f p1_com = transformPoint(T_B_1, r1_com_local);
		const Vector3f p2g_com = transformPoint(T_B_2, r2g_com_local);

		// ------------------------------------------------------------------
		// 5. 系统总质心
		// ------------------------------------------------------------------
		// 【计算量】总质心使用质量加权平均：
		// p_sys = (m_base*p_base + m1*p1 + m2g*p2g) / (m_base + m1 + m2g)
		const Vector3f p_sys_com = (p_base_com * m_base + p1_com * m1 + p2g_com * m2g) / m_total;

		// ------------------------------------------------------------------
		// 6. 缓存用于后续惯量计算
		// ------------------------------------------------------------------
		// 【计算量】惯量函数需要复用“各模块 CoM 在机体系下的位置”以及“系统总质心”。
		_cached_p1 = p1_com;
		_cached_p2 = p2g_com;
		_cached_p3.zero();
		_cached_p4.zero();
		_cached_com = p_sys_com;
		_cached_q[0] = q[0];
		_cached_q[1] = q[1];
		_cached_q[2] = q[2];
		_cached_q[3] = q[3];
		_uam_v5_cache_valid = true;

		return fluToFrdVector(p_sys_com);
	}

	Matrix3f computeSystemInertia_UamV5(const float q[4])
	{
		// ------------------------------------------------------------------
		// UamV5 系统惯量矩阵计算
		//
		// 输出量：I_total，参考点为“系统总质心”，返回坐标系为 PX4 body FRD。
		// 内部计算先使用 SDF FLU/Z-up，再在返回前转换。
		// 使用的核心公式是：
		// I_total = Σ ( R_i * I_i_local * R_i^T + m_i * S(r_i)^T * S(r_i) )
		// 其中：
		// - R_i * I_i_local * R_i^T ：把局部惯量旋转到机体系
		// - m_i * S(r_i)^T * S(r_i)：用平行轴定理搬移到系统总质心
		// ------------------------------------------------------------------
		constexpr float q_eps = 1e-6f;

		// 【计算量】UamV5 动力学模型只对 q[0]/q[1] 敏感，因此缓存失效判断只检查这两个自由度。
		if (fabsf(q[0] - _cached_q[0]) > q_eps || fabsf(q[1] - _cached_q[1]) > q_eps ||
		    !_uam_v5_cache_valid) {
			computeSystemCoM_UamV5(q);
		}

		// ------------------------------------------------------------------
		// 1. 质量参数
		// ------------------------------------------------------------------
		// 【实测参数】以下质量应来自模块级称重。
		constexpr float m_base = 3.6678f;
		constexpr float m1 = 0.593f;
		constexpr float m2g = 0.594f;

		// ------------------------------------------------------------------
		// 2. 各模块局部惯量
		// ------------------------------------------------------------------
		// 【实测参数】局部惯量定义为“模块相对于自身 CoM、在自身局部坐标系下的惯量张量”。
		// 当前使用实测 Ixx / Iyy / Izz 对角项；若后续要提高精度，可再补 Ixy / Ixz / Iyz。
		const Matrix3f I_base_local = diag(Vector3f(0.049436f, 0.052665f, 0.086619f));
		const Matrix3f I1_local = diag(Vector3f(0.000839f, 0.000791f, 0.000943f));
		const Matrix3f I2g_local = diag(Vector3f(0.000302f, 0.002811f, 0.002717f));

		// ------------------------------------------------------------------
		// 3. 主链姿态变换
		// ------------------------------------------------------------------
		// 【编码器/标定参数】主链姿态和 computeSystemCoM_UamV5() 保持完全一致。
		// 这样可以确保 CoM(q) 与 I(q) 来自同一套几何模型。
		constexpr float joint1_zero_offset = 0.f;
		constexpr float joint2_zero_offset = 0.f;
		constexpr float joint1_sign = 1.f;
		constexpr float joint2_sign = 1.f;
		const float theta1 = joint1_sign * (q[0] + joint1_zero_offset);
		const float theta2 = joint2_sign * (q[1] + joint2_zero_offset);

		const Pose3f T_B_J1_mount = makePose(rotX(-M_PI_F), Vector3f(0.115f, 0.0f, -0.09f));
		const Pose3f T_J1_var = makeMdhPose(0.f, 0.f, theta1, 0.f);
		const Pose3f T_B_1 = composePose(T_B_J1_mount, T_J1_var);
		const Pose3f T_1_2_mount = makePose(rotX(-M_PI_2_F), Vector3f(0.0f, 0.0f, 0.072f));
		const Pose3f T_2_var = makeMdhPose(0.f, 0.f, theta2, 0.f);
		const Pose3f T_B_2 = composePose(composePose(T_B_1, T_1_2_mount), T_2_var);

		// ------------------------------------------------------------------
		// 4. 各模块相对系统总质心的位置向量
		// ------------------------------------------------------------------
		// 【计算量】r_i = p_i - p_sys，用于平行轴定理。
		const Vector3f r_base = Vector3f(-0.044f, 0.0f, -0.046f) - _cached_com;
		const Vector3f r1 = _cached_p1 - _cached_com;
		const Vector3f r2g = _cached_p2 - _cached_com;

		// ------------------------------------------------------------------
		// 5. 平行轴定理求系统惯量
		// ------------------------------------------------------------------
		// 【计算量】先从 base 自身局部惯量开始累加，再加入 link1、link2+gripper 的贡献。
		Matrix3f I_total = I_base_local;

		// base：base 坐标系与机体系重合，因此只需要加“搬移项”。
		Matrix3f S_base = skew(r_base);
		I_total += S_base.transpose() * S_base * m_base;

		// link1：先把局部惯量旋转到机体系，再用平行轴定理搬移到系统总质心。
		Matrix3f S1 = skew(r1);
		I_total += T_B_1.R * I1_local * T_B_1.R.transpose() + S1.transpose() * S1 * m1;

		// link2+gripper：当前阶段把 link2 与夹爪合并成一个刚体，
		// 这样你后续只需要实测“模块质量 + 模块局部 CoM + 模块局部惯量”，
		// 不必单独为左右夹爪建立连续动态模型。
		Matrix3f S2g = skew(r2g);
		I_total += T_B_2.R * I2g_local * T_B_2.R.transpose() + S2g.transpose() * S2g * m2g;

		return fluToFrdMatrix(I_total);
	}

	float _mass_total_system{1.5f};
	Vector3f _base_com_offset{};
	Matrix3f _base_inertia_val{};
	eso_common::ModelProfile _model_profile{eso_common::ModelProfile::UavArmV4};

	// 缓存值 (避免重复计算)
	Vector3f _cached_p1{}, _cached_p2{}, _cached_p3{}, _cached_p4{};
	Vector3f _cached_com{};
	float _cached_q[4]{0.f, 0.f, 0.f, 0.f};
	bool _uam_v5_cache_valid{false};
};
