#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
科研风格绘图脚本 - 用于绘制无人机和机械臂控制性能图表
Scientific-style plotting script for UAV and manipulator control performance visualization

本脚本使用 matplotlib 和 numpy 绘制符合学术论文规范的图表
包括：位置跟踪、机械臂关节跟踪、位置误差分析

作者: Auto-generated
日期: 2026-01-03
"""

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import argparse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import math

# ============================================================================
# 全局样式设置 (Global Style Configuration)
# ============================================================================
# 使用 Times New Roman 衬线字体，符合学术论文规范
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['font.size'] = 11  # 适中字号

# 启用 LaTeX 渲染（如果系统支持）
# 注意：如果系统未安装 LaTeX，请将下面两行注释掉，使用 mathtext 替代
# plt.rcParams['text.usetex'] = True
# plt.rcParams['text.latex.preamble'] = r'\usepackage{amsmath}'

# 使用 mathtext 渲染数学公式（不需要 LaTeX 环境）
plt.rcParams['mathtext.fontset'] = 'stix'  # STIX 字体接近 Times New Roman

# 图表线条和网格设置
plt.rcParams['axes.linewidth'] = 1.0
plt.rcParams['grid.linewidth'] = 0.5
plt.rcParams['grid.alpha'] = 0.3
plt.rcParams['lines.linewidth'] = 1.5

# 图例设置
plt.rcParams['legend.frameon'] = True
plt.rcParams['legend.framealpha'] = 0.9
plt.rcParams['legend.edgecolor'] = 'black'
plt.rcParams['legend.fancybox'] = False

# ============================================================================
# 辅助函数 (Helper Functions)
# ============================================================================

def quark_to_euler(q):
    """
    将四元数转换为欧拉角 (Roll, Pitch, Yaw)
    q: 具有 w, x, y, z 属性的四元数对象
    """
    if hasattr(q, 'w'):
        w, x, y, z = q.w, q.x, q.y, q.z
    else:
        w, x, y, z = q

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def interp_data(t_target, t_source, source_data):
    """将源数据插值对齐到目标时间戳"""
    return np.interp(t_target, t_source, source_data)


# ============================================================================
# 数据读取函数 (Data Loading Function)
# ============================================================================

def load_data_from_bag(bagfile, start_time=0.0):
    """
    从 ROS bag 文件读取数据

    参数:
        bagfile: bag 文件路径
        start_time: 相对起始时间 (秒)

    返回:
        data: 包含所有时间序列数据的字典
    """
    print(f"正在打开 bag 文件: {bagfile}")
    bag = rosbag.Bag(bagfile)

    # 数据容器
    data = {
        't_pos': [], 'pos_x': [], 'pos_y': [], 'pos_z': [],
        't_pos_sp': [], 'pos_sp_x': [], 'pos_sp_y': [], 'pos_sp_z': [],
        't_arm': [], 'arm_actual': {}, 'arm_target': {}
    }

    start_time_abs = None

    # 读取话题消息
    print("正在读取消息...")
    for topic, msg, t in bag.read_messages():
        if start_time_abs is None:
            start_time_abs = t.to_sec()

        current_time = t.to_sec() - start_time_abs
        if current_time < start_time:
            continue

        # 1. 实际位置 (来自 /mavros/local_position/pose)
        if topic == '/mavros/local_position/pose':
            data['t_pos'].append(current_time)
            data['pos_x'].append(msg.pose.position.x)
            data['pos_y'].append(msg.pose.position.y)
            data['pos_z'].append(msg.pose.position.z)

        # 2. 目标位置 (来自 /mavros/setpoint_position/local)
        elif topic == '/mavros/setpoint_position/local':
            data['t_pos_sp'].append(current_time)
            data['pos_sp_x'].append(msg.pose.position.x)
            data['pos_sp_y'].append(msg.pose.position.y)
            data['pos_sp_z'].append(msg.pose.position.z)

        # 3. 机械臂实际关节状态
        elif topic == '/uav_arm/joint_states':
            data['t_arm'].append(current_time)
            for i, name in enumerate(msg.name):
                if name not in data['arm_actual']:
                    data['arm_actual'][name] = []
                data['arm_actual'][name].append(msg.position[i])

        # 4. 机械臂目标关节状态
        elif topic == '/uav_arm/target_joint_states':
            for i, name in enumerate(msg.name):
                if name not in data['arm_target']:
                    data['arm_target'][name] = []
                    data['arm_target'][name + '_t'] = []
                data['arm_target'][name].append(msg.position[i])
                data['arm_target'][name + '_t'].append(current_time)

    bag.close()
    print("数据读取完成。")

    return data


# ============================================================================
# 【重要】数据替换接口 (Data Replacement Interface)
# ============================================================================
# 如果您想使用自己的实验数据而非从 bag 文件读取，请按以下方式替换数据：
#
# 1. 注释掉 main() 函数中的 load_data_from_bag() 调用
# 2. 创建一个与下面结构相同的 data 字典
# 3. 将您的实验数据填入对应的数组中
#
# 数据结构示例：
# data = {
#     # 位置数据 (Position data)
#     't_pos': np.array([...]),      # 时间戳 (s)
#     'pos_x': np.array([...]),      # 实际 X 位置 (m)
#     'pos_y': np.array([...]),      # 实际 Y 位置 (m)
#     'pos_z': np.array([...]),      # 实际 Z 位置 (m)
#
#     # 目标位置数据 (Desired position data)
#     't_pos_sp': np.array([...]),   # 时间戳 (s)
#     'pos_sp_x': np.array([...]),   # 期望 X 位置 (m)
#     'pos_sp_y': np.array([...]),   # 期望 Y 位置 (m)
#     'pos_sp_z': np.array([...]),   # 期望 Z 位置 (m)
#
#     # 机械臂数据 (Manipulator data)
#     't_arm': np.array([...]),      # 时间戳 (s)
#     'arm_actual': {                # 实际关节角度 (rad)
#         'joint1': np.array([...]),
#         'joint2': np.array([...]),
#         'joint3': np.array([...]),
#         # ... 更多关节
#     },
#     'arm_target': {                # 期望关节角度 (rad)
#         'joint1': np.array([...]),
#         'joint1_t': np.array([...]),  # 对应时间戳
#         'joint2': np.array([...]),
#         'joint2_t': np.array([...]),
#         # ... 更多关节
#     }
# }
# ============================================================================


# ============================================================================
# 绘图函数 (Plotting Functions)
# ============================================================================

def plot_position_tracking(data):
    """
    【图 1】无人机位置跟踪 (UAV Position Tracking)

    布局：3 行 1 列，共享 X 轴
    内容：分别绘制 x, y, z 三个轴的轨迹
    曲线：蓝色实线 (期望值)，品红色点划线 (实际值)
    """
    # ========================================================================
    # 数据准备 - 如需替换为您的数据，请修改此处
    # Data preparation - Replace here with your own data if needed
    # ========================================================================
    t = np.array(data['t_pos'])           # 时间轴 (Time axis)

    # 实际位置 (Actual position)
    x = np.array(data['pos_x'])           # 实际 X 位置 (m)
    y = np.array(data['pos_y'])           # 实际 Y 位置 (m)
    z = np.array(data['pos_z'])           # 实际 Z 位置 (m)

    # 期望位置 (Desired position) - 插值对齐到实际位置的时间轴
    x_d = interp_data(t, data['t_pos_sp'], data['pos_sp_x'])  # 期望 X 位置 (m)
    y_d = interp_data(t, data['t_pos_sp'], data['pos_sp_y'])  # 期望 Y 位置 (m)
    z_d = interp_data(t, data['t_pos_sp'], data['pos_sp_z'])  # 期望 Z 位置 (m)
    # ========================================================================

    # 计算误差及统计量
    ex = x - x_d
    ey = y - y_d
    ez = z - z_d

    # 保存统计结果供终端打印
    data['stats_x'] = {
        'mean_error': np.mean(ex),
        'mse': np.mean(ex**2),
        'max_abs_error': np.max(np.abs(ex))
    }
    data['stats_y'] = {
        'mean_error': np.mean(ey),
        'mse': np.mean(ey**2),
        'max_abs_error': np.max(np.abs(ey))
    }
    data['stats_z'] = {
        'mean_error': np.mean(ez),
        'mse': np.mean(ez**2),
        'max_abs_error': np.max(np.abs(ez))
    }

    # 计算综合位置指标
    # Combined Position MSE: (1/N) * sum( ||p - pd||^2 )
    pos_err_sq_sum = ex**2 + ey**2 + ez**2
    data['combined_position_mse'] = np.mean(pos_err_sq_sum)

    # Max Position Error: max( ||p - p_d|| )
    pos_err_norm = np.sqrt(pos_err_sq_sum)
    data['max_position_error'] = np.max(pos_err_norm)

    # 创建图形
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(8, 9), sharex=True)

    # 定义曲线样式
    style_desired = {'color': 'blue', 'linestyle': '-', 'linewidth': 1.5, 'label': 'Desired value'}
    style_actual = {'color': 'magenta', 'linestyle': '-.', 'linewidth': 1.5, 'label': 'Actual value'}

    # 子图 1: X 轴
    axes[0].plot(t, x_d, **style_desired)
    axes[0].plot(t, x, **style_actual)
    axes[0].set_ylabel(r'$x$ (m)', fontsize=12)
    axes[0].set_xlabel(r'$t$ (s)', fontsize=12)
    axes[0].tick_params(labelbottom=True)  # 强制显示 X 轴刻度标签
    axes[0].grid(True, linestyle='--', alpha=0.5)

    # 子图 2: Y 轴
    axes[1].plot(t, y_d, **style_desired)
    axes[1].plot(t, y, **style_actual)
    axes[1].set_ylabel(r'$y$ (m)', fontsize=12)
    axes[1].set_xlabel(r'$t$ (s)', fontsize=12)
    axes[1].tick_params(labelbottom=True)  # 强制显示 X 轴刻度标签
    axes[1].grid(True, linestyle='--', alpha=0.5)

    # 子图 3: Z 轴
    axes[2].plot(t, z_d, **style_desired)
    axes[2].plot(t, z, **style_actual)
    axes[2].set_ylabel(r'$z$ (m)', fontsize=12)
    axes[2].set_xlabel(r'$t$ (s)', fontsize=12)
    axes[2].tick_params(labelbottom=True)  # 强制显示 X 轴刻度标签
    axes[2].grid(True, linestyle='--', alpha=0.5)

    # 图例放置在图形顶部外部，横向排列
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.98),
               ncol=2, frameon=True, fontsize=11)

    # 调整子图间距，为顶部图例留出空间
    plt.subplots_adjust(top=0.94, hspace=0.25) # 增加 hspace 以避免 label 重叠

    return fig


def plot_manipulator_tracking(data):
    """
    【图 2】机械臂关节跟踪 (Manipulator Joint Tracking)

    布局：n 行 1 列（n = 关节数量）
    内容：绘制机械臂关节角度的轨迹
    曲线：蓝色实线 (期望值)，品红色点划线 (实际值)
    """
    # 获取关节数量
    all_joint_names = sorted(data['arm_actual'].keys())

    # 仅保留前 4 个关节 (joint1 到 joint4)
    target_joints = [j for j in all_joint_names if j in ['joint1', 'joint2', 'joint3', 'joint4']]

    # 如果没有找到指定的关节，尝试按顺序取前4个
    if not target_joints and len(all_joint_names) > 0:
        target_joints = all_joint_names[:4]

    n_joints = len(target_joints)

    if n_joints == 0:
        print("警告: 未找到机械臂数据，跳过绘制图 2")
        return None

    # 初始化统计字典
    data['arm_stats'] = {}

    # 创建图形 - 使用 2x2 布局
    fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(12, 8), sharex=True)
    axes = axes.flatten()  # 将 2x2 数组展平为 1D 数组, 方便索引

    # 定义曲线样式
    style_desired = {'color': 'blue', 'linestyle': '-', 'linewidth': 1.5, 'label': 'Desired value'}
    style_actual = {'color': 'magenta', 'linestyle': '-.', 'linewidth': 1.5, 'label': 'Actual value'}

    # ========================================================================
    # 数据准备 - 如需替换为您的数据，请修改此处
    # Data preparation - Replace here with your own data if needed
    # ========================================================================
    t = np.array(data['t_arm'])  # 时间轴 (Time axis)
    # ========================================================================

    # 绘制每个关节
    for i, joint_name in enumerate(target_joints):
        # 实际关节角度 (Actual joint angle)
        theta = np.array(data['arm_actual'][joint_name])  # 单位: rad

        # 期望关节角度 (Desired joint angle)
        if joint_name in data['arm_target']:
            theta_d_raw = np.array(data['arm_target'][joint_name])
            t_target = np.array(data['arm_target'][joint_name + '_t'])
            theta_d = interp_data(t, t_target, theta_d_raw)  # 插值对齐
        else:
            theta_d = np.zeros_like(theta)  # 如果没有目标值，使用零

        # 计算误差
        error = theta - theta_d

        # 保存统计结果
        data['arm_stats'][joint_name] = {
            'mean_error': np.mean(error),
            'mse': np.mean(error**2),
            'max_abs_error': np.max(np.abs(error))
        }

        # 绘制
        axes[i].plot(t, theta_d, **style_desired)
        axes[i].plot(t, theta, **style_actual)

        # 提取关节编号用于标签 (假设名字是 jointN)
        joint_label_num = joint_name.replace('joint', '')
        # 如果不是数字结尾，就用索引+1
        if not joint_label_num.isdigit():
             joint_label_num = str(i + 1)

        axes[i].set_ylabel(r'$\theta_{}$ (rad)'.format(joint_label_num), fontsize=12)
        axes[i].set_xlabel(r'$t$ (s)', fontsize=12) # 所有子图都显示 X 轴标签
        axes[i].tick_params(labelbottom=True)  # 强制显示 X 轴刻度标签
        axes[i].grid(True, linestyle='--', alpha=0.5)

    # 图例放置在图形顶部外部，横向排列
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.999),
               ncol=2, frameon=True, fontsize=11)

    # 调整子图间距
    plt.subplots_adjust(top=0.95, hspace=0.3, wspace=0.25)

    return fig


def plot_position_error(data):
    """
    【图 3】位置跟踪误差 (Position Tracking Error)

    数据逻辑：根据位置数据计算欧几里得范数误差
    e(t) = ||p - p_d|| = sqrt((x-x_d)^2 + (y-y_d)^2 + (z-z_d)^2)

    布局：单张图
    曲线：红色实线
    """
    # ========================================================================
    # 数据准备 - 如需替换为您的数据，请修改此处
    # Data preparation - Replace here with your own data if needed
    # ========================================================================
    t = np.array(data['t_pos'])  # 时间轴 (Time axis)

    # 实际位置 (Actual position)
    x = np.array(data['pos_x'])
    y = np.array(data['pos_y'])
    z = np.array(data['pos_z'])

    # 期望位置 (Desired position) - 插值对齐
    x_d = interp_data(t, data['t_pos_sp'], data['pos_sp_x'])
    y_d = interp_data(t, data['t_pos_sp'], data['pos_sp_y'])
    z_d = interp_data(t, data['t_pos_sp'], data['pos_sp_z'])
    # ========================================================================

    # 计算欧几里得范数误差
    e_t = np.sqrt((x - x_d)**2 + (y - y_d)**2 + (z - z_d)**2)

    # 计算平均误差用于绘制参考线
    mean_error = np.mean(e_t)

    # 创建图形
    fig, ax = plt.subplots(figsize=(8, 5))

    # 绘制误差曲线
    ax.plot(t, e_t, color='red', linestyle='-', linewidth=1.5, label='Position Tracking Error')

    # 绘制平均误差参考线 (仅在图3显示)
    ax.axhline(y=mean_error, color='green', linestyle='--', linewidth=1.5, label='mean value')

    # 设置标签
    ax.set_xlabel(r'$t$ (s)', fontsize=12)
    ax.set_ylabel(r'$e(t)$ (m)', fontsize=12)
    ax.set_title('Position Tracking Error', fontsize=13, fontweight='normal')

    # 网格和图例
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='best', fontsize=11)

    plt.tight_layout()

    return fig


# ============================================================================
# 统计输出函数 (Statistics Output Function)
# ============================================================================

def print_statistics(data):
    """
    在终端打印性能统计指标

    包括：
    1. 各轴独立指标 (x, y, z, 关节1-4)
    2. 综合位置指标 (Combined Position Metrics)
    """
    print("\n" + "="*70)
    print(" 性能统计指标 (Performance Statistics)")
    print("="*70)

    # ========================================================================
    # 1. 位置轴统计 (Position Axes Statistics)
    # ========================================================================
    print("\n【位置轴独立指标】(Independent Position Metrics)")
    print("-"*70)

    axes_names = ['x', 'y', 'z']
    for axis_name in axes_names:
        stats_key = f'stats_{axis_name}'
        if stats_key in data:
            stats = data[stats_key]
            print(f"\n  {axis_name.upper()} 轴:")
            print(f"    平均误差 (Mean Error):        {stats['mean_error']:>10.6f} m")
            # print(f"    均方误差 (MSE):                {stats['mse']:>10.6f} m²")
            print(f"    均方误差 (MSE):                {stats['mse']:>10.6f} m^2")
            print(f"    最大绝对误差 (Max Abs Error):  {stats['max_abs_error']:>10.6f} m")

    # ========================================================================
    # 2. 机械臂关节统计 (Manipulator Joint Statistics)
    # ========================================================================
    if 'arm_stats' in data and len(data['arm_stats']) > 0:
        print("\n【机械臂关节独立指标】(Independent Manipulator Metrics)")
        print("-"*70)

        for joint_name in sorted(data['arm_stats'].keys()):
            stats = data['arm_stats'][joint_name]
            joint_num = joint_name.replace('joint', '')
            print(f"\n  关节 {joint_num} (Joint {joint_num}):")
            print(f"    平均误差 (Mean Error):        {stats['mean_error']:>10.6f} rad")
            # print(f"    均方误差 (MSE):                {stats['mse']:>10.6f} rad²")
            print(f"    均方误差 (MSE):                {stats['mse']:>10.6f} rad^2")
            print(f"    最大绝对误差 (Max Abs Error):  {stats['max_abs_error']:>10.6f} rad")

    # ========================================================================
    # 3. 综合位置指标 (Combined Position Metrics)
    # ========================================================================
    if 'combined_position_mse' in data:
        print("\n【综合位置指标】(Combined Position Metrics - x,y,z)")
        print("-"*70)
        print(f"\n  综合位置均方误差 (Combined Position MSE):")
        # print(f"    Formula: (1/N) Σ||p - p_d||²")
        # print(f"    Value:   {data['combined_position_mse']:>10.6f} m²")
        print(f"    Formula: (1/N) sum||p - p_d||^2")
        print(f"    Value:   {data['combined_position_mse']:>10.6f} m^2")

        print(f"\n  最大位置误差 (Max Position Error):")
        print(f"    Formula: Max ||p - p_d||")
        print(f"    Value:   {data['max_position_error']:>10.6f} m")

    print("\n" + "="*70 + "\n")


# ============================================================================
# 主函数 (Main Function)
# ============================================================================

def main():
    """主函数：解析参数、加载数据、绘制图表"""

    # 命令行参数解析
    parser = argparse.ArgumentParser(
        description='科研风格绘图脚本 - 绘制无人机和机械臂控制性能图表',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  python plot_result_classDesign.py experiment.bag
  python plot_result_classDesign.py experiment.bag --start 5.0
  python plot_result_classDesign.py experiment.bag --save
        """
    )
    parser.add_argument('bagfile', help='输入的 ROS bag 文件路径')
    parser.add_argument('--start', type=float, default=0.0,
                        help='相对于 bag 开始的起始时间 (秒), 默认: 0.0')
    parser.add_argument('--save', action='store_true',
                        help='保存图表为 PDF 文件而非显示')
    args = parser.parse_args()

    # ========================================================================
    # 数据加载
    # ========================================================================
    # 从 bag 文件加载数据
    data = load_data_from_bag(args.bagfile, args.start)

    # ========================================================================
    # 【重要】如需使用自己的数据，请注释掉上面的 load_data_from_bag() 调用，
    # 并按照前面 "数据替换接口" 部分的说明创建 data 字典
    # ========================================================================

    # 检查数据完整性
    if len(data['t_pos']) == 0:
        print("错误: 未找到位置数据！")
        return

    # ========================================================================
    # 绘制图表
    # ========================================================================
    print("\n开始绘制图表...")

    # 图 1: 无人机位置跟踪
    print("绘制图 1: 无人机位置跟踪...")
    fig1 = plot_position_tracking(data)

    # 图 2: 机械臂关节跟踪
    print("绘制图 2: 机械臂关节跟踪...")
    fig2 = plot_manipulator_tracking(data)

    # 图 3: 位置跟踪误差
    print("绘制图 3: 位置跟踪误差...")
    fig3 = plot_position_error(data)

    # ========================================================================
    # 终端统计输出
    # ========================================================================
    print_statistics(data)

    # ========================================================================
    # 保存或显示图表
    # ========================================================================
    if args.save:
        print("\n保存图表为 PDF 文件...")
        if fig1:
            fig1.savefig('figure1_position_tracking.pdf', dpi=300, bbox_inches='tight')
            print("  - 已保存: figure1_position_tracking.pdf")
        if fig2:
            fig2.savefig('figure2_manipulator_tracking.pdf', dpi=300, bbox_inches='tight')
            print("  - 已保存: figure2_manipulator_tracking.pdf")
        if fig3:
            fig3.savefig('figure3_position_error.pdf', dpi=300, bbox_inches='tight')
            print("  - 已保存: figure3_position_error.pdf")
        print("完成！")
    else:
        print("\n显示图表...")
        plt.show()


if __name__ == "__main__":
    main()
