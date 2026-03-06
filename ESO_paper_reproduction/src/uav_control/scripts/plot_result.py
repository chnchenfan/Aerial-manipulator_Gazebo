
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import argparse
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu, JointState
from mavros_msgs.msg import DebugValue
import math
import sys

# 设置中文字体（根据系统环境可能需要调整，这里尝试通用设置）
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial']  # 优先使用黑体
plt.rcParams['axes.unicode_minus'] = False # 解决负号显示问题

def quark_to_euler(q):
    """
    将四元数转换为欧拉角 (Roll, Pitch, Yaw)
    q: [w, x, y, z] 或 具有 w, x, y, z 属性的对象
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

def calculate_mse(actual, target):
    """计算均方误差 (MSE)"""
    # 确保长度一致
    min_len = min(len(actual), len(target))
    a = np.array(actual[:min_len])
    t = np.array(target[:min_len])
    return np.mean((a - t)**2)

def interp_data(t_target, t_source, source_data):
    """将源数据插值对齐到目标时间戳"""
    return np.interp(t_target, t_source, source_data)

def main():
    parser = argparse.ArgumentParser(description='从 ROS bag 文件绘制 ESO 结果')
    parser.add_argument('bagfile', help='输入的 ROS bag 文件路径')
    parser.add_argument('--start', type=float, default=0.0, help='相对于 bag 开始的起始时间 (秒)')
    args = parser.parse_args()

    print(f"正在打开 bag 文件: {args.bagfile}")
    bag = rosbag.Bag(args.bagfile)

    # 数据容器
    data = {
        't_pos': [], 'pos_x': [], 'pos_y': [], 'pos_z': [],
        't_pos_sp': [], 'pos_sp_x': [], 'pos_sp_y': [], 'pos_sp_z': [],
        't_att': [], 'roll': [], 'pitch': [], 'yaw': [],
        't_att_sp': [], 'roll_sp': [], 'pitch_sp': [], 'yaw_sp': [], # 来自桥接的 Key-Value
        't_rate': [], 'wx': [], 'wy': [], 'wz': [],
        't_eso_pos': [], 'eso_px': [], 'eso_py': [], 'eso_pz': [], 'eso_dx': [], 'eso_dy': [], 'eso_dz': [],
        't_eso_rate': [], 'eso_wx': [], 'eso_wy': [], 'eso_wz': [], 'eso_tx': [], 'eso_ty': [], 'eso_tz': [],
        't_arm': [], 'arm_actual': {}, 'arm_target': {} # 关节名称字典
    }

    # ESO Key-Value 解析辅助字典
    # 使用字典缓存同步数据，这里简单处理：分别存储并通过插值对齐
    eso_raw = {}
    for key in ['ESO_PX', 'ESO_PY', 'ESO_PZ', 'ESO_DX', 'ESO_DY', 'ESO_DZ',
                'ESO_WX', 'ESO_WY', 'ESO_WZ', 'ESO_TX', 'ESO_TY', 'ESO_TZ',
                'AT_R', 'AT_P', 'AT_Y']:
        eso_raw[key] = {'t': [], 'v': []}

    start_time_abs = None

    # 读取话题消息
    print("正在读取消息...")
    for topic, msg, t in bag.read_messages():
        if start_time_abs is None:
            start_time_abs = t.to_sec()

        current_time = t.to_sec() - start_time_abs
        if current_time < args.start:
            continue

        # 1. 位置 & 姿态 (实际值)
        if topic == '/mavros/local_position/pose':
            data['t_pos'].append(current_time)
            data['pos_x'].append(msg.pose.position.x)
            data['pos_y'].append(msg.pose.position.y)
            data['pos_z'].append(msg.pose.position.z)

            # 姿态时间戳可能略有不同，但通常是同一个消息包
            data['t_att'].append(current_time)
            r, p, y = quark_to_euler(msg.pose.orientation)
            data['roll'].append(r)
            data['pitch'].append(p)
            data['yaw'].append(y)

        # 2. 位置设定点 (目标值)
        elif topic == '/mavros/setpoint_position/local':
            data['t_pos_sp'].append(current_time)
            data['pos_sp_x'].append(msg.pose.position.x)
            data['pos_sp_y'].append(msg.pose.position.y)
            data['pos_sp_z'].append(msg.pose.position.z)

        # 3. 角速度 (实际值)
        elif topic == '/mavros/imu/data':
             data['t_rate'].append(current_time)
             data['wx'].append(msg.angular_velocity.x)
             data['wy'].append(msg.angular_velocity.y)
             data['wz'].append(msg.angular_velocity.z)

        # 4. ESO 调试数据 & 目标姿态 (桥接数据)
        elif topic == '/mavros/debug/named_value_float':
            k = msg.key
            v = msg.value
            if k in eso_raw:
                eso_raw[k]['t'].append(current_time)
                eso_raw[k]['v'].append(v)

        # 5. 机械臂状态
        elif topic == '/uav_arm/joint_states':
            data['t_arm'].append(current_time)
            for i, name in enumerate(msg.name):
                if name not in data['arm_actual']:
                    data['arm_actual'][name] = []
                data['arm_actual'][name].append(msg.position[i])

        elif topic == '/uav_arm/target_joint_states':
             # 注意：目标关节状态的解析结构取决于发布方式
             # 假设是标准的 JointState
             for i, name in enumerate(msg.name):
                if name not in data['arm_target']:
                    data['arm_target'][name] = []
                    data['arm_target'][name + '_t'] = [] # 目标值的时间戳可能不同
                data['arm_target'][name].append(msg.position[i])
                data['arm_target'][name + '_t'].append(current_time)

    bag.close()
    print("数据读取完成。")

    # 后处理：插值对齐时间轴
    # 以实际位置时间轴 t_pos 为基准
    if len(data['t_pos']) == 0:
        print("错误: 未找到位置数据！")
        return

    # 处理桥接数据

    # 目标姿态 (Target Attitude)
    if len(eso_raw['AT_R']['t']) > 0:
        data['t_att_sp'] = eso_raw['AT_R']['t'] # 以 Roll 目标的时间为准
        data['roll_sp'] = eso_raw['AT_R']['v']
        data['pitch_sp'] = np.interp(data['t_att_sp'], eso_raw['AT_P']['t'], eso_raw['AT_P']['v'])
        data['yaw_sp'] = np.interp(data['t_att_sp'], eso_raw['AT_Y']['t'], eso_raw['AT_Y']['v'])

    # ESO 位置估计 (ESO Position)
    if len(eso_raw['ESO_PX']['t']) > 0:
        data['t_eso_pos'] = eso_raw['ESO_PX']['t']
        data['eso_px'] = eso_raw['ESO_PX']['v']
        data['eso_py'] = np.interp(data['t_eso_pos'], eso_raw['ESO_PY']['t'], eso_raw['ESO_PY']['v'])
        data['eso_pz'] = np.interp(data['t_eso_pos'], eso_raw['ESO_PZ']['t'], eso_raw['ESO_PZ']['v'])
        data['eso_dx'] = np.interp(data['t_eso_pos'], eso_raw['ESO_DX']['t'], eso_raw['ESO_DX']['v'])
        data['eso_dy'] = np.interp(data['t_eso_pos'], eso_raw['ESO_DY']['t'], eso_raw['ESO_DY']['v'])
        data['eso_dz'] = np.interp(data['t_eso_pos'], eso_raw['ESO_DZ']['t'], eso_raw['ESO_DZ']['v'])

    # ESO 角速度估计 (ESO Rate)
    if len(eso_raw['ESO_WX']['t']) > 0:
        data['t_eso_rate'] = eso_raw['ESO_WX']['t']
        data['eso_wx'] = eso_raw['ESO_WX']['v']
        data['eso_wy'] = np.interp(data['t_eso_rate'], eso_raw['ESO_WY']['t'], eso_raw['ESO_WY']['v'])
        data['eso_wz'] = np.interp(data['t_eso_rate'], eso_raw['ESO_WZ']['t'], eso_raw['ESO_WZ']['v'])
        data['eso_tx'] = np.interp(data['t_eso_rate'], eso_raw['ESO_TX']['t'], eso_raw['ESO_TX']['v'])
        data['eso_ty'] = np.interp(data['t_eso_rate'], eso_raw['ESO_TY']['t'], eso_raw['ESO_TY']['v'])
        data['eso_tz'] = np.interp(data['t_eso_rate'], eso_raw['ESO_TZ']['t'], eso_raw['ESO_TZ']['v'])


    # --- 绘图 ---
    fig_idx = 1

    # Figure 1: 高度跟踪 (Z vs Zd)
    plt.figure(fig_idx, figsize=(10, 6))
    plt.plot(data['t_pos'], data['pos_z'], label='实际高度 (Actual Z)', linewidth=2)
    plt.plot(data['t_pos_sp'], data['pos_sp_z'], label='目标高度 (Target Z)', linestyle='--', linewidth=2)

    # 计算 Z 轴 MSE
    z_interp = interp_data(data['t_pos'], data['t_pos_sp'], data['pos_sp_z'])
    mse_z = calculate_mse(data['pos_z'], z_interp)
    plt.title(f'图1: 高度跟踪 (MSE: {mse_z:.4f})')
    plt.xlabel('时间 (s)')
    plt.ylabel('高度 (m)')
    plt.legend()
    plt.grid(True)
    fig_idx += 1

    # Figure 2: 平面位置跟踪 (XY Tracking)
    plt.figure(fig_idx, figsize=(10, 6))
    plt.plot(data['t_pos'], data['pos_x'], label='实际 X')
    plt.plot(data['t_pos_sp'], data['pos_sp_x'], label='目标 X', linestyle='--')
    plt.plot(data['t_pos'], data['pos_y'], label='实际 Y')
    plt.plot(data['t_pos_sp'], data['pos_sp_y'], label='目标 Y', linestyle='--')

    x_interp = interp_data(data['t_pos'], data['t_pos_sp'], data['pos_sp_x'])
    y_interp = interp_data(data['t_pos'], data['t_pos_sp'], data['pos_sp_y'])
    mse_x = calculate_mse(data['pos_x'], x_interp)
    mse_y = calculate_mse(data['pos_y'], y_interp)

    plt.title(f'图2: XY 位置跟踪 (MSE X: {mse_x:.4f}, MSE Y: {mse_y:.4f})')
    plt.xlabel('时间 (s)')
    plt.ylabel('位置 (m)')
    plt.legend()
    plt.grid(True)
    fig_idx += 1

    # Figure 3: 综合位置误差 (Overall Position Error)
    # e(t) = ||p - pd||
    # 插值对齐
    sp_x_aligned = interp_data(data['t_pos'], data['t_pos_sp'], data['pos_sp_x'])
    sp_y_aligned = interp_data(data['t_pos'], data['t_pos_sp'], data['pos_sp_y'])
    sp_z_aligned = interp_data(data['t_pos'], data['t_pos_sp'], data['pos_sp_z'])

    pos_err = np.sqrt(
        (np.array(data['pos_x']) - sp_x_aligned)**2 +
        (np.array(data['pos_y']) - sp_y_aligned)**2 +
        (np.array(data['pos_z']) - sp_z_aligned)**2
    )
    avg_pos_err = np.mean(pos_err)

    plt.figure(fig_idx, figsize=(10, 6))
    plt.plot(data['t_pos'], pos_err, label='位置误差模 e(t)')
    plt.axhline(y=avg_pos_err, color='r', linestyle='--', label=f'平均误差: {avg_pos_err:.4f}m')
    plt.title(f'图3: 综合位置误差 e=||p-pd|| (平均: {avg_pos_err:.4f}m)')
    plt.xlabel('时间 (s)')
    plt.ylabel('误差 (m)')
    plt.legend()
    plt.grid(True)
    fig_idx += 1

    # Figure 4: 姿态跟踪 (Attitude Tracking)
    plt.figure(fig_idx, figsize=(12, 8))

    # 插值对齐目标姿态
    r_sp_aligned = interp_data(data['t_att'], data['t_att_sp'], data['roll_sp'])
    p_sp_aligned = interp_data(data['t_att'], data['t_att_sp'], data['pitch_sp'])
    y_sp_aligned = interp_data(data['t_att'], data['t_att_sp'], data['yaw_sp'])

    mse_r = calculate_mse(data['roll'], r_sp_aligned)
    mse_p = calculate_mse(data['pitch'], p_sp_aligned)
    mse_y_val = calculate_mse(data['yaw'], y_sp_aligned)

    plt.subplot(3, 1, 1)
    plt.plot(data['t_att'], np.degrees(data['roll']), label='实际 Roll')
    plt.plot(data['t_att'], np.degrees(r_sp_aligned), label='目标 Roll', linestyle='--')
    plt.title(f'滚转角 Roll (MSE: {mse_r:.4f})')
    plt.ylabel('角度 (deg)')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(data['t_att'], np.degrees(data['pitch']), label='实际 Pitch')
    plt.plot(data['t_att'], np.degrees(p_sp_aligned), label='目标 Pitch', linestyle='--')
    plt.title(f'俯仰角 Pitch (MSE: {mse_p:.4f})')
    plt.ylabel('角度 (deg)')
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(data['t_att'], np.degrees(data['yaw']), label='实际 Yaw')
    plt.plot(data['t_att'], np.degrees(y_sp_aligned), label='目标 Yaw', linestyle='--')
    plt.title(f'偏航角 Yaw (MSE: {mse_y_val:.4f})')
    plt.ylabel('角度 (deg)')
    plt.grid(True)
    plt.legend()

    plt.suptitle('图4: 姿态跟踪 (度)')
    fig_idx += 1

    # Figure 5: ESO 估计性能
    plt.figure(fig_idx, figsize=(12, 10))

    # 5.1: ESO Pos X vs Actual X
    if len(data['t_eso_pos']) > 0:
        actual_x_aligned = interp_data(data['t_eso_pos'], data['t_pos'], data['pos_x'])
        mse_eso_x = calculate_mse(data['eso_px'], actual_x_aligned)

        plt.subplot(2, 2, 1)
        plt.plot(data['t_eso_pos'], data['eso_px'], label='估计 X (Est)')
        plt.plot(data['t_eso_pos'], actual_x_aligned, label='实际 X', linestyle='--')
        plt.title(f'ESO 位置估计 X (MSE: {mse_eso_x:.4f})')
        plt.grid(True)
        plt.legend()

    # 5.2: ESO Rate X vs Actual Rate X
    if len(data['t_eso_rate']) > 0:
        actual_wx_aligned = interp_data(data['t_eso_rate'], data['t_rate'], data['wx'])
        mse_eso_wx = calculate_mse(data['eso_wx'], actual_wx_aligned)

        plt.subplot(2, 2, 2)
        plt.plot(data['t_eso_rate'], data['eso_wx'], label='估计 Wx')
        plt.plot(data['t_eso_rate'], actual_wx_aligned, label='实际 Wx', linestyle='--')
        plt.title(f'ESO 角速度估计 X (MSE: {mse_eso_wx:.4f})')
        plt.grid(True)
        plt.legend()

    # 5.3 ESO 扰动估计 (位置/力相关)
    if len(data['t_eso_pos']) > 0:
        plt.subplot(2, 2, 3)
        plt.plot(data['t_eso_pos'], data['eso_dx'], label='扰动 X')
        plt.plot(data['t_eso_pos'], data['eso_dy'], label='扰动 Y')
        plt.plot(data['t_eso_pos'], data['eso_dz'], label='扰动 Z')
        plt.title('ESO 总扰动估计 (加速度级)')
        plt.grid(True)
        plt.legend()

    # 5.4 ESO 扰动估计 (力矩相关)
    if len(data['t_eso_rate']) > 0:
        plt.subplot(2, 2, 4)
        plt.plot(data['t_eso_rate'], data['eso_tx'], label='力矩 X')
        plt.plot(data['t_eso_rate'], data['eso_ty'], label='力矩 Y')
        plt.plot(data['t_eso_rate'], data['eso_tz'], label='力矩 Z')
        plt.title('ESO 总扰动估计 (力矩级)')
        plt.grid(True)
        plt.legend()

    plt.suptitle('图5: ESO 估计性能')
    fig_idx += 1

    # Figure 6: 机械臂关节
    if len(data['arm_actual']) > 0:
        num_joints = len(data['arm_actual'])
        plt.figure(fig_idx, figsize=(10, 4 * num_joints))

        idx = 1
        for name, values in data['arm_actual'].items():
            plt.subplot(num_joints, 1, idx)
            plt.plot(data['t_arm'], values, label=f'实际 {name}')

            if name in data['arm_target']:
                target_vals = data['arm_target'][name]
                target_times = data['arm_target'][name + '_t']
                plt.plot(target_times, target_vals, label=f'目标 {name}', linestyle='--')

                # Calculate MSE
                t_aligned = interp_data(data['t_arm'], target_times, target_vals)
                mse_arm = calculate_mse(values, t_aligned)
                plt.title(f'{name} 跟踪 (MSE: {mse_arm:.4f})')
            else:
                plt.title(f'{name} (无目标值)')

            plt.grid(True)
            plt.legend()
            idx += 1

        plt.suptitle('图6: 机械臂关节跟踪')
        plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
