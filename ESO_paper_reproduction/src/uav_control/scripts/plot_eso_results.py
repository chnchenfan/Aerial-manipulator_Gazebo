#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import argparse

def plot_bag_data(bag_filename):
    # 1. 准备容器装数据
    time_real = []
    pos_real_x = []

    time_eso = []
    pos_eso_x = []

    time_dist = []
    dist_x = []

    print(f"正在读取文件: {bag_filename} ...")

    # 2. 打开 bag 文件读取数据
    with rosbag.Bag(bag_filename, 'r') as bag:
        # 读取真实位置 (MAVROS)
        for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/pose']):
            time_real.append(msg.header.stamp.to_sec())
            pos_real_x.append(msg.pose.position.x)

        # 读取 ESO 估计位置
        for topic, msg, t in bag.read_messages(topics=['/eso/debug/position']):
            time_eso.append(msg.header.stamp.to_sec())
            pos_eso_x.append(msg.vector.x)

        # 读取 ESO 估计干扰
        for topic, msg, t in bag.read_messages(topics=['/eso/debug/disturbance']):
            time_dist.append(msg.header.stamp.to_sec())
            dist_x.append(msg.vector.x)

    # 3. 时间对齐 (把第一帧作为 0 秒)
    if not time_real:
        print("错误：没读到数据！请检查话题名称是否正确。")
        return

    start_time = time_real[0]
    time_real = [t - start_time for t in time_real]

    if time_eso:
        start_time_eso = time_eso[0]
        time_eso = [t - start_time_eso for t in time_eso]

    if time_dist:
        start_time_dist = time_dist[0]
        time_dist = [t - start_time_dist for t in time_dist]

    print("数据读取完毕，开始画图...")

    # 4. 开始画图 (创建两个子图)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # --- 图 1: 位置对比 ---
    ax1.plot(time_real, pos_real_x, label='Real Position (X)', color='black', linewidth=2)
    if time_eso:
        ax1.plot(time_eso, pos_eso_x, label='ESO Estimate (X)', color='red', linestyle='--', linewidth=2)
    ax1.set_ylabel('Position X (m)')
    ax1.set_title('ESO Estimation Performance')
    ax1.legend()
    ax1.grid(True)

    # --- 图 2: 干扰估计 ---
    if time_dist:
        ax2.plot(time_dist, dist_x, label='Estimated Disturbance (X)', color='blue')
        ax2.set_ylabel('Disturbance Acc (m/s^2)')
        ax2.set_xlabel('Time (s)')
        ax2.legend()
        ax2.grid(True)

        # 可以在这里标出你要展示的重点
        # ax2.annotate('Arm Moving', xy=(10, 1.0), xytext=(12, 2.0),
        #             arrowprops=dict(facecolor='black', shrink=0.05))

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 设置默认读取的文件名
    parser = argparse.ArgumentParser(description='Plot ESO data from rosbag')
    parser.add_argument('bagfile', nargs='?', default='eso_test_01.bag', help='Input bag file name')
    args = parser.parse_args()

    try:
        plot_bag_data(args.bagfile)
    except FileNotFoundError:
        print(f"错误：找不到文件 {args.bagfile}。请确认你已经在终端里录制了数据。")
