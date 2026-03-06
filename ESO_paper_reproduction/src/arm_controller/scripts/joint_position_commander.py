#!/usr/bin/env python3
"""
Improved helper node: Uses Linear Interpolation (Ramp) to move joints smoothly.
Prevents physics explosions caused by sudden torque spikes.
"""
import itertools
import time
from typing import Dict, List
import numpy as np  # 需要安装 numpy

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState  # 用于发布目标关节状态

class JointCommander:
    def __init__(self):
        self.namespace = rospy.get_param("~namespace", "uav_arm")
        # 动作之间的停留时间
        self.hold_time = 3.0
        # 动作过渡时间（越长越柔和，建议设为 2.0 到 5.0 秒）
        self.move_duration = 3.0

        self.sequence: List[Dict[str, float]] = [
            {
                "arm_joint1": 0.0, "arm_joint2": 0.0, "arm_joint3": 0.0,
                "arm_joint4": 0.0, "left_gripper_joint": 0.05
            },
            {
                "arm_joint1": 0.3, "arm_joint2": -0.2, "arm_joint3": 0.3,
                "arm_joint4": -0.15, "left_gripper_joint": 0.05
            },
            {
                "arm_joint1": -0.3, "arm_joint2": 0.2, "arm_joint3": -0.3,
                "arm_joint4": 0.15, "left_gripper_joint": 0.05
            },
        ]

        self.joints = [
            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "left_gripper_joint"
        ]

        self.publishers: Dict[str, rospy.Publisher] = {}
        for joint in self.joints:
            topic = f"/{self.namespace}/{joint}_position_controller/command"
            self.publishers[joint] = rospy.Publisher(topic, Float64, queue_size=10)

        # 【新增】发布目标关节状态话题（用于可视化和调试）
        self.target_joint_state_pub = rospy.Publisher(
            f"/{self.namespace}/target_joint_states", JointState, queue_size=10
        )

        # 记录当前各关节的位置（假设初始都是0）
        self.current_positions = {j: 0.0 for j in self.joints}

        rospy.loginfo("Smooth JointCommander ready!")

    def publish_target_joint_state(self, positions: Dict[str, float]):
        """发布目标关节状态消息"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(positions.keys())
        msg.position = list(positions.values())
        self.target_joint_state_pub.publish(msg)


    def publish_positions(self, positions: Dict[str, float]):
        """发送一组指令"""
        for joint, val in positions.items():
            if joint in self.publishers:
                self.publishers[joint].publish(Float64(val))

    def move_smoothly(self, target_positions: Dict[str, float]):
        """线性插值平滑移动"""
        rospy.loginfo(f"Moving smoothly to: {target_positions}")

        start_positions = self.current_positions.copy()
        steps = int(self.move_duration * 50) # 50Hz 的控制频率

        rate = rospy.Rate(50)

        for i in range(steps):
            if rospy.is_shutdown(): break

            alpha = (i + 1) / steps # 进度 0.0 ~ 1.0
            interp_pos = {}

            for joint in self.joints:
                start = start_positions.get(joint, 0.0)
                end = target_positions.get(joint, 0.0)
                # 线性插值公式：当前 = 起点 + 进度 * (终点 - 起点)
                interp_pos[joint] = start + alpha * (end - start)

            self.publish_positions(interp_pos)
            self.publish_target_joint_state(interp_pos)  # 发布当前插值目标
            rate.sleep()

        # 更新当前位置记录
        self.current_positions = target_positions.copy()

    def run(self):
        # 先发一次0位，锁住机械臂
        self.publish_positions(self.current_positions)
        rospy.sleep(2.0)

        sequence_iter = itertools.cycle(self.sequence)

        while not rospy.is_shutdown():
            # 1. 获取下一个目标
            target = next(sequence_iter)

            # 2. 平滑移动过去 (Move)
            self.move_smoothly(target)

            # 3. 停在原地保持一会儿 (Hold)
            rospy.loginfo(f"Holding for {self.hold_time}s...")
            start_hold = time.time()
            while time.time() - start_hold < self.hold_time:
                if rospy.is_shutdown(): break
                # 持续发送当前目标，保持力矩
                self.publish_positions(target)
                rospy.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node("uav_arm_smooth_commander")
    commander = JointCommander()
    commander.run()
