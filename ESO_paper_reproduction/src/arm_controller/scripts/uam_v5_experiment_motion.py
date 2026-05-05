#!/usr/bin/env python3
"""
Altitude-triggered 0.5 Hz motion script for the uam_v5 arm experiments.
"""

import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


class UamV5ExperimentMotion:
    def __init__(self):
        self.namespace = rospy.get_param("~namespace", "uav_arm").strip("/")
        self.enable_topic = rospy.get_param("~enable_topic", "/experiment/arm_motion_enabled")
        self.command_rate_hz = float(rospy.get_param("~command_rate_hz", 50.0))
        self.frequency_hz = float(rospy.get_param("~frequency_hz", 0.5))
        self.arm_joint1_offset = float(rospy.get_param("~arm_joint1_offset", 0.0))
        self.arm_joint1_amplitude = float(rospy.get_param("~arm_joint1_amplitude", 0.35))
        self.arm_joint2_offset = float(rospy.get_param("~arm_joint2_offset", 0.0))
        self.arm_joint2_amplitude = float(rospy.get_param("~arm_joint2_amplitude", 0.35))
        self.left_hand_offset = float(rospy.get_param("~left_hand_offset", 0.0))
        self.left_hand_amplitude = float(rospy.get_param("~left_hand_amplitude", 0.003))

        self.joints = ["arm_joint1", "arm_joint2", "left_hand_joint"]
        self.publishers = {
            joint: rospy.Publisher(
                f"/{self.namespace}/{joint}_position_controller/command",
                Float64,
                queue_size=10,
            )
            for joint in self.joints
        }
        self.target_joint_state_pub = rospy.Publisher(
            f"/{self.namespace}/target_joint_states", JointState, queue_size=10
        )

        self.motion_enabled = False
        self.motion_start_time = None
        self.have_joint_state = False

        rospy.Subscriber(
            f"/{self.namespace}/joint_states",
            JointState,
            self._joint_state_cb,
            queue_size=50,
        )
        rospy.Subscriber(self.enable_topic, Bool, self._enable_cb, queue_size=1)

    def _joint_state_cb(self, _msg):
        self.have_joint_state = True

    def _enable_cb(self, msg):
        if msg.data and not self.motion_enabled:
            self.motion_enabled = True
            self.motion_start_time = rospy.Time.now()
            rospy.loginfo("Received arm motion enable signal on %s", self.enable_topic)

    def _wait_for_joint_states(self, timeout_s=10.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            if self.have_joint_state:
                return True
            if rospy.Time.now() >= deadline:
                return False
            rate.sleep()
        return False

    def _publish_positions(self, positions):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joints
        msg.position = [positions[joint] for joint in self.joints]
        self.target_joint_state_pub.publish(msg)

        for joint, value in positions.items():
            self.publishers[joint].publish(Float64(value))

    def _neutral_pose(self):
        return {
            "arm_joint1": self.arm_joint1_offset,
            "arm_joint2": self.arm_joint2_offset,
            "left_hand_joint": self.left_hand_offset,
        }

    def _motion_pose(self, elapsed_s):
        phase = 2.0 * math.pi * self.frequency_hz * elapsed_s
        return {
            "arm_joint1": self.arm_joint1_offset + self.arm_joint1_amplitude * math.sin(phase),
            "arm_joint2": self.arm_joint2_offset - self.arm_joint2_amplitude * math.sin(phase),
            "left_hand_joint": self.left_hand_offset + self.left_hand_amplitude * math.sin(phase),
        }

    def run(self):
        if not self._wait_for_joint_states():
            raise RuntimeError("Timed out waiting for /joint_states")

        rospy.loginfo("uam_v5 experiment motion ready, waiting for %s", self.enable_topic)
        rate = rospy.Rate(self.command_rate_hz)

        while not rospy.is_shutdown():
            if self.motion_enabled and self.motion_start_time is not None:
                elapsed_s = (rospy.Time.now() - self.motion_start_time).to_sec()
                positions = self._motion_pose(elapsed_s)
            else:
                positions = self._neutral_pose()

            self._publish_positions(positions)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("uam_v5_experiment_motion")
    UamV5ExperimentMotion().run()
