#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import JointState
from mavros_msgs.msg import Mavlink
from mavros.mavlink import convert_to_rosmsg
from pymavlink.dialects.v10 import common as mavlink1


class UamV5ArmJointStateBridge:
    REQUIRED_JOINTS = ("arm_joint1", "arm_joint2", "left_hand_joint")
    SOURCE_SYSTEM_ID = 1
    SOURCE_COMPONENT_ID = 191

    def __init__(self):
        namespace = rospy.get_param("~namespace", "uav_arm").strip("/")
        self.joint_states_topic = rospy.get_param(
            "~joint_states_topic", f"/{namespace}/joint_states"
        )
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 200.0))
        self.timeout = rospy.Duration.from_sec(
            float(rospy.get_param("~joint_state_timeout", 0.2))
        )
        self.mavlink_topic = rospy.get_param("~mavlink_topic", "/mavlink/to")
        self.position_offsets = {
            "arm_joint1": float(rospy.get_param("~arm_joint1_position_offset", -math.pi)),
            "arm_joint2": float(rospy.get_param("~arm_joint2_position_offset", 1.5 * math.pi)),
            "left_hand_joint": float(rospy.get_param("~left_hand_position_offset", 0.0)),
        }

        self._mavlink = mavlink1.MAVLink(
            None,
            srcSystem=self.SOURCE_SYSTEM_ID,
            srcComponent=self.SOURCE_COMPONENT_ID,
        )
        self._publisher = rospy.Publisher(self.mavlink_topic, Mavlink, queue_size=100)
        self._subscriber = rospy.Subscriber(
            self.joint_states_topic, JointState, self._joint_state_cb, queue_size=50
        )

        self._latest_stamp = None
        self._joint_positions = {name: 0.0 for name in self.REQUIRED_JOINTS}
        self._joint_velocities = {name: 0.0 for name in self.REQUIRED_JOINTS}
        self._joint_state_complete = False
        self._message_keys = (
            "AJQ0",
            "AJQ1",
            "AJQ2",
            "AJQ3",
            "AJD0",
            "AJD1",
            "AJD2",
            "AJD3",
            "AJVAL",
        )
        self._message_index = 0

    def _joint_state_cb(self, msg: JointState) -> None:
        joint_to_index = {name: idx for idx, name in enumerate(msg.name)}
        self._joint_state_complete = all(
            joint in joint_to_index for joint in self.REQUIRED_JOINTS
        )

        if not self._joint_state_complete:
            return

        for joint in self.REQUIRED_JOINTS:
            idx = joint_to_index[joint]
            raw_position = msg.position[idx] if idx < len(msg.position) else 0.0
            raw_velocity = msg.velocity[idx] if idx < len(msg.velocity) else 0.0

            if not math.isfinite(raw_position) or not math.isfinite(raw_velocity):
                self._joint_state_complete = False
                return

            self._joint_positions[joint] = raw_position - self.position_offsets[joint]
            self._joint_velocities[joint] = raw_velocity

        self._latest_stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    def _joint_state_fresh(self, now: rospy.Time) -> bool:
        if not self._joint_state_complete or self._latest_stamp is None:
            return False

        return (now - self._latest_stamp) <= self.timeout

    def _publish_named_value(self, key: str, value: float, now: rospy.Time) -> None:
        if not math.isfinite(value):
            value = 0.0
        time_boot_ms = int(now.to_sec() * 1000.0) & 0xFFFFFFFF
        mav_msg = self._mavlink.named_value_float_encode(
            time_boot_ms, key.encode("ascii"), float(value)
        )
        mav_msg.pack(self._mavlink)
        self._publisher.publish(convert_to_rosmsg(mav_msg, stamp=now))

    def spin(self) -> None:
        rate = rospy.Rate(self.publish_rate_hz)
        rospy.loginfo(
            "uam_v5 arm joint state bridge publishing %s -> %s",
            self.joint_states_topic,
            self.mavlink_topic,
        )

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            fresh = self._joint_state_fresh(now)

            values = {
                "AJQ0": self._joint_positions["arm_joint1"],
                "AJQ1": self._joint_positions["arm_joint2"],
                "AJQ2": self._joint_positions["left_hand_joint"],
                "AJQ3": 0.0,
                "AJD0": self._joint_velocities["arm_joint1"],
                "AJD1": self._joint_velocities["arm_joint2"],
                "AJD2": self._joint_velocities["left_hand_joint"],
                "AJD3": 0.0,
                "AJVAL": 1.0 if fresh else 0.0,
            }

            key = self._message_keys[self._message_index]
            self._publish_named_value(key, values[key], now)
            self._message_index = (self._message_index + 1) % len(self._message_keys)

            rate.sleep()


def main() -> None:
    rospy.init_node("uam_v5_arm_joint_state_bridge")
    bridge = UamV5ArmJointStateBridge()
    bridge.spin()


if __name__ == "__main__":
    main()
