#!/usr/bin/env python3
"""
ROS bag recorder for the uam_v5 flight-manipulator experiments.
"""

import json
import os
import threading
from datetime import datetime
from pathlib import Path

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32


class ExperimentDataRecorder:
    def __init__(self):
        self.namespace = rospy.get_param("~namespace", "uav_arm").strip("/")
        self.experiment_name = rospy.get_param(
            "~experiment_name", "uam_v5_experiment"
        )
        default_output_dir = (
            Path(__file__).resolve().parents[2] / "uav_arm_top" / "data"
        )
        self.output_dir = Path(
            os.path.expanduser(
                rospy.get_param("~output_dir", str(default_output_dir))
            )
        ).resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.output_dir / stamp
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.bag_path = str(self.run_dir / f"{self.experiment_name}.bag")
        self.metadata_path = str(self.run_dir / f"{self.experiment_name}_metadata.json")

        self._lock = threading.Lock()
        self._closed = False
        self._message_counts = {}
        self._bag = rosbag.Bag(self.bag_path, "w")
        self._start_wall_time = datetime.now().isoformat()
        self._first_bag_time = None
        self._analysis_start_time_s = None

        joint_states_topic = f"/{self.namespace}/joint_states"
        target_joint_states_topic = f"/{self.namespace}/target_joint_states"
        self.record_arm_topics = rospy.get_param("~record_arm_topics", True)
        self.record_state_topic = rospy.get_param("~record_state_topic", True)
        self.record_motor_speed_topics = rospy.get_param("~record_motor_speed_topics", False)

        self._subscriptions = [
            rospy.Subscriber(
                "/mavros/local_position/pose",
                PoseStamped,
                self._make_callback("/mavros/local_position/pose"),
                queue_size=200,
            ),
            rospy.Subscriber(
                "/mavros/setpoint_position/local",
                PoseStamped,
                self._make_callback("/mavros/setpoint_position/local"),
                queue_size=200,
            ),
            rospy.Subscriber(
                "/mavros/setpoint_raw/local",
                PositionTarget,
                self._make_callback("/mavros/setpoint_raw/local"),
                queue_size=200,
            ),
            rospy.Subscriber(
                "/experiment/arm_motion_enabled",
                Bool,
                self._make_callback("/experiment/arm_motion_enabled"),
                queue_size=10,
            ),
        ]
        if self.record_arm_topics:
            self._subscriptions.extend([
                rospy.Subscriber(
                    joint_states_topic,
                    JointState,
                    self._make_callback(joint_states_topic),
                    queue_size=100,
                ),
                rospy.Subscriber(
                    target_joint_states_topic,
                    JointState,
                    self._make_callback(target_joint_states_topic),
                    queue_size=100,
                ),
            ])
        if self.record_state_topic:
            self._subscriptions.append(
                rospy.Subscriber(
                    "/mavros/state",
                    State,
                    self._make_callback("/mavros/state"),
                    queue_size=20,
                )
            )
        if self.record_motor_speed_topics:
            for motor_index in range(4):
                motor_speed_topic = f"/motor_speed/{motor_index}"
                self._subscriptions.append(
                    rospy.Subscriber(
                        motor_speed_topic,
                        Float32,
                        self._make_callback(motor_speed_topic),
                        queue_size=200,
                    )
                )

        rospy.on_shutdown(self.close)
        rospy.loginfo("Recording experiment data to %s", self.bag_path)
        rospy.loginfo("Run directory: %s", self.run_dir)

    def _make_callback(self, topic_name):
        def _callback(msg):
            self._write(topic_name, msg)

        return _callback

    def _write(self, topic_name, msg):
        with self._lock:
            if self._closed:
                return
            bag_time = rospy.Time.now()
            if self._first_bag_time is None:
                self._first_bag_time = bag_time
            if (
                topic_name == "/experiment/arm_motion_enabled"
                and getattr(msg, "data", False)
                and self._analysis_start_time_s is None
            ):
                self._analysis_start_time_s = (
                    bag_time - self._first_bag_time
                ).to_sec()
            self._bag.write(topic_name, msg, t=bag_time)
            self._message_counts[topic_name] = self._message_counts.get(topic_name, 0) + 1

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._bag.close()

        metadata = {
            "experiment_name": self.experiment_name,
            "namespace": self.namespace,
            "bag_path": self.bag_path,
            "started_at": self._start_wall_time,
            "stopped_at": datetime.now().isoformat(),
            "message_counts": self._message_counts,
            "analysis_start_time_s": self._analysis_start_time_s,
            "record_arm_topics": self.record_arm_topics,
            "record_state_topic": self.record_state_topic,
            "record_motor_speed_topics": self.record_motor_speed_topics,
        }
        with open(self.metadata_path, "w", encoding="utf-8") as meta_file:
            json.dump(metadata, meta_file, indent=2, ensure_ascii=False)

        rospy.loginfo("Saved bag: %s", self.bag_path)
        rospy.loginfo("Saved metadata: %s", self.metadata_path)


if __name__ == "__main__":
    rospy.init_node("experiment_data_recorder")
    recorder = ExperimentDataRecorder()
    rospy.spin()
    recorder.close()
