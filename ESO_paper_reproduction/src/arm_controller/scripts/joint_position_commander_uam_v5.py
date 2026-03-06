#!/usr/bin/env python3
"""
Smooth motion helper for the uam_v5 arm with built-in logging and plotting.
"""
import csv
import itertools
import os
import time
from datetime import datetime
from typing import Dict, List

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - plotting is optional at runtime
    plt = None


class JointCommander:
    def __init__(self):
        self.namespace = rospy.get_param("~namespace", "uav_arm").strip("/")
        self.hold_time = float(rospy.get_param("~hold_time", 3.0))
        self.move_duration = float(rospy.get_param("~move_duration", 3.0))
        self.command_rate_hz = float(rospy.get_param("~command_rate_hz", 50.0))
        self.log_interval = float(rospy.get_param("~log_interval", 1.0))
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/.ros/uam_v5_arm_logs")
        )
        self.plot_title = rospy.get_param(
            "~plot_title", "uam_v5 Joint Commander Response"
        )

        self.sequence: List[Dict[str, float]] = [
            {
                "arm_joint1": 0.0,
                "arm_joint2": 0.0,
                "left_hand_joint": 0.0,
            },
            {
                "arm_joint1": 0.35,
                "arm_joint2": -0.45,
                "left_hand_joint": 0.008,
            },
            {
                "arm_joint1": -0.35,
                "arm_joint2": 0.25,
                "left_hand_joint": 0.002,
            },
        ]

        self.joints = [
            "arm_joint1", "arm_joint2", "left_hand_joint"
        ]

        self.publishers: Dict[str, rospy.Publisher] = {}
        for joint in self.joints:
            topic = f"/{self.namespace}/{joint}_position_controller/command"
            self.publishers[joint] = rospy.Publisher(topic, Float64, queue_size=10)

        self.target_joint_state_pub = rospy.Publisher(
            f"/{self.namespace}/target_joint_states", JointState, queue_size=10
        )
        self.joint_state_sub = rospy.Subscriber(
            f"/{self.namespace}/joint_states",
            JointState,
            self._joint_state_cb,
            queue_size=50,
        )

        self.current_positions = {j: 0.0 for j in self.joints}
        self.current_targets = self.current_positions.copy()
        self._latest_state = None
        self._samples = []
        self._last_log_time = -1.0
        self._run_start_time = None

        rospy.loginfo("uam_v5 JointCommander ready with logging")

    def _joint_state_cb(self, msg):
        self._latest_state = msg

    def _wait_for_joint_states(self, timeout_s=10.0):
        start = time.time()
        while not rospy.is_shutdown():
            if self._latest_state is not None:
                return True
            if time.time() - start > timeout_s:
                return False
            rospy.sleep(0.05)
        return False

    def publish_target_joint_state(self, positions: Dict[str, float]):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(positions.keys())
        msg.position = list(positions.values())
        self.target_joint_state_pub.publish(msg)

    def publish_positions(self, positions: Dict[str, float]):
        for joint, val in positions.items():
            if joint in self.publishers:
                self.publishers[joint].publish(Float64(val))
        self.current_targets.update(positions)

    def _extract_joint_state(self, msg, joint):
        try:
            idx = msg.name.index(joint)
        except ValueError:
            return 0.0, 0.0, 0.0

        position = msg.position[idx] if idx < len(msg.position) else 0.0
        velocity = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
        effort = msg.effort[idx] if idx < len(msg.effort) else 0.0
        return position, velocity, effort

    def _log_sample(self):
        if self._latest_state is None or self._run_start_time is None:
            return

        elapsed_s = time.time() - self._run_start_time
        sample = {"t": elapsed_s}
        line_parts = [f"t={elapsed_s:6.2f}s"]

        for joint in self.joints:
            target = self.current_targets.get(joint, 0.0)
            position, velocity, effort = self._extract_joint_state(self._latest_state, joint)
            error = target - position
            sample[f"{joint}_target"] = target
            sample[f"{joint}_position"] = position
            sample[f"{joint}_velocity"] = velocity
            sample[f"{joint}_effort"] = effort
            sample[f"{joint}_error"] = error
            line_parts.append(
                f"{joint}: pos={position:+.5f} vel={velocity:+.5f} err={error:+.5f}"
            )

        self._samples.append(sample)

        if elapsed_s - self._last_log_time >= self.log_interval:
            rospy.loginfo(" | ".join(line_parts))
            self._last_log_time = elapsed_s

    def _record_step(self):
        self._log_sample()

    def _save_csv(self, csv_path):
        if not self._samples:
            rospy.logwarn("No samples recorded, skipping CSV export")
            return

        fieldnames = list(self._samples[0].keys())
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._samples)

        rospy.loginfo("Saved CSV log to %s", csv_path)

    def _save_plot(self, png_path):
        if not self._samples:
            return

        if plt is None:
            rospy.logwarn("matplotlib not available, skipping plot generation")
            return

        times = [sample["t"] for sample in self._samples]
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

        for joint in self.joints:
            axes[0].plot(times, [s[f"{joint}_position"] for s in self._samples], label=f"{joint} pos")
            axes[0].plot(times, [s[f"{joint}_target"] for s in self._samples], "--", label=f"{joint} target")
            axes[1].plot(times, [s[f"{joint}_velocity"] for s in self._samples], label=f"{joint} vel")
            axes[2].plot(times, [s[f"{joint}_error"] for s in self._samples], label=f"{joint} err")

        axes[0].set_ylabel("Position")
        axes[1].set_ylabel("Velocity")
        axes[2].set_ylabel("Error")
        axes[2].set_xlabel("Time [s]")

        for axis in axes:
            axis.grid(True, linestyle="--", alpha=0.4)
            axis.legend(loc="upper right", fontsize=8)

        fig.suptitle(self.plot_title)
        fig.tight_layout()
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        rospy.loginfo("Saved plot to %s", png_path)

    def _save_artifacts(self):
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, f"joint_commander_{stamp}.csv")
        png_path = os.path.join(self.output_dir, f"joint_commander_{stamp}.png")
        self._save_csv(csv_path)
        self._save_plot(png_path)

    def move_smoothly(self, target_positions: Dict[str, float]):
        rospy.loginfo(f"Moving smoothly to: {target_positions}")

        start_positions = self.current_positions.copy()
        steps = max(1, int(self.move_duration * self.command_rate_hz))

        rate = rospy.Rate(self.command_rate_hz)

        for i in range(steps):
            if rospy.is_shutdown(): break

            alpha = (i + 1) / steps
            interp_pos = {}

            for joint in self.joints:
                start = start_positions.get(joint, 0.0)
                end = target_positions.get(joint, 0.0)
                interp_pos[joint] = start + alpha * (end - start)

            self.publish_positions(interp_pos)
            self.publish_target_joint_state(interp_pos)
            self._record_step()
            rate.sleep()

        self.current_positions = target_positions.copy()

    def run(self):
        if not self._wait_for_joint_states():
            raise RuntimeError("Timed out waiting for /joint_states")

        self._run_start_time = time.time()
        self.publish_positions(self.current_positions)
        rospy.sleep(2.0)

        sequence_iter = itertools.cycle(self.sequence)

        try:
            while not rospy.is_shutdown():
                target = next(sequence_iter)
                self.move_smoothly(target)
                rospy.loginfo(f"Holding for {self.hold_time}s...")
                start_hold = time.time()
                hold_rate = rospy.Rate(self.command_rate_hz)
                while time.time() - start_hold < self.hold_time:
                    if rospy.is_shutdown():
                        break
                    self.publish_positions(target)
                    self.publish_target_joint_state(target)
                    self._record_step()
                    hold_rate.sleep()
        finally:
            self._save_artifacts()

if __name__ == "__main__":
    rospy.init_node("uam_v5_smooth_commander")
    commander = JointCommander()
    commander.run()
