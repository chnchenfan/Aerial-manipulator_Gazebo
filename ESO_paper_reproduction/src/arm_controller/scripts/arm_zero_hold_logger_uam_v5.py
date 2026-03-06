#!/usr/bin/env python3
"""
Hold the uam_v5 arm at zero target, log joint states, and generate plots.
"""

import csv
import os
import time
from datetime import datetime

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - plotting is optional at runtime
    plt = None


class ZeroHoldLogger:
    def __init__(self):
        self.namespace = rospy.get_param("~namespace", "uav_arm").strip("/")
        self.joints = rospy.get_param(
            "~joints",
            ["arm_joint1", "arm_joint2", "left_hand_joint"],
        )
        self.targets = rospy.get_param(
            "~targets",
            [0.0 for _ in self.joints],
        )
        self.duration = float(rospy.get_param("~duration", 20.0))
        self.command_rate_hz = float(rospy.get_param("~command_rate_hz", 50.0))
        self.log_interval = float(rospy.get_param("~log_interval", 1.0))
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/.ros/uam_v5_arm_logs")
        )

        self._latest_state = None
        self._samples = []
        self._last_log_time = 0.0

        self.publishers = {
            joint: rospy.Publisher(
                f"/{self.namespace}/{joint}_position_controller/command",
                Float64,
                queue_size=10,
            )
            for joint in self.joints
        }
        self.joint_state_sub = rospy.Subscriber(
            f"/{self.namespace}/joint_states",
            JointState,
            self._joint_state_cb,
            queue_size=50,
        )

        rospy.loginfo("ZeroHoldLogger ready for joints: %s", ", ".join(self.joints))

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

    def _publish_targets(self):
        for joint, target in zip(self.joints, self.targets):
            self.publishers[joint].publish(Float64(target))

    def _extract_joint_state(self, msg, joint):
        try:
            idx = msg.name.index(joint)
        except ValueError:
            return 0.0, 0.0, 0.0

        position = msg.position[idx] if idx < len(msg.position) else 0.0
        velocity = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
        effort = msg.effort[idx] if idx < len(msg.effort) else 0.0
        return position, velocity, effort

    def _log_sample(self, elapsed_s):
        msg = self._latest_state
        if msg is None:
            return

        sample = {"t": elapsed_s}
        line_parts = [f"t={elapsed_s:6.2f}s"]

        for joint, target in zip(self.joints, self.targets):
            position, velocity, effort = self._extract_joint_state(msg, joint)
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

        fig.suptitle("uam_v5 Zero-Hold Joint Response")
        fig.tight_layout()
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        rospy.loginfo("Saved plot to %s", png_path)

    def run(self):
        if not self._wait_for_joint_states():
            raise RuntimeError("Timed out waiting for /joint_states")

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, f"zero_hold_{stamp}.csv")
        png_path = os.path.join(self.output_dir, f"zero_hold_{stamp}.png")

        rate = rospy.Rate(self.command_rate_hz)
        start_time = time.time()

        rospy.loginfo(
            "Holding joints at zero for %.1fs, logging to %s",
            self.duration,
            self.output_dir,
        )

        while not rospy.is_shutdown():
            elapsed_s = time.time() - start_time
            if elapsed_s > self.duration:
                break

            self._publish_targets()
            self._log_sample(elapsed_s)
            rate.sleep()

        self._save_csv(csv_path)
        self._save_plot(png_path)
        rospy.loginfo("Zero-hold logging complete")


if __name__ == "__main__":
    rospy.init_node("uam_v5_zero_hold_logger")
    ZeroHoldLogger().run()
