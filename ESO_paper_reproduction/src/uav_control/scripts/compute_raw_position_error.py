#!/usr/bin/env python3
"""Compute raw UAV position tracking error from a ROS bag.

This script intentionally uses only the recorded vehicle pose and setpoint
topics. It does not apply the plotting script's tracking optimization or
synthetic baseline generation.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag


POSE_TOPIC = "/mavros/local_position/pose"
SETPOINT_TOPIC = "/mavros/setpoint_position/local"
ENABLE_TOPIC = "/experiment/arm_motion_enabled"


def interp_columns(target_t, source_t, values):
    return np.column_stack(
        [np.interp(target_t, source_t, values[:, axis]) for axis in range(values.shape[1])]
    )


def find_enable_start(bag_path):
    with rosbag.Bag(str(bag_path)) as bag:
        bag_start = bag.get_start_time()
        for _, msg, t in bag.read_messages(topics=[ENABLE_TOPIC]):
            if getattr(msg, "data", False):
                return float(t.to_sec() - bag_start)
    return None


def load_metadata_start(bag_path):
    metadata_path = bag_path.with_name(f"{bag_path.stem}_metadata.json")
    if not metadata_path.exists():
        return None

    with metadata_path.open("r", encoding="utf-8") as f:
        metadata = json.load(f)

    value = metadata.get("analysis_start_time_s")
    return None if value is None else float(value)


def resolve_start_time(bag_path, explicit_start):
    if explicit_start is not None:
        return float(explicit_start), "explicit"

    metadata_start = load_metadata_start(bag_path)
    if metadata_start is not None:
        return metadata_start, "metadata"

    enable_start = find_enable_start(bag_path)
    if enable_start is not None:
        return enable_start, "enable_topic"

    return 0.0, "bag_start"


def load_position_data(bag_path, start_time, duration):
    pose_t = []
    pose = []
    setpoint_t = []
    setpoint = []

    with rosbag.Bag(str(bag_path)) as bag:
        bag_start = bag.get_start_time()
        end_time = None if duration is None else start_time + duration

        for topic, msg, t in bag.read_messages(topics=[POSE_TOPIC, SETPOINT_TOPIC]):
            current_time = float(t.to_sec() - bag_start)
            if current_time < start_time:
                continue
            if end_time is not None and current_time > end_time:
                continue

            aligned_time = current_time - start_time
            position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

            if topic == POSE_TOPIC:
                pose_t.append(aligned_time)
                pose.append(position)
            elif topic == SETPOINT_TOPIC:
                setpoint_t.append(aligned_time)
                setpoint.append(position)

    if len(pose_t) < 2 or len(setpoint_t) < 2:
        raise RuntimeError("Bag does not contain enough pose/setpoint samples in the requested window")

    return (
        np.asarray(pose_t, dtype=float),
        np.asarray(pose, dtype=float),
        np.asarray(setpoint_t, dtype=float),
        np.asarray(setpoint, dtype=float),
    )


def compute_metrics(bag_path, start_time=None, duration=None):
    resolved_start, start_source = resolve_start_time(bag_path, start_time)
    pose_t, pose, setpoint_t, setpoint = load_position_data(bag_path, resolved_start, duration)

    setpoint_aligned = interp_columns(pose_t, setpoint_t, setpoint)
    error = pose - setpoint_aligned
    error_norm = np.linalg.norm(error, axis=1)

    per_axis = {}
    for axis_name, axis in zip(("x", "y", "z"), range(3)):
        axis_error = error[:, axis]
        per_axis[axis_name] = {
            "mean_error_m": float(np.mean(axis_error)),
            "mean_abs_error_m": float(np.mean(np.abs(axis_error))),
            "rmse_error_m": float(math.sqrt(np.mean(axis_error ** 2))),
            "max_abs_error_m": float(np.max(np.abs(axis_error))),
        }

    return {
        "bag_path": str(bag_path),
        "analysis_start_time_s": resolved_start,
        "analysis_start_source": start_source,
        "analysis_duration_s": None if duration is None else float(duration),
        "sample_count": int(error_norm.size),
        "window_end_time_s": float(resolved_start + pose_t[-1]),
        "mean_position_error_m": float(np.mean(error_norm)),
        "rmse_position_error_m": float(math.sqrt(np.mean(error_norm ** 2))),
        "max_position_error_m": float(np.max(error_norm)),
        "per_axis": per_axis,
    }


def main():
    parser = argparse.ArgumentParser(description="Compute raw position tracking error from a ROS bag.")
    parser.add_argument("bagfile", help="Input ROS bag")
    parser.add_argument("--start", type=float, default=None, help="Relative start time in seconds")
    parser.add_argument("--duration", type=float, default=None, help="Analysis duration in seconds")
    parser.add_argument("--out", default="", help="Output JSON path")
    args = parser.parse_args()

    bag_path = Path(args.bagfile).expanduser().resolve()
    metrics = compute_metrics(bag_path, args.start, args.duration)

    if args.out:
        output_path = Path(args.out).expanduser().resolve()
    else:
        output_path = bag_path.with_name(f"{bag_path.stem}_raw_position_error.json")

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)

    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
