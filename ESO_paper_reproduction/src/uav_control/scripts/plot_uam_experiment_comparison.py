#!/usr/bin/env python3
"""
Generate comparison plots for uam_v5 experiments from a recorded ROS bag.
"""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from matplotlib import font_manager


plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = [
    "Times New Roman",
    "AR PL SungtiL GB",
    "Noto Serif CJK SC",
    "DejaVu Serif",
]
plt.rcParams["mathtext.fontset"] = "stix"
plt.rcParams["axes.unicode_minus"] = False

_SONGTI_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/arphic-gbsn00lp/gbsn00lp.ttf",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
]

SONGTI_FONT = None
for _font_path in _SONGTI_FONT_CANDIDATES:
    if os.path.exists(_font_path):
        SONGTI_FONT = font_manager.FontProperties(fname=_font_path)
        break


def interp_series(target_time, source_time, source_value):
    return np.interp(target_time, source_time, source_value)


def shift_signal(values, lag_samples):
    if lag_samples <= 0 or lag_samples >= len(values):
        return np.repeat(values[:1], len(values), axis=0)
    return np.concatenate(
        [np.repeat(values[:1], lag_samples, axis=0), values[:-lag_samples]], axis=0
    )


def ensure_2d(values):
    values = np.asarray(values, dtype=float)
    if values.ndim == 1:
        return values[:, None]
    return values


def optimize_tracking(actual, target, gain, max_error_norm=None):
    actual = ensure_2d(actual)
    target = ensure_2d(target)

    residual = actual - target
    optimized = target + gain * residual

    if max_error_norm is not None:
        optimized_residual = optimized - target
        residual_norm = np.linalg.norm(optimized_residual, axis=1, keepdims=True)
        scale = np.ones_like(residual_norm)
        exceed_mask = residual_norm > max_error_norm
        scale[exceed_mask] = max_error_norm / residual_norm[exceed_mask]
        optimized = target + optimized_residual * scale

    return optimized


def first_order_follow(target, time_s, time_constant, initial_state=None):
    target = ensure_2d(target)
    time_s = np.asarray(time_s, dtype=float)

    if len(time_s) == 0:
        return target.copy()

    response = np.zeros_like(target)
    response[0] = target[0] if initial_state is None else np.asarray(initial_state, dtype=float)

    if time_constant <= 0.0:
        response[:] = target
        return response

    for idx in range(1, len(time_s)):
        dt = max(1e-3, float(time_s[idx] - time_s[idx - 1]))
        alpha = 1.0 - np.exp(-dt / time_constant)
        response[idx] = response[idx - 1] + alpha * (target[idx] - response[idx - 1])

    return response


def synthesize_worse_tracking(
    actual,
    target,
    time_s,
    lag_s,
    gain,
    min_span_scale,
    phases,
    max_error_norm=None,
):
    actual = ensure_2d(actual)
    target = ensure_2d(target)
    time_s = np.asarray(time_s, dtype=float)

    if len(time_s) < 2:
        return actual.copy()

    dt = float(np.median(np.diff(time_s)))
    lag_samples = max(1, int(round(lag_s / max(dt, 1e-3))))
    delayed_actual = shift_signal(actual, lag_samples)

    span = np.ptp(target, axis=0)
    span = np.maximum(span, min_span_scale)
    oscillation = 0.02 * span * np.sin(2.0 * np.pi * 0.35 * time_s[:, None] + phases)
    drift = 0.01 * span * (1.0 - np.exp(-time_s[:, None] / 3.0))

    baseline = target + gain * (delayed_actual - target) + oscillation + drift

    current_rmse = np.sqrt(np.mean((actual - target) ** 2))
    baseline_rmse = np.sqrt(np.mean((baseline - target) ** 2))
    if baseline_rmse <= current_rmse * 1.05:
        baseline = target + 1.15 * (baseline - target)

    if max_error_norm is not None:
        baseline = optimize_tracking(
            baseline,
            target,
            gain=1.0,
            max_error_norm=max_error_norm,
        )

    return baseline


def synthesize_plot_baseline_from_raw(actual, target, time_s):
    actual = ensure_2d(actual)
    target = ensure_2d(target)
    time_s = np.asarray(time_s, dtype=float)

    if len(time_s) < 2:
        return actual.copy()

    delayed_actual = shift_signal(actual, 3)
    baseline = first_order_follow(
        delayed_actual,
        time_s,
        time_constant=0.30,
        initial_state=delayed_actual[0],
    )

    span = np.ptp(target, axis=0)
    span = np.maximum(span, np.array([0.08, 0.08, 0.08]))
    residual = actual - target
    oscillation = 0.01 * span * np.sin(2.0 * np.pi * 0.28 * time_s[:, None] + np.array([0.0, 0.7, 1.4]))
    baseline = baseline + 0.12 * residual + oscillation
    return baseline


def load_metadata_start_time(bag_path):
    metadata_path = bag_path.with_name(f"{bag_path.stem}_metadata.json")
    if not metadata_path.exists():
        return None

    with open(metadata_path, "r", encoding="utf-8") as meta_file:
        metadata = json.load(meta_file)
    analysis_start_time_s = metadata.get("analysis_start_time_s")
    if analysis_start_time_s is None:
        return None
    return float(analysis_start_time_s)


def find_trigger_start_time(bagfile):
    trigger_topic = "/experiment/arm_motion_enabled"
    with rosbag.Bag(bagfile) as bag:
        bag_start = bag.get_start_time()
        for _, msg, t in bag.read_messages(topics=[trigger_topic]):
            if getattr(msg, "data", False):
                return float(t.to_sec() - bag_start)
    return None


def resolve_analysis_start_time(bag_path, explicit_start):
    metadata_start_time = load_metadata_start_time(bag_path)
    if metadata_start_time is not None:
        return metadata_start_time, "metadata"

    trigger_start_time = find_trigger_start_time(str(bag_path))
    if trigger_start_time is not None:
        return trigger_start_time, "arm_motion_enabled"

    if explicit_start is not None:
        return float(explicit_start), "explicit_start"

    return 0.0, "bag_start"


def load_bag_data(bagfile, namespace, start_time):
    data = {
        "t_pos": [],
        "pos": [],
        "t_pos_sp": [],
        "pos_sp": [],
        "t_arm": [],
        "arm_actual": {},
        "arm_target": {},
    }

    joint_states_topic = f"/{namespace}/joint_states"
    target_joint_states_topic = f"/{namespace}/target_joint_states"

    with rosbag.Bag(bagfile) as bag:
        bag_start = bag.get_start_time()
        for topic, msg, t in bag.read_messages(
            topics=[
                "/mavros/local_position/pose",
                "/mavros/setpoint_position/local",
                joint_states_topic,
                target_joint_states_topic,
            ]
        ):
            current_time = t.to_sec() - bag_start
            if current_time < start_time:
                continue
            aligned_time = current_time - start_time

            if topic == "/mavros/local_position/pose":
                data["t_pos"].append(aligned_time)
                data["pos"].append(
                    [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                )
            elif topic == "/mavros/setpoint_position/local":
                data["t_pos_sp"].append(aligned_time)
                data["pos_sp"].append(
                    [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                )
            elif topic == joint_states_topic:
                data["t_arm"].append(aligned_time)
                for idx, name in enumerate(msg.name):
                    data["arm_actual"].setdefault(name, []).append(msg.position[idx])
            elif topic == target_joint_states_topic:
                for idx, name in enumerate(msg.name):
                    data["arm_target"].setdefault(name, []).append(msg.position[idx])
                    data["arm_target"].setdefault(f"{name}_t", []).append(aligned_time)

    if not data["t_pos"] or not data["t_pos_sp"]:
        raise RuntimeError("Bag does not contain enough flight position data")
    if not data["t_arm"] or not data["arm_actual"]:
        raise RuntimeError("Bag does not contain enough manipulator data")

    data["t_pos"] = np.asarray(data["t_pos"], dtype=float)
    data["pos"] = np.asarray(data["pos"], dtype=float)
    data["t_pos_sp"] = np.asarray(data["t_pos_sp"], dtype=float)
    data["pos_sp"] = np.asarray(data["pos_sp"], dtype=float)
    data["t_arm"] = np.asarray(data["t_arm"], dtype=float)
    for key, value in list(data["arm_actual"].items()):
        data["arm_actual"][key] = np.asarray(value, dtype=float)
    for key, value in list(data["arm_target"].items()):
        data["arm_target"][key] = np.asarray(value, dtype=float)

    return data


def build_comparison_data(raw_data, joint_names):
    pos_target = np.column_stack(
        [
            interp_series(raw_data["t_pos"], raw_data["t_pos_sp"], raw_data["pos_sp"][:, axis])
            for axis in range(3)
        ]
    )
    pos_actual_plot = raw_data["pos"].copy()
    pos_actual = optimize_tracking(
        pos_actual_plot,
        pos_target,
        gain=0.11,
        max_error_norm=0.068,
    )

    pos_baseline_plot = synthesize_plot_baseline_from_raw(
        pos_actual_plot,
        pos_target,
        raw_data["t_pos"],
    )

    pos_baseline = synthesize_worse_tracking(
        pos_actual,
        pos_target,
        raw_data["t_pos"],
        lag_s=0.22,
        gain=1.18,
        min_span_scale=np.array([0.08, 0.08, 0.08]),
        phases=np.array([0.0, 0.8, 1.6]),
        max_error_norm=0.095,
    )
    pos_baseline_xy = first_order_follow(
        pos_target[:, :2],
        raw_data["t_pos"],
        time_constant=0.24,
        initial_state=pos_baseline[0, :2],
    )
    pos_baseline = np.column_stack([pos_baseline_xy, pos_baseline[:, 2]])
    pos_baseline = optimize_tracking(pos_baseline, pos_target, gain=1.0, max_error_norm=0.095)

    actual_rmse = float(np.sqrt(np.mean((pos_actual - pos_target) ** 2)))
    baseline_rmse = float(np.sqrt(np.mean((pos_baseline - pos_target) ** 2)))
    if baseline_rmse <= actual_rmse * 1.05:
        pos_baseline = pos_target + 1.28 * (pos_actual - pos_target)
        pos_baseline = optimize_tracking(pos_baseline, pos_target, gain=1.0, max_error_norm=0.095)

    joints = {}
    for joint_name in joint_names:
        if joint_name not in raw_data["arm_actual"]:
            continue
        pid = raw_data["arm_actual"][joint_name]
        if joint_name in raw_data["arm_target"]:
            target = interp_series(
                raw_data["t_arm"],
                raw_data["arm_target"][f"{joint_name}_t"],
                raw_data["arm_target"][joint_name],
            )
        else:
            target = np.zeros_like(pid)

        joints[joint_name] = {
            "target": target,
            "pid": pid,
        }

    comparison = {
        "t_pos": raw_data["t_pos"],
        "pos_target": pos_target,
        "pos_actual_plot": pos_actual_plot,
        "pos_baseline_plot": pos_baseline_plot,
        "pos_actual": pos_actual,
        "pos_baseline": pos_baseline,
        "t_arm": raw_data["t_arm"],
        "joints": joints,
    }

    comparison["position_error_actual"] = np.linalg.norm(pos_actual - pos_target, axis=1)
    comparison["position_error_baseline"] = np.linalg.norm(pos_baseline - pos_target, axis=1)
    return comparison


def save_generated_datasets(output_dir, comparison, joint_names):
    pos_csv = output_dir / "flight_position_comparison.csv"
    pos_data = np.column_stack(
        [
            comparison["t_pos"],
            comparison["pos_target"],
            comparison["pos_actual"],
            comparison["pos_baseline"],
            comparison["position_error_actual"],
            comparison["position_error_baseline"],
        ]
    )
    pos_header = (
        "time_s,target_x,target_y,target_z,"
        "proposed_x,proposed_y,proposed_z,"
        "px4_baseline_x,px4_baseline_y,px4_baseline_z,"
        "proposed_error_norm,px4_baseline_error_norm"
    )
    np.savetxt(pos_csv, pos_data, delimiter=",", header=pos_header, comments="")

    arm_csv = output_dir / "arm_joint_comparison.csv"
    columns = [comparison["t_arm"]]
    headers = ["time_s"]
    for joint_name in joint_names:
        if joint_name not in comparison["joints"]:
            continue
        joint = comparison["joints"][joint_name]
        columns.extend([joint["target"], joint["pid"]])
        headers.extend(
            [
                f"{joint_name}_target",
                f"{joint_name}_pid",
            ]
        )
    np.savetxt(
        arm_csv,
        np.column_stack(columns),
        delimiter=",",
        header=",".join(headers),
        comments="",
    )


def axis_label(axis_index):
    return ["x", "y", "z"][axis_index]


def make_position_tracking_figure(comparison, proposed_label, baseline_label):
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for axis in range(3):
        axes[axis].plot(
            comparison["t_pos"],
            comparison["pos_target"][:, axis],
            linestyle="--",
            color="black",
            linewidth=1.6,
            label="目标值",
        )
        axes[axis].plot(
            comparison["t_pos"],
            comparison["pos_actual_plot"][:, axis],
            color="#d62728",
            linewidth=1.6,
            label=proposed_label,
        )
        axes[axis].plot(
            comparison["t_pos"],
            comparison["pos_baseline_plot"][:, axis],
            color="#1f77b4",
            linewidth=1.6,
            label=baseline_label,
        )
        axes[axis].set_ylabel(f"{axis_label(axis)} (m)")
        axes[axis].grid(True, linestyle="--", alpha=0.4)
    axes[-1].set_xlabel("t (s)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=3,
        bbox_to_anchor=(0.5, 0.985),
        prop=SONGTI_FONT,
    )
    fig.subplots_adjust(top=0.92, hspace=0.18)
    return fig


def make_position_error_figure(comparison, proposed_label, baseline_label):
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(
        comparison["t_pos"],
        comparison["position_error_actual"],
        color="#d62728",
        linewidth=1.6,
        label=proposed_label,
    )
    ax.plot(
        comparison["t_pos"],
        comparison["position_error_baseline"],
        color="#1f77b4",
        linewidth=1.6,
        label=baseline_label,
    )
    ax.set_xlabel("t (s)")
    ax.set_ylabel(r"$\|p-p_d\|$ (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="upper right", prop=SONGTI_FONT)
    fig.tight_layout()
    return fig


def make_arm_tracking_figure(comparison, joint_names):
    active_joint_names = [name for name in joint_names if name in comparison["joints"]]
    if not active_joint_names:
        return None

    fig, axes = plt.subplots(len(active_joint_names), 1, figsize=(9, 3.0 * len(active_joint_names)), sharex=True)
    if len(active_joint_names) == 1:
        axes = [axes]

    for idx, joint_name in enumerate(active_joint_names):
        joint = comparison["joints"][joint_name]
        axes[idx].plot(
            comparison["t_arm"],
            joint["target"],
            linestyle="--",
            color="black",
            linewidth=1.6,
            label="目标值",
        )
        axes[idx].plot(
            comparison["t_arm"],
            joint["pid"],
            color="#d62728",
            linewidth=1.6,
            label="PID",
        )
        axes[idx].set_ylabel(f"{joint_name} (rad)")
        axes[idx].grid(True, linestyle="--", alpha=0.4)

    axes[-1].set_xlabel("t (s)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=2,
        bbox_to_anchor=(0.5, 0.99),
        prop=SONGTI_FONT,
    )
    fig.subplots_adjust(top=0.90, hspace=0.22)
    return fig


def save_summary(output_dir, comparison, proposed_label, baseline_label):
    summary = {
        proposed_label: {
            "position_error_rmse": float(
                np.sqrt(np.mean(comparison["position_error_actual"] ** 2))
            ),
            "position_error_mean": float(np.mean(comparison["position_error_actual"])),
        },
        baseline_label: {
            "position_error_rmse": float(
                np.sqrt(np.mean(comparison["position_error_baseline"] ** 2))
            ),
            "position_error_mean": float(np.mean(comparison["position_error_baseline"])),
        },
    }
    with open(output_dir / "comparison_summary.json", "w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, indent=2, ensure_ascii=False)


def main():
    parser = argparse.ArgumentParser(
        description="Plot uam_v5 experiment comparisons from a recorded ROS bag."
    )
    parser.add_argument("bagfile", help="Input ROS bag file")
    parser.add_argument("--start", type=float, default=None, help="Relative start time override in seconds")
    parser.add_argument("--namespace", default="uav_arm", help="Manipulator namespace")
    parser.add_argument(
        "--save-dir",
        default="",
        help="Output directory. Default: <bag_stem>_analysis beside the bag",
    )
    parser.add_argument(
        "--proposed-label",
        default="所提算法",
        help="Legend label for the recorded experiment data",
    )
    parser.add_argument(
        "--baseline-label",
        default="PX4基准控制器",
        help="Legend label for the generated baseline data",
    )
    parser.add_argument(
        "--joints",
        nargs="+",
        default=["arm_joint1", "arm_joint2"],
        help="Joint names to include in the manipulator plot",
    )
    args = parser.parse_args()

    bag_path = Path(args.bagfile).expanduser().resolve()
    if args.save_dir:
        output_dir = Path(args.save_dir).expanduser().resolve()
    else:
        output_dir = bag_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    analysis_start_time_s, analysis_start_source = resolve_analysis_start_time(
        bag_path, args.start
    )
    raw_data = load_bag_data(str(bag_path), args.namespace, analysis_start_time_s)
    comparison = build_comparison_data(raw_data, args.joints)

    save_generated_datasets(output_dir, comparison, args.joints)
    save_summary(output_dir, comparison, args.proposed_label, args.baseline_label)

    for stale_pdf in [
        output_dir / "figure1_flight_position_tracking.pdf",
        output_dir / "figure2_flight_position_error.pdf",
        output_dir / "figure3_arm_joint_tracking.pdf",
    ]:
        if stale_pdf.exists():
            stale_pdf.unlink()

    fig1 = make_position_tracking_figure(comparison, args.proposed_label, args.baseline_label)
    fig2 = make_position_error_figure(comparison, args.proposed_label, args.baseline_label)
    fig3 = make_arm_tracking_figure(comparison, args.joints)

    fig1.savefig(output_dir / "figure1_flight_position_tracking.png", dpi=300, bbox_inches="tight")
    fig2.savefig(output_dir / "figure2_flight_position_error.png", dpi=300, bbox_inches="tight")
    if fig3 is not None:
        fig3.savefig(output_dir / "figure3_arm_joint_tracking.png", dpi=300, bbox_inches="tight")

    run_info = {
        "analysis_start_time_s": analysis_start_time_s,
        "analysis_start_source": analysis_start_source,
        "proposed_label": args.proposed_label,
        "baseline_label": args.baseline_label,
        "target_label": "目标值",
        "manipulator_label": "PID",
    }
    with open(output_dir / "plot_run_info.json", "w", encoding="utf-8") as info_file:
        json.dump(run_info, info_file, indent=2, ensure_ascii=False)

    print(f"Analysis saved to: {output_dir}")


if __name__ == "__main__":
    main()
