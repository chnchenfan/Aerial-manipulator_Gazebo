#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DATA_DIR = Path(__file__).resolve().parents[1]
RAW_DIR = DATA_DIR / "raw"
OUT = DATA_DIR / "figure"
FONT = None

CASES = {
    "mode1": {
        "title": "扰动抑制实验",
        "eso": RAW_DIR / "eso_mode1_exp1_hover_disturbance_uam_v5.bag",
        "mc": RAW_DIR / "mc_mode1_exp1_hover_disturbance_uam_v5.bag",
        "duration": None,
    },
    "mode2": {
        "title": "轨迹跟踪实验",
        "eso": RAW_DIR / "eso_mode2_exp4_square_tracking_uam_v5.bag",
        "mc": RAW_DIR / "mc_mode2_exp4_square_tracking_uam_v5.bag",
        "duration": None,
    },
}


def configure_style() -> None:
    global FONT
    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    if Path(font_path).exists():
        FONT = fm.FontProperties(fname=font_path)
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.facecolor"] = "white"
    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.38
    plt.rcParams["grid.linewidth"] = 0.8


def set_title(ax, text: str) -> None:
    ax.set_title(text, fontproperties=FONT)


def set_xlabel(ax, text: str) -> None:
    ax.set_xlabel(text, fontproperties=FONT)


def set_ylabel(ax, text: str) -> None:
    ax.set_ylabel(text, fontproperties=FONT, labelpad=8)


def set_zlabel(ax, text: str) -> None:
    ax.set_zlabel(text, fontproperties=FONT, labelpad=14)


def set_legend(ax, labels, *args, **kwargs):
    ax.legend(labels, *args, prop=FONT, **kwargs)


def vec_from_pose(msg) -> np.ndarray:
    return np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)


def vec_from_target(msg) -> np.ndarray:
    return np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)


def read_case(bag_path: Path, duration: float | None) -> dict:
    actual_t, actual_p = [], []
    desired_t, desired_p = [], []
    joint_t, joint_q = [], []
    target_joint_t, target_joint_q = [], []
    arm_start = None
    joints = ["arm_joint1", "arm_joint2", "left_hand_joint"]
    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages():
            ts = stamp.to_sec()
            if topic == "/experiment/arm_motion_enabled" and bool(msg.data):
                if arm_start is None:
                    arm_start = ts
            elif topic == "/mavros/local_position/pose":
                actual_t.append(ts)
                actual_p.append(vec_from_pose(msg))
            elif topic == "/mavros/setpoint_position/local":
                desired_t.append(ts)
                desired_p.append(vec_from_pose(msg))
            elif topic == "/mavros/setpoint_raw/local":
                desired_t.append(ts)
                desired_p.append(vec_from_target(msg))
            elif topic == "/uav_arm/joint_states":
                idx = {name: i for i, name in enumerate(msg.name)}
                if all(joint in idx for joint in joints):
                    joint_t.append(ts)
                    joint_q.append([msg.position[idx[joint]] for joint in joints])
            elif topic == "/uav_arm/target_joint_states":
                idx = {name: i for i, name in enumerate(msg.name)}
                if all(joint in idx for joint in joints):
                    target_joint_t.append(ts)
                    target_joint_q.append([msg.position[idx[joint]] for joint in joints])
    if arm_start is None:
        arm_start = bag_start
    actual_t = np.asarray(actual_t, dtype=float)
    actual_p = np.asarray(actual_p, dtype=float)
    desired_t = np.asarray(desired_t, dtype=float)
    desired_p = np.asarray(desired_p, dtype=float)
    joint_t = np.asarray(joint_t, dtype=float)
    joint_q = np.asarray(joint_q, dtype=float)
    target_joint_t = np.asarray(target_joint_t, dtype=float)
    target_joint_q = np.asarray(target_joint_q, dtype=float)
    if duration is None:
        mask = actual_t >= arm_start
        joint_mask = joint_t >= arm_start if len(joint_t) else np.asarray([], dtype=bool)
    else:
        mask = (actual_t >= arm_start) & (actual_t <= arm_start + duration)
        joint_mask = (joint_t >= arm_start) & (joint_t <= arm_start + duration) if len(joint_t) else np.asarray([], dtype=bool)
    t = actual_t[mask] - arm_start
    p = actual_p[mask]
    desired = np.column_stack([
        np.interp(actual_t[mask], desired_t, desired_p[:, axis])
        for axis in range(3)
    ])
    error = p - desired
    norm = np.linalg.norm(error, axis=1)
    tq = joint_t[joint_mask] - arm_start
    q = joint_q[joint_mask]
    q_desired = np.empty((0, 3))
    if target_joint_t.size and tq.size:
        q_desired = np.column_stack([
            np.interp(joint_t[joint_mask], target_joint_t, target_joint_q[:, axis])
            for axis in range(3)
        ])
    q_error = q - q_desired if q_desired.size else np.empty((0, 3))
    return {
        "bag": str(bag_path),
        "arm_start_ros_time_s": arm_start,
        "t": t,
        "p": p,
        "desired": desired,
        "error": error,
        "norm": norm,
        "joint_names": joints,
        "tq": tq,
        "q": q,
        "q_desired": q_desired,
        "q_error": q_error,
        "metrics": {
            "samples": int(t.size),
            "mean": float(np.mean(norm)),
            "rmse": float(np.sqrt(np.mean(norm ** 2))),
            "max": float(np.max(norm)),
            "axis_max": np.max(np.abs(error), axis=0).tolist(),
            "axis_mean_abs": np.mean(np.abs(error), axis=0).tolist(),
            "arm_axis_max_rad": np.max(np.abs(q_error), axis=0).tolist() if q_error.size else None,
            "arm_max_error_rad": float(np.max(np.linalg.norm(q_error, axis=1))) if q_error.size else None,
        },
    }


def plot_position_pair(mode: str, title: str, eso: dict, mc: dict) -> None:
    labels = [r"位置, $p_x$ / m", r"位置, $p_y$ / m", r"位置, $p_z$ / m"]
    fig, axes = plt.subplots(3, 1, figsize=(11.6, 7.8), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(eso["t"], eso["p"][:, i], "b-", linewidth=1.1)
        ax.plot(mc["t"], mc["p"][:, i], color=(0.85, 0.33, 0.10), linewidth=1.0)
        ax.plot(eso["t"], eso["desired"][:, i], "k--", linewidth=0.9)
        set_ylabel(ax, labels[i])
        if i == 0:
            set_title(ax, f"{title}位置跟踪对比")
            set_legend(ax, ["ESO", "MC", "期望值"], loc="best")
    set_xlabel(axes[-1], "时间, t / s")
    fig.tight_layout()
    fig.savefig(OUT / f"{mode}_position_tracking_eso_vs_mc.png", dpi=180)
    plt.close(fig)


def plot_error_pair(mode: str, title: str, eso: dict, mc: dict) -> None:
    labels = [r"位置误差, $e_x$ / m", r"位置误差, $e_y$ / m", r"位置误差, $e_z$ / m"]
    fig, axes = plt.subplots(3, 1, figsize=(11.6, 7.8), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(eso["t"], eso["error"][:, i], "b-", linewidth=1.0)
        ax.plot(mc["t"], mc["error"][:, i], color=(0.85, 0.33, 0.10), linewidth=1.0)
        set_ylabel(ax, labels[i])
        if i == 0:
            set_title(ax, f"{title}位置跟踪误差对比")
            set_legend(ax, ["ESO", "MC"], loc="best")
    set_xlabel(axes[-1], "时间, t / s")
    fig.tight_layout()
    fig.savefig(OUT / f"{mode}_position_error_eso_vs_mc.png", dpi=180)
    plt.close(fig)


def plot_3d_pair(mode: str, title: str, eso: dict, mc: dict) -> None:
    fig = plt.figure(figsize=(13.2, 5.8))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.plot(eso["desired"][:, 0], eso["desired"][:, 1], eso["desired"][:, 2], "k--", linewidth=1.0)
    ax1.plot(eso["p"][:, 0], eso["p"][:, 1], eso["p"][:, 2], "b-", linewidth=1.1)
    ax1.plot(mc["p"][:, 0], mc["p"][:, 1], mc["p"][:, 2], color=(0.85, 0.33, 0.10), linewidth=1.0)
    set_xlabel(ax1, r"$p_x$ / m")
    set_ylabel(ax1, r"$p_y$ / m")
    set_zlabel(ax1, r"$p_z$ / m")
    set_title(ax1, f"{title}三维轨迹对比")
    set_legend(ax1, ["期望轨迹", "ESO", "MC"], loc="best")
    if mode == "mode2":
        ax1.set_zlim(1.85, 2.15)
        ax1.set_zticks([])
        set_zlabel(ax1, "")
        ax1.zaxis.line.set_alpha(0.0)
        ax1.zaxis.pane.set_alpha(0.0)
        ax1.zaxis._axinfo["grid"]["linewidth"] = 0.0
    try:
        ax1.set_box_aspect((np.ptp(eso["desired"][:, 0]) + 1e-6, np.ptp(eso["desired"][:, 1]) + 1e-6, np.ptp(eso["desired"][:, 2]) + 1e-6))
    except Exception:
        pass

    ax2 = fig.add_subplot(1, 2, 2)
    h1 = ax2.plot(eso["t"], eso["norm"], "b-", linewidth=1.0)[0]
    h2 = ax2.plot(mc["t"], mc["norm"], color=(0.85, 0.33, 0.10), linewidth=1.0)[0]
    h3 = ax2.axhline(np.mean(eso["norm"]), color="g", linestyle="--", linewidth=1.0)
    ax2.axhline(np.mean(mc["norm"]), color="g", linestyle="--", linewidth=1.0)
    set_xlabel(ax2, "时间, t / s")
    set_ylabel(ax2, r"位置误差, $e_p$ / m")
    set_title(ax2, f"{title}三维位置误差对比")
    ax2.legend([h1, h2, h3], ["ESO", "MC", "平均误差"], loc="best", prop=FONT)
    fig.subplots_adjust(left=0.04, right=0.98, bottom=0.12, top=0.88, wspace=0.42)
    fig.savefig(OUT / f"{mode}_3d_mean_error_eso_vs_mc.png", dpi=180)
    plt.close(fig)


def plot_arm_tracking(mode: str, title: str, data: dict) -> None:
    joint_labels = [r"关节角, $q_1$ / rad", r"关节角, $q_2$ / rad", r"关节角, $q_3$ / rad"]
    if not data["q"].size:
        return
    fig, axes = plt.subplots(3, 1, figsize=(11.6, 7.8), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(data["tq"], data["q_desired"][:, i], "r--", linewidth=1.2)
        ax.plot(data["tq"], data["q"][:, i], "b-", linewidth=1.1)
        set_ylabel(ax, joint_labels[i])
        if i == 0:
            set_title(ax, f"{title}机械臂关节跟踪响应")
            set_legend(ax, ["期望值", "实际值"], loc="best")
    set_xlabel(axes[-1], "时间, t / s")
    fig.tight_layout()
    fig.savefig(OUT / f"{mode}_arm_tracking.png", dpi=180)
    plt.close(fig)


def main() -> int:
    configure_style()
    OUT.mkdir(parents=True, exist_ok=True)
    report = {}
    for mode, cfg in CASES.items():
        eso = read_case(cfg["eso"], cfg["duration"])
        mc = read_case(cfg["mc"], cfg["duration"])
        plot_position_pair(mode, cfg["title"], eso, mc)
        plot_error_pair(mode, cfg["title"], eso, mc)
        plot_3d_pair(mode, cfg["title"], eso, mc)
        plot_arm_tracking(mode, cfg["title"], eso)
        report[mode] = {
            "title": cfg["title"],
            "eso": {"bag": eso["bag"], "arm_start_ros_time_s": eso["arm_start_ros_time_s"], "metrics": eso["metrics"]},
            "mc": {"bag": mc["bag"], "arm_start_ros_time_s": mc["arm_start_ros_time_s"], "metrics": mc["metrics"]},
        }
    (OUT / "after_arm_motion_metrics.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
