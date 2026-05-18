#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import os
import select
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rosbag
from scipy.io import savemat


REPO = Path("/home/cf/PX4_Firmware_clean")
DATA_DIR = REPO / "ESO_paper_reproduction/data"
RAW_ROOT = DATA_DIR / "raw"
FIGURE_ROOT = DATA_DIR / "figure"
RAW_DIR = RAW_ROOT
FIGURE_DIR = FIGURE_ROOT
AUTO_SCRIPT = REPO / "ESO_paper_reproduction/src/uav_control/scripts/auto_tune_uam_v5_eso.py"
RUN_ROOT = RAW_DIR / "fresh_runs"

sys.path.insert(0, str(AUTO_SCRIPT.parent))
spec = importlib.util.spec_from_file_location("auto_tune_uam_v5_eso", AUTO_SCRIPT)
if spec is None or spec.loader is None:
    raise RuntimeError(f"failed to import {AUTO_SCRIPT}")
auto = importlib.util.module_from_spec(spec)
spec.loader.exec_module(auto)
auto.REPO_ROOT = REPO
auto.ROS_ENV = REPO / "ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh"


ESO_PARAMS = {
    "COM_RCL_EXCEPT": 4,
    "COM_RC_IN_MODE": 1,
    "COM_ARM_IMU_ACC": 1.0,
    "MPC_THR_HOVER": 0.72,
    "MPC_USE_HTE": 0,
    "ESO_DYN_FF_EN": 1,
    "ESO_K_BETA": 0.6753983577239936,
    "ESO_MAX_TORQUE": 1.5689237741310784,
    "ESO_TAUS_K": 0.07656162909208254,
    "ESO_TAUS_K_R": 0.04196685956703164,
    "ESO_TAUS_K_P": -0.45179684856908414,
    "ESO_TAUS_K_Y": 0.3153866653296281,
    "ESO_TAUS_OBS_R": 1.1742838498096384,
    "ESO_TAUS_OBS_P": 0.9975129600067933,
    "ESO_TAUS_OBS_Y": 1.0,
    "ESO_TAUS_CTL_R": 0.912981734991624,
    "ESO_TAUS_CTL_P": 1.2030863580577495,
    "ESO_TAUS_CTL_Y": 1.0,
    "ESO_TAUS_LIM": 0.2777730262275409,
    "ESO_TAUS_TAU": 0.13486946443819608,
    "ESO_X_P": 1.57042,
    "ESO_Y_P": 1.82178630576241,
    "ESO_X_I": 0.35494,
    "ESO_Y_I": 0.2910956920072665,
    "ESO_X_VEL_P_ACC": 2.6543,
    "ESO_Y_VEL_P_ACC": 3.4518786353089626,
    "ESO_X_BW": 1.4796421374343431,
    "ESO_Y_BW": 2.686366307273755,
    "ESO_Z_P": 1.54265,
    "ESO_Z_I": 0.13,
    "ESO_Z_VEL_P_ACC": 1.47839,
    "ESO_Z_BW": 1.1519268189679266,
    "ESO_POS_INT_LIM": 0.3682,
    "ESO_XY_VEL_MAX": 3.35,
    "ESO_ACC_HOR": 8.41854,
    "ESO_ACC_HOR_MAX": 9.32449,
    "ESO_JERK_AUTO": 7.10799,
    "ESO_JERK_MAX": 7.4,
    "ESO_ROLL_P": 1.63571,
    "ESO_PITCH_P": 1.57133,
    "ESO_ROLLRATE_P": 0.24055,
    "ESO_PITCHRATE_P": 0.16968,
    "ESO_YAW_P": 0.8,
    "ESO_YAW_WEIGHT": 0.35,
    "ESO_YAWRATE_P": 0.16,
    "ESO_RATE_BW_R": 3.7450942682339265,
    "ESO_RATE_BW_P": 2.272848954518408,
    "ESO_RATE_BW_Y": 0.8,
    "ESO_RATE_I_SC": 0.12430763687449435,
}

COMMON_EXP4_ARGS = {
    "startup_delay_s": 45.0,
    "activation_hold_s": 1.0,
    "path_speed_mps": 0.09621358687658534,
    "use_raw_setpoint": "true",
    "use_td_setpoint": "true",
    "velocity_ff_scale": 0.008279872451230363,
    "td_bandwidth_hz": 0.32880281660571076,
    "td_accel_limit_mps2": 0.19318573839061587,
    "td_vel_limit_mps": 0.22063069013967385,
}

EXPERIMENTS = {
    "mode1": {
        "name": "exp1_hover_disturbance_uam_v5",
        "launch": "exp1_hover_disturbance_uam_v5.launch",
        "run_after_enable_s": 65.0,
        "enable_timeout_s": 120.0,
        "crop_duration_s": 60.0,
        "launch_args": {},
    },
    "mode2": {
        "name": "exp4_square_tracking_uam_v5",
        "launch": "exp4_square_tracking_uam_v5.launch",
        "run_after_enable_s": 105.0,
        "enable_timeout_s": 150.0,
        "crop_duration_s": None,
        "launch_args": COMMON_EXP4_ARGS,
    },
}

STACKS = [
    ("px4_pid", "PX4-PID", {}),
    ("paper_eso", "本文算法 ESO", ESO_PARAMS),
]


def start_process(command: str, log_path: Path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=str(REPO),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        stdin=subprocess.PIPE,
        preexec_fn=os.setsid,
        text=True,
    )
    return proc, log_file


def stop_process(process, log_file=None, grace_s=8.0):
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        deadline = time.time() + grace_s
        while time.time() < deadline and process.poll() is None:
            time.sleep(0.2)
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
    if process.stdin is not None:
        try:
            process.stdin.close()
        except OSError:
            pass
    if log_file is not None:
        log_file.close()


def wait_for_enable_true(timeout_s: float, ros_home: Path) -> bool:
    command = auto.shell_cmd("rostopic echo /experiment/arm_motion_enabled/data", ros_home)
    proc = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=str(REPO),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    try:
        fd = proc.stdout.fileno()
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            ready, _, _ = select.select([fd], [], [], 0.5)
            if not ready:
                if proc.poll() is not None:
                    break
                continue
            line = proc.stdout.readline()
            if line and line.strip().lower() == "true":
                return True
        return False
    finally:
        stop_process(proc)


def launch_arg_text(args: dict) -> str:
    return " ".join(f"{key}:={value}" for key, value in args.items())


def run_fresh_case(stack: str, params: dict, mode: str, cfg: dict, session_dir: Path) -> dict:
    run_root = session_dir / f"{stack}_{mode}_{cfg['name']}"
    recorder_output = run_root / "bags"
    ros_home = run_root / "ros_home"
    recorder_output.mkdir(parents=True, exist_ok=True)

    launch_args = launch_arg_text(cfg["launch_args"])
    launch_cmd = auto.shell_cmd(
        f"roslaunch uav_arm_top {cfg['launch']} gui:=false {launch_args}".strip(),
        ros_home,
    )
    recorder_cmd = auto.shell_cmd(
        "rosrun uav_control experiment_data_recorder.py "
        f"_experiment_name:={cfg['name']} _output_dir:={recorder_output} "
        "_record_arm_topics:=true _record_state_topic:=false _record_motor_speed_topics:=false",
        ros_home,
    )

    launch_proc = recorder_proc = None
    launch_file = recorder_file = None
    try:
        auto.cleanup_sim_processes()
        launch_proc, launch_file = start_process(launch_cmd, run_root / "roslaunch.log")
        time.sleep(18.0)
        if params:
            auto.set_mavros_params(params, run_root / "mavparam_set.log", ros_home)
        else:
            (run_root / "mavparam_set.log").write_text(
                "PX4-PID baseline: no ESO parameters injected by this script.\n",
                encoding="utf-8",
            )
        recorder_proc, recorder_file = start_process(recorder_cmd, run_root / "recorder.log")
        bag_run_dir = auto.wait_for_recorder_run_dir(recorder_output, cfg["name"])
        if not wait_for_enable_true(cfg["enable_timeout_s"], ros_home):
            raise RuntimeError(f"Timed out waiting for arm motion enable in {stack} {mode}")
        time.sleep(cfg["run_after_enable_s"])
    finally:
        if recorder_proc is not None:
            stop_process(recorder_proc, recorder_file)
        if launch_proc is not None:
            stop_process(launch_proc, launch_file)
        auto.cleanup_sim_processes()
        time.sleep(5.0)

    source_bag = bag_run_dir / f"{cfg['name']}.bag"
    source_meta = bag_run_dir / f"{cfg['name']}_metadata.json"
    canonical_base = f"px4_sitl_{mode}_{stack}"
    canonical_bag = RAW_DIR / f"{canonical_base}.bag"
    canonical_meta = RAW_DIR / f"{canonical_base}_metadata.json"
    shutil.copy2(source_bag, canonical_bag)
    if source_meta.exists():
        shutil.copy2(source_meta, canonical_meta)

    return {
        "stack": stack,
        "mode": mode,
        "experiment": cfg["name"],
        "run_root": str(run_root),
        "bag": str(canonical_bag),
        "metadata": str(canonical_meta),
        "params": params,
    }


def vec_from_pose(msg) -> list[float]:
    return [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]


def vec_from_target(msg) -> list[float]:
    return [msg.position.x, msg.position.y, msg.position.z]


def read_bag_series(bag_path: Path) -> dict:
    joints = ["arm_joint1", "arm_joint2", "left_hand_joint"]
    data = {
        "actual_t": [],
        "actual_p": [],
        "desired_t": [],
        "desired_p": [],
        "joint_t": [],
        "joint_q": [],
        "target_joint_t": [],
        "target_joint_q": [],
        "arm_start": None,
        "bag_start": None,
    }
    with rosbag.Bag(str(bag_path), "r") as bag:
        data["bag_start"] = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages():
            ts = stamp.to_sec()
            if topic == "/experiment/arm_motion_enabled" and bool(msg.data):
                if data["arm_start"] is None:
                    data["arm_start"] = ts
            elif topic == "/mavros/local_position/pose":
                data["actual_t"].append(ts)
                data["actual_p"].append(vec_from_pose(msg))
            elif topic == "/mavros/setpoint_position/local":
                data["desired_t"].append(ts)
                data["desired_p"].append(vec_from_pose(msg))
            elif topic == "/mavros/setpoint_raw/local":
                data["desired_t"].append(ts)
                data["desired_p"].append(vec_from_target(msg))
            elif topic == "/uav_arm/joint_states":
                idx = {name: i for i, name in enumerate(msg.name)}
                if all(joint in idx for joint in joints):
                    data["joint_t"].append(ts)
                    data["joint_q"].append([msg.position[idx[joint]] for joint in joints])
            elif topic == "/uav_arm/target_joint_states":
                idx = {name: i for i, name in enumerate(msg.name)}
                if all(joint in idx for joint in joints):
                    data["target_joint_t"].append(ts)
                    data["target_joint_q"].append([msg.position[idx[joint]] for joint in joints])
    if data["arm_start"] is None:
        raise RuntimeError(f"{bag_path} has no /experiment/arm_motion_enabled true marker")
    return {key: np.asarray(value, dtype=float) if isinstance(value, list) else value for key, value in data.items()}


def interp_columns(source_t: np.ndarray, source_y: np.ndarray, target_t: np.ndarray) -> np.ndarray:
    if source_t.size == 0 or source_y.size == 0:
        raise RuntimeError("cannot interpolate empty signal")
    return np.column_stack([
        np.interp(target_t, source_t, source_y[:, axis])
        for axis in range(source_y.shape[1])
    ])


def exp4_expected_complete_s() -> float:
    side_length = 2.0
    corner_hold_s = 3.0
    path_speed = float(COMMON_EXP4_ARGS["path_speed_mps"])
    return 4.0 * (corner_hold_s + side_length / path_speed)


def choose_crop_end(mode: str, rel_t: np.ndarray, desired: np.ndarray, cfg: dict) -> float:
    if mode == "mode1":
        return float(cfg["crop_duration_s"])

    expected = exp4_expected_complete_s()
    xy_norm = np.linalg.norm(desired[:, :2], axis=1)
    traversed_square = np.maximum.accumulate(desired[:, 0]) > 1.5
    traversed_square &= np.maximum.accumulate(desired[:, 1]) > 1.5
    candidates = np.where((rel_t >= expected) & traversed_square & (xy_norm <= 0.08))[0]
    if candidates.size:
        return float(rel_t[candidates[0]])
    return expected


def build_result_from_bag(bag_path: Path, stack: str, mode: str, cfg: dict) -> tuple[dict, dict]:
    series = read_bag_series(bag_path)
    arm_start = float(series["arm_start"])
    actual_t_abs = series["actual_t"]
    desired = interp_columns(series["desired_t"], series["desired_p"], actual_t_abs)
    rel_t_all = actual_t_abs - arm_start
    crop_end = choose_crop_end(mode, rel_t_all, desired, cfg)
    mask = (rel_t_all >= 0.0) & (rel_t_all <= crop_end)
    if np.count_nonzero(mask) < 20:
        raise RuntimeError(f"{bag_path} produced too few samples after crop")

    t_abs = actual_t_abs[mask]
    t = t_abs - arm_start
    p_true = series["actual_p"][mask]
    p_desired = desired[mask]
    q_true = interp_columns(series["joint_t"], series["joint_q"], t_abs)
    q_desired = interp_columns(series["target_joint_t"], series["target_joint_q"], t_abs)
    n = min(t.size, p_true.shape[0], p_desired.shape[0], q_true.shape[0], q_desired.shape[0])
    t = t[:n]
    p_true = p_true[:n, :]
    p_desired = p_desired[:n, :]
    q_true = q_true[:n, :]
    q_desired = q_desired[:n, :]

    p_error = p_true - p_desired
    p_error_abs = np.abs(p_error)
    p_error_norm = np.linalg.norm(p_error, axis=1)
    q_error = q_true - q_desired
    q_error_abs = np.abs(q_error)

    signals = {
        "p_true": {"time": t, "data": p_true},
        "p_actual": {"time": t, "data": p_true},
        "p_desired": {"time": t, "data": p_desired},
        "q_true": {"time": t, "data": q_true},
        "q_desired": {"time": t, "data": q_desired},
        "f": {"time": t, "data": np.zeros((n, 1))},
        "tau": {"time": t, "data": np.zeros((n, 3))},
    }
    metrics = {
        "position_axis_mean": np.mean(p_error_abs, axis=0),
        "position_axis_max": np.max(p_error_abs, axis=0),
        "position_axis_rms": np.sqrt(np.mean(p_error ** 2, axis=0)),
        "position_rms": float(np.sqrt(np.mean(p_error_norm ** 2))),
        "position_max": float(np.max(p_error_norm)),
        "arm_axis_max": np.max(q_error_abs, axis=0),
        "arm_axis_rms": np.sqrt(np.mean(q_error ** 2, axis=0)),
        "arm_rms": float(np.sqrt(np.mean(np.linalg.norm(q_error, axis=1) ** 2))),
        "arm_max": float(np.max(np.linalg.norm(q_error, axis=1))),
        "is_divergent": bool(np.any(np.max(p_error_abs, axis=0) > 10.0)),
        "thrust_saturation_ratio": 0.0,
        "torque_saturation_ratio": 0.0,
    }
    result = {
        "config": {
            "source": "PX4 SITL Gazebo UAM V5",
            "controller": stack,
            "mode": mode,
            "experiment": cfg["name"],
            "crop_start": "first /experiment/arm_motion_enabled true",
            "crop_duration_s": float(t[-1] - t[0]) if t.size else 0.0,
            "bag": str(bag_path),
        },
        "signals": signals,
        "metrics": metrics,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    summary = {
        "bag": str(bag_path),
        "arm_start_ros_time_s": arm_start,
        "samples": int(n),
        "crop_start_s": 0.0,
        "crop_end_s": float(t[-1]),
        "position_axis_mean": metrics["position_axis_mean"].tolist(),
        "position_axis_max": metrics["position_axis_max"].tolist(),
        "position_rms": metrics["position_rms"],
        "position_max": metrics["position_max"],
        "arm_axis_max": metrics["arm_axis_max"].tolist(),
        "is_divergent": metrics["is_divergent"],
    }
    return result, summary


def write_mat_and_summary(run_rows: list[dict]) -> dict:
    report = {}
    for row in run_rows:
        mode = row["mode"]
        stack = row["stack"]
        cfg = EXPERIMENTS[mode]
        result, summary = build_result_from_bag(Path(row["bag"]), stack, mode, cfg)
        mat_path = RAW_DIR / f"px4_sitl_{mode}_{stack}.mat"
        result["output_file"] = str(mat_path)
        savemat(mat_path, {"result": result}, long_field_names=True)
        row["mat"] = str(mat_path)
        row["summary"] = summary
        report.setdefault(mode, {})[stack] = summary
    (RAW_DIR / "px4_sitl_comparison_summary.json").write_text(
        json.dumps({"runs": run_rows, "report": report}, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return report


def main() -> int:
    global RAW_DIR, FIGURE_DIR, RUN_ROOT

    dataset_name = f"px4_sitl_comparison_{time.strftime('%Y%m%d_%H%M%S')}"
    RAW_DIR = RAW_ROOT / dataset_name
    FIGURE_DIR = FIGURE_ROOT / dataset_name
    RUN_ROOT = RAW_DIR / "fresh_runs"

    RAW_DIR.mkdir(parents=True, exist_ok=True)
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    session_dir = RUN_ROOT / time.strftime("%Y%m%d_%H%M%S")
    session_dir.mkdir(parents=True, exist_ok=True)

    run_rows = []
    for stack, label, params in STACKS:
        for mode, cfg in EXPERIMENTS.items():
            print(f"RUN {label} {mode} {cfg['name']}", flush=True)
            run_rows.append(run_fresh_case(stack, params, mode, cfg, session_dir))
            (RAW_DIR / "px4_sitl_run_progress.json").write_text(
                json.dumps(
                    {
                        "dataset": dataset_name,
                        "raw_dir": str(RAW_DIR),
                        "figure_dir": str(FIGURE_DIR),
                        "session_dir": str(session_dir),
                        "runs": run_rows,
                    },
                    indent=2,
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )

    report = write_mat_and_summary(run_rows)
    print(json.dumps({
        "dataset": dataset_name,
        "raw_dir": str(RAW_DIR),
        "figure_dir": str(FIGURE_DIR),
        "session_dir": str(session_dir),
        "report": report,
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
