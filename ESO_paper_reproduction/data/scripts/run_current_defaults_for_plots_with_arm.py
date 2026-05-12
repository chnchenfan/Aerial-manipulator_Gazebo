#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path


REPO = Path("/home/cf/PX4_Firmware_clean")
AUTO_SCRIPT = REPO / "ESO_paper_reproduction/src/uav_control/scripts/auto_tune_uam_v5_eso.py"
OUT_ROOT = REPO / "ESO_paper_reproduction/src/uav_arm_top/auto_tune_data"

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

EXP4_ARGS = (
    "startup_delay_s:=45.0 activation_hold_s:=1.0 "
    "path_speed_mps:=0.09621358687658534 "
    "use_raw_setpoint:=true use_td_setpoint:=true "
    "velocity_ff_scale:=0.008279872451230363 "
    "td_bandwidth_hz:=0.32880281660571076 "
    "td_accel_limit_mps2:=0.19318573839061587 "
    "td_vel_limit_mps:=0.22063069013967385"
)


def start_process(command: str, log_path: Path):
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


def run_experiment_with_arm(stack: str, experiment: dict, out: Path) -> dict:
    params = dict(ESO_PARAMS) if stack == "eso" else {}
    launch_args = ""
    if experiment["name"] == "exp4_square_tracking_uam_v5":
        launch_args = EXP4_ARGS
        if stack == "eso":
            params["__launch_args"] = launch_args
    run_root = out / f"{stack}_with_arm_{experiment['name']}" / experiment["name"]
    run_root.mkdir(parents=True, exist_ok=True)
    recorder_output = run_root / "bags"
    recorder_output.mkdir(parents=True, exist_ok=True)
    ros_home = run_root / "ros_home"

    launch_cmd = auto.shell_cmd(
        f"roslaunch uav_arm_top {experiment['launch']} gui:=false {launch_args}".strip(),
        ros_home,
    )
    recorder_cmd = auto.shell_cmd(
        "rosrun uav_control experiment_data_recorder.py "
        f"_experiment_name:={experiment['name']} _output_dir:={recorder_output} "
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
        recorder_proc, recorder_file = start_process(recorder_cmd, run_root / "recorder.log")
        run_dir = auto.wait_for_recorder_run_dir(recorder_output, experiment["name"])
        if not wait_for_enable_true(experiment["enable_timeout_s"], ros_home):
            raise RuntimeError("Timed out waiting for arm_motion_enabled")
        time.sleep(experiment["run_after_enable_s"])
    finally:
        if recorder_proc is not None:
            stop_process(recorder_proc, recorder_file)
        if launch_proc is not None:
            stop_process(launch_proc, launch_file)
        auto.cleanup_sim_processes()
        time.sleep(5.0)
    bag_path = run_dir / f"{experiment['name']}.bag"
    return {"experiment": experiment["name"], "bag_path": str(bag_path)}


def main() -> int:
    if len(sys.argv) != 2 or sys.argv[1] not in {"eso", "mc"}:
        print("usage: run_current_defaults_for_plots_with_arm.py {eso|mc}", file=sys.stderr)
        return 2
    stack = sys.argv[1]
    out = OUT_ROOT / f"manual_{time.strftime('%Y%m%d_%H%M%S')}_current_default_{stack}_plots_with_arm"
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for experiment in auto.EXPERIMENTS:
        rows.append(run_experiment_with_arm(stack, experiment, out))
        (out / "with_arm_summary.json").write_text(json.dumps({"stack": stack, "results": rows}, indent=2), encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
