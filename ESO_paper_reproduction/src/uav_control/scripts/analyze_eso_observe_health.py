#!/usr/bin/env python3
"""Extract ESO observe-only health metrics from a PX4 ULog.

The window is expressed in ULog seconds since boot. This script intentionally
uses logged uORB status topics only; it does not depend on plotting heuristics.
"""

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
from pyulog import ULog


def dataset(ulog, name):
    for data in ulog.data_list:
        if data.name == name:
            return data
    return None


def window_data(data, start_s, duration_s):
    timestamps = np.asarray(data.data["timestamp"], dtype=float) * 1e-6
    mask = (timestamps >= start_s) & (timestamps <= start_s + duration_s)
    return timestamps[mask], {key: np.asarray(value)[mask] for key, value in data.data.items()}


def vec(values, prefix):
    return np.column_stack([values[f"{prefix}[{axis}]"] for axis in range(3)])


def rms(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return math.nan
    return float(math.sqrt(np.mean(np.square(x))))


def p95_abs(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return math.nan
    return float(np.percentile(np.abs(x), 95))


def finite_ratio(arr):
    arr = np.asarray(arr, dtype=float)
    if arr.size == 0:
        return 0.0
    return float(np.isfinite(arr).sum() / arr.size)


def control_index(name):
    match = re.search(r"\[(\d+)\]", name)
    return int(match.group(1)) if match else 999


def diff_highfreq_ratio(arr):
    """A conservative logged-data proxy for high-frequency content.

    rate_ctrl_status is commonly logged at low rate in SITL. The first-difference
    ratio cannot prove absence of >3 Hz noise when the log rate is too low, but it
    still detects stale jumps and visibly noisy logged observer estimates.
    """
    arr = np.asarray(arr, dtype=float)
    if arr.shape[0] < 3:
        return math.nan
    centered = arr - np.mean(arr, axis=0)
    denom = rms(centered)
    if denom <= 1e-9 or not math.isfinite(denom):
        return 0.0
    return float(rms(np.diff(arr, axis=0)) / denom)


def summarize(ulog_path, start_s, duration_s):
    ulog = ULog(str(ulog_path))
    result = {
        "ulog_path": str(ulog_path),
        "window_start_s": float(start_s),
        "window_duration_s": float(duration_s),
        "window_end_s": float(start_s + duration_s),
        "topics_present": {},
        "hard_fail_reasons": [],
        "notes": [],
    }

    rate = dataset(ulog, "rate_ctrl_status")
    if rate is None:
        result["topics_present"]["rate_ctrl_status"] = False
        result["hard_fail_reasons"].append("missing_rate_ctrl_status")
    else:
        result["topics_present"]["rate_ctrl_status"] = True
        t, values = window_data(rate, start_s, duration_s)
        result["rate_ctrl_status_sample_count"] = int(t.size)
        if t.size >= 2:
            result["rate_ctrl_status_rate_hz"] = float((t.size - 1) / (t[-1] - t[0]))
        else:
            result["rate_ctrl_status_rate_hz"] = math.nan
            result["hard_fail_reasons"].append("rate_ctrl_status_window_samples_lt_2")

        if t.size:
            eso_rate = vec(values, "eso_rate")
            eso_rate_hat = vec(values, "eso_rate_hat")
            eso_rate_error = vec(values, "eso_rate_error")
            disturbance = vec(values, "eso_disturbance_hat")
            tau_s_used = vec(values, "eso_tau_s_used")
            integrator = np.column_stack([
                values["rollspeed_integ"],
                values["pitchspeed_integ"],
                values["yawspeed_integ"],
            ])
            result["eso_finite_ratio"] = finite_ratio(
                np.column_stack([eso_rate, eso_rate_hat, eso_rate_error, disturbance, tau_s_used])
            )
            result["rate_hat_rmse_rad_s"] = rms(eso_rate - eso_rate_hat)
            result["rate_error_rmse_rad_s"] = rms(eso_rate_error)
            result["rate_error_p95_rad_s"] = p95_abs(eso_rate_error)
            result["disturbance_hat_rms"] = rms(disturbance)
            result["disturbance_hat_highfreq_ratio_proxy"] = diff_highfreq_ratio(disturbance)
            result["tau_s_used_max"] = float(np.max(np.abs(tau_s_used)))
            result["rate_integrator_abs_max"] = float(np.max(np.abs(integrator)))
            if result["eso_finite_ratio"] < 1.0:
                result["hard_fail_reasons"].append("nonfinite_eso_status")
            if result["rate_hat_rmse_rad_s"] >= 0.15:
                result["hard_fail_reasons"].append("rate_hat_rmse_ge_0p15")
            if result["disturbance_hat_highfreq_ratio_proxy"] >= 0.35:
                result["hard_fail_reasons"].append("disturbance_hat_highfreq_proxy_ge_0p35")
            if result.get("rate_ctrl_status_rate_hz", 0.0) < 20.0:
                result["notes"].append("rate_ctrl_status log rate below 20 Hz; high-frequency observer noise gate is a low-rate proxy only")

    status = dataset(ulog, "vehicle_status")
    if status is not None:
        result["topics_present"]["vehicle_status"] = True
        _, values = window_data(status, start_s, duration_s)
        nav_state = values.get("nav_state")
        arming_state = values.get("arming_state")
        failsafe = values.get("failsafe")
        if nav_state is not None and nav_state.size:
            result["nav_state_unique"] = sorted({int(x) for x in nav_state.tolist()})
        if arming_state is not None and arming_state.size:
            result["arming_state_unique"] = sorted({int(x) for x in arming_state.tolist()})
        if failsafe is not None and failsafe.size:
            result["failsafe_count"] = int(np.asarray(failsafe, dtype=bool).sum())
            if result["failsafe_count"] > 0:
                result["hard_fail_reasons"].append("failsafe_in_window")
    else:
        result["topics_present"]["vehicle_status"] = False

    control_mode = dataset(ulog, "vehicle_control_mode")
    if control_mode is not None:
        result["topics_present"]["vehicle_control_mode"] = True
        _, values = window_data(control_mode, start_s, duration_s)
        flag = values.get("flag_control_offboard_enabled")
        if flag is not None and flag.size:
            result["offboard_enabled_ratio"] = float(np.asarray(flag, dtype=bool).sum() / flag.size)
            if result["offboard_enabled_ratio"] < 1.0:
                result["hard_fail_reasons"].append("offboard_not_continuous")
    else:
        result["topics_present"]["vehicle_control_mode"] = False

    actuator = dataset(ulog, "actuator_controls_0")
    if actuator is not None:
        result["topics_present"]["actuator_controls_0"] = True
        _, values = window_data(actuator, start_s, duration_s)
        control_cols = [key for key in values if key.startswith("control[")]
        if control_cols:
            ordered = sorted(control_cols, key=control_index)
            controls = np.column_stack([values[key] for key in ordered])
            result["actuator_control_abs_max"] = float(np.max(np.abs(controls)))
            result["actuator_saturation_ratio_0p98"] = float((np.abs(controls) >= 0.98).sum() / controls.size)
            if controls.shape[1] >= 3:
                torque_controls = controls[:, :3]
                result["torque_control_abs_max"] = float(np.max(np.abs(torque_controls)))
                result["torque_saturation_ratio_0p98"] = float(
                    (np.abs(torque_controls) >= 0.98).sum() / torque_controls.size
                )
                if result["torque_saturation_ratio_0p98"] >= 0.01:
                    result["hard_fail_reasons"].append("torque_saturation_ratio_ge_0p01")
    else:
        result["topics_present"]["actuator_controls_0"] = False

    result["observer_health_passed"] = not result["hard_fail_reasons"]
    return result


def main():
    parser = argparse.ArgumentParser(description="Analyze ESO observe-only ULog health metrics.")
    parser.add_argument("ulog", help="Input .ulg file")
    parser.add_argument("--start", type=float, required=True, help="Window start in ULog seconds since boot")
    parser.add_argument("--duration", type=float, required=True, help="Window duration in seconds")
    parser.add_argument("--out", default="", help="Output JSON path")
    args = parser.parse_args()

    metrics = summarize(Path(args.ulog).expanduser().resolve(), args.start, args.duration)
    text = json.dumps(metrics, indent=2, ensure_ascii=False, allow_nan=False)
    if args.out:
        Path(args.out).expanduser().resolve().write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
