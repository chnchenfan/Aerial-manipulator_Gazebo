#!/usr/bin/env python3
"""Offline tau_s feed-forward scale and reference-point calibration.

The script intentionally does not modify PX4 parameters or skill memory. It
uses fresh run artifacts to reconstruct the actuator torque scale, compare the
logged tau_s term against reference-point hypotheses, and estimate per-axis
tau_s signs/scales before any closed-loop tau_s experiment is attempted.
"""

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from pyulog import ULog


DEFAULT_SDF = "Tools/sitl_gazebo/models/uav_arm_v4/uav_arm_v4.sdf"
DEFAULT_MIXER_GEOMETRY = "src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml"
AXES = ("roll", "pitch", "yaw")
PWM_DEFAULT_MIN = 1000.0
PWM_DEFAULT_MAX = 2000.0
UAM_V5_PROFILE = {
    "mass_total": 2.57,
    "com_frd": np.array([0.04517874, 0.00408382, -0.13967335], dtype=float),
    "inertia_diag": np.array(
        [0.02601 + 0.00756734, 0.02942848 + 0.02004877, 0.04177777 + 0.01403721],
        dtype=float,
    ),
}
UAV_ARM_V4_PROFILE = {
    "mass_total": 1.289,
    "com_frd": np.array([-0.01, 0.0, 0.161], dtype=float),
    "inertia_diag": np.array([0.045 + 0.016, 0.045 + 0.016, 0.08 + 0.016], dtype=float),
}


def as_list(value):
    return [float(x) for x in value.strip().split()]


def flu_to_frd(v):
    return np.array([v[0], -v[1], -v[2]], dtype=float)


def parse_sdf_model(sdf_path, convert_flu_to_frd=True):
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    link_positions = {}
    for link in root.findall(".//link"):
        name = link.attrib.get("name", "")
        pose = link.findtext("pose")
        if pose:
            xyz = np.array(as_list(pose)[:3], dtype=float)
            link_positions[name] = flu_to_frd(xyz) if convert_flu_to_frd else xyz

    rotors = []
    motor_by_number = {}
    for plugin in root.findall(".//plugin"):
        filename = plugin.attrib.get("filename", "")
        if "gazebo_motor_model" not in filename:
            continue

        motor_number = int(plugin.findtext("motorNumber"))
        link_name = plugin.findtext("linkName")
        direction = plugin.findtext("turningDirection").strip().lower()
        position = link_positions[link_name]
        motor_constant = float(plugin.findtext("motorConstant"))
        moment_constant = float(plugin.findtext("momentConstant"))
        max_rot_velocity = float(plugin.findtext("maxRotVelocity"))
        rotor_velocity_slowdown = float(plugin.findtext("rotorVelocitySlowdownSim", "10"))
        motor = {
            "motor_number": motor_number,
            "link_name": link_name,
            "direction": direction,
            "position_frd_m": position,
            "motor_constant": motor_constant,
            "moment_constant": moment_constant,
            "max_rot_velocity": max_rot_velocity,
            "rotor_velocity_slowdown_sim": rotor_velocity_slowdown,
        }
        rotors.append(motor)
        motor_by_number[motor_number] = motor

    control_channels = []
    for channel in root.findall(".//plugin[@name='mavlink_interface']/control_channels/channel"):
        name = channel.attrib.get("name", "")
        input_index = int(channel.findtext("input_index"))
        control_channels.append(
            {
                "name": name,
                "input_index": input_index,
                "input_offset": float(channel.findtext("input_offset", "0")),
                "input_scaling": float(channel.findtext("input_scaling", "1")),
                "zero_position_disarmed": float(channel.findtext("zero_position_disarmed", "0")),
                "zero_position_armed": float(channel.findtext("zero_position_armed", "0")),
                "joint_control_type": channel.findtext("joint_control_type", "").strip(),
                "motor_number": input_index if input_index in motor_by_number else None,
            }
        )

    return sorted(rotors, key=lambda r: r["motor_number"]), sorted(control_channels, key=lambda c: c["input_index"])


def parse_quad_wide_geometry(path):
    try:
        import tomllib

        with Path(path).open("rb") as f:
            data = tomllib.load(f)
        rotors = data.get("rotors", [])
        return [
            {
                "name": rotor.get("name", ""),
                "position_frd_m": [float(x) for x in rotor.get("position", [0, 0, 0])],
                "direction": str(rotor.get("direction", "")).lower(),
            }
            for rotor in rotors
        ]
    except Exception:
        # Minimal fallback for the simple PX4 mixer geometry TOML used here.
        rotors = []
        current = None
        for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            if line == "[[rotors]]":
                if current is not None:
                    rotors.append(current)
                current = {}
            elif current is not None and "=" in line:
                key, value = [x.strip() for x in line.split("=", 1)]
                if key == "name":
                    current["name"] = value.strip('"')
                elif key == "position":
                    current["position_frd_m"] = [float(x.strip()) for x in value.strip("[]").split(",")]
                elif key == "direction":
                    current["direction"] = value.strip('"').lower()
        if current is not None:
            rotors.append(current)
        return rotors


def dataset(ulog, name):
    matches = [d for d in ulog.data_list if d.name == name]
    if not matches:
        raise RuntimeError(f"ULog does not contain required dataset: {name}")
    return matches[0].data


def optional_dataset(ulog, name):
    matches = [d for d in ulog.data_list if d.name == name]
    return matches[0].data if matches else None


def columns(data, prefix, count=3):
    return np.column_stack([np.asarray(data[f"{prefix}[{i}]"], dtype=float) for i in range(count)])


def interp_matrix(target_t, source_t, values):
    return np.column_stack([np.interp(target_t, source_t, values[:, i]) for i in range(values.shape[1])])


def interp_vector(target_t, source_t, values):
    return np.interp(target_t, source_t, values)


def interpolate_armed_mask(ulog, target_t):
    armed = optional_dataset(ulog, "actuator_armed")
    if armed is not None and "armed" in armed:
        t = np.asarray(armed["timestamp"], dtype=float) * 1e-6
        values = np.asarray(armed["armed"], dtype=float)
        return np.interp(target_t, t, values) >= 0.5

    status = optional_dataset(ulog, "vehicle_status")
    if status is not None and "arming_state" in status:
        t = np.asarray(status["timestamp"], dtype=float) * 1e-6
        values = np.asarray(status["arming_state"], dtype=float)
        # vehicle_status_s::ARMING_STATE_ARMED == 2.
        return np.interp(target_t, t, values) >= 1.5

    return np.ones_like(target_t, dtype=bool)


def quat_to_dcm(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def load_rate_log(ulog):
    rate = dataset(ulog, "rate_ctrl_status")
    t = np.asarray(rate["timestamp"], dtype=float) * 1e-6
    return {
        "t": t,
        "eso_torque": columns(rate, "eso_torque"),
        "tau_s_raw": columns(rate, "eso_tau_s_raw"),
        "tau_s_used": columns(rate, "eso_tau_s_used"),
        "disturbance_hat": columns(rate, "eso_disturbance_hat"),
    }


def actuator_outputs_to_omega(actuator_outputs, target_t, rotors, args):
    t = np.asarray(actuator_outputs["timestamp"], dtype=float) * 1e-6
    pwm = np.column_stack(
        [np.asarray(actuator_outputs[f"output[{r['motor_number']}]"], dtype=float) for r in rotors]
    )
    pwm_i = interp_matrix(target_t, t, pwm)

    max_omega = np.array([r["max_rot_velocity"] for r in rotors], dtype=float)

    if args.pwm_mode == "normalized-max":
        norm = np.clip((pwm_i - args.pwm_min) / max(args.pwm_max - args.pwm_min, 1e-6), 0.0, 1.0)
        omega = norm * max_omega
    elif args.pwm_mode == "offset-scale":
        omega = np.clip((pwm_i - args.pwm_offset) * args.pwm_omega_scale, 0.0, max_omega)
    else:
        raise ValueError(f"Unsupported pwm mode: {args.pwm_mode}")

    return omega, {
        "source": "actuator_outputs_pwm",
        "pwm_mode": args.pwm_mode,
        "pwm_min": args.pwm_min,
        "pwm_max": args.pwm_max,
        "pwm_offset": args.pwm_offset,
        "pwm_omega_scale": args.pwm_omega_scale,
        "warning": "actuator_outputs are PWM-like values; reconstructed torque depends on this explicit mapping.",
    }


def actuator_outputs_to_gazebo_omega(actuator_outputs, armed_mask, target_t, rotors, control_channels, args):
    """Rebuild the motor reference sent by GazeboMavlinkInterface.

    PX4 sends HIL_ACTUATOR_CONTROLS from actuator_outputs. For multirotors,
    PWM_DEFAULT_MIN..PWM_DEFAULT_MAX is mapped to MAVLink control 0..1. Gazebo
    then applies the per-channel SDF mapping:
    input_reference = (control + input_offset) * input_scaling + zero_position_armed.
    """
    t = np.asarray(actuator_outputs["timestamp"], dtype=float) * 1e-6
    pwm = np.column_stack(
        [np.asarray(actuator_outputs[f"output[{r['motor_number']}]"], dtype=float) for r in rotors]
    )
    pwm_i = interp_matrix(target_t, t, pwm)
    armed = armed_mask[:, None]

    hil_controls = np.where(
        pwm_i > PWM_DEFAULT_MIN / 2.0,
        (pwm_i - args.hil_pwm_min) / max(args.hil_pwm_max - args.hil_pwm_min, 1e-6),
        0.0,
    )

    channels_by_input = {c["input_index"]: c for c in control_channels}
    omega = np.zeros_like(hil_controls)
    channel_meta = []
    for i, rotor in enumerate(rotors):
        channel = channels_by_input.get(rotor["motor_number"], {})
        input_scaling = float(channel.get("input_scaling", args.default_input_scaling))
        input_offset = float(channel.get("input_offset", 0.0))
        zero_armed = float(channel.get("zero_position_armed", args.default_zero_position_armed))
        zero_disarmed = float(channel.get("zero_position_disarmed", 0.0))
        max_omega = float(rotor["max_rot_velocity"])
        ref = (hil_controls[:, i] + input_offset) * input_scaling + zero_armed
        ref = np.where(armed[:, 0], ref, zero_disarmed)
        omega[:, i] = np.clip(ref, 0.0, max_omega)
        channel_meta.append(
            {
                "motor_number": rotor["motor_number"],
                "input_index": channel.get("input_index", rotor["motor_number"]),
                "input_scaling": input_scaling,
                "input_offset": input_offset,
                "zero_position_armed": zero_armed,
                "zero_position_disarmed": zero_disarmed,
            }
        )

    return omega, {
        "source": "actuator_outputs_hil_to_gazebo_motor_reference",
        "hil_pwm_min": args.hil_pwm_min,
        "hil_pwm_max": args.hil_pwm_max,
        "gazebo_control_channels": channel_meta,
        "armed_samples": int(np.count_nonzero(armed_mask)),
        "sample_count": int(target_t.size),
        "note": (
            "This follows PX4 HIL_ACTUATOR_CONTROLS plus the uav_arm_v4 SDF "
            "mavlink_interface mapping. It is the preferred SITL torque source "
            "when Gazebo transport /motor_speed is not captured."
        ),
    }


def actuator_controls0_to_gazebo_omega(actuator_controls, armed_mask, target_t, rotors, control_channels, args):
    """Diagnostic-only direct mapping of actuator_controls_0 channels.

    This is not the normal PX4 SITL motor path for this airframe; it is kept so
    reports can show why actuator_controls_0 should not be used as final motor
    evidence unless the simulator is configured for dynamic direct mixing.
    """
    t = np.asarray(actuator_controls["timestamp"], dtype=float) * 1e-6
    controls = np.column_stack(
        [np.asarray(actuator_controls[f"control[{r['motor_number']}]"], dtype=float) for r in rotors]
    )
    controls_i = interp_matrix(target_t, t, controls)
    channels_by_input = {c["input_index"]: c for c in control_channels}
    omega = np.zeros_like(controls_i)
    for i, rotor in enumerate(rotors):
        channel = channels_by_input.get(rotor["motor_number"], {})
        input_scaling = float(channel.get("input_scaling", args.default_input_scaling))
        input_offset = float(channel.get("input_offset", 0.0))
        zero_armed = float(channel.get("zero_position_armed", args.default_zero_position_armed))
        zero_disarmed = float(channel.get("zero_position_disarmed", 0.0))
        ref = (controls_i[:, i] + input_offset) * input_scaling + zero_armed
        ref = np.where(armed_mask, ref, zero_disarmed)
        omega[:, i] = np.clip(ref, 0.0, float(rotor["max_rot_velocity"]))

    return omega, {
        "source": "actuator_controls_0_direct_diagnostic",
        "warning": (
            "actuator_controls_0 is the attitude control group, not the final "
            "four motor commands in the current PX4 SITL path. Use this only "
            "as a diagnostic comparison, not as physical torque evidence."
        ),
    }


def read_motor_speed_value(msg):
    for attr in ("data", "angular_velocity", "velocity", "motor_speed"):
        if hasattr(msg, attr):
            return float(getattr(msg, attr))
    raise RuntimeError(f"Unsupported motor speed message type: {type(msg)}")


def bag_motor_speeds_to_omega(bag_path, target_t, rotors, time_offset):
    import rosbag

    topics = [f"/motor_speed/{r['motor_number']}" for r in rotors]
    by_topic = {topic: {"t": [], "omega": []} for topic in topics}

    with rosbag.Bag(str(bag_path)) as bag:
        bag_start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            by_topic[topic]["t"].append(float(stamp.to_sec() - bag_start + time_offset))
            by_topic[topic]["omega"].append(read_motor_speed_value(msg))

    if any(len(v["t"]) < 2 for v in by_topic.values()):
        missing = [topic for topic, values in by_topic.items() if len(values["t"]) < 2]
        raise RuntimeError(f"Bag does not contain enough motor speed samples for: {missing}")

    omega = np.column_stack(
        [
            np.interp(target_t, np.asarray(by_topic[topic]["t"], dtype=float), np.asarray(by_topic[topic]["omega"], dtype=float))
            for topic in topics
        ]
    )
    return omega, {
        "source": "rosbag_motor_speed_topics",
        "bag_path": str(bag_path),
        "topics": topics,
        "bag_time_offset_s": time_offset,
    }


def reconstruct_torque(omega, rotors, reference_point, ccw_yaw_positive=True):
    tau = np.zeros((omega.shape[0], 3), dtype=float)
    thrust = np.zeros_like(omega)

    for i, rotor in enumerate(rotors):
        force_mag = rotor["motor_constant"] * omega[:, i] ** 2
        thrust[:, i] = force_mag
        force_body = np.column_stack(
            [np.zeros_like(force_mag), np.zeros_like(force_mag), -force_mag]
        )
        arm = rotor["position_frd_m"] - reference_point
        tau += np.cross(np.broadcast_to(arm, force_body.shape), force_body)

        direction_sign = 1.0 if rotor["direction"] == "ccw" else -1.0
        if not ccw_yaw_positive:
            direction_sign *= -1.0
        tau[:, 2] += direction_sign * rotor["moment_constant"] * force_mag

    return tau, thrust


def linear_fit(x, y):
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    if x.size < 3 or np.std(x) < 1e-9:
        return {"scale": None, "bias": None, "corr": None, "samples": int(x.size)}
    scale, bias = np.polyfit(x, y, 1)
    corr = float(np.corrcoef(x, y)[0, 1]) if np.std(y) > 1e-9 else None
    return {
        "scale": float(scale),
        "bias": float(bias),
        "corr": corr,
        "samples": int(x.size),
        "x_rms": float(math.sqrt(np.mean(x * x))),
        "y_rms": float(math.sqrt(np.mean(y * y))),
    }


def lagged_fit(x, y, max_lag):
    best = None
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            fit = linear_fit(x[-lag:], y[:lag])
        elif lag > 0:
            fit = linear_fit(x[:-lag], y[lag:])
        else:
            fit = linear_fit(x, y)

        score = -1.0 if fit["corr"] is None else abs(fit["corr"])
        if best is None or score > best["score"]:
            best = {"lag_samples": lag, "score": score, **fit}
    best.pop("score", None)
    return best


def summarize_vector(values):
    return {
        "mean": [float(x) for x in np.mean(values, axis=0)],
        "rms": [float(math.sqrt(np.mean(values[:, i] ** 2))) for i in range(values.shape[1])],
        "max_abs": [float(x) for x in np.max(np.abs(values), axis=0)],
    }


def geometry_report(rotors, mixer_rotors, control_channels):
    mixer_by_index = {i: rotor for i, rotor in enumerate(mixer_rotors)}
    channels_by_input = {c["input_index"]: c for c in control_channels}
    rows = []
    position_errors = []
    position_errors_xy = []
    direction_matches = []

    for rotor in rotors:
        idx = rotor["motor_number"]
        mixer = mixer_by_index.get(idx)
        channel = channels_by_input.get(idx)
        sdf_pos = np.asarray(rotor["position_frd_m"], dtype=float)
        if mixer is not None:
            mixer_pos = np.asarray(mixer.get("position_frd_m", [math.nan, math.nan, math.nan]), dtype=float)
            pos_delta = sdf_pos - mixer_pos
            pos_err = float(np.linalg.norm(pos_delta))
            pos_err_xy = float(np.linalg.norm(pos_delta[:2]))
            dir_match = rotor["direction"].lower() == mixer.get("direction", "").lower()
            position_errors.append(pos_err)
            position_errors_xy.append(pos_err_xy)
            direction_matches.append(dir_match)
        else:
            mixer_pos = np.array([math.nan, math.nan, math.nan], dtype=float)
            pos_delta = np.array([math.nan, math.nan, math.nan], dtype=float)
            pos_err = None
            pos_err_xy = None
            dir_match = None

        rows.append(
            {
                "motor_number": idx,
                "sdf_link_name": rotor["link_name"],
                "sdf_position_frd_m": [float(x) for x in sdf_pos],
                "sdf_direction": rotor["direction"],
                "mixer_name": None if mixer is None else mixer.get("name", ""),
                "mixer_position_frd_m": [float(x) for x in mixer_pos],
                "mixer_direction": None if mixer is None else mixer.get("direction", ""),
                "position_delta_sdf_minus_mixer_m": [float(x) for x in pos_delta],
                "position_error_norm_m": pos_err,
                "position_error_xy_norm_m": pos_err_xy,
                "direction_match": dir_match,
                "gazebo_input_index": None if channel is None else channel["input_index"],
                "gazebo_channel_name": None if channel is None else channel["name"],
            }
        )

    return {
        "mixer_geometry": "quad_wide.toml",
        "rows": rows,
        "max_position_error_norm_m": None if not position_errors else float(max(position_errors)),
        "max_position_error_xy_norm_m": None if not position_errors_xy else float(max(position_errors_xy)),
        "all_directions_match": bool(direction_matches) and all(direction_matches),
        "interpretation": (
            "Use SDF rotor positions for reconstructed physical torque; use the PX4 mixer geometry "
            "to interpret what the controller/mixer believed the geometry was."
        ),
    }


def gravity_torque_from_ref(att_q, mass, com, ref):
    gravity_world_ned = np.array([0.0, 0.0, 9.81], dtype=float)
    out = np.zeros((att_q.shape[0], 3), dtype=float)
    arm = com - ref
    for i, q in enumerate(att_q):
        g_body = quat_to_dcm(q).T @ gravity_world_ned
        out[i] = np.cross(arm, g_body) * mass
    return out


def fit_tau_s_reference(att_q, mass, com, tau_s_raw):
    gravity_world_ned = np.array([0.0, 0.0, 9.81], dtype=float)
    lhs = []
    rhs = []
    for q, tau in zip(att_q, tau_s_raw):
        g_body = quat_to_dcm(q).T @ gravity_world_ned
        base = np.cross(com, g_body) * mass
        # tau = cross(com - ref, g) * m = base + cross(g*m, ref)
        a = np.array(
            [
                [0.0, -mass * g_body[2], mass * g_body[1]],
                [mass * g_body[2], 0.0, -mass * g_body[0]],
                [-mass * g_body[1], mass * g_body[0], 0.0],
            ],
            dtype=float,
        )
        lhs.append(a)
        rhs.append(tau - base)

    a_all = np.vstack(lhs)
    b_all = np.concatenate(rhs)
    ref, *_ = np.linalg.lstsq(a_all, b_all, rcond=None)
    return ref


def find_default_ulog(run_dir):
    matches = sorted(Path(run_dir).rglob("*.ulg"))
    if not matches:
        return None
    return matches[-1]


def build_report(args):
    repo = Path(args.repo).resolve()
    sdf_arg = Path(args.sdf).expanduser()
    sdf_path = sdf_arg.resolve() if sdf_arg.is_absolute() else (repo / sdf_arg).resolve()
    rotors, control_channels = parse_sdf_model(sdf_path, convert_flu_to_frd=not args.no_flu_to_frd)
    mixer_arg = Path(args.mixer_geometry).expanduser()
    mixer_path = mixer_arg.resolve() if mixer_arg.is_absolute() else (repo / mixer_arg).resolve()
    mixer_rotors = parse_quad_wide_geometry(mixer_path)

    ulog_path = Path(args.ulog).expanduser() if args.ulog else None
    if ulog_path is None and args.run_dir:
        found = find_default_ulog(args.run_dir)
        if found is not None:
            ulog_path = found
    if ulog_path is None:
        raise RuntimeError("Provide --ulog or --run-dir containing a .ulg file")
    ulog_path = ulog_path.resolve()

    profile = UAM_V5_PROFILE if args.model_profile == "uam_v5" else UAV_ARM_V4_PROFILE
    mass = args.mass_total if args.mass_total is not None else profile["mass_total"]
    com = np.array(args.com, dtype=float) if args.com else profile["com_frd"]
    inertia_diag = np.array(args.inertia_diag, dtype=float) if args.inertia_diag else profile["inertia_diag"]

    ulog = ULog(str(ulog_path))
    rate = load_rate_log(ulog)
    target_t = rate["t"]
    armed_mask = interpolate_armed_mask(ulog, target_t)

    diagnostic_motor_sources = {}

    if args.motor_source == "rosbag_motor_speed":
        if not args.bag:
            raise RuntimeError("--motor-source rosbag_motor_speed requires --bag")
        try:
            omega, omega_meta = bag_motor_speeds_to_omega(
                Path(args.bag).expanduser().resolve(), target_t, rotors, args.bag_time_offset
            )
        except Exception as exc:
            if not args.allow_fallback:
                raise
            actuator_outputs = optional_dataset(ulog, "actuator_outputs")
            if actuator_outputs is None:
                raise RuntimeError(
                    "Bag motor speed read failed and ULog does not contain actuator_outputs"
                ) from exc
            omega, omega_meta = actuator_outputs_to_gazebo_omega(
                actuator_outputs, armed_mask, target_t, rotors, control_channels, args
            )
            omega_meta["fallback_reason"] = str(exc)
    else:
        actuator_outputs = optional_dataset(ulog, "actuator_outputs")
        actuator_controls = optional_dataset(ulog, "actuator_controls_0")

        if args.motor_source == "actuator_outputs_hil":
            if actuator_outputs is None:
                raise RuntimeError("ULog does not contain actuator_outputs")
            omega, omega_meta = actuator_outputs_to_gazebo_omega(
                actuator_outputs, armed_mask, target_t, rotors, control_channels, args
            )

        elif args.motor_source == "actuator_controls0_direct":
            if actuator_controls is None:
                raise RuntimeError("ULog does not contain actuator_controls_0")
            omega, omega_meta = actuator_controls0_to_gazebo_omega(
                actuator_controls, armed_mask, target_t, rotors, control_channels, args
            )

        elif args.motor_source == "actuator_outputs_pwm_legacy":
            if actuator_outputs is None:
                raise RuntimeError("ULog does not contain actuator_outputs")
            omega, omega_meta = actuator_outputs_to_omega(actuator_outputs, target_t, rotors, args)

        else:
            raise ValueError(f"Unsupported motor source: {args.motor_source}")

        if actuator_controls is not None and args.include_diagnostic_motor_sources:
            diag_omega, diag_meta = actuator_controls0_to_gazebo_omega(
                actuator_controls, armed_mask, target_t, rotors, control_channels, args
            )
            diagnostic_motor_sources[diag_meta["source"]] = {
                "metadata": diag_meta,
                "omega_rad_s": summarize_vector(diag_omega),
            }

    att = dataset(ulog, "vehicle_attitude")
    att_t = np.asarray(att["timestamp"], dtype=float) * 1e-6
    att_q = interp_matrix(target_t, att_t, columns(att, "q", 4))
    att_q /= np.linalg.norm(att_q, axis=1)[:, None]

    rotor_center = np.mean(np.array([r["position_frd_m"] for r in rotors]), axis=0)
    refs = {
        "body_origin": np.zeros(3, dtype=float),
        "profile_com": com,
        "rotor_center": rotor_center,
    }

    torque_by_ref = {}
    thrust_summary = None
    for name, ref in refs.items():
        tau, thrust = reconstruct_torque(omega, rotors, ref, ccw_yaw_positive=args.ccw_yaw_positive)
        torque_by_ref[name] = tau
        thrust_summary = thrust

    fit_ref = fit_tau_s_reference(att_q, mass, com, rate["tau_s_raw"])
    refs["tau_s_fitted_ref"] = fit_ref
    torque_by_ref["tau_s_fitted_ref"], _ = reconstruct_torque(
        omega, rotors, fit_ref, ccw_yaw_positive=args.ccw_yaw_positive
    )

    tau_s_hypotheses = {
        name: gravity_torque_from_ref(att_q, mass, com, ref) for name, ref in refs.items()
    }

    max_torque_fits = {}
    residual_fits = {}
    tau_s_reference_fits = {}
    disturbance_proxy = rate["disturbance_hat"] * inertia_diag

    for ref_name, tau_actual in torque_by_ref.items():
        max_torque_fits[ref_name] = {
            axis: linear_fit(rate["eso_torque"][:, i], tau_actual[:, i]) for i, axis in enumerate(AXES)
        }
        residual = tau_actual - rate["eso_torque"]
        residual_fits[ref_name] = {
            "actual_minus_eso_torque": {
                axis: lagged_fit(rate["tau_s_raw"][:, i], residual[:, i], args.max_lag_samples)
                for i, axis in enumerate(AXES)
            },
            "inertia_times_disturbance_hat": {
                axis: lagged_fit(rate["tau_s_raw"][:, i], disturbance_proxy[:, i], args.max_lag_samples)
                for i, axis in enumerate(AXES)
            },
        }

    for name, tau_h in tau_s_hypotheses.items():
        tau_s_reference_fits[name] = {
            axis: linear_fit(tau_h[:, i], rate["tau_s_raw"][:, i]) for i, axis in enumerate(AXES)
        }

    report = {
        "inputs": {
            "repo": str(repo),
            "sdf_path": str(sdf_path),
            "mixer_geometry_path": str(mixer_path),
            "ulog_path": str(ulog_path),
            "model_profile": args.model_profile,
            "mass_total_kg": mass,
            "com_frd_m": [float(x) for x in com],
            "inertia_diag_kg_m2": [float(x) for x in inertia_diag],
            "sample_count": int(target_t.size),
            "time_span_s": [float(target_t[0]), float(target_t[-1])],
            "best_baseline_kept_unchanged": {
                "ESO_DYN_FF_EN": 0,
                "ESO_TAUS_K": 0.0,
                "ESO_MAX_TORQUE": 2.2,
            },
        },
        "rotors": [
            {
                **{k: v for k, v in rotor.items() if k != "position_frd_m"},
                "position_frd_m": [float(x) for x in rotor["position_frd_m"]],
            }
            for rotor in rotors
        ],
        "gazebo_control_channels": control_channels,
        "mixer_sdf_geometry_check": geometry_report(rotors, mixer_rotors, control_channels),
        "motor_speed_reconstruction": omega_meta,
        "diagnostic_motor_sources": diagnostic_motor_sources,
        "reference_points_frd_m": {k: [float(x) for x in v] for k, v in refs.items()},
        "summaries": {
            "omega_rad_s": summarize_vector(omega),
            "thrust_n": summarize_vector(thrust_summary),
            "eso_torque_nm": summarize_vector(rate["eso_torque"]),
            "tau_s_raw_nm": summarize_vector(rate["tau_s_raw"]),
            "tau_s_used_nm": summarize_vector(rate["tau_s_used"]),
            "disturbance_proxy_nm": summarize_vector(disturbance_proxy),
            "tau_actual_nm": {k: summarize_vector(v) for k, v in torque_by_ref.items()},
        },
        "eso_max_torque_axis_fits": max_torque_fits,
        "tau_s_reference_fits": tau_s_reference_fits,
        "tau_s_per_axis_fits": residual_fits,
        "notes": [
            "比例、相关性和符号只作为离线证据；必须经过 fresh 闭环 exp4 验证后才能启用 tau_s。",
            "SITL 中优先使用 actuator_outputs_hil 作为力矩证据，它对应 PX4 HIL_ACTUATOR_CONTROLS 加 SDF mavlink_interface 映射。",
            "actuator_controls_0 在当前机型里只是姿态控制组数据，不是最终四个电机命令，因此只能作为诊断对照。",
            "如果以后能采集 Gazebo transport 的 /motor_speed/*，可使用 --motor-source rosbag_motor_speed，把电机滤波后的执行器动态也纳入分析。",
            "profile_com 是零机械臂/默认构型参考；除非 ulog 中有足够关节状态数据，否则这里不做动态 CoM 重建。",
        ],
    }
    return report


def print_summary(report):
    print("tau_s 动力学前馈离线标定")
    print(f"  ulog 日志: {report['inputs']['ulog_path']}")
    print(f"  样本数: {report['inputs']['sample_count']}")
    motor_meta = report["motor_speed_reconstruction"]
    motor_mode = motor_meta.get("pwm_mode", "direct")
    print(f"  电机转速来源: {motor_meta['source']} ({motor_mode})")
    geom = report["mixer_sdf_geometry_check"]
    print(
        "  mixer/SDF 几何对照: "
        f"最大三维位置差={geom['max_position_error_norm_m']} m, "
        f"最大 XY 力臂差={geom['max_position_error_xy_norm_m']} m, "
        f"转向一致={geom['all_directions_match']}"
    )
    print("  参考点 [m]:")
    for name, ref in report["reference_points_frd_m"].items():
        print(f"    {name}: {ref}")

    print("\nESO_MAX_TORQUE 分轴拟合: tau_actual ~= scale * eso_torque + bias")
    for ref_name, axes in report["eso_max_torque_axis_fits"].items():
        print(f"  {ref_name}:")
        for axis in AXES:
            fit = axes[axis]
            print(
                f"    {axis}: 比例={fit['scale']} 偏置={fit['bias']} 相关性={fit['corr']} 样本数={fit['samples']}"
            )

    print("\ntau_s 分轴拟合: actual_minus_eso_torque ~= scale * tau_s_raw + bias")
    for ref_name, groups in report["tau_s_per_axis_fits"].items():
        print(f"  {ref_name}:")
        for axis in AXES:
            fit = groups["actual_minus_eso_torque"][axis]
            print(
                f"    {axis}: 比例={fit['scale']} 偏置={fit['bias']} 相关性={fit['corr']} 滞后样本={fit['lag_samples']}"
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default="/home/cf/PX4_Firmware_clean", help="PX4 repo root")
    parser.add_argument("--sdf", default=DEFAULT_SDF, help="SDF path relative to repo or absolute")
    parser.add_argument("--mixer-geometry", default=DEFAULT_MIXER_GEOMETRY,
                        help="PX4 multirotor geometry TOML relative to repo or absolute")
    parser.add_argument("--ulog", default="", help="PX4 .ulg file")
    parser.add_argument("--bag", default="", help="Optional ROS bag with /motor_speed/0..3 topics")
    parser.add_argument("--bag-time-offset", type=float, default=0.0, help="Offset added to bag-relative motor-speed time")
    parser.add_argument("--motor-source",
                        choices=("actuator_outputs_hil", "rosbag_motor_speed",
                                 "actuator_controls0_direct", "actuator_outputs_pwm_legacy"),
                        default="actuator_outputs_hil",
                        help="Motor speed source used for physical torque reconstruction")
    parser.set_defaults(allow_fallback=True)
    parser.add_argument("--no-fallback", dest="allow_fallback", action="store_false",
                        help="Fail instead of falling back when the requested motor source is unavailable")
    parser.add_argument("--include-diagnostic-motor-sources", action="store_true",
                        help="Include diagnostic-only motor source summaries in the JSON report")
    parser.add_argument("--run-dir", default="", help="Run directory; newest .ulg below it is used if --ulog is omitted")
    parser.add_argument("--out", default="", help="Output JSON path")
    parser.add_argument("--model-profile", choices=("uam_v5", "uav_arm_v4"), default="uam_v5")
    parser.add_argument("--mass-total", type=float, default=None)
    parser.add_argument("--com", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"))
    parser.add_argument("--inertia-diag", nargs=3, type=float, default=None, metavar=("IXX", "IYY", "IZZ"))
    parser.add_argument("--no-flu-to-frd", action="store_true", help="Do not convert SDF positions from FLU/Z-up to FRD")
    parser.add_argument("--pwm-mode", choices=("normalized-max", "offset-scale"), default="normalized-max")
    parser.add_argument("--pwm-min", type=float, default=900.0)
    parser.add_argument("--pwm-max", type=float, default=2000.0)
    parser.add_argument("--pwm-offset", type=float, default=900.0)
    parser.add_argument("--pwm-omega-scale", type=float, default=1.0)
    parser.add_argument("--hil-pwm-min", type=float, default=PWM_DEFAULT_MIN,
                        help="PWM min used by PX4 HIL_ACTUATOR_CONTROLS mapping")
    parser.add_argument("--hil-pwm-max", type=float, default=PWM_DEFAULT_MAX,
                        help="PWM max used by PX4 HIL_ACTUATOR_CONTROLS mapping")
    parser.add_argument("--default-input-scaling", type=float, default=1000.0,
                        help="Fallback SDF mavlink_interface input_scaling")
    parser.add_argument("--default-zero-position-armed", type=float, default=100.0,
                        help="Fallback SDF mavlink_interface zero_position_armed")
    parser.add_argument("--ccw-yaw-positive", action="store_true", default=True)
    parser.add_argument("--max-lag-samples", type=int, default=5)
    args = parser.parse_args()

    report = build_report(args)
    print_summary(report)

    if args.out:
        out_path = Path(args.out).expanduser().resolve()
    else:
        out_path = Path(report["inputs"]["ulog_path"]).with_name("tau_s_feedforward_calibration.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"\n已写入 {out_path}")


if __name__ == "__main__":
    main()
