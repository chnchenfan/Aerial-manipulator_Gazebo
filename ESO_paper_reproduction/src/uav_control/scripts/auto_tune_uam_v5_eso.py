#!/usr/bin/env python3
"""Run closed-loop SITL tuning for the uam_v5 ESO experiments."""

import argparse
import json
import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

from compute_raw_position_error import compute_metrics


REPO_ROOT = Path("/home/cf/PX4_Firmware_clean")
ROS_ENV = REPO_ROOT / "ESO_paper_reproduction" / "src" / "setup_px4_sitl_ros_env.sh"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "ESO_paper_reproduction" / "src" / "uav_arm_top" / "auto_tune_data"
PASS_THRESHOLD_M = 0.10


BASELINE = {
    "COM_RCL_EXCEPT": 4,
    "COM_RC_IN_MODE": 1,
    "ESO_DYN_FF_EN": 0,
    "ESO_XY_P": 0.80,
    "ESO_XY_I": 0.15,
    "ESO_XY_VEL_P_ACC": 1.00,
    "ESO_XY_BW": 0.70,
    "ESO_Z_P": 0.90,
    "ESO_Z_I": 0.10,
    "ESO_Z_VEL_P_ACC": 0.90,
    "ESO_Z_BW": 0.80,
    "ESO_POS_INT_LIM": 0.12,
    "ESO_XY_VEL_MAX": 2.0,
    "ESO_ACC_HOR": 2.0,
    "ESO_ACC_HOR_MAX": 3.0,
    "ESO_JERK_AUTO": 2.0,
    "ESO_JERK_MAX": 4.0,
    "ESO_ROLL_P": 1.0,
    "ESO_PITCH_P": 1.0,
    "ESO_YAW_P": 0.8,
    "ESO_YAW_WEIGHT": 0.35,
    "ESO_ROLLRATE_P": 0.12,
    "ESO_PITCHRATE_P": 0.12,
    "ESO_YAWRATE_P": 0.16,
    "ESO_RATE_BW_R": 1.5,
    "ESO_RATE_BW_P": 1.5,
    "ESO_RATE_BW_Y": 0.8,
    "ESO_RATE_I_SC": 0.08,
    "ESO_TAUS_K": 0.0,
    "ESO_TAUS_LIM": 0.30,
}


def with_updates(name, updates):
    params = dict(BASELINE)
    params.update(updates)
    return {"name": name, "params": params}


CANDIDATES = [
    with_updates("baseline_conservative", {}),
    with_updates(
        "xy_tracking_1",
        {
            "ESO_XY_P": 0.90,
            "ESO_XY_VEL_P_ACC": 1.15,
            "ESO_XY_BW": 0.90,
            "ESO_ACC_HOR": 2.5,
            "ESO_JERK_AUTO": 3.0,
        },
    ),
    with_updates(
        "xy_tracking_2",
        {
            "ESO_XY_P": 0.95,
            "ESO_XY_VEL_P_ACC": 1.20,
            "ESO_XY_BW": 1.00,
            "ESO_XY_VEL_MAX": 2.5,
            "ESO_ACC_HOR": 2.5,
            "ESO_ACC_HOR_MAX": 3.5,
            "ESO_JERK_AUTO": 3.0,
        },
    ),
    with_updates(
        "z_rate_disturbance_1",
        {
            "ESO_Z_P": 1.05,
            "ESO_Z_VEL_P_ACC": 1.05,
            "ESO_Z_BW": 1.00,
            "ESO_RATE_BW_R": 2.0,
            "ESO_RATE_BW_P": 2.0,
            "ESO_RATE_BW_Y": 1.0,
            "ESO_RATE_I_SC": 0.12,
        },
    ),
    with_updates(
        "combined_tracking_disturbance",
        {
            "ESO_XY_P": 0.90,
            "ESO_XY_VEL_P_ACC": 1.15,
            "ESO_XY_BW": 0.90,
            "ESO_ACC_HOR": 2.5,
            "ESO_JERK_AUTO": 3.0,
            "ESO_Z_P": 1.05,
            "ESO_Z_VEL_P_ACC": 1.05,
            "ESO_Z_BW": 1.00,
            "ESO_RATE_BW_R": 2.0,
            "ESO_RATE_BW_P": 2.0,
            "ESO_RATE_BW_Y": 1.0,
            "ESO_RATE_I_SC": 0.12,
        },
    ),
    with_updates(
        "combined_with_taus",
        {
            "ESO_XY_P": 0.90,
            "ESO_XY_VEL_P_ACC": 1.15,
            "ESO_XY_BW": 0.90,
            "ESO_ACC_HOR": 2.5,
            "ESO_JERK_AUTO": 3.0,
            "ESO_Z_P": 1.05,
            "ESO_Z_VEL_P_ACC": 1.05,
            "ESO_Z_BW": 1.00,
            "ESO_RATE_BW_R": 2.0,
            "ESO_RATE_BW_P": 2.0,
            "ESO_RATE_BW_Y": 1.0,
            "ESO_RATE_I_SC": 0.12,
            "ESO_TAUS_K": 0.10,
            "ESO_TAUS_LIM": 0.20,
        },
    ),
    with_updates(
        "xy_integral_disturbance_1",
        {
            "ESO_XY_P": 0.95,
            "ESO_XY_I": 0.35,
            "ESO_XY_VEL_P_ACC": 1.25,
            "ESO_XY_BW": 1.00,
            "ESO_POS_INT_LIM": 0.50,
            "ESO_XY_VEL_MAX": 2.5,
            "ESO_ACC_HOR": 3.0,
            "ESO_ACC_HOR_MAX": 4.0,
            "ESO_JERK_AUTO": 4.0,
        },
    ),
    with_updates(
        "xy_integral_disturbance_2",
        {
            "ESO_XY_P": 1.10,
            "ESO_XY_I": 0.45,
            "ESO_XY_VEL_P_ACC": 1.45,
            "ESO_XY_BW": 1.15,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.8,
            "ESO_ACC_HOR": 3.5,
            "ESO_ACC_HOR_MAX": 4.5,
            "ESO_JERK_AUTO": 5.0,
        },
    ),
    with_updates(
        "smooth_square_lowlag",
        {
            "ESO_XY_P": 1.20,
            "ESO_XY_I": 0.40,
            "ESO_XY_VEL_P_ACC": 1.70,
            "ESO_XY_BW": 1.25,
            "ESO_Z_P": 1.00,
            "ESO_Z_VEL_P_ACC": 1.00,
            "ESO_Z_BW": 0.95,
            "ESO_POS_INT_LIM": 0.60,
            "ESO_XY_VEL_MAX": 3.0,
            "ESO_ACC_HOR": 4.0,
            "ESO_ACC_HOR_MAX": 5.0,
            "ESO_JERK_AUTO": 6.0,
        },
    ),
    with_updates(
        "integral_tau_attitude_1",
        {
            "ESO_XY_P": 0.95,
            "ESO_XY_I": 0.35,
            "ESO_XY_VEL_P_ACC": 1.20,
            "ESO_XY_BW": 0.95,
            "ESO_POS_INT_LIM": 0.50,
            "ESO_XY_VEL_MAX": 2.5,
            "ESO_ACC_HOR": 3.0,
            "ESO_ACC_HOR_MAX": 4.0,
            "ESO_JERK_AUTO": 4.0,
            "ESO_ROLL_P": 1.30,
            "ESO_PITCH_P": 1.30,
            "ESO_ROLLRATE_P": 0.14,
            "ESO_PITCHRATE_P": 0.14,
            "ESO_RATE_BW_R": 1.80,
            "ESO_RATE_BW_P": 1.80,
            "ESO_RATE_I_SC": 0.10,
            "ESO_TAUS_K": 0.05,
            "ESO_TAUS_LIM": 0.15,
        },
    ),
    with_updates(
        "integral_tau_attitude_2",
        {
            "ESO_XY_P": 0.95,
            "ESO_XY_I": 0.35,
            "ESO_XY_VEL_P_ACC": 1.20,
            "ESO_XY_BW": 0.95,
            "ESO_POS_INT_LIM": 0.50,
            "ESO_XY_VEL_MAX": 2.5,
            "ESO_ACC_HOR": 3.0,
            "ESO_ACC_HOR_MAX": 4.0,
            "ESO_JERK_AUTO": 4.0,
            "ESO_ROLL_P": 1.50,
            "ESO_PITCH_P": 1.50,
            "ESO_ROLLRATE_P": 0.16,
            "ESO_PITCHRATE_P": 0.16,
            "ESO_RATE_BW_R": 2.00,
            "ESO_RATE_BW_P": 2.00,
            "ESO_RATE_I_SC": 0.12,
            "ESO_TAUS_K": 0.10,
            "ESO_TAUS_LIM": 0.20,
        },
    ),
    with_updates(
        "damped_integral_attitude",
        {
            "ESO_XY_P": 0.85,
            "ESO_XY_I": 0.35,
            "ESO_XY_VEL_P_ACC": 1.05,
            "ESO_XY_BW": 0.85,
            "ESO_POS_INT_LIM": 0.50,
            "ESO_XY_VEL_MAX": 2.2,
            "ESO_ACC_HOR": 2.5,
            "ESO_ACC_HOR_MAX": 3.5,
            "ESO_JERK_AUTO": 3.0,
            "ESO_ROLL_P": 1.50,
            "ESO_PITCH_P": 1.50,
            "ESO_ROLLRATE_P": 0.16,
            "ESO_PITCHRATE_P": 0.16,
            "ESO_RATE_BW_R": 2.00,
            "ESO_RATE_BW_P": 2.00,
            "ESO_RATE_I_SC": 0.12,
            "ESO_TAUS_K": 0.05,
            "ESO_TAUS_LIM": 0.15,
        },
    ),
    with_updates(
        "no_dynff_integral_1",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.95,
            "ESO_XY_I": 0.35,
            "ESO_XY_VEL_P_ACC": 1.25,
            "ESO_XY_BW": 1.00,
            "ESO_POS_INT_LIM": 0.50,
            "ESO_XY_VEL_MAX": 2.5,
            "ESO_ACC_HOR": 3.0,
            "ESO_ACC_HOR_MAX": 4.0,
            "ESO_JERK_AUTO": 4.0,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_integral_2",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.90,
            "ESO_XY_I": 0.45,
            "ESO_XY_VEL_P_ACC": 1.15,
            "ESO_XY_BW": 0.90,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.3,
            "ESO_ACC_HOR": 2.8,
            "ESO_ACC_HOR_MAX": 3.8,
            "ESO_JERK_AUTO": 3.5,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_slow_damped",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.85,
            "ESO_XY_I": 0.45,
            "ESO_XY_VEL_P_ACC": 1.05,
            "ESO_XY_BW": 0.85,
            "ESO_POS_INT_LIM": 0.80,
            "ESO_XY_VEL_MAX": 2.0,
            "ESO_ACC_HOR": 2.5,
            "ESO_ACC_HOR_MAX": 3.5,
            "ESO_JERK_AUTO": 3.0,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_slow_midband",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "exp4_xy_rate_boost",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.12,
            "ESO_XY_BW": 0.92,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.7,
            "ESO_ACC_HOR_MAX": 3.8,
            "ESO_JERK_AUTO": 3.4,
            "ESO_ROLL_P": 1.10,
            "ESO_PITCH_P": 1.10,
            "ESO_ROLLRATE_P": 0.14,
            "ESO_PITCHRATE_P": 0.14,
            "ESO_RATE_BW_R": 1.8,
            "ESO_RATE_BW_P": 1.8,
            "ESO_K_BETA": 0.60,
            "ESO_MAX_TORQUE": 1.8,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_hover_bias_trim",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.82,
            "ESO_XY_I": 0.50,
            "ESO_XY_VEL_P_ACC": 1.00,
            "ESO_XY_BW": 0.80,
            "ESO_POS_INT_LIM": 0.90,
            "ESO_XY_VEL_MAX": 2.0,
            "ESO_ACC_HOR": 2.4,
            "ESO_ACC_HOR_MAX": 3.4,
            "ESO_JERK_AUTO": 2.8,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_extra_damped",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.78,
            "ESO_XY_I": 0.45,
            "ESO_XY_VEL_P_ACC": 0.95,
            "ESO_XY_BW": 0.75,
            "ESO_POS_INT_LIM": 0.80,
            "ESO_XY_VEL_MAX": 1.8,
            "ESO_ACC_HOR": 2.2,
            "ESO_ACC_HOR_MAX": 3.2,
            "ESO_JERK_AUTO": 2.5,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_kbeta_low",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_K_BETA": 0.20,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_kbeta_zero",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_K_BETA": 0.0,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_rate_soft",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_K_BETA": 0.20,
            "ESO_RATE_BW_R": 1.2,
            "ESO_RATE_BW_P": 1.2,
            "ESO_RATE_I_SC": 0.05,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_i50",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.50,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.90,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_i55_soft",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.86,
            "ESO_XY_I": 0.55,
            "ESO_XY_VEL_P_ACC": 1.05,
            "ESO_XY_BW": 0.86,
            "ESO_POS_INT_LIM": 1.00,
            "ESO_XY_VEL_MAX": 2.0,
            "ESO_ACC_HOR": 2.5,
            "ESO_ACC_HOR_MAX": 3.5,
            "ESO_JERK_AUTO": 3.0,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_i55_fastvel",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.55,
            "ESO_XY_VEL_P_ACC": 1.20,
            "ESO_XY_BW": 0.90,
            "ESO_POS_INT_LIM": 1.00,
            "ESO_XY_VEL_MAX": 2.2,
            "ESO_ACC_HOR": 2.8,
            "ESO_ACC_HOR_MAX": 3.8,
            "ESO_JERK_AUTO": 3.5,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_i50_lowbw",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.50,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.75,
            "ESO_POS_INT_LIM": 0.90,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_i50_highbw",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.50,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 1.05,
            "ESO_POS_INT_LIM": 0.90,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_i50_torque12",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.50,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.90,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_MAX_TORQUE": 1.20,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_p90_i42",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.90,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_p86_i42",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.86,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_vel115_i42",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.15,
            "ESO_XY_BW": 0.88,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_bw95_i42",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.88,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.10,
            "ESO_XY_BW": 0.95,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.6,
            "ESO_ACC_HOR_MAX": 3.6,
            "ESO_JERK_AUTO": 3.2,
            "ESO_TAUS_K": 0.0,
        },
    ),
    with_updates(
        "no_dynff_midband_p90_vel115",
        {
            "ESO_DYN_FF_EN": 0,
            "ESO_XY_P": 0.90,
            "ESO_XY_I": 0.42,
            "ESO_XY_VEL_P_ACC": 1.15,
            "ESO_XY_BW": 0.90,
            "ESO_POS_INT_LIM": 0.70,
            "ESO_XY_VEL_MAX": 2.1,
            "ESO_ACC_HOR": 2.7,
            "ESO_ACC_HOR_MAX": 3.7,
            "ESO_JERK_AUTO": 3.3,
            "ESO_TAUS_K": 0.0,
        },
    ),
]


EXPERIMENTS = [
    {
        "name": "exp1_hover_disturbance_uam_v5",
        "launch": "exp1_hover_disturbance_uam_v5.launch",
        "duration_s": 10.0,
        "run_after_enable_s": 14.0,
        "enable_timeout_s": 100.0,
    },
    {
        "name": "exp4_square_tracking_uam_v5",
        "launch": "exp4_square_tracking_uam_v5.launch",
        "duration_s": 78.0,
        "run_after_enable_s": 81.0,
        "enable_timeout_s": 120.0,
    },
]


def shell_cmd(command, ros_home=None):
    ros_home_path = Path(ros_home or "/tmp/uam_v5_auto_tune_ros").resolve()
    ros_log_path = ros_home_path / "log"
    return (
        f"mkdir -p {ros_home_path} {ros_log_path} && "
        f"export ROS_HOME={ros_home_path} ROS_LOG_DIR={ros_log_path} && "
        f"source {ROS_ENV} >/tmp/uam_v5_auto_tune_env.log && {command}"
    )


def run_checked(command, timeout=None):
    return subprocess.run(
        ["bash", "-lc", command],
        cwd=str(REPO_ROOT),
        timeout=timeout,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=True,
    )


def cleanup_sim_processes():
    patterns = [
        "/opt/ros/noetic/bin/roslaunch uav_arm_top",
        "experiment_data_recorder.py",
        "/home/cf/PX4_Firmware_clean/build/px4_sitl_default/bin/px4",
        "px4-simulator --instance",
        "gzserver",
        "gzclient",
        "rosmaster --core",
    ]
    for pattern in patterns:
        subprocess.run(
            ["pkill", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    time.sleep(5.0)


def start_process(command, log_path):
    log_file = open(log_path, "w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=str(REPO_ROOT),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        stdin=subprocess.PIPE,
        preexec_fn=os.setsid,
        text=True,
    )
    return process, log_file


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


def wait_for_enable_true(timeout_s, ros_home):
    command = shell_cmd("rostopic echo /experiment/arm_motion_enabled/data", ros_home)
    process = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=str(REPO_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    deadline = time.time() + timeout_s
    try:
        fd = process.stdout.fileno()
        while time.time() < deadline:
            ready, _, _ = select.select([fd], [], [], 0.5)
            if not ready:
                if process.poll() is not None:
                    break
                continue

            line = process.stdout.readline()
            if line and line.strip().lower() == "true":
                return True
        return False
    finally:
        stop_process(process)


def wait_for_recorder_run_dir(output_dir, experiment_name, timeout_s=30.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        candidates = sorted(output_dir.glob(f"*/{experiment_name}.bag"))
        if candidates:
            return candidates[-1].parent
        time.sleep(0.5)
    raise RuntimeError(f"Recorder did not create a bag for {experiment_name}")


def set_mavros_params(params, log_path, ros_home):
    lines = []
    for key, value in params.items():
        command = shell_cmd(f"rosrun mavros mavparam set {key} {value}", ros_home)
        result = subprocess.run(
            ["bash", "-lc", command],
            cwd=str(REPO_ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        lines.append(f"$ {key}={value}\n{result.stdout}\nreturncode={result.returncode}\n")
        if result.returncode != 0:
            raise RuntimeError(f"Failed to set {key}={value}: {result.stdout}")
    log_path.write_text("\n".join(lines), encoding="utf-8")


def run_experiment(candidate_name, params, experiment, output_root):
    run_root = output_root / candidate_name / experiment["name"]
    run_root.mkdir(parents=True, exist_ok=True)
    recorder_output = run_root / "bags"
    recorder_output.mkdir(parents=True, exist_ok=True)

    launch_log = run_root / "roslaunch.log"
    recorder_log = run_root / "recorder.log"
    param_log = run_root / "mavparam_set.log"
    ros_home = run_root / "ros_home"

    launch_args = str(experiment.get("launch_args", "")).strip()
    launch_cmd = shell_cmd(
        f"roslaunch uav_arm_top {experiment['launch']} gui:=false {launch_args}".strip(),
        ros_home,
    )
    recorder_cmd = shell_cmd(
        "rosrun uav_control experiment_data_recorder.py "
        f"_experiment_name:={experiment['name']} _output_dir:={recorder_output} "
        "_record_arm_topics:=false _record_state_topic:=false "
        f"_record_motor_speed_topics:={'true' if experiment.get('record_motor_speed_topics', False) else 'false'}",
        ros_home,
    )

    launch_proc = recorder_proc = None
    launch_file = recorder_file = None

    try:
        cleanup_sim_processes()
        launch_proc, launch_file = start_process(launch_cmd, launch_log)
        time.sleep(18.0)
        set_mavros_params(params, param_log, ros_home)
        recorder_proc, recorder_file = start_process(recorder_cmd, recorder_log)
        run_dir = wait_for_recorder_run_dir(recorder_output, experiment["name"])

        if not wait_for_enable_true(experiment["enable_timeout_s"], ros_home):
            raise RuntimeError("Timed out waiting for /experiment/arm_motion_enabled=true")

        time.sleep(experiment["run_after_enable_s"])
    finally:
        if recorder_proc is not None:
            stop_process(recorder_proc, recorder_file)
        if launch_proc is not None:
            stop_process(launch_proc, launch_file)
        cleanup_sim_processes()
        time.sleep(5.0)

    bag_path = run_dir / f"{experiment['name']}.bag"
    metrics = compute_metrics(
        bag_path,
        start_time=None,
        duration=experiment["duration_s"],
    )
    metrics_path = run_root / "raw_position_error.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")

    snapshot = {
        "candidate": candidate_name,
        "experiment": experiment["name"],
        "params": params,
        "launch_args": launch_args,
        "bag_path": str(bag_path),
        "metrics_path": str(metrics_path),
    }
    (run_root / "run_snapshot.json").write_text(
        json.dumps(snapshot, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return metrics


def build_px4():
    run_checked("CCACHE_DISABLE=1 make px4_sitl_default", timeout=1800)


def build_ros_workspace():
    run_checked("cd ESO_paper_reproduction && catkin_make --pkg uav_control uav_arm_top", timeout=900)


def candidate_passed(results):
    return all(
        result["metrics"]["mean_position_error_m"] < PASS_THRESHOLD_M
        for result in results
    )


def main():
    parser = argparse.ArgumentParser(description="Automatically tune uam_v5 ESO parameters in SITL.")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="Output directory for tuning artifacts")
    parser.add_argument("--start-candidate", type=int, default=0, help="Zero-based candidate index to start from")
    parser.add_argument("--max-candidates", type=int, default=None, help="Maximum candidates to run")
    parser.add_argument(
        "--experiments",
        default=None,
        help="Comma-separated experiment names to run; defaults to all experiments",
    )
    parser.add_argument("--skip-build", action="store_true", help="Skip PX4 build step")
    args = parser.parse_args()

    output_root = Path(args.output_dir).expanduser().resolve() / time.strftime("%Y%m%d_%H%M%S")
    output_root.mkdir(parents=True, exist_ok=True)

    if not args.skip_build:
        build_ros_workspace()
        build_px4()

    all_results = []
    winner = None

    max_candidates = len(CANDIDATES) if args.max_candidates is None else args.max_candidates
    selected_candidates = CANDIDATES[args.start_candidate : args.start_candidate + max_candidates]
    selected_experiments = EXPERIMENTS
    if args.experiments:
        requested_experiments = {name.strip() for name in args.experiments.split(",") if name.strip()}
        selected_experiments = [
            experiment for experiment in EXPERIMENTS if experiment["name"] in requested_experiments
        ]
        missing_experiments = requested_experiments - {experiment["name"] for experiment in selected_experiments}
        if missing_experiments:
            raise ValueError(f"Unknown experiments: {sorted(missing_experiments)}")

    for candidate in selected_candidates:
        candidate_results = []
        for experiment in selected_experiments:
            try:
                metrics = run_experiment(candidate["name"], candidate["params"], experiment, output_root)
                candidate_results.append({"experiment": experiment["name"], "metrics": metrics})
            except Exception as exc:
                candidate_results.append({"experiment": experiment["name"], "error": str(exc)})
                break

        candidate_summary = {
            "candidate": candidate["name"],
            "params": candidate["params"],
            "results": candidate_results,
            "passed": (
                len(candidate_results) == len(selected_experiments)
                and all("metrics" in result for result in candidate_results)
                and candidate_passed(candidate_results)
            ),
        }
        all_results.append(candidate_summary)
        (output_root / "auto_tune_summary.json").write_text(
            json.dumps({"results": all_results, "winner": winner}, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

        if candidate_summary["passed"]:
            winner = candidate_summary
            break

    final_summary = {"results": all_results, "winner": winner}
    summary_path = output_root / "auto_tune_summary.json"
    summary_path.write_text(json.dumps(final_summary, indent=2, ensure_ascii=False), encoding="utf-8")

    print(json.dumps(final_summary, indent=2, ensure_ascii=False))
    if winner is None:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
