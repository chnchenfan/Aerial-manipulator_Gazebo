# uav_control scripts

本目录只保留当前 UAM V5 exp1/exp4 会用到的记录、指标、出图、调参和 tau_s 离线诊断脚本。

## 文件结构

```text
scripts/
├── experiment_data_recorder.py
│   └── exp1/exp4 ROS bag 录制器，保存 bag 和 metadata。
├── compute_raw_position_error.py
│   └── 从 bag 中计算原始位置误差，输出 raw_position_error.json。
├── plot_uam_experiment_comparison.py
│   └── 从 exp1/exp4 bag 中导出 CSV 和论文用 PNG 图。
├── auto_tune_uam_v5_eso.py
│   └── 自动运行 UAM V5 ESO fresh trial，并调用 recorder 和 raw metrics。
├── calibrate_tau_s_feedforward.py
│   └── tau_s 前馈离线诊断脚本；不由 exp1/exp4 launch 自动调用。
└── README.md
    └── 当前文件。
```

## 录制 exp1/exp4

启动 exp1 或 exp4 后，在另一个终端运行：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun uav_control experiment_data_recorder.py _experiment_name:=exp1_hover_disturbance_uam_v5
```

或：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun uav_control experiment_data_recorder.py _experiment_name:=exp4_square_tracking_uam_v5
```

默认输出：

```text
ESO_paper_reproduction/src/uav_arm_top/data/<timestamp>/
├── <experiment>.bag
└── <experiment>_metadata.json
```

录制器会订阅：

- `/mavros/local_position/pose`
- `/mavros/setpoint_position/local`
- `/experiment/arm_motion_enabled`
- `/uav_arm/joint_states`
- `/uav_arm/target_joint_states`
- `/mavros/state`

metadata 中的 `analysis_start_time_s` 来自 `/experiment/arm_motion_enabled` 第一次为 `true` 的时刻。

## 计算原始位置误差

```bash
python3 ESO_paper_reproduction/src/uav_control/scripts/compute_raw_position_error.py \
  /path/to/exp1_hover_disturbance_uam_v5.bag \
  --out /tmp/raw_position_error.json
```

该脚本对齐 `/mavros/local_position/pose` 与 `/mavros/setpoint_position/local`，计算 mean/max/RMS 等原始位置误差。自动调参流程使用该 JSON 作为 hard gate 证据。

## 出图

```bash
python3 ESO_paper_reproduction/src/uav_control/scripts/plot_uam_experiment_comparison.py \
  /path/to/exp4_square_tracking_uam_v5.bag
```

输出包括：

- `flight_position_comparison.csv`
- `arm_joint_comparison.csv`
- `comparison_summary.json`
- `figure1_flight_position_tracking.png`
- `figure2_flight_position_error.png`
- `figure3_arm_joint_tracking.png`

## 自动调参

`auto_tune_uam_v5_eso.py` 面向 UAM V5 ESO staged tuning。它会启动 exp1/exp4 launch、录制 fresh bag、计算 raw position error，并把 trial 结果写入 `auto_tune_data/`。该目录是本地产物，已加入 `.gitignore`。

## 任务参数来源

- exp1 的飞行期望由 `uav_arm_top/src/eso_hover_disturbance_offboard_node.cpp` 发布：默认 `(0, 0, 2.0 m)` 悬停。
- exp4 的飞行期望由 `uav_arm_top/src/eso_square_arm_experiment_node.cpp` 发布：默认 `2.0 m` 高、边长 `2.0 m` 方形轨迹，launch 默认速度 `0.15 m/s`。
- 机械臂扰动由 `arm_controller/scripts/uam_v5_experiment_motion.py` 发布：默认 `0.5 Hz` 正弦运动。
