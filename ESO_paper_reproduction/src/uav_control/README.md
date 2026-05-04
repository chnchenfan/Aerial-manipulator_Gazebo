# uav_control

`uav_control` 提供 exp1/exp4 所需的通用 C++ 控制/轨迹库，以及 ROS bag 录制、原始位置误差计算、实验出图和自动调参脚本。该包不启动飞行任务；飞行任务由 `uav_arm_top` 中的 launch 和 offboard 节点启动。

## 文件结构

```text
uav_control/
├── CMakeLists.txt
│   └── 编译 `uav_core_lib`，安装 exp1/exp4 相关 Python 脚本。
├── package.xml
│   └── 声明 roscpp、rospy、rosbag、geometry_msgs、sensor_msgs、mavros_msgs、tf 等依赖。
├── include/uav_control/
│   ├── ControlUtils.hpp
│   │   └── 坐标转换、误差计算和通用控制辅助接口。
│   └── TrajectoryGenerator.hpp
│       └── 基础轨迹生成接口。
├── src/
│   ├── ControlUtils.cpp
│   │   └── `ControlUtils.hpp` 的实现，被 `uav_core_lib` 导出。
│   └── TrajectoryGenerator.cpp
│       └── `TrajectoryGenerator.hpp` 的实现，被 `uav_core_lib` 导出。
├── scripts/
│   ├── auto_tune_uam_v5_eso.py
│   │   └── UAM V5 ESO 自动调参入口，按 exp1/exp4 launch 生成 fresh bag 并计算 raw metrics。
│   ├── compute_raw_position_error.py
│   │   └── 从 ROS bag 读取当前位置与 setpoint，输出 `raw_position_error.json`。
│   ├── experiment_data_recorder.py
│   │   └── exp1/exp4 录包脚本，保存 bag 和 metadata。
│   ├── plot_uam_experiment_comparison.py
│   │   └── exp1/exp4 后处理出图脚本，生成飞行位置、误差和机械臂跟踪图。
│   ├── calibrate_tau_s_feedforward.py
│   │   └── tau_s 前馈离线标定/诊断脚本，不由 exp1/exp4 launch 自动调用。
│   └── README.md
│       └── 脚本补充说明。
└── README.md
    └── 当前文件。
```

## 录制与指标信息流

`experiment_data_recorder.py` 订阅以下核心话题：

- `/mavros/local_position/pose`：PX4/MAVROS 输出的本地位置。
- `/mavros/setpoint_position/local`：offboard 任务节点发布的位置期望值。
- `/experiment/arm_motion_enabled`：任务有效段标记；第一次变为 `true` 的时刻写入 metadata 的 `analysis_start_time_s`。
- `/uav_arm/joint_states`：Gazebo/ros_control 输出的机械臂实际关节状态。
- `/uav_arm/target_joint_states`：机械臂扰动节点发布的关节期望值。
- `/mavros/state`：飞控连接、模式、解锁状态。

信息流：

```text
exp1/exp4 offboard node
  -> /mavros/setpoint_position/local
PX4 + MAVROS
  -> /mavros/local_position/pose, /mavros/state
uam_v5_experiment_motion.py
  -> /uav_arm/target_joint_states
gazebo_ros_control
  -> /uav_arm/joint_states
experiment_data_recorder.py
  -> data/<timestamp>/<experiment>.bag
  -> data/<timestamp>/<experiment>_metadata.json
compute_raw_position_error.py
  -> raw_position_error.json
plot_uam_experiment_comparison.py
  -> CSV 和 PNG 图
```

## 与顶层任务的关系

exp1 和 exp4 的飞行期望值由 `uav_arm_top` 节点发布；本包只负责记录和评价这些期望值是否被跟踪：

- exp1：期望位置为 `(x=0, y=0, z=2.0 m)` 的悬停点。
- exp4：期望路径为 `z=2.0 m` 高度的正方形，默认边长 `2.0 m`，launch 默认段速度 `0.15 m/s`，角点停留 `3.0 s`。
- 两个实验中机械臂扰动默认频率均为 `0.5 Hz`，有效评价窗口从 `/experiment/arm_motion_enabled=True` 开始。

自动调参脚本 `auto_tune_uam_v5_eso.py` 会围绕这些任务运行 fresh bag、调用 `compute_raw_position_error.py` 得到原始位置误差，并将候选参数的接受/拒绝交给外部 tuning memory 记录。
