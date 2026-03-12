# arm_controller

## English

### 1. Overview

`arm_controller` is the ROS package that brings up `ros_control` PID position controllers for the manipulator joints and provides helper scripts to move the arm during PX4 SITL experiments.

It supports both platform variants in this repository:

- `uav_arm_v4`
- `uam_v5`

### 2. What Was Modified

This package was added to make the arm side of the experiment reproducible:

- separate controller bring-up launch files for `uav_arm_v4` and `uam_v5`,
- dedicated PID configuration YAML files for both models,
- motion scripts for both arm variants,
- a logging tool for the `uam_v5` zero-hold test.

### 3. Key Components

- `launch/controller_bringup.launch`
  - loads `config/joint_pid.yaml`
  - spawns `uav_arm_v4` joint controllers
- `launch/controller_bringup_uam_v5.launch`
  - loads `config/joint_pid_uam_v5.yaml`
  - spawns `uam_v5` joint controllers
- `scripts/joint_position_commander.py`
  - smooth interpolation command script for the `uav_arm_v4` arm
- `scripts/joint_position_commander_uam_v5.py`
  - smooth interpolation command script for the `uam_v5` arm
  - records target, position, velocity, effort, and error
- `scripts/arm_zero_hold_logger_uam_v5.py`
  - holds selected `uam_v5` joints at zero
  - records CSV and PNG outputs

### 4. Interfaces / Launch or Runtime Entry Points

Namespace convention:

- default namespace: `uav_arm`
- controller topics follow:
  - `/<namespace>/<joint>_position_controller/command`
- joint state topic:
  - `/<namespace>/joint_states`
- target joint topic published by the commander scripts:
  - `/<namespace>/target_joint_states`

Launch entry points:

```bash
roslaunch arm_controller controller_bringup.launch
roslaunch arm_controller controller_bringup_uam_v5.launch
```

Runtime scripts:

```bash
rosrun arm_controller joint_position_commander.py
rosrun arm_controller joint_position_commander_uam_v5.py
rosrun arm_controller arm_zero_hold_logger_uam_v5.py
```

### 5. How to Run or Validate

Typical usage is through `uav_arm_top`, but this package can also be checked directly after the Gazebo model is up:

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch arm_controller controller_bringup.launch
rosrun arm_controller joint_position_commander.py
```

For `uam_v5`:

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch arm_controller controller_bringup_uam_v5.launch
rosrun arm_controller joint_position_commander_uam_v5.py
rosrun arm_controller arm_zero_hold_logger_uam_v5.py
```

Validation checks:

- the controller spawner loads without missing-controller errors,
- joint command topics exist in the expected namespace,
- `/uav_arm/joint_states` updates while the scripts run,
- `uam_v5` logging scripts produce CSV/PNG outputs.

### 6. File Map

- `config/joint_pid.yaml`: PID gains for `uav_arm_v4`
- `config/joint_pid_uam_v5.yaml`: PID gains for `uam_v5`
- `launch`: controller bring-up launch files
- `scripts`: motion and logging tools

## 中文

### 1. 概述

`arm_controller` 是机械臂 ROS 控制包，负责加载 `ros_control` 的关节 PID 位置控制器，并提供实验中驱动机械臂的辅助脚本。

它同时支持本仓库里的两套平台：

- `uav_arm_v4`
- `uam_v5`

### 2. 修改了什么

这个包主要补齐了机械臂实验链路中可复现的控制部分：

- 为 `uav_arm_v4` 和 `uam_v5` 分别提供控制器 bring-up launch，
- 为两种模型分别提供 PID 参数文件，
- 为两种机械臂分别提供动作脚本，
- 为 `uam_v5` 增加零位保持与日志记录工具。

### 3. 关键组成

- `launch/controller_bringup.launch`
  - 读取 `config/joint_pid.yaml`
  - 启动 `uav_arm_v4` 的关节控制器
- `launch/controller_bringup_uam_v5.launch`
  - 读取 `config/joint_pid_uam_v5.yaml`
  - 启动 `uam_v5` 的关节控制器
- `scripts/joint_position_commander.py`
  - `uav_arm_v4` 的平滑插值关节命令脚本
- `scripts/joint_position_commander_uam_v5.py`
  - `uam_v5` 的平滑插值关节命令脚本
  - 同时记录目标、位置、速度、力矩和误差
- `scripts/arm_zero_hold_logger_uam_v5.py`
  - 把 `uam_v5` 的指定关节保持在零位
  - 输出 CSV 和 PNG

### 4. 接口 / Launch 与运行入口

命名空间约定：

- 默认 namespace：`uav_arm`
- 控制器命令话题格式：
  - `/<namespace>/<joint>_position_controller/command`
- 关节状态话题：
  - `/<namespace>/joint_states`
- commander 脚本还会发布：
  - `/<namespace>/target_joint_states`

Launch 入口：

```bash
roslaunch arm_controller controller_bringup.launch
roslaunch arm_controller controller_bringup_uam_v5.launch
```

运行脚本：

```bash
rosrun arm_controller joint_position_commander.py
rosrun arm_controller joint_position_commander_uam_v5.py
rosrun arm_controller arm_zero_hold_logger_uam_v5.py
```

### 5. 如何运行或验证

通常这个包由 `uav_arm_top` 顶层 launch 带起，但 Gazebo 模型启动后也可以单独检查：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch arm_controller controller_bringup.launch
rosrun arm_controller joint_position_commander.py
```

`uam_v5` 示例：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch arm_controller controller_bringup_uam_v5.launch
rosrun arm_controller joint_position_commander_uam_v5.py
rosrun arm_controller arm_zero_hold_logger_uam_v5.py
```

验证时重点看：

- controller spawner 启动时不报缺失控制器错误，
- 关节命令话题出现在正确 namespace 下，
- `/uav_arm/joint_states` 会随脚本更新，
- `uam_v5` 日志脚本能正常生成 CSV/PNG。

### 6. 文件索引

- `config/joint_pid.yaml`：`uav_arm_v4` 的 PID 参数
- `config/joint_pid_uam_v5.yaml`：`uam_v5` 的 PID 参数
- `launch`：控制器启动文件
- `scripts`：动作与日志工具
