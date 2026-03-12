# PX4-ESO UAV-Arm Repository

## English

### 1. Overview

This repository is a PX4 `v1.13.2` based research branch for a quadrotor-manipulator platform. The work is centered on one question: how to run a custom ESO-based control stack on PX4 while co-simulating a robotic arm in Gazebo and coordinating the arm through ROS.

The repository is no longer documented as a generic PX4 checkout. It is documented as a modified project with three visible pillars:

- custom PX4 flight-control modules for position, attitude, and rate control,
- two quadrotor-arm platform models: `uav_arm_v4` and `uam_v5`,
- ROS-side top-level demos, arm controllers, and analysis tools.

### 2. What Was Modified

Relative to upstream PX4 `v1.13.2`, the main modifications are:

- Added a custom PX4 ESO control stack:
  - `src/modules/eso_pos_control`
  - `src/modules/eso_att_control`
  - `src/modules/eso_rate_control`
  - `src/modules/eso_common`
- Added a PX4-side joint-state bridge:
  - `src/modules/arm_joint_bridge`
- Added ROS packages for integration and experiments:
  - `ESO_paper_reproduction/src/uav_control`
  - `ESO_paper_reproduction/src/arm_controller`
  - `ESO_paper_reproduction/src/uav_arm_model`
  - `ESO_paper_reproduction/src/uav_arm_top`
- Added two airframe entries that switch PX4 from the stock multicopter controllers to the ESO stack:
  - `ROMFS/px4fmu_common/init.d-posix/airframes/10016_uav_arm_v4`
  - `ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5`
- Added the matching Gazebo models inside the `Tools/sitl_gazebo` submodule:
  - `Tools/sitl_gazebo/models/uav_arm_v4`
  - `Tools/sitl_gazebo/models/uam_v5`
- Added a workspace helper:
  - `ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh`

### 3. Key Components

#### PX4 Control Stack

- `eso_pos_control`: consumes `trajectory_setpoint`, vehicle state, and arm joint state; publishes `vehicle_attitude_setpoint`.
- `eso_att_control`: consumes `vehicle_attitude_setpoint`; publishes `vehicle_rates_setpoint`.
- `eso_rate_control`: consumes `vehicle_rates_setpoint`; publishes torque, thrust, and actuator outputs.
- `eso_common`: holds model-profile abstractions and shared dynamic presets for `uav_arm_v4` and `uam_v5`.
- `arm_joint_bridge`: rebuilds `arm_joint_states` inside PX4 from `debug_key_value` messages derived from MAVLink named values.

#### ROS Integration Layer

- `uav_arm_top`: top-level launch and demo nodes for hover, staged altitude tests, square, and circle trajectories.
- `arm_controller`: ros_control PID bring-up and arm motion helper scripts.
- `uav_arm_model`: URDF/xacro, meshes, and ros_control transmission definitions for both models.
- `uav_control`: ROS-side utilities for force/attitude conversion, trajectory generation, and post-run analysis.

#### Model Variants

- `uav_arm_v4`: serial manipulator configuration with dedicated ROS controllers and Gazebo model assets.
- `uam_v5`: updated arm/hand geometry, different mass and inertia profile, and an additional ROS-to-MAVLink-to-PX4 joint-state bridge chain.

### 4. Interfaces and Runtime Entry Points

#### Airframe Entry

- `10016_uav_arm_v4`: sets `ESO_ARM_MODEL=0`, stops `mc_pos_control`, `mc_att_control`, `mc_rate_control`, then starts the ESO modules.
- `10019_uam_v5`: sets `ESO_ARM_MODEL=1`, starts the same ESO modules, and also starts `arm_joint_bridge`.

#### Top-Level Launch

- `roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch`
- `roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch`
- `roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch`

#### Demo Nodes

- `rosrun uav_arm_top eso_offboard_node`
- `rosrun uav_arm_top eso_offboard_2_5_node`
- `rosrun uav_arm_top eso_square_offboard_node`
- `rosrun uav_arm_top eso_circle_offboard_node`

#### Arm Scripts

- `rosrun arm_controller joint_position_commander.py`
- `rosrun arm_controller joint_position_commander_uam_v5.py`
- `rosrun arm_controller arm_zero_hold_logger_uam_v5.py`

### 5. End-to-End Data Path

For `uav_arm_v4`, the main loop is:

1. Gazebo loads `uav_arm_v4.sdf`.
2. ROS launch loads `uav_arm_v4.urdf.xacro` for ros_control.
3. Offboard demo nodes publish MAVROS setpoints.
4. PX4 airframe `10016_uav_arm_v4` activates the ESO stack.
5. Arm controllers move the manipulator through ROS topics.
6. PX4 position/attitude/rate controllers compensate for the moving arm.

For `uam_v5`, one more bridge is inserted:

1. ROS arm controllers publish commands and observe `/uav_arm/joint_states`.
2. `uam_v5_arm_joint_state_bridge.py` encodes joint position/velocity into MAVLink named values.
3. MAVROS forwards them into PX4 as `debug_key_value`.
4. `arm_joint_bridge` reconstructs `arm_joint_states`.
5. ESO modules use those states for dynamic CoM/inertia-aware compensation.

### 6. How to Run or Validate

#### Build PX4 SITL

```bash
cd /home/cf/PX4_Firmware_clean
make px4_sitl gazebo_uav_arm_v4
make px4_sitl gazebo_uam_v5
```

#### Build the Catkin Workspace

```bash
cd /home/cf/PX4_Firmware_clean/ESO_paper_reproduction
catkin_make
```

#### Prepare Each Terminal

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
```

#### Launch the Full Demo

```bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
```

#### Minimal Validation Checklist

- `rospack find uav_arm_top`
- `rospack find mavlink_sitl_gazebo`
- PX4 console shows `eso_pos_control`, `eso_att_control`, and `eso_rate_control` running
- `uam_v5` additionally shows `arm_joint_bridge` running
- arm scripts can command joints without namespace errors
- analysis scripts can read the recorded bag/log data

### 7. File Map

- `src/modules/eso_common`: model profiles shared by the ESO modules.
- `src/modules/eso_pos_control`: custom multicopter position controller.
- `src/modules/eso_att_control`: custom multicopter attitude controller.
- `src/modules/eso_rate_control`: custom multicopter rate controller.
- `src/modules/arm_joint_bridge`: PX4-side joint-state bridge.
- `ROMFS/px4fmu_common/init.d-posix/airframes`: airframes that enable the custom stack.
- `Tools/sitl_gazebo/models/uav_arm_v4`: Gazebo model for the `uav_arm_v4` platform.
- `Tools/sitl_gazebo/models/uam_v5`: Gazebo model for the `uam_v5` platform.
- `ESO_paper_reproduction/src/uav_arm_model`: ROS model package for both platforms.
- `ESO_paper_reproduction/src/arm_controller`: ROS arm-controller bring-up and scripts.
- `ESO_paper_reproduction/src/uav_arm_top`: top-level demos and launch files.
- `ESO_paper_reproduction/src/uav_control`: ROS utilities and analysis tools.

## 中文

### 1. 概述

这个仓库是基于 PX4 `v1.13.2` 改出来的四旋翼机械臂研究分支，核心目标不是保留一个通用 PX4，而是实现一套可在 PX4 上运行、并能和 Gazebo 机械臂联动的 ESO 控制系统。

整个项目可以概括成三部分：

- PX4 侧自定义 ESO 位置环、姿态环、角速度环，
- 两套飞行器-机械臂模型：`uav_arm_v4` 和 `uam_v5`，
- ROS 侧顶层 demo、机械臂控制、以及实验数据分析工具。

### 2. 修改了什么

相对于上游 PX4 `v1.13.2`，主要改动如下：

- 新增 PX4 自定义 ESO 控制链：
  - `src/modules/eso_pos_control`
  - `src/modules/eso_att_control`
  - `src/modules/eso_rate_control`
  - `src/modules/eso_common`
- 新增 PX4 侧机械臂关节桥接模块：
  - `src/modules/arm_joint_bridge`
- 新增 ROS 侧实验与联调包：
  - `ESO_paper_reproduction/src/uav_control`
  - `ESO_paper_reproduction/src/arm_controller`
  - `ESO_paper_reproduction/src/uav_arm_model`
  - `ESO_paper_reproduction/src/uav_arm_top`
- 新增两套 SITL airframe，用来替换 PX4 官方多旋翼控制器并切入 ESO 控制栈：
  - `ROMFS/px4fmu_common/init.d-posix/airframes/10016_uav_arm_v4`
  - `ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5`
- 在 `Tools/sitl_gazebo` 子模块中新增两套 Gazebo 模型：
  - `Tools/sitl_gazebo/models/uav_arm_v4`
  - `Tools/sitl_gazebo/models/uam_v5`
- 新增一键环境脚本：
  - `ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh`

### 3. 关键组成

#### PX4 控制栈

- `eso_pos_control`：读取 `trajectory_setpoint`、飞行状态和关节状态，输出 `vehicle_attitude_setpoint`。
- `eso_att_control`：读取 `vehicle_attitude_setpoint`，输出 `vehicle_rates_setpoint`。
- `eso_rate_control`：读取 `vehicle_rates_setpoint`，输出推力、力矩和执行器控制量。
- `eso_common`：统一管理 `uav_arm_v4` 与 `uam_v5` 的模型配置、质量、质心和惯量预设。
- `arm_joint_bridge`：把 MAVLink 命名浮点值经 `debug_key_value` 汇总成 PX4 内部的 `arm_joint_states`。

#### ROS 集成层

- `uav_arm_top`：顶层 launch 和 demo 节点，负责悬停、分阶段高度实验、方形轨迹、圆轨迹等实验入口。
- `arm_controller`：负责 ros_control PID 控制器加载，以及机械臂动作脚本。
- `uav_arm_model`：负责两套模型的 URDF/xacro、mesh 和 transmission。
- `uav_control`：负责 ROS 侧工具函数、轨迹生成和实验后处理，不是当前 PX4 主控制器本体。

#### 两种模型

- `uav_arm_v4`：串联机械臂构型，带独立 ROS 控制器与 Gazebo 资源。
- `uam_v5`：更新后的机械臂/夹手构型，质量惯量不同，并额外加入一条 ROS 到 MAVLink 再到 PX4 的关节桥接链路。

### 4. 接口与运行入口

#### Airframe 入口

- `10016_uav_arm_v4`：设置 `ESO_ARM_MODEL=0`，停止 `mc_pos_control`、`mc_att_control`、`mc_rate_control`，然后启动 ESO 模块。
- `10019_uam_v5`：设置 `ESO_ARM_MODEL=1`，启动同样的 ESO 模块，并额外启动 `arm_joint_bridge`。

#### 顶层 Launch

- `roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch`
- `roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch`
- `roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch`

#### Demo 节点

- `rosrun uav_arm_top eso_offboard_node`
- `rosrun uav_arm_top eso_offboard_2_5_node`
- `rosrun uav_arm_top eso_square_offboard_node`
- `rosrun uav_arm_top eso_circle_offboard_node`

#### 机械臂脚本

- `rosrun arm_controller joint_position_commander.py`
- `rosrun arm_controller joint_position_commander_uam_v5.py`
- `rosrun arm_controller arm_zero_hold_logger_uam_v5.py`

### 5. 端到端数据链

`uav_arm_v4` 的主链路如下：

1. Gazebo 加载 `uav_arm_v4.sdf`。
2. ROS launch 加载 `uav_arm_v4.urdf.xacro` 给 ros_control 使用。
3. Offboard demo 节点通过 MAVROS 下发设定值。
4. PX4 的 `10016_uav_arm_v4` airframe 激活 ESO 控制栈。
5. 机械臂 ROS 控制器驱动关节运动。
6. PX4 的位置环、姿态环、角速度环对机械臂扰动进行补偿。

`uam_v5` 比前者多一条桥接链：

1. ROS 机械臂控制器发布命令并读取 `/uav_arm/joint_states`。
2. `uam_v5_arm_joint_state_bridge.py` 把关节位置/速度编码成 MAVLink named value。
3. MAVROS 把这些值送入 PX4 的 `debug_key_value`。
4. `arm_joint_bridge` 重建出 `arm_joint_states`。
5. ESO 模块基于这些状态做动态质心和惯量补偿。

### 6. 如何运行或验证

#### 编译 PX4 SITL

```bash
cd /home/cf/PX4_Firmware_clean
make px4_sitl gazebo_uav_arm_v4
make px4_sitl gazebo_uam_v5
```

#### 编译 Catkin 工作区

```bash
cd /home/cf/PX4_Firmware_clean/ESO_paper_reproduction
catkin_make
```

#### 每个终端先准备环境

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
```

#### 启动整套 Demo

```bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
```

#### 最小检查项

- `rospack find uav_arm_top`
- `rospack find mavlink_sitl_gazebo`
- PX4 控制台能看到 `eso_pos_control`、`eso_att_control`、`eso_rate_control` 正常运行
- `uam_v5` 下还能看到 `arm_joint_bridge` 正常运行
- 机械臂脚本可以正常发命令，且 namespace 不报错
- 后处理脚本能读到 bag 或控制台日志

### 7. 文件索引

- `src/modules/eso_common`：ESO 模块共享模型配置。
- `src/modules/eso_pos_control`：自定义位置控制器。
- `src/modules/eso_att_control`：自定义姿态控制器。
- `src/modules/eso_rate_control`：自定义角速度控制器。
- `src/modules/arm_joint_bridge`：PX4 侧关节桥接模块。
- `ROMFS/px4fmu_common/init.d-posix/airframes`：启用自定义控制栈的 airframe。
- `Tools/sitl_gazebo/models/uav_arm_v4`：`uav_arm_v4` 的 Gazebo 模型。
- `Tools/sitl_gazebo/models/uam_v5`：`uam_v5` 的 Gazebo 模型。
- `ESO_paper_reproduction/src/uav_arm_model`：两套平台的 ROS 模型包。
- `ESO_paper_reproduction/src/arm_controller`：机械臂控制与测试脚本。
- `ESO_paper_reproduction/src/uav_arm_top`：顶层 demo 与 launch。
- `ESO_paper_reproduction/src/uav_control`：ROS 侧工具和分析包。
