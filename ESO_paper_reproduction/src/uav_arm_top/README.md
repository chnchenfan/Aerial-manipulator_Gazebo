# uav_arm_top

## English

### 1. Overview

`uav_arm_top` is the top-level demo package for this repository. It ties together PX4 SITL, Gazebo, MAVROS, the ROS arm controllers, and the offboard demo nodes for the two supported platforms:

- `uav_arm_v4`
- `uam_v5`

### 2. What Was Modified

This package was added to expose the modified system through reproducible launch and demo entry points:

- one integrated launch for `uav_arm_v4`,
- one integrated launch for `uam_v5`,
- one circle-demo launch,
- four offboard demo nodes for hover and trajectory tests,
- one ROS-to-MAVLink bridge script used specifically by the `uam_v5` chain.

### 3. Key Components

#### Launch Files

- `launch/arm_pid_SITL_Gazebo.launch`
  - `uav_arm_v4` full-stack launch
  - loads `uav_arm_v4.urdf.xacro`
  - starts PX4 SITL + Gazebo + MAVROS + `arm_controller/controller_bringup.launch`
- `launch/arm_pid_SITL_Gazebo_uam_v5.launch`
  - `uam_v5` full-stack launch
  - loads `uam_v5.urdf.xacro`
  - starts PX4 SITL + Gazebo + MAVROS + `controller_bringup_uam_v5.launch`
  - also starts `uam_v5_arm_joint_state_bridge.py`
- `launch/circle_offboard_SITL_Gazebo.launch`
  - `uav_arm_v4` full launch plus `eso_circle_offboard_node`

#### Demo Nodes

- `eso_offboard_node`
  - fixed-point offboard setpoint publisher
- `eso_offboard_2_5_node`
  - staged altitude demo used for `uam_v5`
- `eso_square_offboard_node`
  - square trajectory publisher
- `eso_circle_offboard_node`
  - circle trajectory publisher with configurable radius, center, altitude, and waypoint count

#### Bridge Script

- `scripts/uam_v5_arm_joint_state_bridge.py`
  - subscribes to ROS joint states
  - publishes MAVLink named values to `/mavlink/to`
  - provides the `uam_v5` joint-state feed that PX4 reconstructs through `arm_joint_bridge`

### 4. Interfaces / Launch or Runtime Entry Points

Main launch commands:

```bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch
```

Main demo commands:

```bash
rosrun uav_arm_top eso_offboard_node
rosrun uav_arm_top eso_offboard_2_5_node
rosrun uav_arm_top eso_square_offboard_node
rosrun uav_arm_top eso_circle_offboard_node
```

Important runtime difference:

- `uav_arm_v4` launch path uses ROS arm control directly.
- `uam_v5` launch path adds `uam_v5_arm_joint_state_bridge.py`, because the PX4 ESO modules need arm joint state feedback inside PX4 and the model is wired through the named-value bridge chain.

### 5. How to Run or Validate

Prepare the environment:

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
```

Launch the `uav_arm_v4` demo:

```bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
rosrun uav_arm_top eso_offboard_node
rosrun arm_controller joint_position_commander.py
```

Launch the `uam_v5` demo:

```bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
rosrun uav_arm_top eso_offboard_2_5_node
rosrun arm_controller joint_position_commander_uam_v5.py
```

Trajectory demos:

```bash
rosrun uav_arm_top eso_square_offboard_node
rosrun uav_arm_top eso_circle_offboard_node
```

Validation checks:

- the integrated launch starts PX4 SITL, Gazebo, MAVROS, and the arm controller bring-up,
- `uav_arm_v4` and `uam_v5` load the correct URDF and airframe,
- the demo nodes can switch to `OFFBOARD` and arm,
- `uam_v5_arm_joint_state_bridge.py` publishes without MAVLink topic errors.

### 6. File Map

- `launch`: top-level experiment launches
- `src`: offboard demo nodes
- `scripts/uam_v5_arm_joint_state_bridge.py`: `uam_v5` joint-state bridge
- `scripts/*.md`: supporting workflow notes for SITL and bridge debugging

## 中文

### 1. 概述

`uav_arm_top` 是这个仓库的顶层 demo 包，负责把 PX4 SITL、Gazebo、MAVROS、ROS 机械臂控制器，以及 offboard 演示节点串成一套可直接运行的实验入口。

它覆盖两种平台：

- `uav_arm_v4`
- `uam_v5`

### 2. 修改了什么

这个包的作用就是把你改动后的整套系统变成可复现的顶层入口：

- 为 `uav_arm_v4` 提供一套完整 launch，
- 为 `uam_v5` 提供一套完整 launch，
- 为圆轨迹提供单独 launch，
- 提供四个 offboard demo 节点，
- 为 `uam_v5` 增加一条专用的 ROS 到 MAVLink 关节桥接脚本。

### 3. 关键组成

#### Launch 文件

- `launch/arm_pid_SITL_Gazebo.launch`
  - `uav_arm_v4` 全链路启动文件
  - 加载 `uav_arm_v4.urdf.xacro`
  - 启动 PX4 SITL + Gazebo + MAVROS + `arm_controller/controller_bringup.launch`
- `launch/arm_pid_SITL_Gazebo_uam_v5.launch`
  - `uam_v5` 全链路启动文件
  - 加载 `uam_v5.urdf.xacro`
  - 启动 PX4 SITL + Gazebo + MAVROS + `controller_bringup_uam_v5.launch`
  - 额外启动 `uam_v5_arm_joint_state_bridge.py`
- `launch/circle_offboard_SITL_Gazebo.launch`
  - 在 `uav_arm_v4` 完整启动基础上直接带起 `eso_circle_offboard_node`

#### Demo 节点

- `eso_offboard_node`
  - 固定点 offboard 设定值发布器
- `eso_offboard_2_5_node`
  - 面向 `uam_v5` 的分阶段高度实验节点
- `eso_square_offboard_node`
  - 方形轨迹发布器
- `eso_circle_offboard_node`
  - 圆轨迹发布器，可配置半径、圆心、高度和航点数

#### 桥接脚本

- `scripts/uam_v5_arm_joint_state_bridge.py`
  - 订阅 ROS 关节状态
  - 向 `/mavlink/to` 发布 MAVLink named value
  - 为 `uam_v5` 提供 PX4 侧 `arm_joint_bridge` 所需的关节反馈

### 4. 接口 / Launch 与运行入口

主要 launch 命令：

```bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch
```

主要 demo 命令：

```bash
rosrun uav_arm_top eso_offboard_node
rosrun uav_arm_top eso_offboard_2_5_node
rosrun uav_arm_top eso_square_offboard_node
rosrun uav_arm_top eso_circle_offboard_node
```

关键差异：

- `uav_arm_v4` 的顶层链路直接使用 ROS 机械臂控制。
- `uam_v5` 的顶层链路会额外带起 `uam_v5_arm_joint_state_bridge.py`，因为 PX4 内部 ESO 模块需要机械臂关节状态，而这套模型走的是 named-value 桥接链。

### 5. 如何运行或验证

先准备环境：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
```

启动 `uav_arm_v4` demo：

```bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
rosrun uav_arm_top eso_offboard_node
rosrun arm_controller joint_position_commander.py
```

启动 `uam_v5` demo：

```bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
rosrun uav_arm_top eso_offboard_2_5_node
rosrun arm_controller joint_position_commander_uam_v5.py
```

轨迹实验：

```bash
rosrun uav_arm_top eso_square_offboard_node
rosrun uav_arm_top eso_circle_offboard_node
```

验证时重点看：

- 顶层 launch 能正确拉起 PX4 SITL、Gazebo、MAVROS 和 arm controller，
- `uav_arm_v4` 与 `uam_v5` 分别加载了正确的 URDF 和 airframe，
- demo 节点能够进入 `OFFBOARD` 并解锁，
- `uam_v5_arm_joint_state_bridge.py` 发布时不出现 MAVLink 话题错误。

### 6. 文件索引

- `launch`：顶层实验启动文件
- `src`：offboard demo 节点
- `scripts/uam_v5_arm_joint_state_bridge.py`：`uam_v5` 关节桥接脚本
- `scripts/*.md`：SITL 和桥接调试说明
