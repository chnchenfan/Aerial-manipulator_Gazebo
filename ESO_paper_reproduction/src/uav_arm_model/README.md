# uav_arm_model

## English

### 1. Overview

`uav_arm_model` is the ROS model package for the two vehicle-manipulator variants used in this repository:

- `uav_arm_v4`
- `uam_v5`

It provides the URDF/xacro descriptions, meshes, RViz resources, and ros_control transmissions used by the ROS/Gazebo side of the experiments.

### 2. What Was Modified

This package was added to mirror the project-specific Gazebo models and to expose the arm joints to ROS control:

- added `uav_arm_v4.urdf.xacro`,
- added `uam_v5.urdf.xacro`,
- added matching mesh sets for both models,
- added ros_control transmission definitions,
- added Gazebo `libgazebo_ros_control.so` integration in the URDF side.

### 3. Key Components

#### `uav_arm_v4`

- file: `urdf/uav_arm_v4.urdf.xacro`
- main joints:
  - `arm_joint1`
  - `arm_joint2`
  - `arm_joint3`
  - `arm_joint4`
  - `left_gripper_joint`
  - `right_gripper_joint`
- ros_control transmissions exported for:
  - `arm_joint1`
  - `arm_joint2`
  - `arm_joint3`
  - `arm_joint4`
  - `left_gripper_joint`

#### `uam_v5`

- file: `urdf/uam_v5.urdf.xacro`
- main joints:
  - `arm_joint1`
  - `arm_joint2`
  - `left_hand_joint`
  - `right_hand_joint`
- ros_control transmissions exported for:
  - `arm_joint1`
  - `arm_joint2`
  - `left_hand_joint`

### 4. Interfaces / Launch or Runtime Entry Points

The top-level launch files consume this package through:

- `$(find uav_arm_model)/urdf/uav_arm_v4.urdf.xacro`
- `$(find uav_arm_model)/urdf/uam_v5.urdf.xacro`

The ROS-side URDF models correspond to the Gazebo-side SDF models in:

- `Tools/sitl_gazebo/models/uav_arm_v4/uav_arm_v4.sdf`
- `Tools/sitl_gazebo/models/uam_v5/uam_v5.sdf`

The intended mapping is:

- ROS URDF/xacro: ros_control controllers, transmissions, and robot_description
- Gazebo SDF: SITL physics model, motors, sensors, and PX4 plugin integration

### 5. How to Run or Validate

Load either model through the top-level launch:

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
```

Validation checks:

- `/robot_description` is populated,
- joint names in ROS match the controller YAML and scripts,
- Gazebo can spawn the matching model without missing mesh errors,
- ros_control can find the declared transmissions.

### 6. File Map

- `urdf`: xacro definitions for `uav_arm_v4` and `uam_v5`
- `meshes`: STL assets for both model variants
- `rviz/uavBase_arm.rviz`: visualization preset
- `scripts/calc_relative.py`: helper script for relative calculations

## 中文

### 1. 概述

`uav_arm_model` 是这个仓库里两套飞行器-机械臂模型对应的 ROS 模型包：

- `uav_arm_v4`
- `uam_v5`

它负责提供 URDF/xacro、mesh、RViz 资源，以及 ROS/Gazebo 联调时需要的 ros_control transmission 定义。

### 2. 修改了什么

这个包的作用是把项目自定义 Gazebo 模型同步成 ROS 可用的模型描述，并把机械臂关节接进 ROS 控制链：

- 新增 `uav_arm_v4.urdf.xacro`，
- 新增 `uam_v5.urdf.xacro`，
- 为两套模型分别加入 mesh，
- 加入 ros_control transmission，
- 在 URDF 侧加入 Gazebo 的 `libgazebo_ros_control.so` 插件接入。

### 3. 关键组成

#### `uav_arm_v4`

- 文件：`urdf/uav_arm_v4.urdf.xacro`
- 主要关节：
  - `arm_joint1`
  - `arm_joint2`
  - `arm_joint3`
  - `arm_joint4`
  - `left_gripper_joint`
  - `right_gripper_joint`
- 暴露给 ros_control 的 transmission：
  - `arm_joint1`
  - `arm_joint2`
  - `arm_joint3`
  - `arm_joint4`
  - `left_gripper_joint`

#### `uam_v5`

- 文件：`urdf/uam_v5.urdf.xacro`
- 主要关节：
  - `arm_joint1`
  - `arm_joint2`
  - `left_hand_joint`
  - `right_hand_joint`
- 暴露给 ros_control 的 transmission：
  - `arm_joint1`
  - `arm_joint2`
  - `left_hand_joint`

### 4. 接口 / Launch 与运行入口

顶层 launch 会通过以下入口使用这个包：

- `$(find uav_arm_model)/urdf/uav_arm_v4.urdf.xacro`
- `$(find uav_arm_model)/urdf/uam_v5.urdf.xacro`

ROS 侧的 URDF 模型与 Gazebo 侧的 SDF 模型一一对应：

- `Tools/sitl_gazebo/models/uav_arm_v4/uav_arm_v4.sdf`
- `Tools/sitl_gazebo/models/uam_v5/uam_v5.sdf`

这两层的职责分工是：

- ROS URDF/xacro：给 ros_control、transmission、`robot_description` 使用
- Gazebo SDF：给 SITL 物理模型、电机、传感器和 PX4 插件使用

### 5. 如何运行或验证

通过顶层 launch 直接加载模型：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
```

验证时重点看：

- `/robot_description` 已经被加载，
- ROS 中的关节名和 YAML、脚本里使用的名字一致，
- Gazebo 生成模型时不报 mesh 丢失错误，
- ros_control 能找到声明好的 transmission。

### 6. 文件索引

- `urdf`：`uav_arm_v4` 与 `uam_v5` 的 xacro 定义
- `meshes`：两套模型的 STL 资源
- `rviz/uavBase_arm.rviz`：可视化配置
- `scripts/calc_relative.py`：相对量计算辅助脚本
