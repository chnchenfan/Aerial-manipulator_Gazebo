# eso_pos_control

## English

### 1. Overview

`eso_pos_control` is the custom PX4 multicopter position controller used in this repository. It replaces the stock PX4 position loop in the modified SITL airframes and acts as the top controller in the custom ESO chain.

### 2. What Was Modified

Compared with stock PX4 position control behavior, this module was added to:

- run an ESO-based position controller,
- consume arm joint states inside PX4,
- apply arm-aware center-of-mass compensation,
- publish debug values through `debug_key_value` for ROS-side analysis,
- feed the custom attitude controller through `vehicle_attitude_setpoint`.

### 3. Key Components

- `ESOMulticopterPositionControl.cpp/.hpp`
  - PX4 module entry and runtime loop
- `ESOPositionControl/*`
  - control math, position controller, and position ESO implementation
- `ESOTakeoff/*`
  - takeoff behavior support
- `eso_pos_control_params.c`
  - position-loop parameter definitions

Main input topics:

- `vehicle_local_position`
- `trajectory_setpoint`
- `vehicle_control_mode`
- `vehicle_land_detected`
- `vehicle_attitude`
- `vehicle_angular_velocity`
- `arm_joint_states`

Main output topics:

- `vehicle_attitude_setpoint`
- `vehicle_local_position_setpoint`
- `takeoff_status`
- `debug_key_value`

### 4. Interfaces / Launch or Runtime Entry Points

PX4 shell commands:

```bash
eso_pos_control start
eso_pos_control status
eso_pos_control stop
```

In normal use, the module is started by the modified airframes:

- `10021_uav_arm_v4`
- `10019_uam_v5`

Important parameter families:

- `ESO_XY_*`
- `ESO_Z_*`
- `ESO_THR_*`
- `ESO_ACC_*`
- `ESO_JERK_*`
- `ESO_POS_MODE`
- `ESO_ARM_MODEL`

### 5. How to Run or Validate

Run a SITL airframe that enables the custom stack:

```bash
cd /home/cf/PX4_Firmware_clean
make px4_sitl gazebo_uav_arm_v4
```

or:

```bash
make px4_sitl gazebo_uam_v5
```

Then validate:

- `flight_mode_manager status` reports the module running,
- `eso_pos_control status` reports the module running,
- `mc_pos_control status`, `mc_att_control status`, and `mc_rate_control status` report that the stock controllers are not running,
- `uam_v5` also reports `arm_joint_bridge status` running,
- `vehicle_attitude_setpoint` is being published,
- debug values appear on the ROS side through `/mavros/debug/named_value_float`,
- motion of the arm changes the compensation behavior rather than being ignored.

### 6. File Map

- `ESOMulticopterPositionControl.cpp/.hpp`: module runtime
- `ESOPositionControl`: ESO and position-control math
- `ESOTakeoff`: takeoff helper logic
- `eso_pos_control_params.c`: position-loop parameters

## 中文

### 1. 概述

`eso_pos_control` 是这个仓库里自定义的 PX4 多旋翼位置控制器。在修改后的 SITL airframe 中，它会替换官方 `mc_pos_control`，作为整套 ESO 控制链的最上层控制器。

### 2. 修改了什么

相对于 PX4 官方位置环，这个模块主要新增了以下能力：

- 使用 ESO 位置控制逻辑，
- 在 PX4 内部直接读取机械臂关节状态，
- 做与机械臂状态相关的动态质心补偿，
- 通过 `debug_key_value` 输出调试量，便于 ROS 侧分析，
- 通过 `vehicle_attitude_setpoint` 驱动下游自定义姿态控制器。

### 3. 关键组成

- `ESOMulticopterPositionControl.cpp/.hpp`
  - PX4 模块入口与主循环
- `ESOPositionControl/*`
  - 控制数学、位置控制器、位置 ESO 实现
- `ESOTakeoff/*`
  - 起飞逻辑支持
- `eso_pos_control_params.c`
  - 位置环参数定义

主要输入话题：

- `vehicle_local_position`
- `trajectory_setpoint`
- `vehicle_control_mode`
- `vehicle_land_detected`
- `vehicle_attitude`
- `vehicle_angular_velocity`
- `arm_joint_states`

主要输出话题：

- `vehicle_attitude_setpoint`
- `vehicle_local_position_setpoint`
- `takeoff_status`
- `debug_key_value`

### 4. 接口 / 运行入口

PX4 shell 命令：

```bash
eso_pos_control start
eso_pos_control status
eso_pos_control stop
```

正常使用时，这个模块由以下 airframe 自动启动：

- `10021_uav_arm_v4`
- `10019_uam_v5`

重点参数族：

- `ESO_XY_*`
- `ESO_Z_*`
- `ESO_THR_*`
- `ESO_ACC_*`
- `ESO_JERK_*`
- `ESO_POS_MODE`
- `ESO_ARM_MODEL`

### 5. 如何运行或验证

先运行启用了自定义控制栈的 SITL：

```bash
cd /home/cf/PX4_Firmware_clean
make px4_sitl gazebo_uav_arm_v4
```

或者：

```bash
make px4_sitl gazebo_uam_v5
```

之后重点验证：

- `flight_mode_manager status` 显示模块正在运行，
- `eso_pos_control status` 显示模块正在运行，
- `mc_pos_control status`、`mc_att_control status`、`mc_rate_control status` 显示官方控制器未运行，
- `uam_v5` 下 `arm_joint_bridge status` 同样显示正在运行，
- `vehicle_attitude_setpoint` 持续发布，
- ROS 侧能通过 `/mavros/debug/named_value_float` 看到调试量，
- 机械臂运动时，位置环补偿行为确实发生变化，而不是忽略关节状态。

### 6. 文件索引

- `ESOMulticopterPositionControl.cpp/.hpp`：模块主循环
- `ESOPositionControl`：ESO 与位置控制数学
- `ESOTakeoff`：起飞辅助逻辑
- `eso_pos_control_params.c`：位置环参数
