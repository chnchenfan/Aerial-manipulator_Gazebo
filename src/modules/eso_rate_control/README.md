# eso_rate_control

## English

### 1. Overview

`eso_rate_control` is the custom PX4 multicopter angular-rate controller at the bottom of the modified ESO control stack. It converts rate setpoints into torque, thrust, and actuator commands while accounting for model-dependent arm dynamics.

### 2. What Was Modified

This module was added to replace the stock PX4 rate controller with one that:

- works with the custom attitude controller and `eso_rate_aux`,
- updates dynamics using `arm_joint_states`,
- publishes debug values for external analysis,
- exposes its own rate/observer parameter set.

### 3. Key Components

- `ESOMulticopterRateControl.cpp/.hpp`
  - PX4 module entry and runtime loop
- `ESORateControl/*`
  - rate-control and attitude-ESO implementation
- `eso_rate_control_params.c`
  - rate-loop, observer, and limit parameters

Main input topics:

- `vehicle_angular_velocity`
- `vehicle_angular_acceleration`
- `vehicle_rates_setpoint`
- `vehicle_control_mode`
- `vehicle_status`
- `battery_status`
- `control_allocator_status`
- `eso_rate_aux`
- `arm_joint_states`

Main output topics:

- `actuator_controls_0`
- `actuator_controls_status_0`
- `rate_ctrl_status`
- `vehicle_torque_setpoint`
- `vehicle_thrust_setpoint`
- `debug_key_value`

### 4. Interfaces / Launch or Runtime Entry Points

PX4 shell commands:

```bash
eso_rate_control start
eso_rate_control status
eso_rate_control stop
```

Normal activation path:

- `10016_uav_arm_v4`
- `10019_uam_v5`

Important parameter families:

- `ESO_ROLLRATE_*`
- `ESO_PITCHRATE_*`
- `ESO_YAWRATE_*`
- `ESO_INERTIA_*`
- `ESO_RATE_BW_*`
- `ESO_K_BETA`
- `ESO_MAX_TORQUE`
- `ESO_TAUS_*`
- `ESO_BAT_SCALE_EN`
- `ESO_ARM_MODEL`

### 5. How to Run or Validate

Validate inside SITL by checking:

- `eso_rate_control status` shows the module running,
- `vehicle_torque_setpoint` and `vehicle_thrust_setpoint` are published,
- `rate_ctrl_status` updates during flight,
- ROS-side named-value logging shows the exported rate debug values,
- arm motion changes the rate-control compensation path when `arm_joint_states` is available.

### 6. File Map

- `ESOMulticopterRateControl.cpp/.hpp`: module runtime
- `ESORateControl`: rate-control implementation
- `eso_rate_control_params.c`: rate-loop and observer parameters

## 中文

### 1. 概述

`eso_rate_control` 是这个仓库里自定义 ESO 控制栈最底层的 PX4 多旋翼角速度控制器。它把角速度设定值转换成力矩、推力和执行器控制量，同时考虑机械臂带来的模型变化。

### 2. 修改了什么

这个模块替换了 PX4 官方速率环，主要新增了以下能力：

- 与自定义姿态控制器和 `eso_rate_aux` 配套工作，
- 基于 `arm_joint_states` 更新动力学补偿，
- 对外发布调试量，便于实验分析，
- 暴露独立的速率环与观测器参数族。

### 3. 关键组成

- `ESOMulticopterRateControl.cpp/.hpp`
  - PX4 模块入口与主循环
- `ESORateControl/*`
  - 速率控制与姿态 ESO 实现
- `eso_rate_control_params.c`
  - 速率环、观测器与限幅参数

主要输入话题：

- `vehicle_angular_velocity`
- `vehicle_angular_acceleration`
- `vehicle_rates_setpoint`
- `vehicle_control_mode`
- `vehicle_status`
- `battery_status`
- `control_allocator_status`
- `eso_rate_aux`
- `arm_joint_states`

主要输出话题：

- `actuator_controls_0`
- `actuator_controls_status_0`
- `rate_ctrl_status`
- `vehicle_torque_setpoint`
- `vehicle_thrust_setpoint`
- `debug_key_value`

### 4. 接口 / 运行入口

PX4 shell 命令：

```bash
eso_rate_control start
eso_rate_control status
eso_rate_control stop
```

正常启动路径：

- `10016_uav_arm_v4`
- `10019_uam_v5`

重点参数族：

- `ESO_ROLLRATE_*`
- `ESO_PITCHRATE_*`
- `ESO_YAWRATE_*`
- `ESO_INERTIA_*`
- `ESO_RATE_BW_*`
- `ESO_K_BETA`
- `ESO_MAX_TORQUE`
- `ESO_TAUS_*`
- `ESO_BAT_SCALE_EN`
- `ESO_ARM_MODEL`

### 5. 如何运行或验证

建议在 SITL 中验证以下几点：

- `eso_rate_control status` 显示模块正在运行，
- `vehicle_torque_setpoint` 与 `vehicle_thrust_setpoint` 持续发布，
- `rate_ctrl_status` 在飞行中持续更新，
- ROS 侧 named-value 日志能看到导出的速率环调试量，
- 当 `arm_joint_states` 可用时，机械臂动作会影响速率环补偿行为。

### 6. 文件索引

- `ESOMulticopterRateControl.cpp/.hpp`：模块主循环
- `ESORateControl`：速率控制实现
- `eso_rate_control_params.c`：速率环和观测器参数
