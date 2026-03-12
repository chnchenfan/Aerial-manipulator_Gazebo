# eso_att_control

## English

### 1. Overview

`eso_att_control` is the custom PX4 multicopter attitude controller used between the modified position and rate loops. It consumes attitude setpoints and produces rate setpoints while incorporating model-aware arm dynamics.

### 2. What Was Modified

This module was added to replace the stock PX4 attitude loop with a controller that:

- works with the custom ESO position and rate modules,
- uses arm kinematics and dynamic parameters inside PX4,
- reads `arm_joint_states` for online model updates,
- publishes debug values for ROS-side inspection.

### 3. Key Components

- `eso_att_control_main.cpp`
  - module entry and runtime loop
- `eso_att_control.hpp`
  - module state, subscriptions, publications, and parameter bindings
- `ArmKinematics.hpp`
  - arm-kinematics helper used for dynamic compensation
- `ESOAttitudeControl/*`
  - attitude-control implementation
- `eso_att_control_params.c`
  - attitude-loop and model parameters

Main input topics:

- `vehicle_attitude`
- `vehicle_attitude_setpoint`
- `vehicle_angular_velocity`
- `vehicle_control_mode`
- `vehicle_status`
- `vehicle_land_detected`
- `arm_joint_states`

Main output topics:

- `vehicle_rates_setpoint`
- `eso_rate_aux`
- `debug_key_value`

### 4. Interfaces / Launch or Runtime Entry Points

PX4 shell commands:

```bash
eso_att_control start
eso_att_control status
eso_att_control stop
```

Normal activation path:

- started by `10016_uav_arm_v4`
- started by `10019_uam_v5`

Important parameter families:

- `ESO_ROLL_P`
- `ESO_PITCH_P`
- `ESO_YAW_P`
- `ESO_ROLLRATE_MAX`
- `ESO_PITCHRT_MAX`
- `ESO_YAWRATE_MAX`
- `ESO_ARM_MODEL`
- `ESO_MASS_TOTAL`
- `ESO_COM_*`
- `ESO_ARM_I*`

### 5. How to Run or Validate

Validate this module inside a full SITL run:

- `eso_att_control status` shows the module running,
- `vehicle_rates_setpoint` is published when position or offboard setpoints arrive,
- switching between `uav_arm_v4` and `uam_v5` changes the model profile through `ESO_ARM_MODEL`,
- debug attitude values appear on the ROS side through MAVROS named-value forwarding.

### 6. File Map

- `eso_att_control_main.cpp`: module runtime
- `eso_att_control.hpp`: module interface and state
- `ArmKinematics.hpp`: dynamic arm model support
- `ESOAttitudeControl`: inner attitude-control implementation
- `eso_att_control_params.c`: attitude-loop parameters

## 中文

### 1. 概述

`eso_att_control` 是这个仓库里自定义的 PX4 多旋翼姿态控制器，位于自定义位置环和角速度环之间。它读取姿态设定值，输出角速度设定值，并把机械臂的动力学影响纳入控制链。

### 2. 修改了什么

这个模块替换了 PX4 官方姿态环，主要补了以下能力：

- 与自定义 ESO 位置环、速率环配套工作，
- 在 PX4 内部使用机械臂运动学和动力学参数，
- 读取 `arm_joint_states` 做在线模型更新，
- 输出调试量给 ROS 侧检查。

### 3. 关键组成

- `eso_att_control_main.cpp`
  - 模块入口与主循环
- `eso_att_control.hpp`
  - 模块状态、订阅、发布与参数绑定
- `ArmKinematics.hpp`
  - 动态补偿使用的机械臂运动学工具
- `ESOAttitudeControl/*`
  - 姿态控制实现
- `eso_att_control_params.c`
  - 姿态环与模型参数

主要输入话题：

- `vehicle_attitude`
- `vehicle_attitude_setpoint`
- `vehicle_angular_velocity`
- `vehicle_control_mode`
- `vehicle_status`
- `vehicle_land_detected`
- `arm_joint_states`

主要输出话题：

- `vehicle_rates_setpoint`
- `eso_rate_aux`
- `debug_key_value`

### 4. 接口 / 运行入口

PX4 shell 命令：

```bash
eso_att_control start
eso_att_control status
eso_att_control stop
```

正常启动路径：

- 由 `10016_uav_arm_v4` 自动启动
- 由 `10019_uam_v5` 自动启动

重点参数族：

- `ESO_ROLL_P`
- `ESO_PITCH_P`
- `ESO_YAW_P`
- `ESO_ROLLRATE_MAX`
- `ESO_PITCHRT_MAX`
- `ESO_YAWRATE_MAX`
- `ESO_ARM_MODEL`
- `ESO_MASS_TOTAL`
- `ESO_COM_*`
- `ESO_ARM_I*`

### 5. 如何运行或验证

推荐在完整 SITL 中验证：

- `eso_att_control status` 显示模块在运行，
- 当上游位置设定值或 offboard 指令到来时，`vehicle_rates_setpoint` 会持续发布，
- 切换 `uav_arm_v4` 与 `uam_v5` 时，`ESO_ARM_MODEL` 能带来不同模型行为，
- 姿态相关调试量能通过 MAVROS named-value 链路传到 ROS 侧。

### 6. 文件索引

- `eso_att_control_main.cpp`：模块主循环
- `eso_att_control.hpp`：模块接口与内部状态
- `ArmKinematics.hpp`：动态机械臂模型支持
- `ESOAttitudeControl`：姿态控制实现
- `eso_att_control_params.c`：姿态环参数
