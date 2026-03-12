# arm_joint_bridge

## English

### 1. Overview

`arm_joint_bridge` is a PX4-side bridge module that reconstructs manipulator joint state information inside PX4. It is used so the custom ESO controllers can access arm motion even when the arm state originates on the ROS side.

### 2. What Was Modified

This module was added because the `uam_v5` integration path needs arm feedback inside PX4, but that feedback is generated in ROS. The bridge solves the problem by:

- receiving `debug_key_value` messages,
- decoding keys such as `AJQ*`, `AJD*`, and `AJVAL`,
- publishing a PX4-native `arm_joint_states` topic at a fixed rate.

### 3. Key Components

- `ArmJointBridge.cpp/.hpp`
  - module implementation
  - freshness tracking
  - `debug_key_value` parsing
  - `arm_joint_states` publishing

Decoded fields:

- `AJQ0` ... `AJQ3`: joint positions
- `AJD0` ... `AJD3`: joint velocities
- `AJVAL`: validity flag

Published topic:

- `arm_joint_states`

### 4. Interfaces / Launch or Runtime Entry Points

PX4 shell commands:

```bash
arm_joint_bridge start
arm_joint_bridge status
arm_joint_bridge stop
```

Normal runtime path for `uam_v5`:

1. ROS script `uam_v5_arm_joint_state_bridge.py` reads `/uav_arm/joint_states`.
2. It sends MAVLink named values on `/mavlink/to`.
3. MAVROS forwards them into PX4 as `debug_key_value`.
4. `arm_joint_bridge` rebuilds and publishes `arm_joint_states`.
5. `eso_pos_control`, `eso_att_control`, and `eso_rate_control` subscribe to that topic.

The module is automatically started by:

- `ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5`

### 5. How to Run or Validate

Run the `uam_v5` full stack:

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
```

Then validate:

- `arm_joint_bridge status` shows valid decoded data,
- PX4 prints the module as running,
- the ROS bridge script is active,
- moving the `uam_v5` arm changes the decoded joint values instead of leaving them stale.

### 6. File Map

- `ArmJointBridge.cpp`: parsing and publication logic
- `ArmJointBridge.hpp`: module declaration and state
- `CMakeLists.txt`: PX4 module registration

## 中文

### 1. 概述

`arm_joint_bridge` 是一个运行在 PX4 内部的机械臂关节桥接模块，用来把 ROS 侧产生的机械臂关节反馈重建成 PX4 原生话题，供自定义 ESO 控制器使用。

### 2. 修改了什么

之所以要加这个模块，是因为 `uam_v5` 的联调链路里，机械臂反馈最先出现在 ROS 侧，而 PX4 内部控制器又必须直接拿到关节状态。这个模块通过以下方式解决问题：

- 监听 `debug_key_value`，
- 解析 `AJQ*`、`AJD*`、`AJVAL` 等键值，
- 按固定频率发布 PX4 原生的 `arm_joint_states`。

### 3. 关键组成

- `ArmJointBridge.cpp/.hpp`
  - 模块实现
  - 新鲜度检查
  - `debug_key_value` 解析
  - `arm_joint_states` 发布

解析的字段：

- `AJQ0` ... `AJQ3`：关节位置
- `AJD0` ... `AJD3`：关节速度
- `AJVAL`：有效标志

发布的话题：

- `arm_joint_states`

### 4. 接口 / 运行入口

PX4 shell 命令：

```bash
arm_joint_bridge start
arm_joint_bridge status
arm_joint_bridge stop
```

`uam_v5` 的正常运行链如下：

1. ROS 脚本 `uam_v5_arm_joint_state_bridge.py` 读取 `/uav_arm/joint_states`。
2. 它向 `/mavlink/to` 发送 MAVLink named value。
3. MAVROS 把这些值送进 PX4 的 `debug_key_value`。
4. `arm_joint_bridge` 重建并发布 `arm_joint_states`。
5. `eso_pos_control`、`eso_att_control`、`eso_rate_control` 再订阅这个话题。

该模块会由以下 airframe 自动启动：

- `ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5`

### 5. 如何运行或验证

启动 `uam_v5` 全链路：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
```

之后重点验证：

- `arm_joint_bridge status` 能显示解码后的有效数据，
- PX4 控制台中能看到模块正常运行，
- ROS 桥接脚本在持续发布，
- 手动驱动 `uam_v5` 机械臂时，PX4 内部解码出的关节值会同步变化，而不是一直超时失效。

### 6. 文件索引

- `ArmJointBridge.cpp`：解析与发布逻辑
- `ArmJointBridge.hpp`：模块声明与状态
- `CMakeLists.txt`：PX4 模块注册
