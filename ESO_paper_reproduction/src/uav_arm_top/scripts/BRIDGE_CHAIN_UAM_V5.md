# `uam_v5` Joint Bridge 链路说明与排障记录

## 1. 什么是“桥接链路”

这里的“桥接链路”指的是：

- ROS 侧已经存在机械臂关节状态话题 `/uav_arm/joint_states`
- 但 PX4 的 ESO 控制器不能直接订阅 ROS 话题
- 因此需要把 ROS 关节状态逐级转换成 PX4 能消费的 `uORB` 消息 `arm_joint_states`

这条链路的目标是：

- 让 `eso_att_control`
- `eso_rate_control`
- `eso_pos_control`

能够读取 `uam_v5` 机械臂的实时关节角度和角速度，并据此更新动态 CoM / inertia。

---

## 2. 整条链路的输入输出

### 输入

ROS 话题：

- `/uav_arm/joint_states`

消息类型：

- `sensor_msgs/JointState`

实际关节含义：

- `arm_joint1`
- `arm_joint2`
- `left_hand_joint`

### 输出

PX4 `uORB` 话题：

- `arm_joint_states`

消息类型：

- [`msg/arm_joint_states.msg`](/home/cf/PX4_Firmware/msg/arm_joint_states.msg)

字段语义：

- `q[0] = arm_joint1`
- `q[1] = arm_joint2`
- `q[2] = left_hand_joint`
- `q[3] = 0`
- `dq[0] = arm_joint1 velocity`
- `dq[1] = arm_joint2 velocity`
- `dq[2] = left_hand_joint velocity`
- `dq[3] = 0`
- `valid = 当前桥接数据是否新鲜且完整`

---

## 3. 信息如何传递

完整链路如下：

1. Gazebo + `ros_control` 产生 `/uav_arm/joint_states`
2. ROS 节点 `uam_v5_arm_joint_state_bridge.py` 订阅 `/uav_arm/joint_states`
3. 该节点把关节状态编码成多个 MAVLink `NAMED_VALUE_FLOAT`
4. 节点将编码后的 MAVLink ROS 消息发布到 `/mavlink/to`
5. `mavros` 将消息转发给 PX4
6. PX4 `mavlink_receiver` 接收到 `NAMED_VALUE_FLOAT`
7. PX4 将其转换成 `debug_key_value` uORB
8. PX4 模块 `arm_joint_bridge` 订阅 `debug_key_value`
9. `arm_joint_bridge` 聚合 `AJQ* / AJD* / AJVAL`
10. `arm_joint_bridge` 发布 `arm_joint_states`
11. `eso_att_control / eso_rate_control / eso_pos_control` 订阅 `arm_joint_states`

---

## 4. 涉及的文件

### ROS 侧

- 桥接脚本  
  [`ESO_paper_reproduction/src/uav_arm_top/scripts/uam_v5_arm_joint_state_bridge.py`](/home/cf/PX4_Firmware/ESO_paper_reproduction/src/uav_arm_top/scripts/uam_v5_arm_joint_state_bridge.py)

- `uam_v5` launch  
  [`ESO_paper_reproduction/src/uav_arm_top/launch/arm_pid_SITL_Gazebo_uam_v5.launch`](/home/cf/PX4_Firmware/ESO_paper_reproduction/src/uav_arm_top/launch/arm_pid_SITL_Gazebo_uam_v5.launch)

- ROS 包构建与安装  
  [`ESO_paper_reproduction/src/uav_arm_top/CMakeLists.txt`](/home/cf/PX4_Firmware/ESO_paper_reproduction/src/uav_arm_top/CMakeLists.txt)  
  [`ESO_paper_reproduction/src/uav_arm_top/package.xml`](/home/cf/PX4_Firmware/ESO_paper_reproduction/src/uav_arm_top/package.xml)

### PX4 侧

- 桥接模块  
  [`src/modules/arm_joint_bridge/ArmJointBridge.cpp`](/home/cf/PX4_Firmware/src/modules/arm_joint_bridge/ArmJointBridge.cpp)  
  [`src/modules/arm_joint_bridge/ArmJointBridge.hpp`](/home/cf/PX4_Firmware/src/modules/arm_joint_bridge/ArmJointBridge.hpp)  
  [`src/modules/arm_joint_bridge/CMakeLists.txt`](/home/cf/PX4_Firmware/src/modules/arm_joint_bridge/CMakeLists.txt)  
  [`src/modules/arm_joint_bridge/Kconfig`](/home/cf/PX4_Firmware/src/modules/arm_joint_bridge/Kconfig)

- `uORB` 消息定义  
  [`msg/arm_joint_states.msg`](/home/cf/PX4_Firmware/msg/arm_joint_states.msg)

- MAVLink 接收与 `debug_key_value` 转发  
  [`src/modules/mavlink/mavlink_receiver.cpp`](/home/cf/PX4_Firmware/src/modules/mavlink/mavlink_receiver.cpp)

- `uam_v5` airframe 启动  
  [`ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5)

- 旧 `uav_arm_v4` airframe  
  [`ROMFS/px4fmu_common/init.d-posix/airframes/10016_uav_arm_v4`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/10016_uav_arm_v4)

- 避免官方 `mc_*` 控制器被重新拉起  
  [`ROMFS/px4fmu_common/init.d/rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps)

### ESO 动态模型消费侧

- [`src/modules/eso_att_control/eso_att_control_main.cpp`](/home/cf/PX4_Firmware/src/modules/eso_att_control/eso_att_control_main.cpp)
- [`src/modules/eso_rate_control/ESOMulticopterRateControl.cpp`](/home/cf/PX4_Firmware/src/modules/eso_rate_control/ESOMulticopterRateControl.cpp)
- [`src/modules/eso_pos_control/ESOMulticopterPositionControl.cpp`](/home/cf/PX4_Firmware/src/modules/eso_pos_control/ESOMulticopterPositionControl.cpp)

---

## 5. ROS 侧桥接脚本做了什么

脚本位置：

- [`uam_v5_arm_joint_state_bridge.py`](/home/cf/PX4_Firmware/ESO_paper_reproduction/src/uav_arm_top/scripts/uam_v5_arm_joint_state_bridge.py)

功能：

- 订阅 `/uav_arm/joint_states`
- 提取 3 个关节的位置和速度
- 编码成以下键名：
  - `AJQ0`
  - `AJQ1`
  - `AJQ2`
  - `AJQ3`
  - `AJD0`
  - `AJD1`
  - `AJD2`
  - `AJD3`
  - `AJVAL`
- 发布到 `/mavlink/to`

编码规则：

- `AJQ0 = arm_joint1 position`
- `AJQ1 = arm_joint2 position`
- `AJQ2 = left_hand_joint position`
- `AJQ3 = 0`
- `AJD0 = arm_joint1 velocity`
- `AJD1 = arm_joint2 velocity`
- `AJD2 = left_hand_joint velocity`
- `AJD3 = 0`
- `AJVAL = 1.0` 当且仅当 joint state 新鲜且三个关节都存在

当前实现的关键点：

- 使用 `pymavlink.dialects.v10.common`
- 发送到 `/mavlink/to`
- 默认发送频率 `200 Hz`
- 不是每周期发送 9 个键，而是“轮询发送”，每次只发 1 个键

---

## 6. PX4 侧桥接模块做了什么

模块位置：

- [`src/modules/arm_joint_bridge/ArmJointBridge.cpp`](/home/cf/PX4_Firmware/src/modules/arm_joint_bridge/ArmJointBridge.cpp)

功能：

- 订阅 `debug_key_value`
- 识别键名 `AJQ* / AJD* / AJVAL`
- 缓存每个字段的值和时间戳
- 以固定周期发布 `arm_joint_states`

有效性规则：

- `AJVAL > 0.5`
- 且 `q[0..3]`、`dq[0..3]` 对应字段都在 freshness 窗口内
- freshness timeout 当前为 `200 ms`

如果任意条件不满足：

- `valid = false`

---

## 7. 相关参数的含义

### PX4 参数

#### `ESO_ARM_MODEL`

定义位置：

- [`src/modules/eso_att_control/eso_att_control_params.c`](/home/cf/PX4_Firmware/src/modules/eso_att_control/eso_att_control_params.c)

含义：

- `0 = uav_arm_v4`
- `1 = uam_v5`

作用：

- 让 ESO 控制器在运行时选择对应的模型 profile

如何检查：

- PX4 shell: `param show ESO_ARM_MODEL`

你当前验证结果：

- `ESO_ARM_MODEL = 1`

这说明当前 airframe 已经正确切到了 `uam_v5` profile。

### ROS 私有参数

定义在脚本中：

- [`uam_v5_arm_joint_state_bridge.py`](/home/cf/PX4_Firmware/ESO_paper_reproduction/src/uav_arm_top/scripts/uam_v5_arm_joint_state_bridge.py)

参数含义：

- `~namespace`
  - 默认 `uav_arm`
  - 用于生成 joint state 话题名

- `~joint_states_topic`
  - 默认 `/<namespace>/joint_states`
  - ROS 输入话题

- `~publish_rate_hz`
  - 默认 `200.0`
  - 桥接节点发送 MAVLink 的频率

- `~joint_state_timeout`
  - 默认 `0.2`
  - joint state 超时阈值，超时则 `AJVAL=0.0`

- `~mavlink_topic`
  - 默认 `/mavlink/to`
  - 桥接节点发布给 MAVROS 的话题

---

## 8. 如何通过现象判断问题出在哪一段

### 情况 A

现象：

- `/uav_arm/joint_states` 正常
- `listener debug_key_value` 看不到 `AJ*`

结论：

- ROS -> MAVROS -> PX4 这段没打通

重点检查：

- `uam_v5_arm_joint_state_bridge.py` 是否在运行
- 是否发到了正确话题 `/mavlink/to`
- `pymavlink` 编码是否兼容

### 情况 B

现象：

- `listener debug_key_value` 能看到 `AJ*`
- `listener arm_joint_states` 仍全零或 `valid=false`

结论：

- `arm_joint_bridge` 聚合逻辑有问题，或者字段未在 freshness 时间内收齐

重点检查：

- `arm_joint_bridge status`
- 字段发送节奏是否过快导致覆盖

### 情况 C

现象：

- `arm_joint_states.valid = true`
- `mc_*` 和 `eso_*` 同时 running

结论：

- 启动链冲突

重点检查：

- [`10019_uam_v5`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5)
- [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps)

### 情况 D

现象：

- `arm_joint_states.valid = true`
- 但飞行时仍明显不稳

结论：

- 桥接链路已通，问题转移到 ESO 动态补偿模型或控制参数

重点检查：

- `ArmKinematics`
- `ESOModelProfile`
- ESO 增益

---

## 9. 这次实际遇到过的报错与含义

### 报错 1

报错原文：

```text
TypeError: must be str or None, not bytes
```

出现场景：

- `pymavlink` 调用 `named_value_float_encode()`

含义：

- 当前环境里的 `pymavlink` 实现对 `name` 的类型要求和最初脚本假设不一致

当时问题表现：

- 桥接节点直接崩溃
- ROS launch 中 `uam_v5_arm_joint_state_bridge` 退出

解决方式：

- 修正 `named_value_float_encode()` 的参数传递方式
- 随后统一改成更稳定的 MAVLink v1 dialect

### 报错 2

现象：

- `mc_rate_control`
- `mc_att_control`
- `mc_pos_control`
- `eso_att_control`
- `eso_rate_control`
- `eso_pos_control`

同时显示 `running`

含义：

- 不是模块重复编译，而是启动脚本把两套控制器都拉起来了

根因：

- [`10019_uam_v5`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5) 中虽然手动 `stop`/`start`
- 但后续 [`rc.vehicle_setup`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.vehicle_setup) 会调用 [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps)
- `rc.mc_apps` 原本只排除了 `10016`，没有排除 `10019`

解决方式：

- 修改 [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps)
- 让 `10019` 也跳过官方 `mc_*`

### 报错 3

现象：

- `listener debug_key_value` 只能看到例如 `ESO_TZ`
- 看不到 `AJQ* / AJD* / AJVAL`
- `arm_joint_states` 全零且 `valid=false`

含义：

- ROS 桥接消息根本没进入 PX4

根因：

- 桥接脚本最初发布到了 `/mavros/mavlink/to`
- 但实际 `mavros` 订阅的是 `/mavlink/to`

解决方式：

- 将桥接脚本默认发布话题改为 `/mavlink/to`

### 报错 4

现象：

- `debug_key_value` 能看到零星 `AJ*`
- `arm_joint_states` 仍然 `valid=false`

含义：

- 字段没有在 freshness 窗口内被聚合完整

根因：

- ROS 节点一轮一次性发送 9 个 `NAMED_VALUE_FLOAT`
- `debug_key_value` 是单消息覆盖式数据流，不适合瞬时 burst 发送

解决方式：

- 改为轮询发送
- 每次只发 1 个键
- 发送频率提到 `200 Hz`

---

## 10. 实际排障过程全流程

### 第一步：先确认启动链

检查命令：

```text
mc_rate_control status
mc_att_control status
mc_pos_control status
eso_att_control status
eso_rate_control status
eso_pos_control status
arm_joint_bridge status
param show ESO_ARM_MODEL
```

发现：

- `ESO_ARM_MODEL = 1`
- 但一开始 `mc_*` 和 `eso_*` 同时 running

处理：

- 修复 [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps)

### 第二步：确认 ROS 侧源数据是正常的

检查命令：

```text
rostopic echo /uav_arm/joint_states
```

发现：

- `arm_joint1 / arm_joint2 / left_hand_joint` 均有位置和速度

结论：

- Gazebo / ROS 控制器正常

### 第三步：确认 PX4 末端话题异常

检查命令：

```text
listener arm_joint_states
```

发现：

- `q = 0`
- `dq = 0`
- `valid = false`

结论：

- 问题在桥接链路中间

### 第四步：检查 PX4 是否收到原始键值

检查命令：

```text
listener debug_key_value
```

最初发现：

- 只能看到 `ESO_TZ`
- 看不到任何 `AJ*`

结论：

- ROS 桥接消息根本没进 PX4

处理：

- 修复桥接脚本发布话题，从 `/mavros/mavlink/to` 改到 `/mavlink/to`
- 同时改为 MAVLink v1 dialect

### 第五步：消息开始进入，但仍聚合失败

再次检查：

```text
listener debug_key_value
listener arm_joint_states
```

发现：

- `debug_key_value` 能开始看到 `AJ*`
- 但 `arm_joint_states.valid` 仍可能为 `false`

结论：

- 发送模式不适合 `debug_key_value` 覆盖式通道

处理：

- 改成轮询发送 9 个键
- 每周期只发一个键

### 第六步：最终验证成功

最终现象：

```text
TOPIC: debug_key_value
  key: "AJD1"

TOPIC: arm_joint_states
  q: [-0.0019, -0.0258, -0.0000, 0.0000]
  dq: [0.0120, 0.3490, 0.0017, 0.0000]
  valid: True
```

结论：

- 整条桥接链路已成功打通

---

## 11. 最终修复点总结

最终不是单点问题，而是 4 个问题叠加：

1. `10019_uam_v5` 启动 ESO 后，公共脚本又把官方 `mc_*` 重启了
2. ROS 桥接节点最初有 `pymavlink` 编码兼容问题
3. ROS 桥接消息发错了话题
4. 原始发送方式不适合 `debug_key_value` 聚合

最终解决方式分别是：

1. 在 [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps) 排除 `10019`
2. 修复 `named_value_float_encode` 调用，并切到 MAVLink v1
3. 改成发布到 `/mavlink/to`
4. 改成 `200 Hz` 轮询发送单键

---

## 12. 当前链路已达成的状态

当前已经确认成立：

- `uam_v5` 启动时只运行 `eso_*`，不再运行官方 `mc_*`
- `ROS /uav_arm/joint_states` 能传到 PX4
- PX4 能正确生成 `arm_joint_states`
- `arm_joint_states.valid = true`
- `q/dq` 数值会随真实机械臂状态变化

这说明：

- 桥接链路已完成
- 下一阶段问题如果再出现，优先看 ESO 动态模型与控制稳定性，不再优先怀疑桥接本身

