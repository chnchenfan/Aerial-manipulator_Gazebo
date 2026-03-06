# 为什么 `uav_arm_v4` / `mc_*` 以前没出现，`uam_v5` 这次出现了

## 1. 结论先说

这次 `uam_v5` 无法保持 `OFFBOARD`、一进入就切到 `AUTO.RTL`，根因不是：

- 不是 `ESO` 控制器天然不能飞
- 不是 `mc_*` 控制器天然更稳定
- 也不是 `uam_v5` 模型本身先天错误

真正原因是：

- 触发问题的是 `Commander` 层的 `RC loss failsafe`
- 这个逻辑和 `mc_*` / `eso_*` 控制器本身是两层东西
- `uav_arm_v4` 和 `uam_v5` 使用的是不同的参数库
- `uam_v5` 当前加载到的参数状态，恰好会在无 RC 的 `OFFBOARD` 场景下触发 `RTL`

也就是说，这次暴露出来的是“参数和模式管理问题”，不是“控制律本身一定错误”。

## 2. 现象回顾

`uam_v5` 下运行 `eso_offboard_2_5_node` 时，出现了这组关键现象：

- ROS 端反复打印 `Offboard enabled`
- 没有看到 `Vehicle armed`
- PX4 打印 `Failsafe mode activated`
- `/mavros/state` 显示：
  - `armed: False`
  - `mode: "AUTO.RTL"`

同时，PX4 还收到了位置设定点：

- `MSG_ID=84 ... z=-2.000 type_mask=2552`

这说明：

- Offboard 设定点已经发到飞控
- 问题不在“上游没发 setpoint”
- 而是在飞控模式管理阶段，刚进入 `OFFBOARD` 就被安全机制拉走

## 3. 真正起作用的是哪一层

### 3.1 控制器层

控制器层包括：

- `mc_rate_control`
- `mc_att_control`
- `mc_pos_control`
- `eso_rate_control`
- `eso_att_control`
- `eso_pos_control`

它们负责的是：

- 姿态环
- 角速度环
- 位置环
- 推力 / 力矩计算

### 3.2 Commander 层

`Commander` 负责的是：

- 能不能进入 `OFFBOARD`
- 无 RC / 无 offboard setpoint 时是否触发 failsafe
- 触发后切 `POSCTL` / `LAND` / `RTL`
- 能不能解锁

这部分逻辑主要在：

- [commander_params.c](/home/cf/PX4_Firmware/src/modules/commander/commander_params.c)
- [state_machine_helper.cpp](/home/cf/PX4_Firmware/src/modules/commander/state_machine_helper.cpp)

所以：

- `mc_*` 和 `eso_*` 的差异，不直接决定会不会触发 `RC loss failsafe`
- 这次问题虽然在 `uam_v5 + ESO` 场景里出现，但触发器其实是 `Commander`

## 4. 为什么 `AUTO.RTL`

这次已经确认：

- `NAV_RCL_ACT = 2`

它的定义在 [commander_params.c](/home/cf/PX4_Firmware/src/modules/commander/commander_params.c)：

- `1` = Hold
- `2` = Return
- `3` = Land
- `5` = Terminate
- `6` = Lockdown

因此：

- `NAV_RCL_ACT = 2`
- 意味着一旦 `RC loss` 触发，默认反应就是 `Return`
- 最终表现为 `AUTO.RTL`

这正好和实际现象一致。

## 5. 触发链条是什么

触发链条如下：

1. ROS 节点向 PX4 发送 Offboard 位置设定点
2. PX4 接受到 setpoint，因此允许尝试进入 `OFFBOARD`
3. 但此时 `Commander` 检测到没有可用 RC / stick 输入
4. 当前参数又没有豁免 `OFFBOARD` 下的 `RC loss`
5. 因此触发 `RC loss failsafe`
6. 由于 `NAV_RCL_ACT = 2`，模式被切到 `AUTO.RTL`

所以 ROS 端才会看到：

- 先打印 `Offboard enabled`
- 下一轮又发现已经不在 `OFFBOARD`
- 然后继续请求 `OFFBOARD`
- 周而复始

## 6. 为什么 `uav_arm_v4` 以前没出现

最重要的原因不是 airframe 名字不同，而是参数库不同。

PX4 SITL 会按 `SYS_AUTOSTART` 加载不同的参数文件：

- `10016_uav_arm_v4` 使用 `eeprom/parameters_10016`
- `10019_uam_v5` 使用 `eeprom/parameters_10019`

这意味着：

- `uav_arm_v4` 和 `uam_v5` 不是共享同一套参数
- 一个机型里改过的参数，不会自动带到另一个机型

因此，`uav_arm_v4` 以前没出问题，最常见的解释是：

- `parameters_10016` 中已经保存过更适合 SITL/offboard 的值
- 而 `parameters_10019` 还保留着另一套更严格的值

## 7. 为什么 `mc_*` 以前没出现

这通常也不是因为 `mc_*` 本身“更正确”，而是因为当时场景不同。

常见原因有四类。

### 7.1 当时不是真正的“纯 Offboard 无 RC”

可能以前运行 `mc_*` 时：

- 有 joystick / manual input
- 或没进入持续的 `OFFBOARD`
- 或模式切换过程不同

这样就不会触发当前这条 `RC loss -> RTL` 链。

### 7.2 当时参数已经被改过

比如以前 `10016` 这套参数里可能已经有：

- `COM_RC_IN_MODE = 1`
- 或 `COM_RCL_EXCEPT` 包含了 `Offboard`

那么同样的无 RC 场景就不会炸出来。

### 7.3 当时跑的是另外一套启动/节点组合

这次排查过程中还确认过一个事实：

- `offboard_commander_node` 在你的 catkin `devel/` 里有旧二进制残留
- 但当前源码树里已经没有这份源文件

所以不同时间运行的节点，很可能不是同一个实现，行为也可能不同。

### 7.4 `mc_*` 与 `eso_*` 只是把问题暴露出来的背景不同

`mc_*` 时可能压根没把你带到“无 RC 的纯 Offboard 测试链路”里；
而这次 `uam_v5 + ESO + 新 airframe + 新参数库`，刚好把这个问题暴露得非常明显。

## 8. SITL 默认值为什么没帮你挡住

POSIX/SITL 的公共启动脚本 [rcS](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/rcS#L135) 里，其实已经给了一个更适合仿真的默认值：

```sh
param set-default COM_RC_IN_MODE 1
```

这里：

- `set-default` 的含义不是“强制覆盖”
- 而是“只有在用户没有保存过这个参数时，才采用这个默认值”

`COM_RC_IN_MODE = 1` 的意思是：

- `Joystick only`
- 关闭传统 RC 输入检查

如果这一默认值真的生效，那么你不应该那么容易触发“无 RC 就 RTL”的问题。

因此这次能暴露出来，恰恰说明：

- 当前 `10019` 使用的保存参数覆盖了 SITL 默认值
- 即 `parameters_10019` 中该参数很可能已经不是 `1`

这也是为什么：

- 同样是 SITL
- 你以前没遇到
- 这次却遇到了

## 9. 这次最终怎么解决

本次临时解决方案是：

```sh
param set COM_RCL_EXCEPT 4
```

### 9.1 这个 `4` 是什么意思

`COM_RCL_EXCEPT` 是位掩码参数。

定义在 [commander_params.c](/home/cf/PX4_Firmware/src/modules/commander/commander_params.c)：

- `bit 0 = 1` Mission
- `bit 1 = 2` Hold
- `bit 2 = 4` Offboard

因此：

- `COM_RCL_EXCEPT = 4`
- 就是“仅在 `OFFBOARD` 模式下忽略 `RC loss`”

### 9.2 为什么一设它就能飞

因为它正好切断了这条错误触发链：

- 原来：无 RC -> `RC loss` -> `NAV_RCL_ACT=2` -> `AUTO.RTL`
- 设置后：在 `OFFBOARD` 下忽略 `RC loss`

因此：

- 模式不再被立刻拉走
- Offboard 位置控制可以持续生效
- 飞机就能正常起飞

## 10. 为什么这不是“永久结论”

这次结论是：

- `uam_v5` 当前的“飞不起来”主要是因为 `Commander` 参数状态

但这不等于：

- `uam_v5` 动力学建模已经完全没问题
- `ESO` 控制效果已经完全验证通过

它只是说明：

- 在真正评估控制效果之前
- 先要把 `OFFBOARD / RC loss / failsafe` 这层模式问题理顺

否则测试到的不是控制性能，而只是模式管理反复把你踢出 `OFFBOARD`。

## 11. 明天上实物前最值得先核对的参数

建议分别在 `10016` 和 `10019` 下面对比：

```sh
param show COM_RC_IN_MODE
param show COM_RCL_EXCEPT
param show COM_RC_OVERRIDE
param show NAV_RCL_ACT
param show COM_OBL_RC_ACT
param show COM_OF_LOSS_T
```

重点看：

- `COM_RC_IN_MODE`
- `COM_RCL_EXCEPT`
- `NAV_RCL_ACT`

因为本次问题最直接就是这三者联动出来的。

## 12. 一句话总结

`uav_arm_v4` 和以前的 `mc_*` 没出现这个现象，不是因为它们天然不会触发，而是因为：

- 当时的参数库不是这套
- 当时的 offboard/RC 条件不是这次这套组合
- SITL 默认值或历史保存值刚好没有把 `RC loss -> RTL` 暴露出来

这次 `uam_v5` 只是把一个原本存在、但之前没被触发的 `Commander` 参数问题显性化了。
