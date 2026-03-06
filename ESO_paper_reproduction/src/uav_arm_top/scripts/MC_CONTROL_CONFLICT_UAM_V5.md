# `mc_*` 抢控制权问题说明与修复记录

## 1. 问题是什么

在 `uam_v5` 切换到 ESO 控制链之后，期望运行的是：

- `eso_att_control`
- `eso_rate_control`
- `eso_pos_control`

而不应该再运行官方多旋翼控制器：

- `mc_att_control`
- `mc_rate_control`
- `mc_pos_control`

但实际联调时发现：

- `eso_*` 显示 `running`
- `mc_*` 也同时显示 `running`

这就是“`mc_*` 抢控制权”。

它的本质不是模块编译冲突，而是：

- 两套控制器同时被启动
- 两套控制器都在向相同或相邻控制链路发布控制量
- 最终导致控制权不确定，ESO 链路不能独占工作

---

## 2. 为什么会发生

根因是 PX4 的启动过程分成两段：

1. 载具 airframe 脚本先执行  
   这里执行的是：
   - [`ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5)

2. 后面公共启动脚本继续执行  
   这里会走：
   - [`ROMFS/px4fmu_common/init.d-posix/rcS`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/rcS)
   - [`ROMFS/px4fmu_common/init.d/rc.vehicle_setup`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.vehicle_setup)
   - [`ROMFS/px4fmu_common/init.d/rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps)

问题就出在第二段。

---

## 3. 详细启动顺序

### 第一步：`10019_uam_v5` 执行

在 [`10019_uam_v5`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5) 里，我们做了这些事：

- `. ${R}etc/init.d/rc.mc_defaults`
- `mc_rate_control stop`
- `mc_att_control stop`
- `mc_pos_control stop`
- `eso_att_control start`
- `eso_rate_control start`
- `eso_pos_control start`
- `arm_joint_bridge start`

看起来像是已经把官方控制器停掉了。

### 第二步：`rcS` 继续跑

`rcS` 在加载完 airframe 以后，不会停止。
它还会继续执行公共逻辑。

关键位置：

- [`rcS`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/rcS)

里面会调用：

- `. ${R}etc/init.d/rc.vehicle_setup`

### 第三步：`rc.vehicle_setup` 再次进入多旋翼公共启动

对于多旋翼，`VEHICLE_TYPE = mc`。

于是 [`rc.vehicle_setup`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.vehicle_setup) 会执行：

- `. ${R}etc/init.d/rc.interface`
- `. ${R}etc/init.d/rc.mc_apps`

### 第四步：`rc.mc_apps` 又把 `mc_*` 启起来

在 [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps) 里，原先逻辑是：

```sh
if ! param compare SYS_AUTOSTART 10016
then
    mc_rate_control start
    mc_att_control start
    mc_hover_thrust_estimator start
    flight_mode_manager start
    mc_pos_control start
fi
```

它只排除了 `SYS_AUTOSTART = 10016` 的情况。

但 `uam_v5` 的 airframe 是：

- `SYS_AUTOSTART = 10019`

所以对 `10019` 而言，这个判断条件仍然成立，`mc_*` 被再次启动。

这就是“为什么明明在 airframe 里 stop 了，后面还是 running”。

---

## 4. 原先这段判断是什么意思

原始代码：

```sh
if ! param compare SYS_AUTOSTART 10016
then
    ...
fi
```

它的意思是：

- 如果当前机型不是 `10016`
- 就启动官方多旋翼控制器 `mc_*`

这里的设计背景是：

- `10016_uav_arm_v4` 这套机型已经是特殊机型
- 它使用自定义 ESO 控制器
- 所以公共脚本不应该再给它启动官方 `mc_*`

也就是说，原作者已经在用 `10016` 作为“特殊机型例外”。

只是当时还没有把 `10019_uam_v5` 也加入这个例外。

---

## 5. 这里面几个数字分别是什么意思

### `10016`

含义：

- `uav_arm_v4` 的 airframe autostart ID

对应文件：

- [`ROMFS/px4fmu_common/init.d-posix/airframes/10016_uav_arm_v4`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/10016_uav_arm_v4)

用途：

- PX4 启动时会把 `SYS_AUTOSTART` 设成这个数字
- 公共启动脚本用它判断当前启动的是不是 `uav_arm_v4`

### `10019`

含义：

- `uam_v5` 的 airframe autostart ID

对应文件：

- [`ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5)

用途：

- PX4 启动时会把 `SYS_AUTOSTART` 设成这个数字
- 应该被视为与 `10016` 类似的“特殊控制链机型”

### `SYS_AUTOSTART`

含义：

- PX4 当前启动机型的编号

它不是“控制器参数”，而是“当前机型 profile 的选择参数”。

比如：

- `10016` 表示 `uav_arm_v4`
- `10019` 表示 `uam_v5`

可以用这个参数判断：

- 当前实际加载的是哪个 airframe
- 公共脚本是否会走某个机型分支

检查命令：

```text
param show SYS_AUTOSTART
```

### `MAV_TYPE = 2`

这个数字也出现在 airframe 里。

含义：

- MAVLink 标准里的 `MAV_TYPE_QUADROTOR`

作用：

- 告诉上层系统这是四旋翼机型

这个数字和控制器抢占问题没有直接关系，但 airframe 初始化里通常都会设。

### `ESO_ARM_MODEL = 0`

含义：

- 运行时模型选择：`uav_arm_v4`

### `ESO_ARM_MODEL = 1`

含义：

- 运行时模型选择：`uam_v5`

这个参数负责：

- 控制 ESO 使用哪套动态模型 profile

它和 `mc_*` 抢控制权不是同一件事，但都在 airframe 启动里被设置。

---

## 6. 当时做了什么修改

最终修改的文件是：

- [`ROMFS/px4fmu_common/init.d/rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps)

修改前：

```sh
if ! param compare SYS_AUTOSTART 10016
then
    ...
fi
```

修改后：

```sh
if ! param compare SYS_AUTOSTART 10016 && ! param compare SYS_AUTOSTART 10019
then
    ...
fi
```

这段新增的内容：

- `&& ! param compare SYS_AUTOSTART 10019`

意思是：

- 如果当前机型既不是 `10016`
- 也不是 `10019`
- 才允许启动官方 `mc_*`

换句话说：

- `10016` 和 `10019` 都被划入“特殊控制链机型”
- 它们不再由公共脚本自动启动官方 `mc_*`

---

## 7. 为什么不是只在 `10019_uam_v5` 里 stop 一次就完了

因为 `10019_uam_v5` 只是启动早期的一步。

后续还会执行：

- `rc.vehicle_setup`
- `rc.mc_apps`

所以如果只在 airframe 里写：

- `mc_rate_control stop`
- `mc_att_control stop`
- `mc_pos_control stop`

那么这些模块可能在后续又被公共脚本重启。

因此真正可靠的解决方式不是“早停一次”，而是：

- 从公共启动逻辑里把 `10019` 排除出去

也就是要改：

- [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps)

---

## 8. 如何判断问题是否已经解决

### 检查命令

在 PX4 shell 里执行：

```text
mc_rate_control status
mc_att_control status
mc_pos_control status
eso_att_control status
eso_rate_control status
eso_pos_control status
```

### 解决前的现象

- `mc_*` 是 `running`
- `eso_*` 也是 `running`

### 解决后的现象

- `mc_rate_control` -> `not running`
- `mc_att_control` -> `not running`
- `mc_pos_control` -> `not running`
- `eso_att_control` -> `running`
- `eso_rate_control` -> `running`
- `eso_pos_control` -> `running`

你后面的实际结果已经证明修复成功：

```text
mc_rate_control status -> not running
mc_att_control status -> not running
mc_pos_control status -> not running
eso_att_control status -> running
eso_rate_control status -> running
eso_pos_control status -> running
```

---

## 9. 这次修改涉及的参数与数字总结

### 与“抢控制权”直接相关

- `SYS_AUTOSTART`
  - 含义：当前 airframe 机型编号
  - 关键数字：
    - `10016 = uav_arm_v4`
    - `10019 = uam_v5`

### 与 airframe 初始化相关

- `MAV_TYPE`
  - `2 = MAV_TYPE_QUADROTOR`

### 与 ESO 模型选择相关

- `ESO_ARM_MODEL`
  - `0 = uav_arm_v4`
  - `1 = uam_v5`

---

## 10. 最终结论

`mc_*` 抢控制权的真正原因不是：

- ESO 模块写错了
- airframe 没 stop 干净
- PX4 同时编译了两套控制器

真正原因是：

- `10019_uam_v5` 在 airframe 阶段虽然停掉了 `mc_*`
- 但后续公共脚本 [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps) 又根据 `SYS_AUTOSTART` 重新把官方控制器拉起来了
- 原逻辑只豁免了 `10016`，没有豁免 `10019`

最终解决方法是：

- 在 [`rc.mc_apps`](/home/cf/PX4_Firmware/ROMFS/px4fmu_common/init.d/rc.mc_apps) 中，把 `10019` 加入和 `10016` 同等级的例外机型

这样 `uam_v5` 才能真正独占 ESO 控制链，不再被官方 `mc_*` 抢控制权。

