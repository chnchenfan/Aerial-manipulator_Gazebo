# eso_common

## English

### 1. Overview

`eso_common` is the shared model-profile layer for the custom PX4 ESO stack. It defines the platform-dependent mass, center-of-mass, and inertia presets used by the other ESO modules.

### 2. What Was Modified

This module was added to avoid hard-coding one vehicle model into every controller. Instead, the controllers resolve a model profile and load a consistent set of dynamic parameters for:

- `uav_arm_v4`
- `uam_v5`

### 3. Key Components

- `ESOModelProfile.hpp`
  - `ModelProfile`
  - `DynamicsProfile`
  - `kUavArmV4Profile`
  - `kUamV5Profile`
  - `getDynamicsProfile`
  - `resolveModelProfile`

### 4. Interfaces / Launch or Runtime Entry Points

There is no standalone runtime module here. It is included by:

- `eso_pos_control`
- `eso_att_control`
- `eso_rate_control`

The main selector is the PX4 parameter:

- `ESO_ARM_MODEL`

Mapping:

- `0` -> `uav_arm_v4`
- `1` -> `uam_v5`

### 5. How to Run or Validate

Validation is indirect:

- start SITL with airframe `10016_uav_arm_v4` or `10019_uam_v5`,
- check that the ESO modules pick the correct model-specific mass/CoM/inertia behavior,
- confirm both models run without manually editing controller source constants.

### 6. File Map

- `ESOModelProfile.hpp`: all shared model-profile definitions

## 中文

### 1. 概述

`eso_common` 是自定义 PX4 ESO 控制栈的共享模型层，用来统一管理不同平台对应的质量、质心和惯量预设。

### 2. 修改了什么

这个模块的意义在于，不再把某一套飞行器参数硬编码到每个控制器里，而是通过统一的模型配置接口给以下两套平台提供动态参数：

- `uav_arm_v4`
- `uam_v5`

### 3. 关键组成

- `ESOModelProfile.hpp`
  - `ModelProfile`
  - `DynamicsProfile`
  - `kUavArmV4Profile`
  - `kUamV5Profile`
  - `getDynamicsProfile`
  - `resolveModelProfile`

### 4. 接口 / 运行入口

这个目录本身不是独立运行模块，而是被以下控制器直接包含：

- `eso_pos_control`
- `eso_att_control`
- `eso_rate_control`

主要选择参数是：

- `ESO_ARM_MODEL`

对应关系：

- `0` -> `uav_arm_v4`
- `1` -> `uam_v5`

### 5. 如何运行或验证

这个模块的验证方式是间接的：

- 用 `10016_uav_arm_v4` 或 `10019_uam_v5` 启动 SITL，
- 检查 ESO 模块是否切换到了对应模型的质量、质心和惯量行为，
- 确认不用手工改控制源码常数也能在两种模型上运行。

### 6. 文件索引

- `ESOModelProfile.hpp`：全部共享模型配置定义
