# uav_control

## English

### 1. Overview

`uav_control` is the ROS-side utility and analysis package for this repository. It is not the active flight controller that runs inside PX4. Instead, it provides reusable math helpers, trajectory generation logic, and post-processing scripts used around the PX4-based ESO experiments.

### 2. What Was Modified

This package was added to support the modified PX4 control workflow with:

- `ControlUtils`: force-to-attitude/thrust mapping, quaternion-to-Euler conversion, force saturation, and CoM calculation.
- `TrajectoryGenerator`: reference generation for hover, circle, spiral, and attitude-sine test modes.
- plotting and log-analysis scripts for rosbag data and PX4 console logs.

The important project choice is that the main runtime controller now lives in PX4 modules, while this package stays as the ROS-side support layer.

### 3. Key Components

- `include/uav_control/ControlUtils.hpp`
  - `forceToAttitudeThrust`
  - `quaternionToEuler`
  - `saturateForce`
  - `calculateCoM`
- `include/uav_control/TrajectoryGenerator.hpp`
  - `TrajectoryMode`
  - `TrajectoryParameters`
  - `update`
  - `updateAttitude`
- `scripts/plot_result.py`
  - full bag-based experiment plotting
- `scripts/plot_eso_results.py`
  - quick ESO position/disturbance plot
- `scripts/hover_rate_stats.py`
  - PX4 console log statistics for hover/rate behavior
- `scripts/plot_result_classDesign.py`
  - publication-style plotting variant

### 4. Interfaces / Runtime Entry Points

- Built library: `uav_core_lib`
- Installed script entry:
  - `rosrun uav_control plot_eso_results.py <bag>`
- Direct script usage:
  - `python3 scripts/plot_result.py <bag> --start <sec>`
  - `python3 scripts/hover_rate_stats.py --log <px4.log> ...`

Input data used by the scripts:

- `/mavros/local_position/pose`
- `/mavros/setpoint_position/local`
- `/mavros/imu/data`
- `/mavros/debug/named_value_float`
- `/uav_arm/joint_states`
- `/uav_arm/target_joint_states`

### 5. How to Run or Validate

Build the workspace:

```bash
cd /home/cf/PX4_Firmware_clean/ESO_paper_reproduction
catkin_make
source devel/setup.bash
```

Example post-processing:

```bash
python3 src/uav_control/scripts/plot_result.py test_flight.bag --start 15.5
python3 src/uav_control/scripts/plot_eso_results.py eso_test_01.bag
python3 src/uav_control/scripts/hover_rate_stats.py --log /tmp/px4.log --start-sec 20 --end-sec 80
```

Validate this package by checking:

- `catkin_make` builds `uav_core_lib`
- `rosrun uav_control plot_eso_results.py <bag>` starts successfully
- the plotting scripts can find the expected topics or log patterns

### 6. File Map

- `include/uav_control`: reusable control and trajectory headers
- `src`: C++ implementations of utility logic
- `scripts`: data analysis and plotting scripts
- `scripts/result_2m`, `scripts/result_5m`: stored experiment notes

## 中文

### 1. 概述

`uav_control` 是这个仓库的 ROS 侧工具与分析包，不是当前真正运行在 PX4 内部的主飞控控制器。它的作用是给整个实验链路提供数学工具、轨迹生成器，以及 bag/日志后处理脚本。

### 2. 修改了什么

这个包主要为自定义 PX4 ESO 工作流补了三类能力：

- `ControlUtils`：力到姿态/油门映射、四元数转欧拉角、力限幅、质心计算。
- `TrajectoryGenerator`：生成悬停、画圆、螺旋、姿态正弦测试等参考轨迹。
- ROS bag 与 PX4 控制台日志的分析脚本。

这里的关键设计是：主控制闭环放在 PX4 自定义模块里，这个包只保留 ROS 侧的工具和分析职责。

### 3. 关键组成

- `include/uav_control/ControlUtils.hpp`
  - `forceToAttitudeThrust`
  - `quaternionToEuler`
  - `saturateForce`
  - `calculateCoM`
- `include/uav_control/TrajectoryGenerator.hpp`
  - `TrajectoryMode`
  - `TrajectoryParameters`
  - `update`
  - `updateAttitude`
- `scripts/plot_result.py`
  - 完整实验 bag 图形分析
- `scripts/plot_eso_results.py`
  - 快速查看 ESO 位置/扰动结果
- `scripts/hover_rate_stats.py`
  - 从 PX4 控制台日志统计悬停与速率环行为
- `scripts/plot_result_classDesign.py`
  - 偏论文风格的绘图版本

### 4. 接口 / 运行入口

- 编译出的库：`uav_core_lib`
- 安装后的脚本入口：
  - `rosrun uav_control plot_eso_results.py <bag>`
- 直接运行脚本：
  - `python3 scripts/plot_result.py <bag> --start <sec>`
  - `python3 scripts/hover_rate_stats.py --log <px4.log> ...`

这些脚本主要读取以下数据：

- `/mavros/local_position/pose`
- `/mavros/setpoint_position/local`
- `/mavros/imu/data`
- `/mavros/debug/named_value_float`
- `/uav_arm/joint_states`
- `/uav_arm/target_joint_states`

### 5. 如何运行或验证

先编译工作区：

```bash
cd /home/cf/PX4_Firmware_clean/ESO_paper_reproduction
catkin_make
source devel/setup.bash
```

后处理示例：

```bash
python3 src/uav_control/scripts/plot_result.py test_flight.bag --start 15.5
python3 src/uav_control/scripts/plot_eso_results.py eso_test_01.bag
python3 src/uav_control/scripts/hover_rate_stats.py --log /tmp/px4.log --start-sec 20 --end-sec 80
```

验证时重点看：

- `catkin_make` 能编出 `uav_core_lib`
- `rosrun uav_control plot_eso_results.py <bag>` 可以正常启动
- 绘图或统计脚本能找到预期话题和日志模式

### 6. 文件索引

- `include/uav_control`：可复用控制与轨迹头文件
- `src`：C++ 实现
- `scripts`：分析与绘图脚本
- `scripts/result_2m`、`scripts/result_5m`：实验记录
