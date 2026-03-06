# `uam_v5` Arm Tools

这份说明对应两个脚本：

- `arm_zero_hold_logger_uam_v5.py`
- `joint_position_commander_uam_v5.py`

它们都依赖同一套仿真已经启动：

```bash
cd /home/cf/PX4_Firmware/ESO_paper_reproduction
source devel/setup.bash
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
```

建议在另一个终端里再执行一次：

```bash
cd /home/cf/PX4_Firmware/ESO_paper_reproduction
source devel/setup.bash
```

## 1. `arm_zero_hold_logger_uam_v5.py`

### 用途
- 持续把 `arm_joint1`、`arm_joint2`、`left_hand_joint` 保持在目标值
- 默认目标值都是 `0`
- 记录 `joint_states`
- 打印日志
- 保存 `csv` 和 `png`

### 运行方式

默认运行 20 秒：

```bash
rosrun arm_controller arm_zero_hold_logger_uam_v5.py
```

运行 40 秒：

```bash
rosrun arm_controller arm_zero_hold_logger_uam_v5.py _duration:=40
```

### 可调参数

- `~namespace`
  - 默认值：`uav_arm`
  - 作用：控制器和 `joint_states` 的 ROS namespace

- `~joints`
  - 默认值：`[arm_joint1, arm_joint2, left_hand_joint]`
  - 作用：要控制和记录的关节列表

- `~targets`
  - 默认值：`[0.0, 0.0, 0.0]`
  - 作用：每个关节的目标值

- `~duration`
  - 默认值：`20.0`
  - 作用：记录总时长，单位秒

- `~command_rate_hz`
  - 默认值：`50.0`
  - 作用：发命令和采样的频率

- `~log_interval`
  - 默认值：`1.0`
  - 作用：终端打印一次日志的时间间隔，单位秒

- `~output_dir`
  - 默认值：`~/.ros/uam_v5_arm_logs`
  - 作用：保存 `csv/png` 的目录

### 输出文件

默认输出目录：

```bash
~/.ros/uam_v5_arm_logs
```

文件名格式：

- `zero_hold_YYYYMMDD_HHMMSS.csv`
- `zero_hold_YYYYMMDD_HHMMSS.png`

### 修改输出目录

例如改到桌面：

```bash
rosrun arm_controller arm_zero_hold_logger_uam_v5.py \
  _output_dir:=/home/cf/Desktop/uam_v5_logs
```

### 示例

运行 40 秒，0.5 秒打印一次日志，并修改输出目录：

```bash
rosrun arm_controller arm_zero_hold_logger_uam_v5.py \
  _duration:=40 \
  _log_interval:=0.5 \
  _output_dir:=/home/cf/Desktop/uam_v5_logs
```

## 2. `joint_position_commander_uam_v5.py`

### 用途
- 按预设动作序列驱动机械臂
- 动作过程中同步记录 `joint_states`
- 打印日志
- 保存 `csv` 和 `png`

当前内置动作序列是：

1. 回到零位
2. 移动到 `{arm_joint1: 0.35, arm_joint2: -0.45, left_hand_joint: 0.008}`
3. 移动到 `{arm_joint1: -0.35, arm_joint2: 0.25, left_hand_joint: 0.002}`
4. 循环执行

### 运行方式

```bash
rosrun arm_controller joint_position_commander_uam_v5.py
```

### 可调参数

- `~namespace`
  - 默认值：`uav_arm`
  - 作用：控制器和 `joint_states` 的 ROS namespace

- `~hold_time`
  - 默认值：`3.0`
  - 作用：每个目标点保持的时间，单位秒

- `~move_duration`
  - 默认值：`3.0`
  - 作用：从当前点插值移动到下一个目标点的时间，单位秒

- `~command_rate_hz`
  - 默认值：`50.0`
  - 作用：发命令和记录的频率

- `~log_interval`
  - 默认值：`1.0`
  - 作用：终端打印一次日志的时间间隔，单位秒

- `~output_dir`
  - 默认值：`~/.ros/uam_v5_arm_logs`
  - 作用：保存 `csv/png` 的目录

- `~plot_title`
  - 默认值：`uam_v5 Joint Commander Response`
  - 作用：输出图片标题

### 输出文件

默认输出目录：

```bash
~/.ros/uam_v5_arm_logs
```

文件名格式：

- `joint_commander_YYYYMMDD_HHMMSS.csv`
- `joint_commander_YYYYMMDD_HHMMSS.png`

### 修改输出目录

例如：

```bash
rosrun arm_controller joint_position_commander_uam_v5.py \
  _output_dir:=/home/cf/Desktop/uam_v5_logs
```

### 示例

缩短移动时间，延长保持时间，并修改输出目录：

```bash
rosrun arm_controller joint_position_commander_uam_v5.py \
  _move_duration:=2.0 \
  _hold_time:=4.0 \
  _log_interval:=0.5 \
  _output_dir:=/home/cf/Desktop/uam_v5_logs
```

## 3. 常见说明

### 图里为什么有速度

虽然这两个脚本本质上是位置目标测试，但图里仍然记录速度，因为：

- 位置接近目标，不代表已经稳定停住
- `D` 项和速度直接相关
- 调 PID 时，速度曲线能直接看出抖动和欠阻尼

### 如果没有生成图

常见原因：

- `matplotlib` 没有成功导入
- 脚本提前被中断
- 输出目录没有写权限

脚本即使不出图，通常也会优先保存 `csv`。

### 如果想看输出目录下有什么文件

```bash
ls ~/.ros/uam_v5_arm_logs
```

或：

```bash
ls /home/cf/Desktop/uam_v5_logs
```
