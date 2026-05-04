# arm_controller

`arm_controller` 是 UAM V5 机械臂 ROS 控制包。它不直接决定无人机飞行轨迹，而是为 Gazebo 中的机械臂关节加载 PID 控制器，并在 exp1/exp4 中按顶层任务节点发出的使能信号产生机械臂扰动。

## 文件结构

```text
arm_controller/
├── CMakeLists.txt
│   └── 声明 ROS 依赖，并安装 Python 控制脚本。
├── package.xml
│   └── 声明 controller_manager、joint_state_controller、effort_controllers、sensor_msgs、std_msgs 等运行依赖。
├── config/
│   ├── joint_pid_uam_v5.yaml
│   │   └── UAM V5 的 arm_joint1、arm_joint2、left_hand_joint 位置控制器 PID 参数。
│   ├── joint_pid.yaml
│   │   └── 旧 uav_arm_v4 控制器参数；exp1/exp4 不使用。
│   └── 111.txt
│       └── 本地调试遗留文件；exp1/exp4 不使用。
├── launch/
│   ├── controller_bringup_uam_v5.launch
│   │   └── exp1/exp4 使用的 UAM V5 机械臂控制器启动文件。
│   └── controller_bringup.launch
│       └── 旧 uav_arm_v4 控制器启动文件；exp1/exp4 不使用。
├── scripts/
│   ├── uam_v5_experiment_motion.py
│   │   └── exp1/exp4 的机械臂正弦扰动命令节点。
│   ├── joint_position_commander_uam_v5.py
│   │   └── UAM V5 手动关节命令工具；不由 exp1/exp4 自动启动。
│   ├── arm_zero_hold_logger_uam_v5.py
│   │   └── UAM V5 零位保持与记录工具；不由 exp1/exp4 自动启动。
│   ├── joint_position_commander.py
│   │   └── 旧 uav_arm_v4 手动关节命令工具；exp1/exp4 不使用。
│   └── README_uam_v5_tools.md
│       └── UAM V5 手动工具说明。
└── README.md
    └── 当前文件。
```

## Launch 说明

### `launch/controller_bringup_uam_v5.launch`

功能：在命名空间 `uav_arm` 下加载 `config/joint_pid_uam_v5.yaml`，并通过 `controller_manager/spawner` 启动三个控制器：

- `joint_state_controller`
- `arm_joint1_position_controller`
- `arm_joint2_position_controller`
- `left_hand_joint_position_controller`

涉及文件：

- `launch/controller_bringup_uam_v5.launch`
- `config/joint_pid_uam_v5.yaml`
- Gazebo/URDF 中定义的 `arm_joint1`、`arm_joint2`、`left_hand_joint`
- `scripts/uam_v5_experiment_motion.py` 发布到这些控制器的 command 话题

信息流：

```text
uav_arm_top/launch/arm_pid_SITL_Gazebo_uam_v5.launch
  -> include controller_bringup_uam_v5.launch
  -> rosparam load joint_pid_uam_v5.yaml
  -> controller_manager/spawner
  -> /uav_arm/<joint>_position_controller/command
  -> gazebo_ros_control
  -> /uav_arm/joint_states
```

## exp1/exp4 中的机械臂任务

`scripts/uam_v5_experiment_motion.py` 在 exp1 和 exp4 中由顶层 launch 自动启动。默认参数如下：

- 使能话题：`/experiment/arm_motion_enabled`
- 发布频率：`50 Hz`
- 扰动频率：`0.5 Hz`
- `arm_joint1 = -pi + 0.35 * sin(2*pi*0.5*t)`
- `arm_joint2 = 1.5*pi - 0.10 - 0.35 * sin(2*pi*0.5*t)`
- `left_hand_joint = 0.005 + 0.003 * sin(2*pi*0.5*t)`

使能前节点持续发布中立位；收到 `/experiment/arm_motion_enabled=True` 后开始按正弦轨迹运动。该机械臂运动是 exp1 悬停抗扰和 exp4 方形轨迹跟踪中的外部扰动源。
