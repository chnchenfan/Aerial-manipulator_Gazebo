# uav_arm_top

`uav_arm_top` 是 UAM V5 exp1/exp4 的顶层任务包。它负责启动 PX4 SITL、Gazebo、MAVROS、UAM V5 机械臂控制器、机械臂状态桥接节点，以及两个 offboard 飞行任务节点。

## 文件结构

```text
uav_arm_top/
├── CMakeLists.txt
│   └── 编译 offboard 任务节点，安装 `uam_v5_arm_joint_state_bridge.py`。
├── package.xml
│   └── 声明 roscpp、rospy、geometry_msgs、sensor_msgs、std_msgs、mavros_msgs、uav_control、arm_controller 等依赖。
├── launch/
│   ├── arm_pid_SITL_Gazebo_uam_v5.launch
│   │   └── exp1/exp4 共同使用的 UAM V5 SITL 基础启动链路。
│   ├── exp1_hover_disturbance_uam_v5.launch
│   │   └── exp1 悬停抗扰实验顶层 launch。
│   ├── exp4_square_tracking_uam_v5.launch
│   │   └── exp4 方形轨迹跟踪实验顶层 launch。
│   ├── arm_pid_SITL_Gazebo.launch
│   │   └── 旧 uav_arm_v4 启动文件；exp1/exp4 不使用。
│   └── circle_offboard_SITL_Gazebo.launch
│       └── 旧圆形轨迹 demo；exp1/exp4 不使用。
├── src/
│   ├── eso_hover_disturbance_offboard_node.cpp
│   │   └── exp1 飞行任务节点，发布固定悬停位置期望值。
│   ├── eso_square_arm_experiment_node.cpp
│   │   └── exp4 飞行任务节点，发布方形轨迹位置期望值。
│   ├── eso_offboard_node.cpp
│   │   └── 旧 offboard demo；exp1/exp4 不使用。
│   ├── eso_offboard_2_5_node.cpp
│   │   └── 旧 2m/5m 高度 demo；exp1/exp4 不使用。
│   ├── eso_square_offboard_node.cpp
│   │   └── 旧方形轨迹 demo；exp1/exp4 不使用。
│   └── eso_circle_offboard_node.cpp
│       └── 旧圆形轨迹 demo；exp1/exp4 不使用。
├── scripts/
│   ├── uam_v5_arm_joint_state_bridge.py
│   │   └── 把 ROS 关节状态转换为 MAVLink `NAMED_VALUE_FLOAT`，供 PX4 ESO 模块读取。
│   ├── BRIDGE_CHAIN_UAM_V5.md
│   │   └── UAM V5 关节状态桥接链路说明。
│   ├── SIM_WORKFLOW_UAV_ARM_V4_UAM_V5.md
│   │   └── 旧仿真流程记录，包含 v4 和 v5。
│   ├── MC_CONTROL_CONFLICT_UAM_V5.md
│   │   └── UAM V5 与 PX4 multicopter 控制模块冲突排查记录。
│   ├── OFFBOARD_RCL_DIFFERENCE_V4_V5.md
│   │   └── v4/v5 offboard 参数差异记录。
│   └── GIT_WORKFLOW_PX4_FIRMWARE_CLEAN.md
│       └── 本仓库工作流记录。
├── data/
│   └── 本地实验输出目录，已加入 `.gitignore`，不提交。
├── auto_tune_data/
│   └── 自动调参 fresh bag、metrics 和 Optuna DB 输出目录，已加入 `.gitignore`，不提交。
├── CIRCLE_TRAJECTORY_GUIDE.md
│   └── 旧圆形轨迹说明；exp1/exp4 不使用。
└── README.md
    └── 当前文件。
```

## Launch 说明

### `launch/arm_pid_SITL_Gazebo_uam_v5.launch`

功能：启动 UAM V5 的基础仿真链路，不单独定义 exp1/exp4 任务。

涉及文件：

- `uav_arm_model/urdf/uam_v5.urdf.xacro`
- `Tools/sitl_gazebo/models/uam_v5/uam_v5.sdf`
- `arm_controller/launch/controller_bringup_uam_v5.launch`
- `arm_controller/config/joint_pid_uam_v5.yaml`
- `scripts/uam_v5_arm_joint_state_bridge.py`
- PX4 上游 `px4/launch/mavros_posix_sitl.launch`

信息流：

```text
arm_pid_SITL_Gazebo_uam_v5.launch
  -> load /robot_description from uam_v5.urdf.xacro
  -> include px4/launch/mavros_posix_sitl.launch with vehicle=uam_v5
  -> Gazebo loads Tools/sitl_gazebo/models/uam_v5/uam_v5.sdf
  -> include arm_controller/controller_bringup_uam_v5.launch
  -> start uam_v5_arm_joint_state_bridge.py
```

### `launch/exp1_hover_disturbance_uam_v5.launch`

功能：启动悬停抗扰实验。该 launch 先 include `arm_pid_SITL_Gazebo_uam_v5.launch`，再启动 exp1 offboard 节点和机械臂扰动节点。

涉及文件：

- `launch/exp1_hover_disturbance_uam_v5.launch`
- `launch/arm_pid_SITL_Gazebo_uam_v5.launch`
- `src/eso_hover_disturbance_offboard_node.cpp`
- `arm_controller/scripts/uam_v5_experiment_motion.py`
- `uav_control/scripts/experiment_data_recorder.py`，需要另一个终端手动运行或由调参脚本启动
- `uav_control/scripts/compute_raw_position_error.py`，用于后处理指标

信息流：

```text
eso_hover_disturbance_offboard_node
  -> /mavros/setpoint_position/local = fixed hover setpoint
  -> /mavros/set_mode, /mavros/cmd/arming
  -> /experiment/arm_motion_enabled after reach/hold condition
uam_v5_experiment_motion.py
  <- /experiment/arm_motion_enabled
  -> /uav_arm/<joint>_position_controller/command
  -> /uav_arm/target_joint_states
uam_v5_arm_joint_state_bridge.py
  <- /uav_arm/joint_states
  -> /mavlink/to NAMED_VALUE_FLOAT
PX4 ESO modules
  <- MAVLink named values and vehicle state
  -> motor/attitude/rate/position control
```

任务参数：

- 飞行任务：起飞后悬停，机械臂开始正弦运动以形成抗扰测试。
- 默认位置期望值：`x=0.0 m, y=0.0 m, z=2.0 m`。
- 默认激活条件：高度达到 `1.9 m`，距离目标小于 `0.20 m`，保持 `1.0 s` 后发布 `/experiment/arm_motion_enabled=True`。
- 机械臂扰动：默认 `0.5 Hz`，由 `uam_v5_experiment_motion.py` 发布。
- 飞行 setpoint 发布频率：`20 Hz`。

### `launch/exp4_square_tracking_uam_v5.launch`

功能：启动方形轨迹跟踪实验。该 launch 先 include `arm_pid_SITL_Gazebo_uam_v5.launch`，再启动 exp4 offboard 节点和机械臂扰动节点。

涉及文件：

- `launch/exp4_square_tracking_uam_v5.launch`
- `launch/arm_pid_SITL_Gazebo_uam_v5.launch`
- `src/eso_square_arm_experiment_node.cpp`
- `arm_controller/scripts/uam_v5_experiment_motion.py`
- `uav_control/scripts/experiment_data_recorder.py`，需要另一个终端手动运行或由调参脚本启动
- `uav_control/scripts/compute_raw_position_error.py`，用于后处理指标

信息流：

```text
eso_square_arm_experiment_node
  -> /mavros/setpoint_position/local = square trajectory setpoint
  -> /mavros/set_mode, /mavros/cmd/arming
  -> /experiment/arm_motion_enabled when first corner is reached
uam_v5_experiment_motion.py
  <- /experiment/arm_motion_enabled
  -> joint position controller commands
PX4 + MAVROS
  <- position setpoints
  -> /mavros/local_position/pose and /mavros/state
experiment_data_recorder.py
  <- pose, setpoint, joint state, enable marker
  -> bag + metadata
```

任务参数：

- 飞行任务：在 `z=2.0 m` 高度跟踪水平正方形路径，同时机械臂做正弦扰动。
- 默认路径：`(0,0,2)` -> `(2,0,2)` -> `(2,2,2)` -> `(0,2,2)` -> `(0,0,2)`。
- 默认边长：`2.0 m`。
- launch 默认路径速度：`0.15 m/s`。
- 节点内部最小速度保护：`path_speed_mps >= 0.05 m/s`。
- 默认角点停留：`3.0 s`。
- 默认平滑段：`smooth_segments=false`，即线性插值。
- 默认激活条件：高度达到 `1.9 m`，距离首个角点小于 `0.20 m`，保持 `1.0 s` 后发布 `/experiment/arm_motion_enabled=True`。
- 机械臂扰动：默认 `0.5 Hz`。
- 飞行 setpoint 发布频率：`20 Hz`。

## 常用运行方式

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch uav_arm_top exp1_hover_disturbance_uam_v5.launch
```

或：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch uav_arm_top exp4_square_tracking_uam_v5.launch
```

另一个终端录制：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun uav_control experiment_data_recorder.py _experiment_name:=exp1_hover_disturbance_uam_v5
```

录制输出默认写入 `ESO_paper_reproduction/src/uav_arm_top/data/<timestamp>/`。该目录是本地实验产物，已加入 `.gitignore`。
