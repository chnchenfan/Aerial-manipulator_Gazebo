# `uav_arm_v4` / `uam_v5` 仿真启动说明

## 1. 适用范围

这份文档说明两套模型在 `PX4_Firmware_clean` 里的标准仿真流程：

- `uav_arm_v4`
- `uam_v5`

覆盖内容：

- PX4 编译
- catkin 工作区编译
- `roslaunch` 启动
- `rosrun` 常用节点启动
- 常见检查命令

预留内容：

- 实物联调占位

默认工作仓库：

```bash
/home/cf/PX4_Firmware_clean
```

## 2. 模型和入口文件在哪里

### 2.1 Gazebo 模型

- `Tools/sitl_gazebo/models/uav_arm_v4/uav_arm_v4.sdf`
- `Tools/sitl_gazebo/models/uam_v5/uam_v5.sdf`

### 2.2 ROS 启动文件

- `ESO_paper_reproduction/src/uav_arm_top/launch/arm_pid_SITL_Gazebo.launch`
- `ESO_paper_reproduction/src/uav_arm_top/launch/arm_pid_SITL_Gazebo_uam_v5.launch`

### 2.3 PX4 airframe

- `ROMFS/px4fmu_common/init.d-posix/airframes/10016_uav_arm_v4`
- `ROMFS/px4fmu_common/init.d-posix/airframes/10019_uam_v5`

其中：

- `10016_uav_arm_v4` 会启动 `eso_att_control`、`eso_rate_control`、`eso_pos_control`
- `10019_uam_v5` 除了上面三个 ESO 模块，还会启动 `arm_joint_bridge`

## 3. 第一次使用或代码改动后的标准编译流程

### 3.1 PX4 固件编译

在仓库根目录：

```bash
cd /home/cf/PX4_Firmware_clean
```

如果你刚切了分支、改了很多 PX4 代码，或者想做一次干净编译：

```bash
make clean
```

编译 `uav_arm_v4`：

```bash
make px4_sitl gazebo_uav_arm_v4
```

编译 `uam_v5`：

```bash
make px4_sitl gazebo_uam_v5
```

说明：

- 这里的 `make px4_sitl gazebo_<model>` 主要作用是编译并生成对应 SITL 目标
- 日常启动仿真时，真正拉起 PX4/Gazebo 的通常是后面的 `roslaunch`

### 3.2 ROS catkin 工作区编译

```bash
cd /home/cf/PX4_Firmware_clean/ESO_paper_reproduction
catkin_make
```

### 3.3 每个新终端的推荐环境准备

先在仓库根目录生成一次 SITL 构建目录：

```bash
cd /home/cf/PX4_Firmware_clean
DONT_RUN=1 make px4_sitl_default gazebo
```

之后每开一个新终端，直接执行：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
```

这个脚本会自动完成：

- `source ESO_paper_reproduction/devel/setup.bash`
- `source Tools/setup_gazebo.bash`
- 补全 `ROS_PACKAGE_PATH`

建议先验证：

```bash
rospack find uav_arm_top
rospack find mavlink_sitl_gazebo
```

如果这两个都能找到，再执行 `roslaunch` / `rosrun`。

## 4. `uav_arm_v4` 仿真启动

### 4.1 启动整套仿真

终端 1：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch
```

这条 `launch` 会启动：

- PX4 SITL
- Gazebo
- MAVROS
- `uav_arm_v4` 机械臂控制器

### 4.2 常用飞行测试节点

终端 2：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun uav_arm_top eso_offboard_node
```

其他可选轨迹节点：

```bash
rosrun uav_arm_top eso_square_offboard_node
rosrun uav_arm_top eso_circle_offboard_node
```

### 4.3 常用机械臂测试节点

终端 3：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun arm_controller joint_position_commander.py
```

## 5. `uam_v5` 仿真启动

### 5.1 启动整套仿真

终端 1：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch
```

这条 `launch` 会启动：

- PX4 SITL
- Gazebo
- MAVROS
- `uam_v5` 机械臂控制器
- `uam_v5_arm_joint_state_bridge.py`

### 5.2 常用飞行测试节点

终端 2：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun uav_arm_top eso_offboard_2_5_node
```

补充：

- `eso_offboard_2_5_node` 默认先保持 `2 m`，再升到 `5 m`
- `eso_offboard_node` 默认目标高度是 `0.5 m`

如果只想做固定点悬停，也可以运行：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun uav_arm_top eso_offboard_node
```

### 5.3 常用机械臂测试节点

终端 3：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun arm_controller joint_position_commander_uam_v5.py
```

固定零位并记录日志：

```bash
source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh
rosrun arm_controller arm_zero_hold_logger_uam_v5.py
```

## 6. `uam_v5` 桥接链路的最小检查项

启动 `uam_v5` 后，建议最少检查这几项。

PX4 shell：

```bash
param show ESO_ARM_MODEL
eso_att_control status
eso_rate_control status
eso_pos_control status
mc_att_control status
mc_rate_control status
mc_pos_control status
arm_joint_bridge status
listener arm_joint_states
```

期望：

- `ESO_ARM_MODEL = 1`
- `eso_*` 处于 `running`
- `arm_joint_bridge` 处于 `running`
- `arm_joint_states.valid = True`

ROS 侧：

```bash
rostopic echo /uav_arm/joint_states
```

## 7. Offboard 仿真前的参数提醒

纯 SITL、没有遥控器时，如果直接跑 Offboard，可能触发 `RC loss failsafe`。

已知可用的仿真参数处理方式：

```bash
param set COM_RCL_EXCEPT 4
```

含义：

- 在 `OFFBOARD` 模式下忽略 `RC loss`

如果看到：

- `Failsafe mode activated`
- `/mavros/state` 变成 `AUTO.RTL`

优先先检查这一项。

## 8. 常用排查命令

### 8.1 PX4 侧

```bash
listener vehicle_status
listener vehicle_command_ack
listener debug_key_value
listener arm_joint_states
```

### 8.2 ROS 侧

```bash
rostopic list
rostopic echo /mavros/state
rostopic echo /uav_arm/joint_states
rosnode list
```

## 9. 推荐的终端分工

### 9.1 `uav_arm_v4`

- 终端 1：`roslaunch uav_arm_top arm_pid_SITL_Gazebo.launch`
- 终端 2：`rosrun uav_arm_top eso_offboard_node`
- 终端 3：`rosrun arm_controller joint_position_commander.py`

### 9.2 `uam_v5`

- 终端 1：`roslaunch uav_arm_top arm_pid_SITL_Gazebo_uam_v5.launch`
- 终端 2：`rosrun uav_arm_top eso_offboard_2_5_node`
- 终端 3：`rosrun arm_controller joint_position_commander_uam_v5.py`

## 10. 实物联调占位

这一节先留占位，后续实物联调时补充。

注意：

- `setup_px4_sitl_ros_env.sh` 是仿真专用脚本
- 它会加载 Gazebo 和 SITL 相关环境
- 实物联调时不要直接复用这条脚本
- 实物后续应使用单独的 `MAVROS + catkin` 环境准备流程

后续计划补充：

- 遥控器接管参数
- 实物 `OFFBOARD`/`RC loss` 安全参数
- 实物解锁前检查表
- 实物机械臂零位检查
- 实物日志采集流程
- 实物急停与接管流程
