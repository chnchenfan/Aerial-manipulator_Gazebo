# uav_arm_top 实验入口

本包保留 Exp1 和 Exp4 的 ROS 实验入口。完整仓库结构、PX4 底层 ESO 控制器、Gazebo SDF 模型、数据位置和 MATLAB 绘图流程见仓库根目录 `README.md`。

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

通用 `uam_v5` PX4-SITL + Gazebo + 机械臂控制器 bringup 已移动到：

```bash
roslaunch uav_control arm_pid_SITL_Gazebo_uam_v5.launch
```
