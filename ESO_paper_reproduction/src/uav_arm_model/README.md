# uav_arm_model

`uav_arm_model` 提供 UAM V5 在 ROS/Gazebo 中使用的机器人描述文件和可视化网格。exp1/exp4 启动时，顶层 launch 会把 `uam_v5.urdf.xacro` 加载到 `/robot_description`，供 `gazebo_ros_control` 和关节控制器使用。

## 文件结构

```text
uav_arm_model/
├── CMakeLists.txt
│   └── catkin 包定义；当前包主要提供资源文件。
├── package.xml
│   └── 声明 xacro、urdf、robot_state_publisher 等模型相关依赖。
├── urdf/
│   ├── uam_v5.urdf.xacro
│   │   └── exp1/exp4 使用的 UAM V5 ROS 机器人描述，定义 base、机械臂、关节和 ros_control 接口。
│   └── uav_arm_v4.urdf.xacro
│       └── 旧 uav_arm_v4 模型；exp1/exp4 不使用。
├── meshes/
│   ├── uam_v5/
│   │   ├── base_link.STL
│   │   ├── arm_fix_link.STL
│   │   ├── arm_link1.STL
│   │   ├── arm_link2.STL
│   │   ├── left_hand_link.STL
│   │   ├── right_hand_link.STL
│   │   ├── rotor_0.STL
│   │   ├── rotor_1.STL
│   │   ├── rotor_2.STL
│   │   ├── rotor_3.STL
│   │   ├── camera_link.STL
│   │   └── camera_bracket_link.STL
│   │       └── UAM V5 专用可视化/碰撞网格。
│   ├── base_link.STL
│   ├── arm_link1.STL
│   ├── arm_link2.STL
│   ├── arm_link3.STL
│   ├── arm_link4.STL
│   ├── left_gripper_link.STL
│   ├── right_gripper_link.STL
│   ├── rotor_0.STL
│   ├── rotor_1.STL
│   ├── rotor_2.STL
│   └── rotor_3.STL
│       └── 旧模型网格；exp1/exp4 不直接使用。
├── scripts/
│   └── calc_relative.py
│       └── 本地几何计算辅助脚本；exp1/exp4 不自动调用。
├── rviz/
│   └── uavBase_arm.rviz
│       └── RViz 可视化配置；exp1/exp4 不自动启动。
└── README.md
    └── 当前文件。
```

## exp1/exp4 模型链路

涉及文件：

- `urdf/uam_v5.urdf.xacro`
- `meshes/uam_v5/*.STL`
- `arm_controller/config/joint_pid_uam_v5.yaml`
- `uav_control/launch/arm_pid_SITL_Gazebo_uam_v5.launch`
- `Tools/sitl_gazebo/models/uam_v5/uam_v5.sdf`

信息流：

```text
arm_pid_SITL_Gazebo_uam_v5.launch
  -> xacro uam_v5.urdf.xacro
  -> /robot_description
  -> gazebo_ros_control reads joints/transmissions
  -> arm_controller spawns position controllers
  -> /uav_arm/joint_states
  -> uam_v5_arm_joint_state_bridge.py
  -> MAVLink NAMED_VALUE_FLOAT
  -> PX4 ESO controller modules
```

`uam_v5.urdf.xacro` 描述 ROS 侧机械臂控制接口；`Tools/sitl_gazebo/models/uam_v5/uam_v5.sdf` 描述 Gazebo/PX4 侧飞行器动力学、传感器和插件。两者需要保持关节名称、坐标定义和质量/惯量假设一致，否则 exp1/exp4 中的机械臂扰动和 PX4 ESO 补偿会不一致。
