# ESO 圆形轨迹飞行案例

本文档描述如何使用 `eso_circle_offboard_node` 来实现无人机在2米圆形轨迹上的飞行。

## 概述

`eso_circle_offboard_node` 是基于现有的 `eso_square_offboard_node` 改进的节点，用于控制PX4自驾仪的无人机沿着圆形轨迹飞行。无人机将生成并追踪沿圆周均匀分布的多个航点。

## 主要特性

- **参数化圆形轨迹**：支持自定义圆的半径、中心位置和高度
- **动态航点生成**：自动生成圆周上均匀分布的航点
- **闭环轨迹**：自动闭合航点序列以实现连续飞行
- **航点追踪**：使用距离判断达到航点条件，并自动切换到下一个航点
- **实时日志输出**：输出飞行状态和航点信息

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `circle_radius` | 2.0 | 圆的半径（单位：米） |
| `altitude` | 2.0 | 飞行高度（单位：米） |
| `center_x` | 0.0 | 圆心X坐标 |
| `center_y` | 0.0 | 圆心Y坐标 |
| `reach_tol_m` | 0.2 | 航点到达容差（单位：米） |
| `num_waypoints` | 20 | 圆周上的航点数 |

## 文件结构

```
src/uav_arm_top/
├── src/
│   └── eso_circle_offboard_node.cpp    # 圆形轨迹节点源代码
├── launch/
│   └── circle_offboard_SITL_Gazebo.launch  # Gazebo仿真启动文件
├── CMakeLists.txt                      # 编译配置（已更新）
└── package.xml
```

## 使用方法

### 1. 编译

```bash
cd ~/PX4_Firmware
catkin build uav_arm_top
```

### 2. 运行仿真

#### 方法一：使用Launch文件启动（推荐）

```bash
roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch
```

启动时可以自定义参数：

```bash
roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch \
    circle_radius:=2.5 \
    altitude:=3.0 \
    center_x:=1.0 \
    center_y:=1.0 \
    num_waypoints:=32
```

#### 方法二：单独启动节点

首先启动PX4/Gazebo仿真，然后在新的终端启动节点：

```bash
rosrun uav_arm_top eso_circle_offboard_node \
    _circle_radius:=2.0 \
    _altitude:=2.0 \
    _center_x:=0.0 \
    _center_y:=0.0
```

### 3. 输出日志示例

```
[ INFO] [1234567890.123]: Circle Offboard Node initialized with parameters:
[ INFO] [1234567890.124]:   Radius: 2.00 m
[ INFO] [1234567890.125]:   Altitude: 2.00 m
[ INFO] [1234567890.126]:   Center: (0.00, 0.00)
[ INFO] [1234567890.127]:   Waypoints: 20
[ INFO] [1234567890.128]: Generated 21 waypoints for circular path
[ INFO] [1234567890.200]: Waiting for FCU connection...
[ INFO] [1234567890.500]: Offboard enabled
[ INFO] [1234567890.501]: Vehicle armed
[ INFO] [1234567890.502]: Starting circular trajectory...
[ INFO] [1234567891.000]: Reached waypoint 0, moving to next waypoint
[ INFO] [1234567891.100]: Reached waypoint 1, moving to next waypoint
...
```

## 参数配置示例

### 案例1：标准2米圆形（推荐）

```bash
roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch \
    circle_radius:=2.0 \
    altitude:=2.0 \
    num_waypoints:=20
```

**预期行为**：无人机将在高度2米处，沿着半径2米、中心在原点的圆形轨迹飞行。

### 案例2：大圆形轨迹

```bash
roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch \
    circle_radius:=5.0 \
    altitude:=3.0 \
    num_waypoints:=32
```

**预期行为**：无人机将在高度3米处，沿着半径5米的圆形轨迹飞行，使用32个航点以获得更平滑的轨迹。

### 案例3：偏移中心的圆形

```bash
roslaunch uav_arm_top circle_offboard_SITL_Gazebo.launch \
    circle_radius:=2.0 \
    altitude:=2.0 \
    center_x:=3.0 \
    center_y:=3.0
```

**预期行为**：无人机将绕着位置(3.0, 3.0)的中心点飞行圆形，高度2米。

## 代码实现细节

### 航点生成函数

```cpp
std::vector<Waypoint> generate_circular_path(
    double center_x, double center_y, double altitude,
    double radius, int num_waypoints)
```

该函数使用参数方程生成圆周上的航点：
- x = center_x + radius × cos(angle)
- y = center_y + radius × sin(angle)
- z = altitude（常数）

其中 angle 从 0 到 2π 均匀分布在 num_waypoints 个点上。

### 航点追踪逻辑

1. 计算当前位置到目标航点的3D距离
2. 如果距离小于 `reach_tol_m`，标记为"到达"状态
3. 在到达状态下立即切换到下一个航点
4. 循环遍历所有航点（最后一个航点是起点，实现闭合）

## 与原始正方形节点的对比

| 特性 | 正方形节点 | 圆形节点 |
|------|----------|--------|
| 轨迹形状 | 正方形 | 圆形 |
| 参数化方式 | 边长 | 半径 + 航点数 |
| 航点生成 | 手动定义四个角 | 动态计算 |
| 弹性 | 低 | 高 |
| 轨迹平滑性 | 有折角 | 可调节光滑度 |

## 故障排除

### 问题1：无人机不起飞
- **原因**：FCU（飞控单元）连接失败
- **解决**：检查Gazebo仿真是否正确启动，确保MAVROS连接建立

### 问题2：无人机偏离轨迹
- **原因**：航点容差设置过大或飞行器响应延迟
- **解决**：
  - 减小 `reach_tol_m` 参数（如 0.1）
  - 增加 `num_waypoints` 以获得更多中间航点

### 问题3：轨迹不光滑
- **原因**：航点数量不足
- **解决**：增加 `num_waypoints` 参数（如32或64）

## 扩展建议

1. **动态轨迹修改**：通过ROS服务动态更改圆形参数
2. **多个圆形**：实现图形8形轨迹或其他复杂形状
3. **速度控制**：添加航点间速度设置
4. **时间同步**：实现基于时间而非距离的航点切换
5. **可视化**：集成RViz可视化轨迹

## 相关文件

- **源代码**：[eso_circle_offboard_node.cpp](src/eso_circle_offboard_node.cpp)
- **原始正方形节点**：[eso_square_offboard_node.cpp](src/eso_square_offboard_node.cpp)
- **Launch文件**：[circle_offboard_SITL_Gazebo.launch](launch/circle_offboard_SITL_Gazebo.launch)

## 许可证

本项目遵循与PX4固件相同的许可证。

---

**更新日期**：2026年1月6日
