# 科研风格绘图脚本使用说明

## 概述

`plot_result_classDesign.py` 是一个专为学术论文设计的科研风格绘图脚本，使用 matplotlib 和 numpy 库绘制符合学术规范的高质量图表。

## 功能特性

### 1. 全局样式设置

- **字体**: 使用 Times New Roman 衬线体，符合学术论文规范
- **字号**: 适中字号（11pt），适合论文排版
- **LaTeX 支持**:
  - 支持 LaTeX 渲染（需要系统安装 LaTeX）
  - 默认使用 mathtext 渲染数学公式（无需 LaTeX 环境）
- **网格**: 所有子图均开启网格，增强可读性
- **线条样式**: 专业的线条宽度和样式设置

### 2. 绘制的图表

#### 图 1: 无人机位置跟踪 (Position Tracking)

- **布局**: 3 行 1 列，共享 X 轴
- **内容**: 分别绘制 x, y, z 三个轴的位置轨迹
- **曲线定义**:
  - **Desired value (期望值)**: 蓝色实线 (`color='blue'`, `linestyle='-'`)
  - **Actual value (实际值)**: 品红色点划线 (`color='magenta'`, `linestyle='-.'`)
- **标签**:
  - Y 轴标签: $x$ (m), $y$ (m), $z$ (m)
  - X 轴标签: $t$ (s)
- **图例**: 放置在图形最顶部外部，横向排列

#### 图 2: 机械臂关节跟踪 (Manipulator Joint Tracking)

- **布局**: n 行 1 列（n = 关节数量）
- **内容**: 绘制机械臂各关节角度的轨迹
- **曲线样式**: 与图 1 保持一致
  - 蓝色实线: 期望关节角度
  - 品红色点划线: 实际关节角度
- **标签**:
  - Y 轴标签: $\theta_1$ (rad), $\theta_2$ (rad), $\theta_3$ (rad), ...
  - X 轴标签: $t$ (s)
- **图例**: 同图 1，置于顶部外部

#### 图 3: 位置跟踪误差 (Position Error)

- **数据逻辑**: 根据图 1 的数据计算欧几里得范数误差
  $$e(t) = ||\mathbf{p} - \mathbf{p}_d|| = \sqrt{(x-x_d)^2 + (y-y_d)^2 + (z-z_d)^2}$$
- **布局**: 单张图 (1 行 1 列)
- **曲线样式**: 红色实线
- **标签**:
  - Y 轴标签: $e(t)$ (m)
  - X 轴标签: $t$ (s)
- **标题**: "Position Tracking Error"

## 数据来源

本脚本使用与 `plot_result.py` 相同的数据来源，从 ROS bag 文件读取以下话题：

### 无人机数据

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 实际位置和姿态 |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 目标位置 |

### 机械臂数据

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/uav_arm/joint_states` | `sensor_msgs/JointState` | 实际关节状态 |
| `/uav_arm/target_joint_states` | `sensor_msgs/JointState` | 目标关节状态 |

## 使用方法

### 基本用法

```bash
# 绘制 bag 文件中的数据
python plot_result_classDesign.py experiment.bag

# 从指定时间开始绘制（跳过前 5 秒）
python plot_result_classDesign.py experiment.bag --start 5.0

# 保存图表为 PDF 文件
python plot_result_classDesign.py experiment.bag --save
```

### 命令行参数

- `bagfile`: (必需) 输入的 ROS bag 文件路径
- `--start`: (可选) 相对于 bag 开始的起始时间（秒），默认 0.0
- `--save`: (可选) 保存图表为 PDF 文件而非显示

### 输出文件

使用 `--save` 参数时，脚本会在当前目录生成以下 PDF 文件：

- `figure1_position_tracking.pdf` - 位置跟踪图
- `figure2_manipulator_tracking.pdf` - 机械臂关节跟踪图
- `figure3_position_error.pdf` - 位置误差图

## 如何替换为自己的实验数据

如果您想使用自己的实验数据而非从 bag 文件读取，请按以下步骤操作：

### 步骤 1: 准备数据

创建一个包含以下结构的 `data` 字典：

```python
data = {
    # 位置数据 (Position data)
    't_pos': np.array([0.0, 0.1, 0.2, ...]),      # 时间戳 (s)
    'pos_x': np.array([0.0, 0.1, 0.2, ...]),      # 实际 X 位置 (m)
    'pos_y': np.array([0.0, 0.0, 0.0, ...]),      # 实际 Y 位置 (m)
    'pos_z': np.array([5.0, 5.1, 5.2, ...]),      # 实际 Z 位置 (m)

    # 目标位置数据 (Desired position data)
    't_pos_sp': np.array([0.0, 0.1, 0.2, ...]),   # 时间戳 (s)
    'pos_sp_x': np.array([0.0, 0.0, 0.0, ...]),   # 期望 X 位置 (m)
    'pos_sp_y': np.array([0.0, 0.0, 0.0, ...]),   # 期望 Y 位置 (m)
    'pos_sp_z': np.array([5.0, 5.0, 5.0, ...]),   # 期望 Z 位置 (m)

    # 机械臂数据 (Manipulator data)
    't_arm': np.array([0.0, 0.1, 0.2, ...]),      # 时间戳 (s)
    'arm_actual': {                                # 实际关节角度 (rad)
        'joint1': np.array([0.0, 0.1, 0.2, ...]),
        'joint2': np.array([0.0, 0.1, 0.2, ...]),
        'joint3': np.array([0.0, 0.1, 0.2, ...]),
        # ... 更多关节
    },
    'arm_target': {                                # 期望关节角度 (rad)
        'joint1': np.array([0.0, 0.0, 0.0, ...]),
        'joint1_t': np.array([0.0, 0.1, 0.2, ...]),  # 对应时间戳
        'joint2': np.array([0.0, 0.0, 0.0, ...]),
        'joint2_t': np.array([0.0, 0.1, 0.2, ...]),
        # ... 更多关节
    }
}
```

### 步骤 2: 修改代码

在 `main()` 函数中，找到以下代码段：

```python
# 从 bag 文件加载数据
data = load_data_from_bag(args.bagfile, args.start)
```

将其注释掉，并替换为您自己的数据加载代码：

```python
# 从 bag 文件加载数据
# data = load_data_from_bag(args.bagfile, args.start)

# 使用自己的数据
data = {
    't_pos': your_time_array,
    'pos_x': your_x_array,
    'pos_y': your_y_array,
    'pos_z': your_z_array,
    # ... 其他数据
}
```

### 步骤 3: 数据替换位置标注

脚本中已在关键位置添加了醒目的注释，标明数据替换点：

```python
# ========================================================================
# 数据准备 - 如需替换为您的数据，请修改此处
# Data preparation - Replace here with your own data if needed
# ========================================================================
```

您可以在这些位置直接替换数据数组。

## 依赖库

```bash
pip install numpy matplotlib rosbag
```

- `numpy`: 数值计算和数组操作
- `matplotlib`: 绘图库
- `rosbag`: ROS bag 文件读取（如使用自己的数据可不安装）

## LaTeX 渲染配置

### 方法 1: 使用系统 LaTeX（推荐用于论文）

如果您的系统已安装 LaTeX，可以启用完整的 LaTeX 渲染以获得最佳效果：

1. 在脚本中找到以下代码：
   ```python
   # plt.rcParams['text.usetex'] = True
   # plt.rcParams['text.latex.preamble'] = r'\usepackage{amsmath}'
   ```

2. 取消注释：
   ```python
   plt.rcParams['text.usetex'] = True
   plt.rcParams['text.latex.preamble'] = r'\usepackage{amsmath}'
   ```

### 方法 2: 使用 mathtext（默认，无需 LaTeX）

脚本默认使用 matplotlib 的 mathtext 渲染数学公式，无需安装 LaTeX，效果已接近 Times New Roman 风格。

## 注意事项

1. **时间对齐**: 脚本会自动对不同话题的时间戳进行插值对齐
2. **数据完整性**: 如果某些数据缺失（如机械臂数据），相应的图表会被跳过
3. **图表质量**: 使用 `--save` 参数保存时，图表分辨率为 300 DPI，适合论文发表
4. **字体问题**: 如果 Times New Roman 字体未安装，matplotlib 会自动使用替代字体

## 示例输出

运行脚本后，您将看到三张独立的图表窗口（或保存为 PDF 文件）：

1. **图 1**: 三个子图分别显示 x, y, z 轴的位置跟踪效果
2. **图 2**: n 个子图分别显示 n 个关节的角度跟踪效果
3. **图 3**: 单张图显示综合位置误差随时间的变化

## 常见问题

### Q1: 如何修改曲线颜色和样式？

在各绘图函数中找到 `style_desired` 和 `style_actual` 字典，修改其中的参数：

```python
style_desired = {'color': 'blue', 'linestyle': '-', 'linewidth': 1.5}
style_actual = {'color': 'magenta', 'linestyle': '-.', 'linewidth': 1.5}
```

### Q2: 如何调整图表尺寸？

在 `plt.subplots()` 调用中修改 `figsize` 参数：

```python
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(8, 9))  # (宽, 高) 单位: 英寸
```

### Q3: 如何修改字体大小？

在脚本开头的全局样式设置中修改：

```python
plt.rcParams['font.size'] = 11  # 修改为您需要的字号
```

### Q4: 如何只绘制某一张图？

在 `main()` 函数中注释掉不需要的绘图函数调用：

```python
# fig1 = plot_position_tracking(data)  # 注释掉不绘制图 1
fig2 = plot_manipulator_tracking(data)
fig3 = plot_position_error(data)
```

## 作者与维护

- **创建日期**: 2026-01-03
- **脚本版本**: 1.0
- **基于**: `plot_result.py` 的数据结构

## 许可

本脚本遵循与 PX4 项目相同的许可协议。
