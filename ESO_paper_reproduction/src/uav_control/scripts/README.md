# ESO 结果可视化 (ESO Result Visualization)

本目录包含用于分析 ESO 控制器及机械臂性能的可视化脚本。

## 前置条件 (Prerequisites)

1. **已修改的 PX4 固件**：确保已编译并烧录了包含 `debug_key_value` 桥接修改的固件 (涉及 `eso_pos_control`, `eso_rate_control`, `eso_att_control`)。
2. **MAVROS**：标准的 MAVROS 环境。

## 使用方法 (Usage)

### 1. 录制数据 (Record Data)

使用 rosbag 录制以下话题。注意 `/mavros/debug/named_value_float` 对于获取 ESO 数据至关重要。

```bash
rosbag record /mavros/local_position/pose \
              /mavros/setpoint_position/local \
              /mavros/imu/data \
              /mavros/debug/named_value_float \
              /uav_arm/joint_states \
              /uav_arm/target_joint_states \
              -O test_flight.bag
```

### 2. 运行可视化 (Run Visualization)

使用 `plot_result.py` 生成 6 张性能分析图。可以通过参数指定起始时间，跳过起飞前的等待。

```bash
# 基本用法 (从 bag 开始处分析)
python3 plot_result.py test_flight.bag

# 从 bag 的 15.5 秒处开始分析
python3 plot_result.py test_flight.bag --start 15.5
```

### 3. 输出图表 (Output Figures)

脚本将生成 6 张交互式图表：

1.  **高度跟踪 (Height Tracking)**：实际 Z vs 目标 Z (附 MSE)。
2.  **XY 跟踪 (XY Tracking)**：实际 XY vs 目标 XY (附 MSE)。
3.  **综合位置误差 (Overall Position Error)**：随时间变化的位置误差模长 $e(t) = ||p - p_d||$，以及平均误差。
4.  **姿态跟踪 (Attitude Tracking)**：Roll/Pitch/Yaw 实际 vs 目标 (通过桥接获取)，附 MSE。
5.  **ESO 估计 (ESO Estimation)**：
    *   估计 vs 实际位置 (X)
    *   估计 vs 实际角速度 (X)
    *   估计扰动 (力 & 力矩)
6.  **机械臂 (Robotic Arm)**：关节角度 实际 vs 目标 (附 MSE)。

## 话题映射 (Topic Mappings)

脚本使用以下话题映射关系：

| 数据 | 来源话题 | 说明 |
|------|-------------|------|
| **实际位置** | `/mavros/local_position/pose` | 标准 MAVROS |
| **目标位置** | `/mavros/setpoint_position/local` | 标准 MAVROS |
| **实际姿态** | `/mavros/local_position/pose` | 四元数转欧拉角 |
| **目标姿态** | `/mavros/debug/named_value_float` | 键: `AT_R`, `AT_P`, `AT_Y` |
| **实际角速度** | `/mavros/imu/data` | 标准 MAVROS |
| **ESO 估计位置**| `/mavros/debug/named_value_float` | 键: `ESO_PX`, `ESO_PY`, `ESO_PZ` |
| **ESO 估计扰动** | `/mavros/debug/named_value_float` | 键: `ESO_DX`, `ESO_DY`, `ESO_DZ` |
| **ESO 估计角速度** | `/mavros/debug/named_value_float` | 键: `ESO_WX`, `ESO_WY`, `ESO_WZ` |
| **ESO 估计力矩** | `/mavros/debug/named_value_float` | 键: `ESO_TX`, `ESO_TY`, `ESO_TZ` |
| **机械臂关节** | `/uav_arm/joint_states` | 标准 ROS |

## MSE 计算说明

均方误差 (MSE) 计算公式为分析时段内实际值与目标值之差的平方平均数：

$$ MSE = \frac{1}{N} \sum_{i=1}^{N} (Actual_i - Target_i)^2 $$

计算前会对数据进行线性插值以对齐时间戳。

## 悬停统计脚本（控制台日志）

如果你已经在 PX4 控制台中开启了以下低频打印：
- `rate: e(w-sp)[...] ... w_d[...]`
- `rate term1: ... int[...]`

可以使用 `hover_rate_stats.py` 直接做悬停统计，计算：
- `mean(e_xyz)`
- `mean(w_d_y)`
- `mean(int_y)`

### 基本用法

```bash
python3 hover_rate_stats.py \
  --log /path/to/px4.log \
  --start-sec 20 \
  --end-sec 80 \
  --out /tmp/hover_stats.json
```

### 参数说明

- `--log`（必填）：控制台日志路径
- `--start-sec`（可选，默认 `0.0`）：统计起始时间（秒）
- `--end-sec`（可选，默认不限制）：统计结束时间（秒）
- `--time-mode`（可选，默认 `auto`）：`auto|timestamp|synthetic`
- `--sample-period`（可选，默认 `0.5`）：`synthetic` 模式采样周期（秒）
- `--out`（可选）：输出 JSON 路径
- `--dump-csv`（可选）：导出样本 CSV 路径（调试用）

### 时间模式说明

- `auto`：优先使用行内时间戳；若缺失则自动回退到 `synthetic`
- `timestamp`：强制使用行内时间戳；缺失即报错
- `synthetic`：按样本索引 × `sample-period` 构造时间

### 输出字段

- `mean_e`: `[mean_ex, mean_ey, mean_ez]`
- `mean_w_d_y`
- `mean_int_y`
- `sample_count_rate`
- `sample_count_term1`
- `time_mode_used`
- `window: {start_sec, end_sec}`
- `warnings`
