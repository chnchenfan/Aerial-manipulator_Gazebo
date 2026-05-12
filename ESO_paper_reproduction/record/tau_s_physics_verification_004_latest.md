# tau_s 动力学前馈验证记录 004（最新）

## 本轮目标

- 针对 `exp1_hover_disturbance_uam_v5` 继续调参，用户指定新目标：
  - `mean_position_error_m < 0.02 m`
  - `rmse_position_error_m < 0.02 m`
  - `max_position_error_m < 0.04 m`
- 每次先让 Optuna 跑一批候选，再由 Codex 介入做方向分析。
- 本轮重点判断三类方向：
  1. 纯控制器窄域调参；
  2. exp1 加 TD setpoint；
  3. 动力学前馈 / tau_s / obs-ctrl split 参与调参。

## 使用的脚本与数据路径

### 运行脚本

- 第一轮、第二轮入口脚本：
  `/tmp/optuna_uam_v5_strict_batch.py`
- 第三轮动力学前馈入口脚本：
  `/tmp/optuna_uam_v5_exp1_dyn_batch.py`
- 通用 SITL/录包/指标基础脚本：
  `ESO_paper_reproduction/src/uav_control/scripts/auto_tune_uam_v5_eso.py`
- 原始位置误差计算脚本：
  `ESO_paper_reproduction/src/uav_control/scripts/compute_raw_position_error.py`

### 代码改动

- 为验证 exp1 TD 方向，本轮给 exp1 hover 节点临时接入 TD setpoint 参数：
  - `ESO_paper_reproduction/src/uav_arm_top/src/eso_hover_disturbance_offboard_node.cpp`
  - `ESO_paper_reproduction/src/uav_arm_top/launch/exp1_hover_disturbance_uam_v5.launch`
- 新增/使用的 launch 参数：
  - `use_td_setpoint`
  - `td_bandwidth_hz`
  - `td_accel_limit_mps2`
  - `td_vel_limit_mps`
- 编译命令：
  `catkin_make -DCATKIN_WHITELIST_PACKAGES='uav_control;uav_arm_top'`
- 编译结果：通过。

### Optuna 输出

- 第一轮 controller-only：
  - summary:
    `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_075756_uam_v5_strict_exp1_td_controller/uam_v5_strict_exp1_td_controller_summary.json`
  - study DB:
    `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/uam_v5_strict_exp1_td_controller.db`
- 第二轮 exp1 TD setpoint：
  - summary:
    `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_095606_uam_v5_strict_exp1_tdsetpoint_controller/uam_v5_strict_exp1_tdsetpoint_controller_summary.json`
  - study DB:
    `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/uam_v5_strict_exp1_tdsetpoint_controller.db`
- 第三轮 dynamics/tau_s：
  - summary:
    `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_103938_uam_v5_strict_exp1_dyn_taus/uam_v5_strict_exp1_dyn_taus_summary.json`
  - study DB:
    `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/uam_v5_strict_exp1_dyn_taus.db`

## 接受标准

本轮按用户新目标评估：

| 指标 | 目标 |
| --- | ---: |
| mean | `< 0.02 m` |
| RMSE | `< 0.02 m` |
| max | `< 0.04 m` |

所有结果均使用 fresh SITL bag 的 `raw_position_error.json`，不使用旧结论或 README 作为通过证据。

## 第一轮：controller-only 窄域 Optuna

### 方法

- 实验：`exp1_hover_disturbance_uam_v5`
- 计划 trial 数：50
- 实际完成：50
- TD：关闭
- 动力学前馈 / tau_s：关闭
- 搜索重点：
  - `ESO_X/Y_P`
  - `ESO_X/Y_I`
  - `ESO_X/Y_VEL_P_ACC`
  - `ESO_X/Y_BW`
  - `ESO_Z_*`
  - `ESO_ROLL/PITCH_*`
  - `ESO_RATE_BW_R/P`
  - `ESO_K_BETA`
  - `ESO_MAX_TORQUE`

### 最优结果

最优 candidate：
`uam_v5_strict_exp1_td_controller_trial_0049`

fresh bag：
`ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_075756_uam_v5_strict_exp1_td_controller/uam_v5_strict_exp1_td_controller_trial_0049/exp1_hover_disturbance_uam_v5/bags/20260512_093444/exp1_hover_disturbance_uam_v5.bag`

指标窗口：

- `analysis_start_time_s = 10.108`
- `analysis_start_source = metadata`
- `analysis_duration_s = 10.0`
- `sample_count = 300`

| 指标 | 数值 m | 是否达标 |
| --- | ---: | --- |
| mean | 0.03213 | 否 |
| RMSE | 0.03502 | 否 |
| max | 0.06435 | 否 |
| x max | 0.03950 | 贴近 |
| y max | 0.05410 | 否 |
| z max | 0.03469 | 是 |

关键参数：

| 参数 | 值 |
| --- | ---: |
| `ESO_X_P` | 1.60890 |
| `ESO_Y_P` | 1.88044 |
| `ESO_X_I` | 0.32030 |
| `ESO_Y_I` | 0.27217 |
| `ESO_X_VEL_P_ACC` | 2.72731 |
| `ESO_Y_VEL_P_ACC` | 3.23521 |
| `ESO_X_BW` | 1.33149 |
| `ESO_Y_BW` | 2.67953 |
| `ESO_Z_P` | 1.56182 |
| `ESO_Z_VEL_P_ACC` | 1.52920 |
| `ESO_Z_BW` | 1.16835 |
| `ESO_POS_INT_LIM` | 0.37797 |
| `ESO_ACC_HOR` | 9.07077 |
| `ESO_ACC_HOR_MAX` | 9.54404 |
| `ESO_JERK_AUTO` | 7.09473 |
| `ESO_ROLL_P` | 1.59024 |
| `ESO_PITCH_P` | 1.67356 |
| `ESO_ROLLRATE_P` | 0.23721 |
| `ESO_PITCHRATE_P` | 0.15599 |
| `ESO_RATE_BW_R` | 3.20276 |
| `ESO_RATE_BW_P` | 2.52735 |
| `ESO_RATE_I_SC` | 0.11503 |
| `ESO_K_BETA` | 0.65273 |
| `ESO_MAX_TORQUE` | 1.51735 |

### 判断

- 纯控制器调参能把 `z max` 压入 `0.04 m`，但 `mean/RMSE` 仍卡在 `0.032/0.035 m`。
- 主峰值来自 `Y` 轴，`y max = 0.05410 m`。
- 该轮可作为后续动力学前馈的控制器基线，但不能作为最终通过候选。

## 第二轮：exp1 加 TD setpoint

### 方法

- 实验：`exp1_hover_disturbance_uam_v5`
- 计划 trial 数：50
- 实际完成：20
- 中止原因：前 20 个 fresh trial 已显示 TD 方向相对第一轮退化，继续跑满 50 次不符合预算策略。
- TD：开启并纳入 Optuna 搜索
  - `td_bandwidth_hz`
  - `td_accel_limit_mps2`
  - `td_vel_limit_mps`
- 动力学前馈 / tau_s：关闭

### 最优结果

最优 candidate：
`uam_v5_strict_exp1_tdsetpoint_controller_trial_0018`

fresh bag：
`ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_095606_uam_v5_strict_exp1_tdsetpoint_controller/uam_v5_strict_exp1_tdsetpoint_controller_trial_0018/exp1_hover_disturbance_uam_v5/bags/20260512_103223/exp1_hover_disturbance_uam_v5.bag`

指标窗口：

- `analysis_start_time_s = 10.112`
- `analysis_start_source = metadata`
- `analysis_duration_s = 10.0`
- `sample_count = 300`

| 指标 | 数值 m | 是否达标 |
| --- | ---: | --- |
| mean | 0.03422 | 否 |
| RMSE | 0.03871 | 否 |
| max | 0.07225 | 否 |
| x max | 0.01675 | 是 |
| y max | 0.06165 | 否 |
| z max | 0.04338 | 否 |

关键 TD 参数：

| 参数 | 值 |
| --- | ---: |
| `td_bandwidth_hz` | 0.18923 |
| `td_accel_limit_mps2` | 0.35426 |
| `td_vel_limit_mps` | 0.27754 |

### 判断

- TD setpoint 明显改善 `x max`，但 `mean/RMSE/max` 均劣于第一轮最优。
- `z max` 从第一轮最优的 `0.03469 m` 退化到 `0.04338 m`。
- exp1 是定点悬停扰动实验，测量窗内目标点基本不动；误差主要来自机械臂扰动，不是轨迹 setpoint 突变。
- 因此 TD setpoint 对 exp1 判定为 rejected direction。

## 第三轮：dyn/tau_s 动力学前馈 Optuna

### 方法

- 实验：`exp1_hover_disturbance_uam_v5`
- 计划 trial 数：50
- 实际完成：50
- TD：关闭
- 动力学前馈：
  - `ESO_DYN_FF_EN=1`
  - 搜索 `ESO_K_BETA`
  - 搜索 `ESO_MAX_TORQUE`
  - 搜索 `ESO_TAUS_K`
  - 搜索 `ESO_TAUS_K_R/P/Y`
  - 搜索 `ESO_TAUS_OBS_R/P`
  - 搜索 `ESO_TAUS_CTL_R/P`
  - 搜索 `ESO_TAUS_LIM`
  - 搜索 `ESO_TAUS_TAU`
- 控制器参数以第一轮最优 trial 49 为中心做窄域微调。
- 评分函数加重 `Y max`，同时惩罚 `Z` 退化。

### 最优结果

最优 candidate：
`uam_v5_strict_exp1_dyn_taus_trial_0048`

fresh bag：
`ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_103938_uam_v5_strict_exp1_dyn_taus/uam_v5_strict_exp1_dyn_taus_trial_0048/exp1_hover_disturbance_uam_v5/bags/20260512_121448/exp1_hover_disturbance_uam_v5.bag`

指标窗口：

- `analysis_start_time_s = 2.284`
- `analysis_start_source = metadata`
- `analysis_duration_s = 10.0`
- `sample_count = 300`

| 指标 | 数值 m | 是否达标 |
| --- | ---: | --- |
| mean | 0.03297 | 否 |
| RMSE | 0.03505 | 否 |
| max | 0.05602 | 否 |
| x max | 0.03111 | 是 |
| y max | 0.04927 | 否 |
| z max | 0.03817 | 是 |

关键动力学参数：

| 参数 | 值 |
| --- | ---: |
| `ESO_DYN_FF_EN` | 1 |
| `ESO_K_BETA` | 0.66223 |
| `ESO_MAX_TORQUE` | 1.51322 |
| `ESO_TAUS_K` | 0.09511 |
| `ESO_TAUS_K_R` | 0.01495 |
| `ESO_TAUS_K_P` | -0.31641 |
| `ESO_TAUS_K_Y` | 0.59446 |
| `ESO_TAUS_OBS_R` | 1.12303 |
| `ESO_TAUS_OBS_P` | 1.16166 |
| `ESO_TAUS_CTL_R` | 0.95928 |
| `ESO_TAUS_CTL_P` | 1.38200 |
| `ESO_TAUS_LIM` | 0.23873 |
| `ESO_TAUS_TAU` | 0.15964 |

关键控制器参数：

| 参数 | 值 |
| --- | ---: |
| `ESO_X_P` | 1.57042 |
| `ESO_Y_P` | 1.93110 |
| `ESO_X_I` | 0.35494 |
| `ESO_Y_I` | 0.30566 |
| `ESO_X_VEL_P_ACC` | 2.65430 |
| `ESO_Y_VEL_P_ACC` | 3.61775 |
| `ESO_X_BW` | 1.57395 |
| `ESO_Y_BW` | 2.62867 |
| `ESO_Z_P` | 1.54265 |
| `ESO_Z_VEL_P_ACC` | 1.47839 |
| `ESO_Z_BW` | 1.15777 |
| `ESO_POS_INT_LIM` | 0.36820 |
| `ESO_ACC_HOR` | 8.41854 |
| `ESO_ACC_HOR_MAX` | 9.32449 |
| `ESO_JERK_AUTO` | 7.10799 |
| `ESO_ROLL_P` | 1.63571 |
| `ESO_PITCH_P` | 1.57133 |
| `ESO_ROLLRATE_P` | 0.24055 |
| `ESO_PITCHRATE_P` | 0.16968 |
| `ESO_RATE_BW_R` | 3.54258 |
| `ESO_RATE_BW_P` | 2.29786 |
| `ESO_RATE_I_SC` | 0.13414 |

### 判断

- 第三轮相对第一轮有明确削峰收益：
  - `max`: `0.06435 -> 0.05602`
  - `y max`: `0.05410 -> 0.04927`
  - `x max`: `0.03950 -> 0.03111`
- `z max` 仍保持达标：
  - `0.03817 < 0.04`
- 但 `mean/RMSE` 没有实质改善：
  - 第一轮：`0.03213 / 0.03502`
  - 第三轮：`0.03297 / 0.03505`
- 因此 dyn/tau_s 方向成立，但当前仍没有达到用户指定的 `0.02/0.02/0.04`。

## 三轮综合对比

| 轮次 | 完成 trial | 方向 | mean m | RMSE m | max m | x max | y max | z max | 结论 |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 50 | controller-only | 0.03213 | 0.03502 | 0.06435 | 0.03950 | 0.05410 | 0.03469 | 基线最优，均值/RMSE 平台 |
| 2 | 20 | exp1 TD setpoint | 0.03422 | 0.03871 | 0.07225 | 0.01675 | 0.06165 | 0.04338 | 退化，判负 |
| 3 | 50 | dyn/tau_s | 0.03297 | 0.03505 | 0.05602 | 0.03111 | 0.04927 | 0.03817 | 削峰有效，均值/RMSE 未改善 |

## 动力学模型与 SDF 对应关系说明

本轮重新核对后，结论修正如下：

- `ArmKinematics.hpp` 的 UAM V5 实时动力学链路确实被使用，不是废代码。
- 姿态环会用 `arm_joint_states` 调用：
  - `computeSystemCoM(q)`
  - `computeSystemInertia(q)`
  并更新 tau_s 使用的 COM 和惯量。
- 速率环会用 `arm_joint_states` 调用：
  - `computeSystemInertia(q)`
  并用实时惯量计算：
  - `M(omega_r_dot - disturbance_hat)`
  - `omega x (M omega)`
- 位置环会用 `arm_joint_states` 调用：
  - `computeSystemCoM(q)`
  并用动态 COM 计算 `a_f_ned`。
- `tau_s` 中只保留重力力矩项：
  `tau_s = p_com x g_body * mass`
  这是合理的，因为速率环已经用实时惯量统一计算了陀螺/科氏项，避免重复补偿。

但也需要注意：

- `ArmKinematics.hpp` 里的 UAM V5 质量、几何、局部惯量是代码常量，来源对齐 SDF/实测整理值，但不是运行时自动读取 SDF。
- 当前 UAM V5 模型只让 q0/q1 影响连续 `CoM(q)` 和 `I(q)`；q2/q3 保留接口兼容但不进入连续补偿。
- SDF 中 `arm_link2=0.594 kg`，左右手各 `0.02 kg`；当前 `ArmKinematics` 中 `m2g=0.594 kg` 并注释为 link2 已含 gripper mass。若 SDF 是权威物理模型，这里存在约 `0.04 kg` 的合并口径差异。
- 工程上无需追求精确完全对应；当前更合理的定位是：
  `实时粗模型补偿 + ESO residual + Optuna 小范围校正`。

## Rejected Directions

- exp1 TD setpoint：
  - 理由：对定点悬停扰动实验没有压低主要误差源，且 `mean/RMSE/max` 全面劣于 controller-only 最优。
  - 后续不建议继续在 exp1 上搜索 TD。
- 单纯继续 controller-only 宽域：
  - 理由：第一轮已显示均值/RMSE 平台在 `0.032/0.035 m` 左右，继续大范围搜索收益低。

## 当前可保留候选

### 最佳 controller-only 基线

- candidate:
  `uam_v5_strict_exp1_td_controller_trial_0049`
- 用途：
  作为无动力学前馈回退基线。
- 指标：
  `mean=0.03213, RMSE=0.03502, max=0.06435`

### 最佳 dyn/tau_s 候选

- candidate:
  `uam_v5_strict_exp1_dyn_taus_trial_0048`
- 用途：
  作为下一轮动力学前馈窄域中心。
- 指标：
  `mean=0.03297, RMSE=0.03505, max=0.05602`
- 结论：
  削峰优于 controller-only，但不能通过新目标。

## 下一次调参建议

下一轮不建议做模型校准大改，也不建议继续 TD。建议保留当前工程近似模型，围绕第三轮最优做更窄的 50 次 Optuna：

### 搜索中心

- 以 `uam_v5_strict_exp1_dyn_taus_trial_0048` 为中心。
- 固定：
  - `use_td_setpoint=false`
  - `ESO_DYN_FF_EN=1`
  - `ESO_MAX_TORQUE` 在 `1.45-1.65` 小范围；
  - `ESO_TAUS_TAU` 在 `0.12-0.20`；
  - `ESO_TAUS_LIM` 在 `0.18-0.28`。

### 下一轮应重点调的参数

- tau_s：
  - `ESO_TAUS_K`: `0.06-0.14`
  - `ESO_TAUS_K_P`: 围绕负值搜索，建议 `-0.55 ~ -0.10`
  - `ESO_TAUS_K_Y`: 保留正向搜索，建议 `0.25 ~ 0.70`
  - `ESO_TAUS_K_R`: 收窄到 `-0.20 ~ 0.25`
  - `ESO_TAUS_OBS_P`: `0.9-1.4`
  - `ESO_TAUS_CTL_P`: `0.8-1.5`
  - `ESO_TAUS_OBS_R/CTL_R`: 保守收窄，避免引入 x 轴峰值。
- 控制器：
  - `ESO_Y_P`: `1.80-2.05`
  - `ESO_Y_I`: `0.28-0.36`
  - `ESO_Y_VEL_P_ACC`: `3.45-3.85`
  - `ESO_Y_BW`: `2.50-2.80`
  - `ESO_X_BW`: `1.45-1.70`
  - `ESO_Z_BW`: `1.10-1.22`
  - `ESO_RATE_BW_R`: `3.30-3.75`
  - `ESO_RATE_BW_P`: `2.20-2.55`

### 评分策略

- 继续使用用户新目标：
  `mean/RMSE < 0.02 m, max < 0.04 m`
- 下一轮评分应显式避免“只削峰、不降均值”的局部最优：
  - 提高 `mean` 与 `RMSE` 权重；
  - `Y max` 仍作为主要峰值项；
  - `Z max > 0.04` 直接重罚；
  - 如果 `mean > 0.034`，即使 max 较低也不应排第一。

### 预期

- 单靠下一轮窄域可能继续压 `max` 到 `0.05 m` 附近。
- 要达到 `mean/RMSE < 0.02 m`，仅靠 exp1 控制器/tau_s 参数可能不足，需要后续考虑：
  - 是否缩小机械臂扰动幅值/频率作为分阶段验证；
  - 是否进一步检查 `arm_joint_states` 时延、关节命令与实际 Gazebo joint state 的相位差；
  - 是否记录/分析 `tau_s_raw`、`tau_s_used`、dynamic COM、dynamic inertia 与 position error 的相位关系。

## 本轮结论

- 三轮 fresh Optuna 均未达到用户的新 strict gate。
- exp1 TD setpoint 被拒绝。
- dyn/tau_s 方向有效，主要体现在削峰：
  `max 0.06435 -> 0.05602`。
- 但当前均值/RMSE 平台仍在 `0.033/0.035 m` 左右。
- 下一轮应围绕 dyn/tau_s 最优 trial 48 做更窄的 50 次搜索，并修改评分函数，避免继续只优化 max。

## exp4 当前情况补充

### exp4 最优 summary

当前 exp4 TD/velocity-FF 窄域 Optuna 的全局最优 summary 位于：

`ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_002216_exp4_td_vff_narrow/optuna_exp4_td_vff_narrow_summary.json`

最优 candidate：
`optuna_exp4_td_vff_narrow_trial_0004`

fresh bag：
`ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_002216_exp4_td_vff_narrow/optuna_exp4_td_vff_narrow_trial_0004/exp4_square_tracking_uam_v5/bags/20260512_003524/exp4_square_tracking_uam_v5.bag`

指标窗口：

- `analysis_start_time_s = 9.492`
- `analysis_start_source = metadata`
- `analysis_duration_s = 78.0`
- `sample_count = 2340`

| 指标 | 数值 m | 说明 |
| --- | ---: | --- |
| mean | 0.04736 | 已接近旧目标，但未达到新目标 |
| RMSE | 0.05043 | 接近旧目标，未达到新目标 |
| max | 0.10384 | 仍明显高于新目标 |
| x max | 0.06757 | 未达新目标 |
| y max | 0.09549 | 当前主要峰值之一 |
| z max | 0.06485 | 未达新目标 |

关键 launch / TD 参数：

| 参数 | 值 |
| --- | ---: |
| `path_speed_mps` | 0.10789 |
| `use_raw_setpoint` | true |
| `use_td_setpoint` | true |
| `velocity_ff_scale` | 0.01010 |
| `td_bandwidth_hz` | 0.27723 |
| `td_accel_limit_mps2` | 0.25355 |
| `td_vel_limit_mps` | 0.23293 |

关键控制器参数：

| 参数 | 值 |
| --- | ---: |
| `ESO_Y_P` | 1.78587 |
| `ESO_Y_I` | 0.20973 |
| `ESO_Y_VEL_P_ACC` | 3.54011 |
| `ESO_Y_BW` | 2.31287 |
| `ESO_Z_P` | 1.47993 |
| `ESO_Z_VEL_P_ACC` | 1.41950 |
| `ESO_Z_BW` | 1.36645 |
| `ESO_POS_INT_LIM` | 0.45585 |
| `ESO_ACC_HOR` | 8.02900 |
| `ESO_ACC_HOR_MAX` | 9.28130 |
| `ESO_JERK_AUTO` | 6.79792 |
| `ESO_ROLL_P` | 1.52877 |
| `ESO_ROLLRATE_P` | 0.21775 |
| `ESO_RATE_BW_R` | 3.55234 |

### exp4 与 exp1 的 TD 结论不同

- exp1 是定点悬停扰动实验：
  - 测量窗内目标 setpoint 基本不动；
  - 主要误差来自机械臂周期扰动；
  - 因此 exp1 加 TD setpoint 对主要误差源帮助很小，并且本轮 fresh 结果显示退化。
- exp4 是方形轨迹跟踪实验：
  - 目标点沿方形路径移动，并且有拐角和速度/加速度变化；
  - 原始 setpoint 的变化会直接引入轨迹跟踪瞬态峰值；
  - TD setpoint 可以平滑参考轨迹、限制速度/加速度、降低拐角冲击。
- 因此：
  - exp1：TD rejected direction；
  - exp4：TD 是必要方向，应继续保留并纳入后续 Optuna。

### exp4 已观察到的趋势

- 宽域 TD/velocity-FF 搜索显示：
  - `path_speed_mps` 降低有明显收益；
  - `velocity_ff_scale` 过大容易放大 Y 轴峰值；
  - TD 过快或加速度限制过高会使 Y max 变差。
- 窄域搜索后，最佳组合集中在：
  - `path_speed_mps ~= 0.108`
  - `velocity_ff_scale ~= 0.01`
  - `td_bandwidth_hz ~= 0.28`
  - `td_accel_limit_mps2 ~= 0.25`
  - `td_vel_limit_mps ~= 0.23`
- 当前 exp4 的主要问题仍是：
  - `Y max` 和 overall max 高；
  - mean/RMSE 虽较好，但距离用户新目标 `0.02/0.02/0.04` 仍很远。

### 后续顺序

- 按 `uam-v5-eso-autotune` 的 staged workflow，仍应先把 `exp1_hover_disturbance_uam_v5` 调到新目标或至少形成稳定可接受候选。
- 在 exp1 未通过前，不建议继续大预算调 exp4。
- exp1 通过后，再进入 exp4 下一轮 50 次 Optuna：
  - 保留 TD setpoint；
  - TD的参数以 `optuna_exp4_td_vff_narrow_trial_0004` 为中心，其它参数以exp1的最优结果为中心；
  - 固定或窄域搜索低 `path_speed_mps`；
  - 保持 `velocity_ff_scale` 小范围；
  - 联动搜索 dyn/tau_s 与 Y/Z 峰值相关参数；
  - 评分函数应同时重罚 `Y max`、`Z max`、overall max，避免只优化 mean。

## 第四轮：dyn/tau_s 窄域二次搜索

### 方法

- 实验：`exp1_hover_disturbance_uam_v5`
- 入口脚本：
  `/tmp/optuna_uam_v5_exp1_dyn_taus_narrow2.py`
- 输出 summary：
  `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_125621_uam_v5_strict_exp1_dyn_taus_narrow2/uam_v5_strict_exp1_dyn_taus_narrow2_summary.json`
- study DB：
  `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/uam_v5_strict_exp1_dyn_taus_narrow2.db`
- 搜索中心：
  `uam_v5_strict_exp1_dyn_taus_trial_0048`
- 先做 fresh baseline repeat，再跑 50 个 Optuna trial。
- 固定：
  - `use_td_setpoint=false`
  - `ESO_DYN_FF_EN=1`
- 搜索空间按第三轮建议收窄，并提高 `mean/RMSE` 权重，避免只削峰。

### baseline repeat

candidate：
`uam_v5_strict_exp1_dyn_taus_narrow2_baseline_repeat`

fresh bag：
`ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_125621_uam_v5_strict_exp1_dyn_taus_narrow2/uam_v5_strict_exp1_dyn_taus_narrow2_baseline_repeat/exp1_hover_disturbance_uam_v5/bags/20260512_125742/exp1_hover_disturbance_uam_v5.bag`

| 指标 | 数值 m | 是否达标 |
| --- | ---: | --- |
| mean | 0.03037 | 否 |
| RMSE | 0.03401 | 否 |
| max | 0.06642 | 否 |
| x max | 0.01844 | 是 |
| y max | 0.05620 | 否 |
| z max | 0.03988 | 是 |

说明：

- 同一中心参数 fresh repeat 的 mean 优于第三轮记录值 `0.03297`，说明该参数附近存在一定 SITL 重复噪声。
- 但 max 退化到 `0.06642`，不能用 repeat 作为通过证据。

### 最佳综合评分候选

candidate：
`uam_v5_strict_exp1_dyn_taus_narrow2_trial_0006`

fresh bag：
`ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_125621_uam_v5_strict_exp1_dyn_taus_narrow2/uam_v5_strict_exp1_dyn_taus_narrow2_trial_0006/exp1_hover_disturbance_uam_v5/bags/20260512_130928/exp1_hover_disturbance_uam_v5.bag`

| 指标 | 数值 m | 是否达标 |
| --- | ---: | --- |
| mean | 0.02656 | 否 |
| RMSE | 0.02891 | 否 |
| max | 0.05925 | 否 |
| x max | 0.01839 | 是 |
| y max | 0.04870 | 否 |
| z max | 0.03410 | 是 |

关键参数：

| 参数 | 值 |
| --- | ---: |
| `ESO_TAUS_K` | 0.07656 |
| `ESO_TAUS_K_R` | 0.04197 |
| `ESO_TAUS_K_P` | -0.45180 |
| `ESO_TAUS_K_Y` | 0.31539 |
| `ESO_TAUS_OBS_R` | 1.17428 |
| `ESO_TAUS_OBS_P` | 0.99751 |
| `ESO_TAUS_CTL_R` | 0.91298 |
| `ESO_TAUS_CTL_P` | 1.20309 |
| `ESO_TAUS_LIM` | 0.27777 |
| `ESO_TAUS_TAU` | 0.13487 |
| `ESO_K_BETA` | 0.67540 |
| `ESO_MAX_TORQUE` | 1.56892 |
| `ESO_Y_P` | 1.82179 |
| `ESO_Y_I` | 0.29110 |
| `ESO_Y_VEL_P_ACC` | 3.45188 |
| `ESO_Y_BW` | 2.68637 |
| `ESO_X_BW` | 1.47964 |
| `ESO_Z_BW` | 1.15193 |
| `ESO_RATE_BW_R` | 3.74509 |
| `ESO_RATE_BW_P` | 2.27285 |
| `ESO_RATE_I_SC` | 0.12431 |

判断：

- 这是本轮最重要的新方向：mean/RMSE 从第三轮约 `0.033/0.035` 降到 `0.0266/0.0289`。
- `x max` 和 `z max` 都过新目标，但 `y max=0.04870`，overall max 仍为 `0.05925`。
- 该候选说明 dyn/tau_s 窄域能压低均值平台，但还没有解决 Y 轴峰值。

### 最佳削峰候选

candidate：
`uam_v5_strict_exp1_dyn_taus_narrow2_trial_0033`

fresh bag：
`ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_125621_uam_v5_strict_exp1_dyn_taus_narrow2/uam_v5_strict_exp1_dyn_taus_narrow2_trial_0033/exp1_hover_disturbance_uam_v5/bags/20260512_140341/exp1_hover_disturbance_uam_v5.bag`

| 指标 | 数值 m | 是否达标 |
| --- | ---: | --- |
| mean | 0.03149 | 否 |
| RMSE | 0.03322 | 否 |
| max | 0.05416 | 否 |
| x max | 0.03775 | 是 |
| y max | 0.04109 | 接近 |
| z max | 0.03855 | 是 |

判断：

- 这是本轮最好的 max/Y-max 方向：
  `max=0.05416, y max=0.04109`。
- 代价是 mean/RMSE 回到 `0.0315/0.0332`。
- 它不应替代 trial 0006，但可作为下一轮 Y 峰值约束的第二中心。

### 第四轮综合结论

- 51 条 fresh evidence（baseline repeat + 50 trials）中无 strict pass。
- 第四轮相对第三轮有实质进展：
  - 最佳 mean：`0.03297 -> 0.02656`
  - 最佳 RMSE：`0.03505 -> 0.02891`
  - 最佳 max：`0.05602 -> 0.05416`
  - 最佳 y max：`0.04927 -> 0.04109`
- 但没有一个候选同时满足：
  `mean < 0.02, RMSE < 0.02, max < 0.04`。
- 当前分裂成两个方向：
  1. 低均值方向：trial 0006；
  2. 低峰值/Y 轴方向：trial 0033。

### 下一次调参建议

下一轮不应回到宽域搜索。建议做“双中心窄域”或“两阶段”搜索：

1. 以 trial 0006 为主中心，固定其低均值特征，重点压 `Y max`：
   - 保持 `ESO_TAUS_K` 约 `0.065-0.095`
   - 保持 `ESO_TAUS_K_P` 负向约 `-0.52 ~ -0.38`
   - 保持较低 `ESO_TAUS_K_Y` 约 `0.25-0.40`
   - 重点搜索 `ESO_Y_BW`, `ESO_Y_P`, `ESO_Y_I`, `ESO_Y_VEL_P_ACC`, `ESO_RATE_BW_R/P`
2. 以 trial 0033 为副中心，尝试把低 Y 峰值方向的 mean 拉低：
   - 保持 `Y max` 约束；
   - 收窄 `ESO_MAX_TORQUE` 到 `1.58-1.65`；
   - 收窄 `ESO_TAUS_LIM` 到 `0.25-0.28`；
   - 避免 `x max` 超过 `0.04`。
3. 评分函数需要加入硬约束式筛选：
   - `z max > 0.04` 直接强惩罚；
   - `y max > 0.045` 强惩罚；
   - `mean > 0.030` 的候选除非 `max < 0.052`，否则不应排第一。

若下一轮仍不能把 mean/RMSE 压到 `0.02` 附近，应开始诊断 `arm_joint_states` 时延、关节实际相位、`tau_s_raw/tau_s_used` 与 position error 的相位关系，而不是继续纯参数搜索。

## 第五轮：双中心窄域搜索（提前中止）

### 方法

- 实验：`exp1_hover_disturbance_uam_v5`
- 入口脚本：
  `/tmp/optuna_uam_v5_exp1_dyn_taus_twocenter.py`
- 输出 summary：
  `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_144055_uam_v5_strict_exp1_dyn_taus_twocenter/uam_v5_strict_exp1_dyn_taus_twocenter_summary.json`
- study DB：
  `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/uam_v5_strict_exp1_dyn_taus_twocenter.db`
- 计划：
  - 先 fresh repeat 第四轮低均值中心 trial 0006；
  - 再 fresh repeat 第四轮低峰值中心 trial 0033；
  - 然后做 50 次双中心 Optuna。
- 实际：
  - 完成 2 个 repeat + 约 25 个 Optuna trial 后提前中止。

### 中止原因

前 20 多个 fresh trial 没有超过第四轮最优，且两个中心 repeat 均退化：

| candidate | mean m | RMSE m | max m | x max | y max | z max | 判断 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `low_mean_repeat` | 0.03275 | 0.03500 | 0.05908 | 0.03295 | 0.05616 | 0.03811 | 低均值中心不稳定 |
| `low_peak_repeat` | 0.03927 | 0.04255 | 0.08326 | 0.05048 | 0.06379 | 0.03230 | 低峰值中心明显退化 |
| best partial `trial_0012` | 0.03109 | 0.03346 | 0.05953 | 0.02535 | 0.04897 | 0.03901 | 未优于第四轮 |

对比第四轮保留候选：

- 第四轮 trial 0006：
  `mean=0.02656, RMSE=0.02891, max=0.05925`
- 第四轮 trial 0033：
  `mean=0.03149, RMSE=0.03322, max=0.05416`

因此，第五轮双中心窄域判定为 rejected / diagnostic only，不继续跑满 50 次。

### 结论更新

- 第四轮 trial 0006 仍是当前 exp1 最佳低均值候选。
- 第四轮 trial 0033 仍是当前 exp1 最佳削峰/Y-max 候选。
- 第五轮说明当前参数局部存在明显 repeat 噪声，继续收窄纯参数搜索不稳。
- 下一步不建议继续同类窄域 Optuna。应转入诊断：
  - `arm_joint_states` 与 Gazebo 实际关节状态/命令的相位差；
  - `tau_s_raw`、`tau_s_used`、dynamic COM、dynamic inertia 与 position error 的相位关系；
  - 是否存在 tau_s 方向正确但时延/滤波导致补偿相位滞后；
  - 必要时再基于诊断结果重开小范围参数搜索。

## 第六轮：暂停 exp1，exp4 复用 exp1 最优参数，仅调 TD/路径参数

### 方法修正

用户指出最终需要 `exp1` 和 `exp4` 使用同一套控制器 / ESO / dyn/tau_s 参数，`exp4` 只允许比 `exp1` 多任务层 TD 参数。

因此中止原先沿历史 exp4 专用 no-dynff 参数继续搜索的方向，改为：

- 控制器、ESO、dyn/tau_s 全部固定为当前 exp1 最优：
  `uam_v5_strict_exp1_dyn_taus_narrow2_trial_0006`
- exp4 只搜索 launch / TD / path 参数：
  - `path_speed_mps`
  - `velocity_ff_scale`
  - `td_bandwidth_hz`
  - `td_accel_limit_mps2`
  - `td_vel_limit_mps`
- 固定：
  - `use_raw_setpoint=true`
  - `use_td_setpoint=true`
  - `ESO_DYN_FF_EN=1`
  - exp1 best 的 tau_s 参数全保留

### 运行信息

- 入口脚本：
  `/tmp/optuna_exp4_from_exp1_best_td_only.py`
- 输出 summary：
  `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_20260512_160308_exp4_from_exp1_best_td_only/optuna_exp4_from_exp1_best_td_only_summary.json`
- study DB：
  `ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/optuna_exp4_from_exp1_best_td_only.db`
- fresh evidence：
  3 个 seed + 20 个 Optuna trial，共 23 条。

### 当前结果

- 旧 gate（`mean <= 0.05, RMSE <= 0.05, max <= 0.07`）：0 条通过。
- strict gate（`mean < 0.02, RMSE < 0.02, max < 0.04`）：0 条通过。

但复用 exp1 参数后的 exp4 明显优于历史 exp4 专用 no-dynff best：

| 对比 | mean | RMSE | max | 说明 |
| --- | ---: | ---: | ---: | --- |
| 历史 exp4 专用 best `optuna_exp4_td_vff_narrow_trial_0007` | 0.04541 | 0.04842 | 0.11049 | no-dynff，exp4 专用控制器 |
| 本轮最低 max `trial_0010` | 0.04081 | 0.04342 | 0.08095 | 复用 exp1 best，仅调 TD |
| 本轮综合/Y 峰值 best `trial_0003` | 0.04030 | 0.04300 | 0.08393 | `y max=0.06844` |

### 最低 overall max 候选

candidate：
`exp4_from_exp1_best_td_only_trial_0010`

| 指标 | 数值 m | 是否达旧 gate |
| --- | ---: | --- |
| mean | 0.04081 | 是 |
| RMSE | 0.04342 | 是 |
| max | 0.08095 | 否 |
| x max | 0.07190 | 接近 |
| y max | 0.07771 | 否 |
| z max | 0.07114 | 接近 |

TD / path 参数：

| 参数 | 值 |
| --- | ---: |
| `path_speed_mps` | 0.09564 |
| `velocity_ff_scale` | 0.00833 |
| `td_bandwidth_hz` | 0.32246 |
| `td_accel_limit_mps2` | 0.20853 |
| `td_vel_limit_mps` | 0.22790 |

判断：

- 这是当前最接近 old max gate 的同参数 exp4 候选。
- 三轴 max 都集中在 `0.071-0.078 m`，说明不是单一轴严重失控，而是整体峰值还需小幅压低。

### 综合评分 / Y 峰值最优候选

candidate：
`exp4_from_exp1_best_td_only_trial_0003`

| 指标 | 数值 m | 是否达旧 gate |
| --- | ---: | --- |
| mean | 0.04030 | 是 |
| RMSE | 0.04300 | 是 |
| max | 0.08393 | 否 |
| x max | 0.08055 | 否 |
| y max | 0.06844 | 是 |
| z max | 0.06557 | 是 |

TD / path 参数：

| 参数 | 值 |
| --- | ---: |
| `path_speed_mps` | 0.09621 |
| `velocity_ff_scale` | 0.00828 |
| `td_bandwidth_hz` | 0.32880 |
| `td_accel_limit_mps2` | 0.19319 |
| `td_vel_limit_mps` | 0.22063 |

判断：

- `Y` 与 `Z` 峰值已低于 `0.07 m`。
- 剩余瓶颈转移到 `X max=0.08055 m`。

### 本轮结论

- 用户提出的“同一套参数”约束是正确的，后续不应再用 exp4 专用控制器参数作为最终候选。
- 当前 exp1 best 参数直接迁移到 exp4，并只调 TD/path 后，exp4 性能优于历史 exp4 专用 no-dynff 参数。
- 当前最优区间集中在：
  - `path_speed_mps ~= 0.095-0.104`
  - `velocity_ff_scale ~= 0.000-0.008`
  - `td_bandwidth_hz ~= 0.24-0.33`
  - `td_accel_limit_mps2 ~= 0.19-0.29`
  - `td_vel_limit_mps ~= 0.21-0.23`
- 下一轮建议继续 TD/path-only 搜索，但收窄到 `trial_0010` 和 `trial_0003` 附近：
  - 主目标：让 X/Y/Z 三轴 max 同时低于 `0.07 m`；
  - 不再改控制器 / ESO / tau_s，保持与 exp1 完全一致。
