# tau_s 动力学前馈继续验证日志 002

记录日期：2026-04-28

## 本轮约束

- 调参/验证原始数据继续输出到：
  `/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/uav_arm_top/auto_tune_data`
- 文档只放到：
  `/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/record`
- 后续动力学前馈文档只追加或新建编号文档，不删除旧文档。
- 本轮继续围绕 pitch-only tau_s 小比例前馈验证，不修改 skill memory 中当前 best 参数。

## 上一轮结论继承

- `ESO_MAX_TORQUE=2.2` 在 pitch 轴基本可信，pitch 轴 `tau_actual ~= eso_torque` 的 scale 约 `0.98-1.00`。
- `tau_s_raw -> actual_minus_eso_torque` 的 profile CoM pitch 轴有效比例稳定在约 `0.088`，相关性约 `0.96`。
- pitch-only `ESO_TAUS_K=0.088` 相对同批 baseline 连续两次降低 mean/RMSE/max，但 mean 仍未低于 `0.07 m`。
- roll/yaw 没有足够稳定证据，不继续打开。

## 本轮计划

1. 恢复 `auto_tune_uam_v5_eso.py` 默认输出目录到 `auto_tune_data`。
2. 保留 `record` 目录只作为中文文档目录。
3. 继续 fresh exp4 小范围验证 pitch-only tau_s：
   - `ESO_TAUS_K=0.06`
   - `ESO_TAUS_K=0.08`
   - `ESO_TAUS_K=0.10`
4. 每条 run 生成 `raw_position_error.json`、`run_snapshot.json`、bag、ulog。
5. 对有效候选补跑 `calibrate_tau_s_feedforward.py`，检查物理力矩标尺和 per-axis 拟合是否仍自洽。

## 运行记录

### 1. 路径修正

已恢复调参/验证默认输出路径：

`/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/uav_arm_top/auto_tune_data`

本轮中文文档路径：

`/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/record/tau_s_physics_verification_002.md`

上一轮文档 `tau_s_physics_verification_zh.md` 保留，不删除、不覆盖。

### 2. pitch-only 比例 sweep

输出目录：

`/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/manual_20260428_123610_taus_pitch_sweep`

| 候选 | 参数 | mean (m) | RMSE (m) | max (m) |
|---|---|---:|---:|---:|
| `ff_pitch_k060` | `ESO_TAUS_K=0.06` | 0.08419 | 0.11072 | 0.47268 |
| `ff_pitch_k080` | `ESO_TAUS_K=0.08` | 0.08474 | 0.10750 | 0.41847 |
| `ff_pitch_k100` | `ESO_TAUS_K=0.10` | 0.08615 | 0.10716 | 0.37623 |

观察：

- `K=0.06` 的 mean 最低，但 max 较大。
- `K=0.10` 的 max 最低，但 mean 明显变差。
- `K=0.08` 居中，RMSE/max 比 `K=0.06` 好，但 mean 略差。
- 这说明增大 pitch-only tau_s 比例主要在削峰，不能持续改善平均误差。

### 3. pitch-only 滤波时间常数 sweep

输出目录：

`/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/uav_arm_top/auto_tune_data/manual_20260428_125819_taus_pitch_filter`

| 候选 | 参数 | mean (m) | RMSE (m) | max (m) |
|---|---|---:|---:|---:|
| `ff_pitch_k088_tau005` | `ESO_TAUS_K=0.088`, `ESO_TAUS_TAU=0.05` | 0.08066 | 0.10356 | 0.39317 |
| `ff_pitch_k088_tau020` | `ESO_TAUS_K=0.088`, `ESO_TAUS_TAU=0.20` | 0.08001 | 0.10354 | 0.37538 |

观察：

- `TAU=0.20` 比 `TAU=0.05` 略好，说明更平滑的注入对本轮 exp4 没有坏处。
- 但两者 mean 都仍高于 `0.07 m`，不能作为默认启用依据。
- 相比上一轮 `K=0.088, TAU=0.10` 的最好单次 `0.07938 m`，本轮结果没有突破，只是复现了“有收益但不达标”的趋势。

### 4. 离线物理标定结果

本轮所有有效 run 都补跑了：

`ESO_paper_reproduction/src/uav_control/scripts/calibrate_tau_s_feedforward.py`

报告位置在各 run 的：

`ros_home/log/2026-04-28/tau_s_feedforward_calibration.json`

主要结论：

- 电机力矩重建继续使用 `actuator_outputs -> HIL_ACTUATOR_CONTROLS -> Gazebo motor omega`。
- SDF 与 mixer 的最大 XY 力臂差仍约 `0.066 m`，电机转向一致。
- pitch 轴 `ESO_MAX_TORQUE=2.2` 仍基本可信：body origin 参考下 pitch scale 约 `0.98-1.02`。
- profile CoM 下 pitch 轴 `tau_s_raw -> residual` 有效比例仍稳定在约 `0.087-0.088`，相关性约 `0.965-0.971`。
- roll/yaw 仍没有稳定证据，不应启用。

### 5. 本轮结论

本轮进一步确认：

- `tau_s` 的 pitch 轴物理方向、比例和参考点是自洽的。
- pitch-only 小比例注入确实能降低 RMSE/max，尤其能削峰。
- 但它不能把 exp4 mean 稳定推到 `<0.07 m`，所以还不是最终可默认启用的控制项。

默认参数仍应保持：

- `ESO_TAUS_K=0.0`
- `ESO_DYN_FF_EN=0`
- `ESO_MAX_TORQUE=2.2`

如果继续做，优先方向不是继续单独增大 `ESO_TAUS_K`，而是：

1. 保留 pitch-only `K≈0.088` 作为可选削峰候选。
2. 继续查为什么同一外环 best 在今天的 baseline 附近退到 `0.08-0.09 m`。
3. 如果要冲 `<0.07 m`，应把 pitch-only tau_s 和外环响应微调联合验证，而不是只扫 tau_s。
