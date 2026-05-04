# tau_s 动力学前馈验证记录 003

## 本轮目标

- 将动力学前馈调优流程沉淀为独立 Codex skill，降低后续上下文 token 消耗。
- 固化 `tau_s` 物理标尺、参考点、per-axis 比例/符号、滞后/滤波、闭环 fresh 验证的证据链。
- 明确采用“Codex 主导 + 调优算法辅助”的工作方式：先离线物理拟合缩小范围，再用 Optuna TPE 做少量 fresh 验证。

## 已完成

- 新建专用 skill：`/home/cf/.codex/skills/uam-v5-dyn-ff-tuning`。
- 新增 `SKILL.md`，记录固定路径、不可破坏规则、证据标准、默认 pitch-only 策略和与 `uam-v5-eso-autotune` 的协作关系。
- 新增 `references/dyn_ff_workflow.md`，记录动力学前馈验证流程：
  - 电机/力矩物理标尺；
  - mixer 与 SDF 几何对照；
  - `tau_s` 参考点比较；
  - per-axis 比例、符号与 lag 拟合；
  - same-batch `K=0` baseline 对比和 fresh exp4 接受标准。
- 新增 `references/code_map.md`，记录 `tau_s` 从姿态环计算、发布、速率环接收、限幅/缩放/低通、进入 ESO 和控制力矩的代码路径。
- 新增 `references/algorithms.md`，记录调优算法策略：
  - 默认使用离线拟合 + Optuna TPE 分阶段贝叶斯；
  - 小网格只用于确认；
  - CMA-ES 暂不作为默认；
  - 候选必须优于同批 `K=0` baseline。
- 新增 `assets/report_template_zh.md`，作为后续编号中文记录模板。
- 新增 `agents/openai.yaml`，便于界面展示新 skill。

## 固化规则

- 原始调参/验证输出仍保持在：
  `/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/uav_arm_top/auto_tune_data`
- 动力学前馈过程文档放在：
  `/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/record`
- 旧文档不删除、不覆盖；后续每轮新聊天/新阶段创建新编号文档。
- 默认基线继续保持：
  - `ESO_TAUS_K=0.0`
  - `ESO_DYN_FF_EN=0`
  - `ESO_MAX_TORQUE=2.2`
- 在 fresh 证据满足标准之前，不把动力学前馈写成默认打开。

## 后续代码方向

- 当前 `tau_s` 结构仍是耦合的：
  - `u_nominal = ... + tau_s_used`
  - `torque_physical = ... - tau_s_used`
- 下一步优先改造为 ESO 残差分解结构：
  - `tau_s_obs`：给 ESO nominal input，用于告诉观测器“这部分是已建模扰动”；
  - `tau_s_ctrl`：给最终控制力矩，用于保守地直接补偿；
  - 先只做 pitch-only，roll/yaw 保持关闭。

## 实物联系

- 电机/桨/ESC 标尺：仿真中用 Gazebo motor model 和 actuator output 重建电机力矩；实物中对应电机台架或飞行日志反推。
- 机架几何：仿真中比较 SDF 转子位置和 PX4 mixer 几何；实物中对应真实电机安装位置和飞控认为的布局是否一致。
- CoM/机械臂姿态：`tau_s = r x mg` 对参考点非常敏感；实物中对应机械臂负载重心、飞控坐标原点和推力中心的定义。
- ESO 残差：`tau_s` 只补偿可建模重力力矩，剩余柔性、延迟、推力曲线误差、摩擦和未建模耦合交给 ESO 估计。

## 本轮结论

- 已完成动力学前馈专用 skill 的落地。
- 该 skill 将后续工作约束为：先物理验证，再算法辅助，最后 fresh 闭环验证。
- 当前没有修改 PX4 控制代码、默认参数或 skill memory。

## 2026-04-28 追加：tau_s obs/ctrl 拆分代码落地

### 做了什么

- 在速率控制器中把原来的单一路径 `tau_s_used` 拆成两路：
  - `tau_s_obs`：进入 ESO nominal input；
  - `tau_s_ctrl`：进入最终控制力矩。
- 保留原有安全链路：
  - 有限性保护；
  - `ESO_TAUS_LIM` 单轴限幅；
  - `ESO_TAUS_K` 主开关；
  - `ESO_TAUS_K_R/P/Y` 共同分轴启用/符号；
  - `ESO_TAUS_TAU` 一阶低通。
- 新增 PX4 参数：
  - `ESO_TAUS_OBS_R`
  - `ESO_TAUS_OBS_P`
  - `ESO_TAUS_OBS_Y`
  - `ESO_TAUS_CTL_R`
  - `ESO_TAUS_CTL_P`
  - `ESO_TAUS_CTL_Y`
- 新参数默认都是 `1.0`，因此：
  - 当前默认 `ESO_TAUS_K=0.0` 时行为仍然关闭；
  - 如果沿用旧的非零 `ESO_TAUS_K` 实验参数，默认也等效于旧的耦合路径。

### 改动路径

- `src/modules/eso_rate_control/ESORateControl/RateControl.cpp`
  - 原来：`u_nominal = ... + tau_s_used`
  - 原来：`torque_physical = ... - tau_s_used`
  - 现在：`u_nominal = ... + tau_s_obs`
  - 现在：`torque_physical = ... - tau_s_ctrl`
- `src/modules/eso_rate_control/ESORateControl/RateControl.hpp`
  - 新增 observer/control 两组分轴比例缓存和 setter。
- `src/modules/eso_rate_control/ESOMulticopterRateControl.cpp`
  - 参数更新时把新参数传入 `ESORateControl`。
- `src/modules/eso_rate_control/ESOMulticopterRateControl.hpp`
  - 新增 6 个参数句柄。
- `src/modules/eso_rate_control/eso_rate_control_params.c`
  - 新增 6 个 PX4 参数定义。

### 为什么这么做

- 之前 `tau_s_used` 同时给 ESO 和控制器使用，会把“观测器知道的模型扰动”和“控制器实际打到电机的前馈补偿”绑定在一起。
- 这会导致一个问题：如果模型前馈幅值、参考点或相位只有部分正确，ESO 和控制器可能互相补偿同一项，甚至互相打架。
- 拆分后可以让 ESO 较充分地知道“可建模重力扰动”，同时让控制器只保守地直接补偿一部分。
- 这更符合当前假设：
  `真实扰动 = tau_s 可建模部分 + ESO residual 未建模部分`

### 和实物的联系

- `tau_s_obs` 对应“控制系统知道负载重心会产生某个重力力矩”，类似把机械臂/负载模型告诉观测器。
- `tau_s_ctrl` 对应“真正打到电机上的前馈力矩”，实物中需要更保守，因为电机/桨/ESC、延迟、结构柔性和推力曲线都不可能完全等于模型。
- 分开后，后续可以让 `ESO_TAUS_OBS_P` 接近离线拟合出的模型比例，而让 `ESO_TAUS_CTL_P` 从更小值开始，降低过补偿风险。

### 验证

- 已执行编译：
  `ninja -C build/px4_sitl_default modules__eso_rate_control px4`
- 编译通过。

### 下一步

- 先不打开默认前馈，继续保持：
  - `ESO_TAUS_K=0.0`
  - `ESO_DYN_FF_EN=0`
  - `ESO_MAX_TORQUE=2.2`
- 下一轮 fresh 验证建议只做 pitch-only：
  - `ESO_TAUS_K_R=0`
  - `ESO_TAUS_K_P=1`
  - `ESO_TAUS_K_Y=0`
  - `ESO_TAUS_OBS_P` 在离线拟合比例附近；
  - `ESO_TAUS_CTL_P` 从更小值开始；
  - roll/yaw 的 obs/ctl 参数保持不会实际生效。

## 2026-04-28 追加：pitch obs/ctrl split fresh exp4 验证

### 候选设置

- baseline：当前 best 外环参数，`ESO_TAUS_K=0.0`。
- 候选：`pitch_obs088_ctl044_tau020`
  - `ESO_TAUS_K=0.088`
  - `ESO_TAUS_TAU=0.20`
  - `ESO_TAUS_K_R=0.0`
  - `ESO_TAUS_K_P=1.0`
  - `ESO_TAUS_K_Y=0.0`
  - `ESO_TAUS_OBS_P=1.0`
  - `ESO_TAUS_CTL_P=0.5`
- 物理含义：
  - pitch 轴的 ESO nominal input 看到 `0.088 * tau_s_raw_pitch`；
  - pitch 轴的控制力矩只直接扣除 `0.044 * tau_s_raw_pitch`；
  - roll/yaw 不实际注入。

### fresh exp4 结果

| run dir | candidate | mean m | RMSE m | max m | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `manual_20260428_161254_taus_obs_ctrl_split` | `k0_baseline_after_split` | 0.08632 | 0.11417 | 0.50855 | 同批 baseline |
| `manual_20260428_161254_taus_obs_ctrl_split` | `pitch_obs088_ctl044_tau020` | 0.08358 | 0.10663 | 0.41038 | mean/RMSE/max 均改善 |
| `manual_20260428_161859_taus_obs_ctrl_split` | `k0_baseline_after_split` | 0.08208 | 0.11357 | 0.52570 | 同批 baseline |
| `manual_20260428_161859_taus_obs_ctrl_split` | `pitch_obs088_ctl044_tau020` | 0.09270 | 0.11496 | 0.43980 | max 改善，但 mean/RMSE 变差 |

### 离线标定结果

- 四条 run 均已生成 `tau_s_feedforward_calibration.json`。
- 电机力矩重建来源均为：
  `actuator_outputs_hil_to_gazebo_motor_reference`
- mixer/SDF 几何差异仍存在：
  - 最大三维位置差约 `0.225 m`
  - 最大 XY 力臂差约 `0.066 m`
  - 转向一致
- pitch 轴 `ESO_MAX_TORQUE` 物理标尺仍稳定：
  - body origin pitch scale 约 `0.981-0.992`
  - pitch 相关性约 `0.984-0.989`
- profile CoM 下的 pitch residual 拟合仍稳定：
  - scale 约 `0.0845-0.0888`
  - 相关性约 `0.961-0.968`
  - 最优滞后约 `5` 个样本

### 本轮判断

- obs/ctrl 拆分后的 pitch-only 候选具有稳定削峰能力：两次 fresh exp4 的 max 均明显低于同批 baseline。
- 但 mean 不稳定：第一次改善，第二次变差。
- 因此它不能写入默认值，也不能判定为“正确且有稳定收益”。
- 更准确的结论是：
  `tau_s` 的 pitch 物理标尺和 residual 相关性可信，obs/ctrl split 结构方向成立，但当前比例/滤波还只达到“削峰有效、平均误差收益不足”。

### 下一步建议

- 不要增大直接控制补偿，`ESO_TAUS_CTL_P=0.5` 已经出现 mean 不稳定。
- 下一轮优先测试更保守的控制路径：
  - `ESO_TAUS_K=0.088`
  - `ESO_TAUS_TAU=0.20`
  - `ESO_TAUS_OBS_P=1.0`
  - `ESO_TAUS_CTL_P=0.25` 或 `0.0`
- 如果 `CTL_P=0.0` 仍能降低 disturbance/residual，但不改善 mean，说明 `tau_s` 更适合做 ESO nominal model，而不是直接控制前馈。
- 默认参数继续关闭。

## 2026-04-28 追加：更保守 CTL_P 扫描

### 候选设置

- baseline：当前 best 外环参数，`ESO_TAUS_K=0.0`。
- `pitch_obs088_ctl022_tau020`：
  - `ESO_TAUS_K=0.088`
  - `ESO_TAUS_TAU=0.20`
  - `ESO_TAUS_K_R=0.0`
  - `ESO_TAUS_K_P=1.0`
  - `ESO_TAUS_K_Y=0.0`
  - `ESO_TAUS_OBS_P=1.0`
  - `ESO_TAUS_CTL_P=0.25`
- `pitch_obs088_ctl000_tau020`：
  - `ESO_TAUS_K=0.088`
  - `ESO_TAUS_TAU=0.20`
  - `ESO_TAUS_K_R=0.0`
  - `ESO_TAUS_K_P=1.0`
  - `ESO_TAUS_K_Y=0.0`
  - `ESO_TAUS_OBS_P=1.0`
  - `ESO_TAUS_CTL_P=0.0`

### fresh exp4 结果

| run dir | candidate | mean m | RMSE m | max m | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `manual_20260428_163202_taus_obs_ctrl_conservative` | `k0_baseline_after_split` | 0.08667 | 0.11599 | 0.48180 | 同批 baseline |
| `manual_20260428_163202_taus_obs_ctrl_conservative` | `pitch_obs088_ctl022_tau020` | 0.08314 | 0.10854 | 0.42405 | mean/RMSE/max 均改善 |
| `manual_20260428_163202_taus_obs_ctrl_conservative` | `pitch_obs088_ctl000_tau020` | 0.08746 | 0.11450 | 0.45580 | 只改善 RMSE/max，mean 略差 |

### 离线标定结果

- 三条 run 均已生成 `tau_s_feedforward_calibration.json`。
- `pitch_obs088_ctl022_tau020` 中：
  - body origin pitch `ESO_MAX_TORQUE` scale = `0.98936`
  - pitch 相关性 = `0.98679`
  - profile CoM pitch residual scale = `0.08940`
  - profile CoM pitch residual 相关性 = `0.96787`
  - 最优滞后 = `5` 个样本
- `pitch_obs088_ctl000_tau020` 中：
  - body origin pitch `ESO_MAX_TORQUE` scale = `1.01392`
  - pitch 相关性 = `0.99551`
  - profile CoM pitch residual scale = `0.08768`
  - profile CoM pitch residual 相关性 = `0.96560`
  - 最优滞后 = `5` 个样本

### 本轮判断

- `CTL_P=0.25` 比 `CTL_P=0.5` 更稳，本批 fresh exp4 对 mean/RMSE/max 都有收益。
- `CTL_P=0.0` 说明“只让 ESO nominal model 知道 tau_s、不直接补偿控制力矩”不足以改善 mean；直接控制路径仍需要一个小比例。
- 当前最有希望的候选是：
  `ESO_TAUS_K=0.088, ESO_TAUS_TAU=0.20, ESO_TAUS_OBS_P=1.0, ESO_TAUS_CTL_P=0.25`
- 但该候选目前只有一次 fresh 改善，还未达到“两次 fresh exp4 稳定优于同批 K=0 baseline”的接受标准。
- 默认参数继续保持关闭。

### 下一步建议

- 对 `pitch_obs088_ctl022_tau020` 做至少一次同批复验。
- 若复验仍改善 mean/RMSE/max，再考虑围绕 `CTL_P=0.20-0.35` 做小范围 Optuna/TPE 或网格确认。
- 若复验不稳，说明主要收益仍是削峰，不能作为默认前馈打开。

## 2026-04-28 追加：`CTL_P=0.25` 同批复验

### fresh exp4 复验结果

| run dir | candidate | mean m | RMSE m | max m | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `manual_20260428_164430_taus_obs_ctrl_conservative` | `k0_baseline_after_split` | 0.08373 | 0.11653 | 0.55429 | 同批 baseline |
| `manual_20260428_164430_taus_obs_ctrl_conservative` | `pitch_obs088_ctl022_tau020` | 0.08337 | 0.10669 | 0.41546 | mean/RMSE/max 均改善 |
| `manual_20260428_164430_taus_obs_ctrl_conservative` | `pitch_obs088_ctl000_tau020` | 0.08084 | 0.10607 | 0.41800 | 本批也改善，但前一批 mean 不稳 |

### 离线标定结果

- 三条 run 均已生成 `tau_s_feedforward_calibration.json`。
- 电机力矩重建来源均为：
  `actuator_outputs_hil_to_gazebo_motor_reference`
- mixer/SDF 几何差异保持不变：
  - 最大三维位置差约 `0.225 m`
  - 最大 XY 力臂差约 `0.066 m`
  - 转向一致
- pitch 轴 `ESO_MAX_TORQUE` 物理标尺仍稳定：
  - body origin pitch scale 约 `0.9827-1.0137`
  - pitch 相关性约 `0.9838-0.9940`
- profile CoM 下的 pitch residual 拟合仍稳定：
  - scale 约 `0.0865-0.0874`
  - 相关性约 `0.9643-0.9684`
  - 最优滞后仍为 `5` 个样本

### 当前判断

- `pitch_obs088_ctl022_tau020` 已完成两次 fresh exp4 同批复验，且两次都相对各自 `K=0` baseline 改善 mean/RMSE/max。
- 该候选仍只应视为“可进入更小范围确认”的候选，不应直接写成默认打开；原因是第二次 mean 改善幅度很小，主要收益仍集中在 RMSE 和 max 削峰。
- `pitch_obs088_ctl000_tau020` 本批表现最好，但上一批 mean 略差，因此不能替代 `CTL_P=0.25` 作为当前首选。
- 当前首选继续保持：
  `ESO_TAUS_K=0.088, ESO_TAUS_TAU=0.20, ESO_TAUS_K_R=0, ESO_TAUS_K_P=1, ESO_TAUS_K_Y=0, ESO_TAUS_OBS_P=1.0, ESO_TAUS_CTL_P=0.25`
- 默认参数继续关闭。

### 下一步建议

- 围绕 `ESO_TAUS_CTL_P=0.0-0.25` 做更小范围确认，优先比较 `CTL_P=0.0/0.125/0.25`。
- 若 `CTL_P=0.0` 再次稳定改善，说明 `tau_s` 更适合主要进入 ESO nominal model，直接控制前馈可以进一步减小。
- 若 `CTL_P=0.125/0.25` 更稳，则保持小比例直接控制路径。

## 2026-04-28 追加：三轴策略与 exp4/exp1 状态

### 是否纳入三轴

- 当前不建议同时纳入 roll/pitch/yaw。
- pitch 轴证据最稳定：
  - profile CoM residual scale 长期在约 `0.086-0.089` 附近；
  - 相关性多次达到约 `0.96-0.97`；
  - 多数有效批次最优 lag 为 `5` 个样本。
- roll 轴目前只在个别批次出现中等相关性，符号/lag 不如 pitch 稳定。
- yaw 轴重建仍受电机反扭矩和混控几何影响，相关性和符号不满足闭环启用标准。
- 结论：继续 pitch-only；roll 需要单独离线证据稳定后再进入，yaw 继续关闭。

### 当前是否都在做 exp4

- 最近所有有效 closed-loop fresh 对比确实都是：
  `exp4_square_tracking_uam_v5`
- 原因是 tau_s 重力扰动和机械臂运动耦合在 exp4 中更明显，适合先筛 pitch-only 候选。
- 但候选不能只靠 exp4 接受；后续必须补 `exp1_hover_disturbance_uam_v5`，确认 hover disturbance 下不劣化。

### exp4 小确认结果

| run dir | candidate | mean m | RMSE m | max m | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `manual_20260428_170713_taus_obs_ctrl_confirm` | `k0_baseline_after_split` | 0.08222 | 0.11412 | 0.53317 | 同批 baseline |
| `manual_20260428_170713_taus_obs_ctrl_confirm` | `pitch_obs088_ctl000_tau020` | 0.08157 | 0.10607 | 0.39551 | mean/RMSE/max 均改善 |
| `manual_20260428_170713_taus_obs_ctrl_confirm` | `pitch_obs088_ctl011_tau020` | - | - | - | 自动化等待 `/experiment/arm_motion_enabled=true` 超时，无效 |

### exp4 离线标定补充

- `k0_baseline_after_split`：
  - body origin pitch `ESO_MAX_TORQUE` scale = `0.98822`
  - pitch 相关性 = `0.98810`
  - profile CoM pitch residual scale = `0.08799`
  - profile CoM pitch residual 相关性 = `0.96756`
  - lag = `5`
- `pitch_obs088_ctl000_tau020`：
  - body origin pitch `ESO_MAX_TORQUE` scale = `0.84592`
  - pitch 相关性 = `0.88831`
  - profile CoM pitch residual scale = `0.09569`
  - profile CoM pitch residual 相关性 = `0.85559`
  - lag = `-2`
- 判断：
  - `CTL_P=0.0` 在这批 exp4 闭环指标再次改善，但离线物理自洽性弱于前几批，不能替代 `CTL_P=0.25`。
  - `CTL_P=0.125` 和本批后续 `CTL_P=0.25` 未得到有效自动化结果，需要重跑。

### exp1 尝试

- 尝试运行：
  `manual_20260428_171839_taus_obs_ctrl_exp1_validate`
- baseline 在等待 `/experiment/arm_motion_enabled=true` 时超时，脚本未生成标准 `raw_position_error.json`。
- 虽然 recorder 保存了 bag，但手工诊断 metrics 起点来自 `bag_start`，mean 达到异常的 `93 m`，说明该 bag 不满足标准分析窗口，不能作为 fresh pass 证据。
- 结论：目前还没有有效 exp1 证据；下一步应先修复/确认 exp1 自动化 enable 条件，再做 baseline + pitch-only 候选对比。
