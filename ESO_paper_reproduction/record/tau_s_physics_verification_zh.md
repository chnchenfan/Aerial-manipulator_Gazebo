# tau_s 动力学前馈物理标定与收益验证日志

记录日期：2026-04-28

## 1. 本次做了什么

1. 修正了离线力矩重建链路。
   - 修改脚本：`ESO_paper_reproduction/src/uav_control/scripts/calibrate_tau_s_feedforward.py`
   - 原来主要依赖 `actuator_outputs` 的 PWM 近似映射。
   - 现在优先使用真实 SITL 链路：`actuator_outputs -> HIL_ACTUATOR_CONTROLS -> Gazebo mavlink_interface -> motor omega`。
   - SDF 中使用的关键映射是：`input_scaling=1000`、`zero_position_armed=100`、`motorConstant=5.84e-06`、`momentConstant=0.06`。

2. 增加了 PX4 mixer 与 SDF 几何对照。
   - 相关文件：
     - `ROMFS/px4fmu_common/mixers/quad_w.main.mix`
     - `src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml`
     - `Tools/sitl_gazebo/models/uav_arm_v4/uav_arm_v4.sdf`
   - 结果：电机转向一致；SDF 与 mixer 的最大 XY 力臂差约 `0.066 m`。
   - 解释：控制器/mixer 认为的机架几何和 Gazebo 实际施加力矩的几何不是完全一致。

3. 用 fresh exp4 做了两组同批对照。
   - 基线：`ESO_TAUS_K=0`
   - 候选：pitch-only，`ESO_TAUS_K=0.088`、`ESO_TAUS_K_R=0`、`ESO_TAUS_K_P=1`、`ESO_TAUS_K_Y=0`
   - 当前 best 参数和 skill memory 没有修改。

## 2. 结果路径

所有本次验证记录已移动到：

`/home/cf/PX4_Firmware_clean/ESO_paper_reproduction/record`

具体目录：

- `manual_20260428_105455_taus_physics_verify`：第一次尝试，因 sandbox 限制导致 MAVROS XML-RPC 启动失败，不作为控制效果证据。
- `manual_20260428_105558_taus_physics_verify`：第一组有效 fresh exp4 对照。
- `manual_20260428_110640_taus_physics_verify_repeat2`：第二组有效 fresh exp4 对照。

每条有效 run 内保留：

- `raw_position_error.json`：轨迹误差指标。
- `run_snapshot.json`：参数、bag 路径、实验配置快照。
- `mavparam_set.log`：实际写入 PX4 的参数日志。
- `tau_s_feedforward_calibration.json`：离线力矩标定报告。
- `.bag` 和 `.ulg`：ROS/PX4 原始日志。

## 3. fresh exp4 指标

| 组别 | 参数 | mean (m) | RMSE (m) | max (m) |
|---|---|---:|---:|---:|
| 第一组 baseline | `ESO_TAUS_K=0` | 0.08965 | 0.11995 | 0.52154 |
| 第一组 pitch-only | `ESO_TAUS_K=0.088` | 0.07938 | 0.10198 | 0.38804 |
| 第二组 baseline | `ESO_TAUS_K=0` | 0.08981 | 0.11672 | 0.50650 |
| 第二组 pitch-only | `ESO_TAUS_K=0.088` | 0.08618 | 0.10692 | 0.33398 |

结论：

- pitch-only 前馈连续两次相对同批 baseline 有收益。
- 收益主要体现在 `mean`、`RMSE` 和 `max` 都下降，尤其是最大误差下降明显。
- 但两次 mean 都没有低于 `0.07 m`，所以还不能写入默认参数。

## 4. 力矩标尺结论

用 body origin 作为参考点重建 `tau_actual` 后，`ESO_MAX_TORQUE` 分轴拟合大致为：

- pitch：约 `0.98-1.00`，相关性约 `0.99`，说明 `ESO_MAX_TORQUE=2.2` 在 pitch 轴基本正确。
- roll：约 `0.63-0.66`，说明 roll 轴物理尺度还不完全匹配。
- yaw：约 `0.29-0.34`，说明 yaw 轴不适合用当前同一套标尺直接解释。

所以当前不能说全轴动力学前馈都正确；只能说 pitch 轴的物理标尺比较可信。

## 5. 参考点与有效比例结论

离线拟合 `actual_minus_eso_torque ~= scale * tau_s_raw + bias` 后，稳定证据出现在 profile CoM 的 pitch 轴：

- 第一组 baseline：pitch scale `0.08789`，相关性 `0.96278`
- 第一组 pitch-only：pitch scale `0.08882`，相关性 `0.96644`
- 第二组 baseline：pitch scale `0.08813`，相关性 `0.96090`
- 第二组 pitch-only：pitch scale `0.08841`，相关性 `0.96251`

这说明：

- pitch 轴 `tau_s_raw` 和实际需要补偿的残差高度相关。
- 有效比例约为 `0.088`，不是 1.0。
- roll/yaw 没有同等级别的稳定证据，因此不应该打开。

## 6. 为什么这样做

不能直接相信模型算出来的 `tau_s`，原因是模型前馈从仿真走到实物会经过多层误差：

- 电机指令到转速、推力的标尺可能不准。
- PX4 mixer 的机架几何和 Gazebo/实物的真实电机位置可能不一致。
- `tau = r x mg` 对参考点非常敏感，参考点错会直接造成力矩符号或量级错误。
- 机械臂和载荷会改变系统 CoM，而真实系统还会有柔性、延迟、摩擦和电机动态。
- 闭环控制器会把一部分模型误差吸收到反馈项和 ESO 扰动估计里，所以模型力矩不能全量注入。

## 7. 和实物的联系

- SDF 的 `motorConstant`、`momentConstant` 对应实物里的电机/桨推力和反扭矩标定。
- SDF 转子位置和 PX4 mixer 几何对照，对应实物里实际量尺测出来的四个电机坐标。
- `profile_com` 和参考点拟合，对应实物里机械臂、夹爪、载荷相对机体原点和推力中心的位置。
- per-axis scale 约 `0.088`，对应实物中模型只解释了一部分真实扰动，剩下部分来自电机动态、结构柔性、CoM 误差和控制器闭环耦合。

## 8. 最终结论

当前 `tau_s` 动力学前馈的结论是：

**pitch 轴物理标尺基本正确、参考点证据稳定、小比例注入有收益；但还没有达到最终默认启用标准。**

默认参数应继续保持：

- `ESO_TAUS_K=0.0`
- `ESO_DYN_FF_EN=0`
- `ESO_MAX_TORQUE=2.2`

下一步如果继续做，应只围绕 pitch 轴做更小范围验证，例如 `ESO_TAUS_K=0.06-0.10`，并要求至少两次 fresh exp4 同时优于同批 baseline 且 mean 接近或低于 `0.07 m`。
