# 2026-07-09：静态接触平衡工作点搜索

## 任务

暂停继续调 LQR 权重或控制增益，先新增独立 `equilibrium search / static contact balance search`。目标是寻找满足静态接触平衡、无持续漂移、无长期限幅、接触稳定的 true equilibrium，再用于后续线性化和 LQR。

## 清理

已删除 `tasks/` 下历史残留的 `.csv`、`.png`、`.jpg`、`.jpeg` 实验输出文件，只保留 markdown 任务记录。

## 修改

- `run_smoke.py` 新增 `--equilibrium-search` 诊断模式。
- 搜索模式不启用完整 LQR，不追全局 `x=0`。
- 每个候选使用：

```text
F_l = F_l0 + k_l * (L_ref - L) - d_l * dL
Tp  = weak theta/pitch damping + Tp_bias
T   = wheel damping + optional weak pitch-to-wheel term
```

- 默认扫描：

```text
L_ref = [0.326, 0.335, 0.345, 0.35]
F_l0_scale = [0.8, 1.0, 1.2, 1.5]
```

- 可选扫描：

```text
--equilibrium-tp-biases
--equilibrium-wheel-dampings
--equilibrium-wheel-pitch-kps
--equilibrium-wheel-pitch-kds
```

- 每组输出：

```text
L_mean
phi_mean
theta_mean
dx_RMS
dphi_RMS
dtheta_RMS
dL_RMS
T_mean
Tp_mean
F_l0
T_sat_ratio
Tp_sat_ratio
joint_sat_ratio
contact_force_mean
contact_force_min
slip_indicator
qualified
```

其中 `joint_sat_ratio` 当前表示腿部或轮端任一 actuator 饱和的比例。

## 合格标准

当前门禁阈值：

```text
abs(phi_mean)   < 0.08 rad
abs(theta_mean) < 0.08 rad
dx_RMS          < 0.05 m/s
dphi_RMS        < 0.08 rad/s
dtheta_RMS      < 0.08 rad/s
dL_RMS          < 0.015 m/s
T_sat_ratio     <= 0.01
Tp_sat_ratio    <= 0.01
joint_sat_ratio <= 0.01
contact_min     > 5 N
slip_indicator  < 0.15
```

全局 `x` 不作为合格标准，因为平地轮腿机器人具有平移对称性。

## 验证结果

默认 16 组扫描，3 秒 rollout、末 1 秒评估：

```text
没有候选满足 equilibrium 标准。
低 F_l0_scale 组：速度/RMS 很小、接触稳定、不饱和，但 phi≈-0.60~-0.71 rad，theta≈0.61~0.71 rad。
高 F_l0_scale 或较高 L_ref 组：可能离地，contact_force_min=0。
best by score: L_ref=0.326, F_l0_scale=0.8, qualified=False。
```

该 best candidate：

```text
L_mean≈0.282 m
phi_mean≈-0.600 rad
theta_mean≈0.607 rad
dx_RMS≈2.5e-6
contact_force_mean≈27 N
contact_force_min≈27 N
T_sat_ratio=0
Tp_sat_ratio=0
joint_sat_ratio=0
```

结论：该点是速度近零的被动接触构型，但不是 upright equilibrium。由于 `phi` 和 `theta` 都是大角度，不能用于 LQR 线性化。

`Tp_bias = [-1, 0, 1]` 小范围扫描：

```text
未显著改变大角度被动接触点。
phi/theta 仍约为 -0.60/0.61 到 -0.70/0.71 rad。
```

wheel pitch-to-wheel 小范围符号扫描：

```text
wheel_pitch_kp = [-4, 4]
wheel_pitch_kd = [-0.8, 0.8]
```

结果：

```text
能产生 T≈±2.3~2.6 N*m；
接触仍稳定；
但 phi/theta 仍保持大角度；
没有合格 upright equilibrium。
```

## 当前结论

当前实现已完成 equilibrium search 模式，但尚未找到 true equilibrium。

不能继续执行：

```text
equilibrium -> linearization -> LQR
```

因为第一步还没有合格工作点。当前搜索找到的只是：

```text
quasi-static fallen contact configuration
```

不是：

```text
upright static contact equilibrium
```

后续应继续改进工作点搜索策略，例如更合理的轮端倒立摆临时控制、初态/IK 初始化、或直接求解约束静力平衡，但仍不能把 LQR-settle 末端点当作线性化工作点。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

默认 equilibrium search：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-steps 3000 --equilibrium-eval-steps 1000 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

Tp bias 诊断：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-steps 2500 --equilibrium-eval-steps 800 --equilibrium-l-refs 0.326 0.335 0.345 --equilibrium-fl0-scales 0.8 1.0 --equilibrium-tp-biases -1 0 1 --equilibrium-wheel-dampings 0.12 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

wheel pitch-to-wheel 诊断：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-steps 2500 --equilibrium-eval-steps 800 --equilibrium-l-refs 0.326 0.335 --equilibrium-fl0-scales 0.8 1.0 --equilibrium-tp-biases 0 --equilibrium-wheel-dampings 0.12 --equilibrium-wheel-pitch-kps -4 4 --equilibrium-wheel-pitch-kds -0.8 0.8 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

