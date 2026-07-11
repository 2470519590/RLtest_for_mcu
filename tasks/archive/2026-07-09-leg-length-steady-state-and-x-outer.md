# 2026-07-09：腿长稳态误差与 x 外环接口

## 任务

优先处理 `L_target = 0.35 m` 下虚拟腿长稳态误差，再为固定点控制加入“位置误差生成目标速度 `v_ref`，内层 LQR 跟踪 `v_ref`/平衡”的接口。

## 修改

- 在 `src/robot_smoke/model_smoke.py` 中为 VMC 腿长通道加入：
  - 支撑前馈 `F_ff`。
  - 条件积分 `k_i * I_l`。
  - 积分限幅和饱和防 windup。
  - 腿长力、积分项、关节力矩的 history 诊断字段。
- 新增 `--lqr-x-outer-kp` 和 `--lqr-x-outer-max-v`：
  - `v_ref = clip(-k_x_outer * x, -v_max, v_max)`。
  - 启用外环时，内层 LQR 使用 `dx - v_ref`。
  - 默认 `k_x_outer = 0`，先不默认介入腿长问题。
- 拆出 `--wheel-balance-max-pitch-torque`，允许单独限制 `Tp`。
- 加入 `--virtual-rod-gravity-comp-scale`：
  - 估算 `F_g_est_per_leg = m_total * g / 2`。
  - 当前模型估算 `F_g_est_per_leg ≈ 51.23 N`。
  - 若传入 scale，则用 `scale * F_g_est_per_leg` 覆盖腿长前馈。
- VMC 控制分配改为支撑优先：
  - 先计算 `tau_support = J_l^T F_l`。
  - 再加入危险区分支保护 `tau_guard`。
  - `Tp` 只使用剩余关节力矩余量，按 `scale_theta ∈ [0, 1]` 动态缩放。
- 更新 `docs/CONTROL_THEORY.md` 中腿长 PI/前馈和 x 外环公式。
- 重写 `docs/ERROR_CATALOG.md` 为可读中文，并记录本轮 LQR 工作点错误经验。

## 验证结果

默认 10 秒 smoke：

```text
final_left_length  = 0.326470 m
final_right_length = 0.326470 m
target_length      = 0.35 m
final theta        = -0.00900869 rad
final pitch        = 0.00870207 rad
final x            = -0.512322 m
final x_rate       = 0.106395 m/s
branch violation   = 0
saturated_steps    = 0
max_abs_ctrl       = 0.724033
```

支撑优先分配默认 10 秒 smoke：

```text
final_left_length       = 0.326470 m
left_theta_force_scale  = 1.0
right_theta_force_scale = 1.0
saturated_steps         = 0
```

结论：默认工况没有触发 `Tp` 缩放，说明当前静差不是由 `Tp` 抢到关节限幅直接造成，而是闭环工作点和 LQR/VMC 耦合造成。

重力补偿 scale 实验：

```text
scale_g = 1.0
F_g_comp = 51.23 N
final_left_length = 0.325160 m
```

结论：按静态 `m*g/2` 给前馈没有直接改善腿长；改变前馈会同时改变自动 LQR 的线性化矩阵和 K，不能只按静态重力推断结果。

接触工作点实验：

```text
lqr_design_steps = 8
3 秒内腿长可到约 0.349 ~ 0.350 m
10 秒会进入大角度模态并发散
```

结论：本轮没有彻底解决 0.35 m 腿长稳态；已确认默认稳定路径无饱和且腿型正常，但腿长仍有约 2.35 cm 静差。短时接触工作点能修腿长，但当前 K 长时不稳定，不能作为默认。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

默认 10 秒 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --virtual-rod-steps 10000 --constraint-steps 1200 --check-constraints --history-csv tasks\leg_length_default_final_10s_history.csv --history-plot tasks\leg_length_default_final_10s_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

支撑优先分配验证：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --virtual-rod-steps 10000 --constraint-steps 1200 --check-constraints --history-csv tasks\support_priority_gravity_default_10s_history.csv --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

重力补偿 scale 验证：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --virtual-rod-steps 10000 --virtual-rod-gravity-comp-scale 1.0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1 --history-csv tasks\support_priority_gravity_scale100_10s_history.csv --history-sample-interval 10
```

短时接触工作点实验：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --virtual-rod-steps 3000 --lqr-design-steps 8 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

可视化命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --visualize --visualize-seconds 10
```

## 后续

- 不能继续单纯加 `F_ff` 或 `k_i`，否则会把 LQR 工作点和长时稳定性推坏。
- 下一步应重做接触工作点线性化的输入尺度和 K 约束，特别是 `Tp` 输入列、`lqr_design_steps`、`Q/R` 和长时 rollout。
- x 外环已具备接口，但应在腿长工作点真正稳定后再默认启用。

## 追加：F_l 通道辨识与工作点接口

### 修改

- 新增 `--diagnostics-only`，用于 F_l 通道/pulse 诊断后直接退出，避免普通 smoke 输出混入。
- 新增 `--print-static-operating-point`，在虚拟杆测试末尾打印：
  - `X0 = [theta, dtheta, x, dx, pitch, dpitch]`
  - `U0 = [T, Tp]`
  - `F_l_cmd / F_l_raw / L0 / dL0`
  - `tau_support`
  - `tau_total_before_clip / tau_total_after_clip`
  - `contact_force0`
  - `J0`
  - `q0`
- VMC 诊断新增 `tau_support = J_l^T F_l`，用于区分“前馈没进 VMC”和“前馈进了但工作点没变”。
- LQR 控制律接口改为：

```text
U = U0 - K * (X - X0)
```

默认 `X0=0, U0=0`，保持旧命令行为不变。

### 验证结果

F_l sweep：

```text
F_l=-100: L_mean_tail≈0.109, dL_initial≈-1.342, tau_support=[-14.3, 14.3]
F_l=-50 : L_mean_tail≈0.099, dL_initial≈-0.654, tau_support=[-7.15, 7.15]
F_l=0   : L_mean_tail≈0.139, dL_initial≈0
F_l=50  : L_mean_tail≈0.406, dL_initial≈+0.586, tau_support=[7.15, -7.15]
F_l=100 : L_mean_tail≈0.395, dL_initial≈+1.037, tau_support=[14.3, -14.3]
F_l=150 : L_mean_tail≈0.395, dL_initial≈+1.232, tau_support=[21.45, -21.45]
```

结论：`+F_l` 在初始响应上确实让虚拟腿伸长，符号没有反。大正 `F_l` 会使接触法向力接近 0，说明腿长通道有效，但不能单独代表带接触的平衡工作点。

F_l pulse：

```text
base_force=60N
+20N pulse: dL_mean_during_pulse≈-0.0213
-20N pulse: dL_mean_during_pulse≈+0.0546
```

结论：这个 pulse 是在常数 `F_l=60N` 自由 settle 后的构型上做的，settle 后腿长约 `0.399m`，不是当前 LQR 稳定点 `0.326m`。该结果不能直接推翻 sweep 的符号结论，后续若要做“稳定姿态附近 pulse”，应使用 LQR/工作点 settle 后再注入脉冲。

默认 10 秒 LQR smoke 的末端工作点：

```text
final L≈0.32647 m
X0≈[-0.00904, 0.04351, -0.51222, 0.09863, 0.00876, 0.05623]
U0≈[-3.542, 8.0]
F_l_cmd≈78.60 N
tau_support≈[11.24, -11.24]
tau_total_before_clip≈[2.84, -15.05]
contact_force0≈48.33 N/leg
saturated_steps=0
```

结论：默认末端不是 `X0=0, U0=0` 的静态平衡点，而且 `Tp` 在正限幅。腿长短不是因为 `F_l` 没有进入 VMC，也不是因为 `Tp` 被支撑优先分配削弱。

gravity scale 对比：

```text
scale off: F_l_cmd≈78.60 N, tau_support≈[11.24, -11.24], final L≈0.32647 m
scale=1 : F_l_cmd≈71.01 N, tau_support≈[10.15, -10.15], final L≈0.32516 m
```

结论：重力补偿确实进入了 VMC，并改变了 `tau_support`；但稳定腿长没有改善，说明主问题是整体接触/LQR 工作点，而不是前馈没有接线。

把 `L_ref` 直接改为 `0.32647m` 的 10 秒实验：

```text
final L≈0.34035 m
final theta≈0.123 rad
final x≈-17.03 m
max wheel speed≈56.3 rad/s
自动线性化得到的 Tp 输入列为 0
```

结论：不能只把 `L_ref` 改成当前长度。该目标下局部线性化失去 `Tp` 通道，系统高速漂移。需要一起重构 `X0/U0/F_l0` 和线性化工作点。

用默认末端点作为 `X0/U0` 的接口验证：

```text
X0=[-0.00904064, 0.0435069, -0.512224, 0.0986286, 0.0087583, 0.0562267]
U0=[-3.54216, 8]
```

结果可运行并输出工作点，但长时仍出现位置漂移和 `Tp` 翻转。该实验只证明新接口接通，不能把这个末端点作为合格静态工作点。

### 本轮命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

F_l sweep：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --fl-channel-test --diagnostics-only --fl-channel-steps 1000 --fl-channel-forces -100 -50 0 50 100 150
```

F_l pulse：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --fl-pulse-test --diagnostics-only --fl-pulse-base-force 60 --fl-pulse-delta-force 20 --fl-pulse-settle-steps 1000 --fl-pulse-steps 100
```

默认工作点记录：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --virtual-rod-steps 10000 --print-static-operating-point --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

gravity scale 工作点记录：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --virtual-rod-steps 10000 --virtual-rod-gravity-comp-scale 1.0 --print-static-operating-point --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

可视化命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --visualize --visualize-seconds 10 --print-static-operating-point
```
