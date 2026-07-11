# 2026-07-09：wheel-pd 固定点低频摇动调参

## 任务

用户反馈当前已经可以平衡，但会反复低频大摇动，不能稳定停在固定点。本轮目标是在不改机械 XML、不做训练的前提下，继续优化 wheel-pd 中层和腿部 VMC 阻尼，降低固定点附近的低频摆动。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- wheel-pd 中层新增实验保留参数：
  - `--wheel-balance-direct-x-kp`
  - `--wheel-balance-direct-x-kd`
- wheel-pd 中层新增固定点停车门控阻尼参数：
  - `--wheel-balance-parking-wheel-speed-kd`
  - `--wheel-balance-parking-x-threshold`
  - `--wheel-balance-parking-pitch-threshold`
  - `--wheel-balance-parking-theta-threshold`
- 直接平移项默认关闭，避免与腿部 VMC 饱和耦合后放大摆动。
- 新增相位实验参数：
  - `--virtual-rod-theta-pitch-ff`
  - `--wheel-balance-world-theta-kp`
  - `--wheel-balance-world-theta-kd`
- `theta_pitch_ff` 和世界系 `theta + pitch` 的 `Tp` 阻尼经测试不作为当前默认值。
- 调整默认 wheel-pd 参数：
  - `wheel_balance_pitch_kp = 20`
  - `wheel_balance_pitch_kd = 5`
  - `wheel_balance_x_kp = 2.3`
  - `wheel_balance_x_kd = 2.2`
  - `wheel_balance_wheel_speed_kd = 0.05`
  - `wheel_balance_parking_wheel_speed_kd = 0.04`
  - `wheel_balance_max_pitch_target = 0.2 rad`
- 调整腿部 VMC 阻尼默认值：
  - `virtual_rod_theta_kp = 50`
  - `virtual_rod_theta_kd = 10`
  - `virtual_rod_joint_kd = 8`
- 更新 `docs/CONTROL_THEORY.md`，只记录固定点阻尼控制律和默认参数。

## 对比结论

基线 2 秒表现仍存在明显低频漂移和摇摆：

```text
tail x RMS        ≈ 0.3707 m
tail x_rate RMS   ≈ 1.2736 m/s
tail wheel RMS    ≈ 14.9016 rad/s
final x           ≈ 0.6261 m
final x_rate      ≈ 1.1138 m/s
```

当前默认 4 秒表现：

```text
max_left_branch_violation  = 0
max_right_branch_violation = 0
final x                    = 0.0405218 m
final x_rate               = -0.0508983 m/s
final pitch                = -0.258986 rad
final theta                = 0.255011 rad
final wheel speed          = -12.9227 rad/s
```

当前默认参数显著减小了固定点水平漂移和水平速度，但仍未做到完全静止；尾段仍有 pitch/theta 互相耦合的小幅摆动，腿部 actuator 仍长时间饱和。

加入停车门控阻尼后的 4 秒结果：

```text
max_left_branch_violation  = 0
max_right_branch_violation = 0
final x                    = 0.0592036 m
final x_rate               = 0.0323599 m/s
final pitch                = -0.25536 rad
final theta                = 0.251054 rad
final wheel speed          = -1.94254 rad/s
```

6 秒结果：

```text
max_left_branch_violation  = 0
max_right_branch_violation = 0
final x                    = -0.13951 m
final x_rate               = -0.153703 m/s
final pitch                = 0.228249 rad
final theta                = -0.224447 rad
final wheel speed          = -0.552546 rad/s
```

结论：停车门控阻尼能显著降低末端轮速，但尾段 RMS 没有全面优于无停车阻尼版本。当前主要矛盾已经从“轮子停不住”转为 `pitch/theta` 相位耦合，后续不应继续单纯增大轮速阻尼。

相位调参结论：

```text
theta_pitch_ff = +1.0：方向错误，位置明显漂移。
theta_pitch_ff = -0.5：短时有效，但 4 秒会推走固定点。
world_theta_kp/kd = 10/2：能注入小幅 Tp，但由于 theta + pitch 本来已经接近 0，收益有限。
virtual_rod_theta_kp/kd/joint_kd = 50/10/8：当前更有效，末端轮速接近 0。
```

当前默认 6 秒相位调参结果：

```text
max_left_branch_violation  = 0
max_right_branch_violation = 0
final x                    = -0.127093 m
final x_rate               = -0.200105 m/s
final pitch                = 0.210369 rad
final theta                = -0.205198 rad
final wheel speed          = -0.0555699 rad/s
```

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

默认 4 秒固定点 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 4000 --constraint-steps 4000 --check-constraints --history-csv tasks\wheel_pd_stationary_default_4s_history.csv --history-plot tasks\wheel_pd_stationary_default_4s_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

停车门控 6 秒 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 6000 --constraint-steps 6000 --check-constraints --history-csv tasks\wheel_pd_parking_default_6s_history.csv --history-plot tasks\wheel_pd_parking_default_6s_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

当前默认相位调参 6 秒 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 6000 --constraint-steps 6000 --check-constraints --history-csv tasks\wheel_pd_phase_default_6s_history.csv --history-plot tasks\wheel_pd_phase_default_6s_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 输出文件

- `tasks/wheel_pd_stationary_default_4s_history.csv`
- `tasks/wheel_pd_stationary_default_4s_history.png`
- `tasks/wheel_pd_parking_default_6s_history.csv`
- `tasks/wheel_pd_parking_default_6s_history.png`
- `tasks/wheel_pd_phase_default_6s_history.csv`
- `tasks/wheel_pd_phase_default_6s_history.png`

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --visualize --visualize-seconds 10
```

## 后续风险

- 腿部 VMC 仍长时间饱和，说明低层腿长/腿角保持能力仍是固定点静止的主要瓶颈。
- 当前默认参数优先压低水平漂移和大幅低频摇动；如果继续追求轮速归零，需要进一步处理腿部饱和和 pitch/theta 的能量交换。
