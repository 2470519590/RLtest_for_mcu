# 2026-07-09：整体 LQR + VMC 平衡框架重构

## 任务

用户要求停止在原有分层 PD 参数上继续小修小补，将平衡控制框架从：

```text
轮端 pitch PD + 腿部 theta VMC PD
```

重构为：

```text
整体 LQR + VMC
```

目标状态和输入：

```text
X = [theta, dtheta, x, dx, phi, dphi]^T
U = [T, Tp]^T
U = -K * (X - X_ref)
```

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 新增 `DEFAULT_LQR_K`，并通过 `--lqr-k` 支持传入 `2x6` 常量矩阵。
- `--balance-control lqr` 模式下，中层输出：
  - `T`：轮端总合力矩。
  - `Tp`：虚拟腿角向等效平衡力矩。
- `T` 分配到左右轮：

```text
tau_left_wheel  = 0.5 * T
tau_right_wheel = 0.5 * T
```

- `Tp` 分配到左右腿 VMC 角向通道：

```text
F_theta_left  += 0.5 * Tp
F_theta_right += 0.5 * Tp
```

- `--balance-control lqr` 模式下关闭主 `theta` PD：

```text
effective_theta_kp = 0
effective_theta_kd = 0
```

- VMC 保留腿长控制和分支 guard：

```text
F_l = k_l * (L_target - L) - d_l * dL
tau_joint = J^T * [F_l, Tp_side]^T + tau_guard
```

- 对 `T`、`Tp` 使用现有中层输出变化率限制。
- 对 VMC 任务空间和关节力矩执行限幅：

```text
F_l_max       = 250
F_theta_max   = 80
tau_joint_max = 45 N*m
```

- 增强 LQR 模式控制台诊断输出：
  - `K` 矩阵。
  - 状态顺序和输入顺序。
  - `theta_pd_main_disabled`。
  - `T/Tp` 限幅。
  - VMC 任务空间和关节力矩限幅。
  - actuator 饱和统计。

- 更新 `docs/CONTROL_THEORY.md`，只记录整体 LQR + VMC 的控制公式、物理量定义、限幅和符号检查量。

## 验证结果

语法检查通过。

整体 LQR 短 smoke 通过 finite、约束检查和腿型分支检查：

```text
result: PASS finite model/load/step smoke
max_left_branch_violation  = 0
max_right_branch_violation = 0
final_left_branch_violation  = 0
final_right_branch_violation = 0
```

当前占位 `K` 尚不能稳定站立：

```text
final theta = 0.704697 rad
final pitch = -0.704936 rad
final x     = -0.0267443 m
final dx    = 0.29637 m/s
```

仍然满足：

```text
theta + pitch ≈ 0
```

这说明虚拟腿世界系接近竖直，但车体和腿之间的内部相对模态仍未被当前占位 LQR 增益消除。框架已经改为整体 LQR + VMC，但下一步必须围绕符号、尺度和线性化模型求取有效 `K`，不能把当前占位矩阵当作正式 Riccati 增益。

当前 LQR 输出幅值很小，轮端 actuator 未饱和，腿部 actuator 仍有饱和：

```text
max_abs_lqr_wheel_torque = 0.173367 N*m
max_abs_lqr_pitch_torque = 0.254147
left_wheel_motor ctrl range  = [-0.00722363, -0.00053739]
right_wheel_motor ctrl range = [-0.00722363, -0.00053739]
left_front_motor sat_neg/sat_pos  = 48 / 118
left_rear_motor sat_neg/sat_pos   = 32 / 47
right_front_motor sat_neg/sat_pos = 48 / 118
right_rear_motor sat_neg/sat_pos  = 32 / 47
```

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

整体 LQR 短 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --lqr-gain-scale 0.002 --virtual-rod-steps 1000 --constraint-steps 1000 --check-constraints --history-csv tasks\integrated_lqr_refactor_1000_history.csv --history-plot tasks\integrated_lqr_refactor_1000_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 输出文件

- `tasks/integrated_lqr_refactor_1000_history.csv`
- `tasks/integrated_lqr_refactor_1000_history.png`

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --lqr-gain-scale 0.002 --visualize --visualize-seconds 10
```

## 后续

- 先确认 `theta`、`phi`、轮端力矩 `T`、虚拟腿角向力矩 `Tp`、雅可比转置符号。
- 再基于当前 MuJoCo 模型或低阶近似模型线性化并求解 Riccati 方程，替换当前占位 `K`。
- 调试目标仍然是低速站立稳定，不加入强化学习。
