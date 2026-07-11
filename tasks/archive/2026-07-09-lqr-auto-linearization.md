# 2026-07-09：LQR 自动线性化与真实 K 求解

## 任务

继续处理整体 `LQR + VMC` 框架，不再使用手填占位 `K`，而是在当前 MuJoCo 五连杆轮腿模型上直接构造局部工作点、做 reduced-state 有限差分线性化，并自动求取 `K`。

目标：

- 确认 `T / Tp` 的物理正方向。
- 让 `theta / pitch` 不再回到反相坏模态。
- 保持腿型分支不凹陷。
- 把结果写进理论文档和任务记录。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 新增自动 LQR 设计流程：
  - 接触一致工作点构造。
  - reduced-state 有限差分 `A_d, B_d` 线性化。
  - 有限时域 Riccati 迭代近似。
  - 自动生成 `K(2x6)`。
- 自动设计模式默认假设：

```text
lqr_wheel_sign = +1
lqr_pitch_sign = +1
wheel_balance_torque_rate_limit = 0
```

- 自动设计模式默认 `lqr_gain_scale = 1`。
- `virtual_rod_test`、`constraint_check`、`visualize` 在自动 LQR 模式下会先 warm-start 到 LQR 设计工作点，再释放 base 运行。
- 新增 LQR 设计诊断输出：
  - `Q / R`
  - `state_eps / input_eps`
  - `operating_state`
  - `A_d / B_d`
  - `K`
  - Riccati 迭代步数
  - `closed_loop_max_abs_eig`

## 关键结论

1. 单纯把占位 `K` 接到现有 smoke 不够，必须先解决工作点和输入符号。
2. 直接沿用旧默认：

```text
lqr_wheel_sign = -1
wheel_balance_torque_rate_limit = 120
```

会把自动线性化得到的 `K` 推回错误方向或严重限速，闭环会重新掉进坏模态。

3. 当前有效的自动设计路径是：

```text
工作点 warm-start
+ sign_T  = +1
+ sign_Tp = +1
+ rate limit = 0
```

4. 当前自动 `K` 已经能把：

```text
theta -> 0
pitch -> 0
```

压回零附近，不再出现之前那种持续的 `pitch ≈ -theta` 大幅反相摆动。

5. 当前残余问题从“平衡坏模态”转成了“慢速 x 漂移和低层腿部长期饱和”。

## 代表性验证结果

推荐权重：

```text
Q = diag([120, 8, 80, 16, 180, 15])
R = diag([0.8, 0.4])
```

1200 步自动 LQR smoke：

```text
max_left_branch_violation  = 0.0149586
max_right_branch_violation = 0.0149586
final_left_branch_violation  = 0
final_right_branch_violation = 0

final theta      = -0.0132474 rad
final pitch      =  0.0136178 rad
final x          = -0.155066 m
final x_rate     = -0.233599 m/s
final wheel speed = -0.475392 rad/s
```

说明：

- `theta / pitch` 已经被拉回零附近。
- 腿型分支最终无凹陷。
- 仍存在慢速 `x` 漂移，闭环还没有做到严格固定点停车。

当前自动生成 `K`：

```text
[
  [-0.745389,  55.7277, -2.9571, -3.24068,  98.2384, -94.5431],
  [ 2.52325, -267.705, -1.90368, -1.90542, 39.8354, 452.972 ],
]
```

2000 步延长 smoke：

```text
final theta      = -0.0307204 rad
final pitch      =  0.0309556 rad
final x          = -0.330718 m
final x_rate     = -0.239053 m/s
final wheel speed = 1.88966 rad/s
```

结论：

- upright 平衡模态已经建立。
- 固定点停车还没有完成，`x` 方向仍有慢漂移。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

自动线性化 LQR smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --lqr-q-diag 120 8 80 16 180 15 --virtual-rod-steps 1200 --constraint-steps 1200 --check-constraints --history-csv tasks\integrated_lqr_auto_defaultlike_1200_history.csv --history-plot tasks\integrated_lqr_auto_defaultlike_1200_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

2000 步延长 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --lqr-q-diag 120 8 80 16 180 15 --virtual-rod-steps 2000 --constraint-steps 2000 --check-constraints --history-csv tasks\integrated_lqr_auto_defaultlike_2000_history.csv --history-plot tasks\integrated_lqr_auto_defaultlike_2000_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --lqr-q-diag 120 8 80 16 180 15 --visualize --visualize-seconds 10
```

## 后续

- 继续处理 `x / dx` 漂移，优先通过 `Q_x / Q_dx`、工作点和输入模型改进，而不是把 `theta` PD 再加回来。
- 如果要重新引入 `T / Tp` rate limit，必须把它作为设计模型的一部分重新线性化，而不是在线性化之后再额外硬夹。
- 低层腿部饱和仍然很重，后续需要继续核对 `F_l`、`Tp`、`J^T` 尺度。
