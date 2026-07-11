# 任务：改回 wheel source 并修正 equilibrium 的 `L0` 截面语义

## 目标

1. 平衡状态重新与文章对齐，`x / dx` 默认使用轮子自身速度来源。
2. 修正旧记录中的 `L_ref` 表述：这里的腿长参数应理解为局部线性化冻结截面 `L0_slice`，不是要搜索出唯一真实腿长。
3. 在给定 `L0_slice` 下寻找 `F_l0 / X0 / U0 / J0` 一致的 upright 接触工作点。

## 修改

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- `LQR` 状态默认 `x_source` 改为 `wheel`
- equilibrium search 里的 `state.x / state.x_rate` 改为 wheel source。
- 新增推荐参数名 `--equilibrium-l-slices`，旧 `--equilibrium-l-refs` 仅作为兼容别名保留。
- 输出中的 `L_ref` 改为 `L0_slice`。
- slip 指标改为：

```text
slip = | base_x_dot - r * omega_avg |
```

- `X0` / HUD / 工作点打印统一到新的状态语义

## 已确认结论

### 1. 速度状态

当前主平衡状态默认已经是：

```text
x      = r * 0.5 * (q_left_wheel + q_right_wheel)
dx     = r * 0.5 * (dq_left_wheel + dq_right_wheel)
r      = 0.085 m
```

`base_x / base_x_dot` 不再作为默认平衡状态输入，只作为漂移与 slip 诊断量。

### 2. 新对称几何下的 `L0_slice`

静态约束构型检查表明：

- `L0_slice = 0.38` 与 `0.395` 可以形成正常接地支撑构型。
- `L0_slice = 0.41` 在旧静态检查中接触裕度较差。

这些值只能说明“这些冻结腿长截面值得用于局部建模检查”，不能说明真实机器人只能在 `0.38 ~ 0.395 m` 之间工作。实际腿长 `L_cmd(t)` 是连续可变的，后续应按多个 `L0_slice` 做增益调度。

```text
L0_slice ∈ candidate slices
```

### 3. `F_l0`

静态约束构型检查中，`F_l0_scale = 0.1 ~ 0.4` 都可以在部分 `L0_slice` 附近维持接触，说明新几何下的 `F_l0` 必须按截面重新辨识，不能继续沿用旧值：

```text
F_l0 = 60 N
```

### 4. 自由 equilibrium search 现状

即使状态源改回 wheel source，当前自由 equilibrium search 仍全部失败。

主要现象：

- `contact_force_min = 0`
- `dx_RMS` 很大
- `slip_indicator` 很大
- 姿态进入大角度坏模态

这说明下一步主问题已经变成：

```text
equilibrium search 的轮端临时控制还不对
```

当前的：

```text
T = wheel_damping + weak pitch/world-theta feedback
```

还不足以形成文章里那种“轮子主动跑到重心下方”的临时倒立摆平衡。

## 验证命令

### 1. 语法检查

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

### 2. 静态构型检查

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-static-pose-check --diagnostics-only --equilibrium-l-slices 0.35 0.38 0.395 0.41 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 0.1 0.2 0.3 0.4
```

### 3. 自由 equilibrium search（wheel source）

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 0.38 0.395 0.41 --equilibrium-fl0-scales 0.1 0.2 0.3 0.4 --equilibrium-theta-refs 0 --equilibrium-steps 900 --equilibrium-eval-steps 350 --equilibrium-wheel-com-kps 80 --equilibrium-wheel-dampings 0.12 --equilibrium-wheel-pitch-kps 0 --equilibrium-wheel-pitch-kds 0 --equilibrium-wheel-world-theta-kps 0 --equilibrium-wheel-world-theta-kds 0 --equilibrium-wheel-base-dx-kds 0.0
```

## 结果摘要

- 旧结论“`0.38 ~ 0.395` 是合理腿长工作区”应降级为“这些是可检查的冻结 `L0` 截面”。
- 当前自由 equilibrium search 还没有找到合格工作点
- 因此当前阶段不能直接线性化，也不能直接拿这些末端点当 `X0/U0`

## 下一步

下一步不该再扫更多旧式 wheel damping，而应把 equilibrium search 的轮端临时控制改成更接近文章的倒立摆思路：

1. 用 wheel source 的 `x / dx`
2. 轮端负责把轮子跑到重心下方
3. 腿长环只负责高度
4. `Tp` 只做姿态/腿角支撑，不和 wheel 环互相顶
