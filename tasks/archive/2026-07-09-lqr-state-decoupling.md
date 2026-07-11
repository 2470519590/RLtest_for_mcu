# 2026-07-09：LQR 状态扰动解耦与长时漂移检查

## 任务

针对可视化中“小车短时稳住后持续加速、虚拟杆没有保持世界竖直”的现象，继续检查整体 `LQR + VMC` 的符号、尺度和线性化构造。

## 修改

- 修正 `src/robot_smoke/model_smoke.py` 中 LQR 状态扰动构造顺序。
- 构造线性化状态时，先设置 `x / dx / pitch / pitch_rate`，再通过腿部 IK 把世界系 `theta` 补偿到目标值。
- `_apply_lqr_theta_state_perturbation()` 改为锁住当前 base 位姿，而不是固定锁回 reset 位姿。
- 更新 `docs/CONTROL_THEORY.md` 中 `theta` 的定义：
  - 当前 `theta = atan2(r_x, -r_z)` 是世界系虚拟腿角。
  - `theta = 0` 才表示虚拟腿接近世界竖直。
  - `theta + phi` 不能再当作世界系腿角。

## 关键结论

线性化构造修正前，纯 `pitch` 扰动会在 `A_d` 中夹带一个接近 `-1` 的虚假 `theta` 位移。修正后该耦合显著减小，10 秒 smoke 不再出现原先的高速前冲发散。

当前仍未完成“固定点长时静止”：

- 10 秒内虚拟腿世界角可压到小角度。
- 分支 violation 最终为 0。
- 轮速不再像修正前一样增长到约 `92 rad/s`。
- 但 `x / dx` 仍存在慢漂或低频回拉，20 秒测试仍会出现位置模式继续变大。

因此本轮结论是：已修正一个 LQR 线性化坐标错误，但固定点站立还需要继续处理平移模态和输出尺度。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

修正后的 10 秒默认 LQR smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --virtual-rod-steps 10000 --constraint-steps 1200 --check-constraints --history-csv tasks\lqr_theta_pitch_decoupled_10s_history.csv --history-plot tasks\lqr_theta_pitch_decoupled_10s_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

代表性输出：

```text
final theta = -0.0475399 rad
final pitch = 0.0481609 rad
final x = -4.2104 m
final x_rate = -0.153331 m/s
final wheel speed = -1.40823 rad/s
final branch violation = 0
```

对比修正前 10 秒默认 LQR：

```text
final theta = -0.482585 rad
final pitch = 0.478571 rad
final x = 12.1175 m
final x_rate = 7.83683 m/s
final wheel speed = 92.501 rad/s
```

## 后续

- 继续处理 `x / dx` 长时漂移。
- 优先检查平移模态的可控尺度、轮地位移定义和 LQR 输出限幅模型。
- 不应把旧的强 `theta` PD 加回主控制器。
