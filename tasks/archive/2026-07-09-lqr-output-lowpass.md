# 2026-07-09：LQR 输出低通滤波实验

## 任务

在整体 LQR + VMC 主路径中，对 LQR 输出的：

```text
U = [T, Tp]^T
```

加入可选一阶低通滤波，并观察 6 个电机实际输出力矩图。

## 修改

新增参数：

```text
--lqr-output-lowpass-hz
```

滤波公式：

```text
tau_f = 1 / (2 * pi * f_c)
alpha = dt_control / (tau_f + dt_control)

T_f[k]  = T_f[k-1]  + alpha * (T_raw[k]  - T_f[k-1])
Tp_f[k] = Tp_f[k-1] + alpha * (Tp_raw[k] - Tp_f[k-1])
```

`f_c = 0` 表示关闭低通。滤波只作用在 LQR 输出进入轮端和 VMC 前，不改变 LQR 状态、K、工作点和 VMC 映射定义。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

20 Hz 低通：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 3000 --history-sample-interval 1 --lqr-output-lowpass-hz 20 --motor-torque-plot tasks\2026-07-09-motor-torque-lpf20hz.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

10 Hz 低通：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 3000 --history-sample-interval 1 --lqr-output-lowpass-hz 10 --motor-torque-plot tasks\2026-07-09-motor-torque-lpf10hz.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 验证结果

20 Hz：

- 3 秒 smoke finite。
- `saturated_steps = 0`。
- `max_abs_lqr_wheel_torque ≈ 7.51 N*m`。
- `max_abs_lqr_pitch_torque ≈ 7.72 N*m`。
- `final theta ≈ -0.0213 rad`。
- `final pitch ≈ 0.0183 rad`。
- `final x_rate ≈ -0.247 m/s`。
- 力矩图显示原先持续高频纹波基本被压掉。

10 Hz：

- 3 秒 smoke finite。
- `saturated_steps = 0`。
- `max_abs_lqr_wheel_torque ≈ 6.77 N*m`。
- `max_abs_lqr_pitch_torque = 8 N*m`。
- `final theta ≈ -0.0216 rad`。
- `final pitch ≈ 0.0185 rad`。
- `final x_rate ≈ -0.266 m/s`。
- 力矩更平滑，但启动后轮端振铃和水平漂移略更差。

## 结论

LQR 输出低通可以明显降低 6 个电机输出力矩的高频纹波，说明前一轮看到的高频输出主要来自 LQR 输出和 VMC 映射的离散高频分量。20 Hz 当前比 10 Hz 更合适：能压掉大部分高频纹波，同时相位滞后更小。

下一步不应加回旧 PD/PI，而应在整体 LQR + VMC 框架内继续处理：

- LQR 线性化模型是否包含执行器带宽。
- `Q/R` 是否过度激励高频状态。
- `Tp` 经 `J_theta^T` 映射后的腿部力矩尺度。
- 位置外环 `x -> v_ref` 与滤波相位滞后的耦合。
