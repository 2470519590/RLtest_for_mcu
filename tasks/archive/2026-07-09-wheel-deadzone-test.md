# 2026-07-09：轮端控制量死区实验

## 任务

在保留 LQR 输出低通的基础上，给轮端 actuator 输出加入死区：

```text
if abs(ctrl_wheel_i) < ctrl_deadzone:
    ctrl_wheel_i = 0
```

本轮按用户建议测试：

```text
ctrl_deadzone = 0.002
```

## 修改

新增参数：

```text
--wheel-ctrl-deadzone
```

死区只作用于左右轮 actuator 的 `ctrl`，不作用于：

- LQR 状态。
- LQR 输出 `T/Tp`。
- VMC 腿部 `F_l/Tp_side`。
- 四个腿部驱动电机。

当前轮电机 `gear = 12`，因此：

```text
ctrl_deadzone = 0.002
tau_deadzone = 12 * 0.002 = 0.024 N*m
```

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

20 Hz LQR 输出低通 + 轮端死区：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 3000 --history-sample-interval 1 --lqr-output-lowpass-hz 20 --wheel-ctrl-deadzone 0.002 --motor-torque-plot tasks\2026-07-09-motor-torque-lpf20hz-wheel-deadzone0002.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 验证结果

- 语法检查通过。
- 3 秒 smoke finite。
- `saturated_steps = 0`。
- `final theta ≈ -0.0213 rad`。
- `final pitch ≈ 0.0183 rad`。
- `final x_rate ≈ -0.247 m/s`。
- 结果与仅使用 `20 Hz` LQR 输出低通几乎一致。

## 结论

`ctrl_deadzone = 0.002` 对当前主行为影响很小，因为滤波后的轮端控制量大部分时间远大于该阈值。该死区可保留作为抑制极小轮端输出的测试工具，但当前电机力矩主要变化仍由 LQR 输出和 VMC 映射决定。
