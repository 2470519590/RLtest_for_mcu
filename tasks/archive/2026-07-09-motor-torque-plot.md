# 2026-07-09：六电机输出力矩曲线

## 任务

在整体 LQR + VMC 主路径基础上，输出 6 个 MuJoCo actuator 的实际力矩曲线。

## 力矩定义

当前图中电机输出力矩按已确认 actuator 语义计算：

```text
tau_motor = gear * ctrl
```

对应 6 个 actuator：

```text
left_front_motor
left_rear_motor
left_wheel_motor
right_front_motor
right_rear_motor
right_wheel_motor
```

## 修改

- `LqrHistorySample` 新增 6 个实际电机力矩字段。
- 新增 `--motor-torque-plot PATH`。
- 图中分三行：
  - 左腿前/后驱动电机力矩。
  - 右腿前/后驱动电机力矩。
  - 左/右轮电机力矩，并叠加 `0.5*T target` 作为轮端目标对照。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

默认采样间隔：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 3000 --motor-torque-plot tasks\2026-07-09-motor-torque.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

逐步采样：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 3000 --history-sample-interval 1 --motor-torque-plot tasks\2026-07-09-motor-torque-step1.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 验证结果

- 语法检查通过。
- 已生成：

```text
tasks\2026-07-09-motor-torque.png
tasks\2026-07-09-motor-torque-step1.png
```

- 3 秒 smoke finite。
- `saturated_steps = 0`，但图上可见腿部驱动和轮驱动存在持续高频力矩纹波。
- 轮电机实际力矩与 `0.5*T target` 基本重合，说明轮端 actuator 输出跟随中层 `T` 分配。

## 结论

当前看到的电机力矩抖动不是 viewer 显示层的问题。下一步应在整体 LQR + VMC 框架内检查离散线性化、`K` 的高频模态、`Tp` 经 VMC 映射后的腿部力矩纹波，以及是否需要在 LQR 设计模型中显式考虑执行器带宽或输出平滑。
