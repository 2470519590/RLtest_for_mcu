# 2026-07-09：LQR 慢速漂移修正

## 任务

处理默认 `LQR + VMC` 下小车不再加速摔倒后仍慢速向后匀速漂移的问题，并把此前错误经验写入 `docs/ERROR_CATALOG.md`。

## 修改

- 调整自动 LQR 默认权重：

```text
Q = diag([700, 50, 10, 300, 700, 60])
R = diag([2.0, 1.2])
```

- 自动 LQR 默认 `T / Tp` 输出限幅从 `12 N*m` 降到 `8 N*m`。
- LQR 分支预留可选的 `base_x / base_x_dot` 直接修正项，复用现有参数：

```text
T = T_LQR + k_x_direct * x + d_x_direct * dx
```

当前默认仍取：

```text
k_x_direct = 0
d_x_direct = 0
```

- 更新 `docs/CONTROL_THEORY.md` 中当前默认权重和限幅。
- 更新 `docs/ERROR_CATALOG.md`，记录：
  - LQR pitch 扰动夹带虚假 theta。
  - 轮关节速度阻尼不能替代车体平移阻尼。

## 验证结果

调整前默认 10 秒 LQR smoke：

```text
final theta = -0.0475399 rad
final pitch = 0.0481609 rad
final x = -4.2104 m
final x_rate = -0.153331 m/s
final wheel speed = -1.40823 rad/s
final branch violation = 0
```

调整后默认 10 秒 LQR smoke：

```text
final theta = 0.0183106 rad
final pitch = -0.0176305 rad
final x = 0.760309 m
final x_rate = -0.00506068 m/s
final wheel speed = 0.19864 rad/s
final branch violation = 0
```

20 秒检查仍存在低频残余漂移：

```text
final theta = 0.0458163 rad
final pitch = -0.0422398 rad
final x = 1.65524 m
final x_rate = 0.258763 m/s
final wheel speed = 3.05131 rad/s
final branch violation = 0
```

结论：本轮修复了用户观察到的默认 10 秒内慢速向后匀速漂移，默认 smoke 已接近停车；但固定点长时完全静止仍未完成，后续需要继续处理低频位置模态。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

默认 10 秒 LQR smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --virtual-rod-steps 10000 --constraint-steps 1200 --check-constraints --history-csv tasks\lqr_drift_default_after_tuning_10s_history.csv --history-plot tasks\lqr_drift_default_after_tuning_10s_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

本地可视化：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --balance-control lqr --visualize --visualize-seconds 10
```

## 后续

- 继续处理 20 秒尺度上的低频位置模态。
- 优先检查 `Tp` 长时间限幅、支撑腿长压缩和 `x` 参考固定方式。
- 不建议继续增大轮关节速度阻尼。
