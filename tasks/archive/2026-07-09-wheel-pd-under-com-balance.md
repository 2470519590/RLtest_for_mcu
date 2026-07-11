# 2026-07-09：轮式倒立摆 wheel-pd 中层

## 任务

用户指出可视化里看不到明显纠正平衡的措施，要求采用文章或开源轮式倒立摆思路：保持虚拟腿角为 `0`、高度不变，通过轮子运动到重心下方来平衡。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 新增 `--balance-control wheel-pd|lqr`。
- 默认 `--balance-control wheel-pd`。
- 默认 `--lqr-wheel-sign` 从 `+1` 改为 `-1`，匹配当前 wheel-pd 的轮端方向。
- 新增 wheel-pd 参数：
  - `--wheel-balance-pitch-kp`
  - `--wheel-balance-pitch-kd`
  - `--wheel-balance-x-kp`
  - `--wheel-balance-x-kd`
  - `--wheel-balance-max-torque`
- wheel-pd 中层只输出轮端力矩 `T`，不再向腿部额外注入 `Tp`。
- 腿部 VMC 继续保持：
  - `l_target = 0.35 m`
  - `theta_target = 0 rad`
- 更新 `docs/CONTROL_THEORY.md` 的 wheel-pd 控制律。

## 控制律

```text
T_raw = -(
  k_phi * phi
+ d_phi * dphi
+ k_x   * x
+ d_x   * dx
)

T = sign_T * clip(T_raw, -T_max, T_max)
Tp = 0
```

当前默认：

```text
k_phi = 14
d_phi = 2.5
k_x   = 2
d_x   = 1.2
T_max = 16 N*m
sign_T = -1
```

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

默认 wheel-pd smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 800 --constraint-steps 800 --check-constraints --history-csv tasks\wheel_pd_under_com_history.csv --history-plot tasks\wheel_pd_under_com_history.png --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 验证结果

- 语法检查通过。
- 默认 wheel-pd 800 step 状态 finite。
- 闭链最大误差约 `2.65903e-05 m`。
- 最终姿态明显优于旧 LQR 缩放版本：
  - `final pitch = 0.0148992 rad`
  - `final theta = -0.0146617 rad`
  - `final_base_height = 0.454485 m`
- 轮子有明显主动纠正：
  - `max_abs_left_wheel_speed_rad_s = 42.283`
  - `max_abs_right_wheel_speed_rad_s = 42.283`
  - `max_abs_lqr_wheel_torque = 4.39023`
- `Tp = 0`，腿部不再额外承担中层姿态补偿。
- 仍存在问题：
  - `max_left_branch_violation = 0.00542509`
  - `max_right_branch_violation = 0.00542509`
  - 腿部 VMC 饱和 `566` step。

## 结论

wheel-pd 模式已经能看到倒立摆式纠正动作：轮子主动跑位，最终车体 pitch 和虚拟腿 theta 都接近 `0`。

但当前仍不能判定为完成平衡。主要问题从“轮子没有纠正动作”变成了：

- 轮端力矩有振荡。
- 腿部 VMC 长时间饱和。
- 早期存在小幅分支 violation。

下一步应继续做：

- 降低 wheel-pd 振荡。
- 优化腿部 VMC 饱和阶段的抗压和分支保护。
- 让腿部保持高度/腿角时不把控制量长期打满。

## 输出文件

- `tasks/wheel_pd_under_com_history.csv`
- `tasks/wheel_pd_under_com_history.png`

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --visualize --visualize-seconds 10
```
