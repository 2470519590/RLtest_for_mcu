# 2026-07-09：虚拟杆 VMC 底层控制

## 任务

用户反馈大冲击下仍会凹进去，认为底层控制器参数或性能不足。参考用户给出的 VMC 设计思路，改进当前虚拟杆底层控制。

当前不修改机械 XML，不推进 LQR、行为克隆、PPO 或 residual RL。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 新增 `--virtual-rod-control vmc|position`。
- 默认虚拟杆控制从旧的电机角度位置伺服切换为 `vmc`。
- 默认虚拟杆腿长设为 `0.35 m`，默认腿角为 `0 rad`。
- 保留旧逻辑：使用 `--virtual-rod-control position` 可回到电机角目标 PD。
- 新增每个 actuator 的 ctrl min/max 和正负限幅步数输出。
- 新增虚拟杆 VMC：
  - 在任务空间计算虚拟杆长度轴向力。
  - 在任务空间计算虚拟杆角向力矩。
  - 用虚拟雅可比转置映射到前后驱动关节力矩。
  - 再按 actuator gear 和 ctrlrange 写入 MuJoCo motor ctrl。
- 新增接近危险分支时的提前 guard：
  - 每步计算 base 机体系下腿型指标。
  - 腿型接近危险 margin 时，提前拉回分支安全 IK 目标。
- 更新 `docs/CONTROL_THEORY.md`。

## 控制频率

- MuJoCo timestep 为 `0.001 s`。
- 虚拟杆控制每个 `mj_step` 前更新一次。
- 当前底层控制名义频率为 `1000 Hz`，满足至少 `100 Hz`。

## 控制逻辑

```text
F_l = k_l * (l_target - l) - d_l * dl
F_theta = k_theta * (theta_target - theta) - d_theta * dtheta
tau_joint = J_virtual^T * [F_l, F_theta]
ctrl_i = clip(tau_i / gear_i, ctrlrange_i)
```

当前虚拟雅可比使用动态扫描确认过的局部映射：

```text
d l     / d [q_front, q_rear] = [ 0.143, -0.143]
d theta / d [q_front, q_rear] = [-0.424, -0.424]
```

没有使用纯 `mj_forward` qpos 数值微分，因为当前模型是树结构加 equality 闭链约束，直接扰动 rear drive qpos 不能正确反映闭链动态影响。

## 默认参数

默认目标：

```text
l_target = 0.35 m
theta_target = 0 rad
```

默认 VMC 增益：

```text
virtual_rod_length_kp = 1200
virtual_rod_length_kd = 55
virtual_rod_theta_kp = 35
virtual_rod_theta_kd = 2.2
virtual_rod_joint_kd = 4.0
```

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

锁机身 VMC：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --lock-base --check-constraints --left-rod-length 0.44 --right-rod-length 0.44 --left-rod-theta 0.145 --right-rod-theta 0.145 --virtual-rod-steps 800 --constraint-steps 800
```

真实场景 VMC：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --check-constraints --constraint-steps 800
```

## 验证结果

- 语法检查通过。
- 锁机身 800 step：
  - 最终约 `l=0.437863 m, theta=0.146469 rad`。
  - 左右 `max_branch_violation = 0`。
  - 闭链最大误差约 `1.02e-6 m`。
- 真实场景 800 step：
  - 状态 finite。
  - 左右 `max_branch_violation = 0`。
  - 默认目标为 `l=0.35 m, theta=0 rad`。
  - 闭链最大误差约 `2.52e-5 m`。
  - `final_base_height` 约 `0.370 m`。
  - `max_abs_ctrl = 1`，有 256 step 至少一个 actuator 饱和。
  - `left_front_motor`：`min=-1, max=1`，负向饱和 67 step，正向饱和 123 step。
  - `left_rear_motor`：`min=-1, max=1`，负向饱和 42 step，正向饱和 66 step。
  - `right_front_motor`：`min=-1, max=1`，负向饱和 67 step，正向饱和 123 step。
  - `right_rear_motor`：`min=-1, max=1`，负向饱和 42 step，正向饱和 66 step。
  - 左右 wheel motor 始终为 `0`。

## 结论

本轮底层控制从“位置伺服追 IK 目标”升级为“虚拟杆 VMC + 分支提前 guard”。数值 smoke 表明真实场景 800 step 内 branch violation 为 0，但仍没有站稳。下一步若视觉仍出现明显凹陷，需要同时检查：

- 可视化是否对应最新 `vmc` 模式。
- 视觉凹陷时 branch 指标是否同步变正。
- 当前 actuator ctrlrange `[-1, 1]` 和 gear 是否限制了抗冲击力矩。
- 是否需要在 VMC 之外继续加入机身姿态/轮速平衡控制。

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --visualize
```

旧位置伺服对比命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --visualize --virtual-rod-control position --left-rod-length 0.44 --right-rod-length 0.44 --left-rod-theta 0.145 --right-rod-theta 0.145 --motor-servo-kp 30 --motor-servo-kd 1.2
```
