# 2026-07-08：虚拟杆 IK 正常分支控制

## 任务

优化底层虚拟杆控制，避免 IK 追到“凹进去”的合法但错误分支。当前不继续优先修改机械 XML，不推进 LQR、行为克隆、PPO 或 residual RL。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 新增 `LegBranchMetrics`，记录 `front_dx`、`rear_dx`、`below_front`、`below_rear`、`elbow_span` 和 `violation`。
- branch 指标在 base 机体系中计算，避免车身倒下后被世界坐标误判。
- 将虚拟杆 IK 从单点线性近似升级为分支感知候选搜索。
- 新增 CLI 参数：
  - `--leg-branch normal|off`，默认 `normal`。
  - `--ik-search-radius`，默认 `0.25`。
  - `--ik-search-samples`，默认 `5`。
- smoke 输出新增：
  - `max_left_branch_violation`
  - `max_right_branch_violation`
  - `final_left_branch_violation`
  - `final_right_branch_violation`
- 更新 `docs/CONTROL_THEORY.md` 记录 IK 多解/分支选择结论。

## 控制结论

当前“凹进去”按控制问题处理：不是继续优先改连杆机构，而是在虚拟杆中间层保证正常腿型分支。

正常分支要求在 base 机体系中满足：

- carrier 在两个 elbow 下方。
- front_elbow 在 carrier 前方。
- rear_elbow 在 carrier 后方。
- 两个 elbow 的 x 间距保持正且明显。

LQR 之前必须由中间层保证腿型分支，不能让上层控制掩盖 IK 分支选择问题。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

锁机身虚拟杆 + 约束：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --lock-base --check-constraints --left-rod-length 0.44 --right-rod-length 0.44 --left-rod-theta 0.145 --right-rod-theta 0.145 --motor-servo-kp 30 --motor-servo-kd 1.2
```

真实场景：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --check-constraints --constraint-steps 800 --left-rod-length 0.44 --right-rod-length 0.44 --left-rod-theta 0.145 --right-rod-theta 0.145 --motor-servo-kp 30 --motor-servo-kd 1.2
```

## 验证结果

- 语法检查通过。
- 锁机身 800 step：
  - 闭链最大误差约 `7.29e-7 m`。
  - 左右 `max_branch_violation = 0`。
  - 左右 `final_branch_violation = 0`。
  - 最终虚拟杆约 `l=0.435963 m, theta=0.153996 rad`。
- 真实场景 800 step：
  - 状态 finite。
  - 闭链最大误差约 `2.43e-5 m`。
  - 左右 `max_branch_violation = 0`。
  - 左右 `final_branch_violation = 0`。
  - `final_base_height` 约 `0.191 m`，说明仍会倒下；这不是本轮失败。

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --visualize --left-rod-length 0.44 --right-rod-length 0.44 --left-rod-theta 0.145 --right-rod-theta 0.145 --motor-servo-kp 30 --motor-servo-kd 1.2
```

## 下一步

用户本地看可视化，确认启动控制时腿型是否保持正常平行四边形分支。若视觉仍有凹陷，先对照 branch 指标判断是显示/视角问题、base 坐标指标 margin 不足，还是需要进一步约束 IK 搜索。
