# 2026-07-09：LQR 中层控制 smoke

## 任务

按用户给出的 LQR/VMC 分层思路，在现有虚拟杆 VMC 底层之上接入一个轻量 LQR 中层 smoke。

本轮只做本地 smoke 接口，不做训练平台，不做 PPO、行为克隆或 residual RL。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 新增 LQR 中层状态：
  - `theta`
  - `theta_rate`
  - `x`
  - `x_rate`
  - `pitch`
  - `pitch_rate`
- 新增 `--lqr-test`。
- 新增 `--lqr-gain-scale`，默认 `0.02`。
- 新增 `--lqr-x-reference`，默认 `0.0`。
- `--lqr-test` 自动启用虚拟杆测试路径。
- LQR 输出：
  - `T`：左右轮平均分配的总轮端力矩。
  - `Tp`：平均分配到左右虚拟腿角向 VMC 的补偿力矩。
- 输出中新增：
  - `lqr_test`
  - `lqr_gain_scale`
  - `lqr_x_reference`
  - `max_abs_lqr_wheel_torque`
  - `max_abs_lqr_pitch_torque`
  - `final_lqr_state`
- 更新 `docs/CONTROL_THEORY.md`，只记录 LQR 状态、公式和映射关系。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

LQR 短 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 200 --constraint-steps 200 --check-constraints --lqr-gain-scale 0.02
```

LQR 800 step smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 800 --constraint-steps 800 --check-constraints --lqr-gain-scale 0.02
```

## 验证结果

- 语法检查通过。
- 200 step LQR smoke：
  - 状态 finite。
  - 左右 `max_branch_violation = 0`。
  - `max_abs_lqr_wheel_torque = 0.163594`。
  - `max_abs_lqr_pitch_torque = 0.5057`。
  - actuator 没有饱和。
- 800 step LQR smoke：
  - 状态 finite。
  - 闭链最大误差约 `2.49486e-05 m`。
  - 左右 `max_branch_violation = 0`。
  - `max_abs_lqr_wheel_torque = 1.78378`。
  - `max_abs_lqr_pitch_torque = 2.55107`。
  - wheel motor 没有饱和，最大 ctrl 约 `0.0743241`。
  - 腿部 VMC 仍有饱和，至少一个 actuator 饱和 `213` step。
  - `final_base_height` 约 `0.370373 m`。

## 结论

LQR 中层接口已接入现有 VMC 底层，并能在本地 smoke 中保持 finite 和正常腿型分支。

当前结果不能说明整车已经站稳。800 step 中腿部 VMC 仍会饱和，说明后续需要继续做 LQR 增益、力矩符号和轮端/腿端分配的物理调参。

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --visualize --visualize-seconds 10 --lqr-gain-scale 0.02
```
