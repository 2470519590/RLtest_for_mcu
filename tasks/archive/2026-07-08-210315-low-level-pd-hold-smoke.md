# 2026-07-08 21:03:15：最小底层 PD hold smoke

## 任务

在已有 MuJoCo smoke 工具中加入最小底层控制检查，先验证控制路径能跑通，不推进 LQR。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 在 `run_smoke.py` 默认流程中加入 actuated joint 的 PD hold smoke。
- 更新 `docs/CONTROL_THEORY.md` 记录结果。

## 控制方式

对每个 actuator 绑定的 joint，目标为 `model.qpos0` 对应位置：

```text
ctrl = kp * (qpos0 - qpos) - kd * qvel
```

再按 actuator ctrlrange 裁剪。

默认参数：

- `pd_hold_steps = 500`
- `kp = 0.6`
- `kd = 0.08`

## 验证命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py
```

## 验证结果

- 语法检查：通过。
- smoke：通过。
- PD hold 状态保持 finite。
- `final_base_height`：0.190235
- `final_qpos_norm`：3.66733
- `final_qvel_norm`：5.78479
- `max_abs_ctrl`：0.423526
- `saturated_steps`：0

## 结论

- 底层控制代码路径已跑通。
- 这不是稳定站立控制。
- 当前 reset 初态或控制目标还不够合理，机器人高度明显下降。
- 仍然不能推进正式 LQR。

## 下一步

1. 定义合理 reset 初态。
2. 明确底层控制目标。
3. 再做短 rollout 检查。
