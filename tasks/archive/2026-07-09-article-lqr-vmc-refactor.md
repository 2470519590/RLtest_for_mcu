# 2026-07-09：整体 LQR + VMC 主路径重构

## 任务

按轮腿机器人文章式控制框架清理主控制路径：

```text
X = [theta, dtheta, x, dx, phi, dphi]^T
U = [T, Tp]^T
U = U0 - K * (X - X0)
F_l = F_l0 + k_l * (L_ref - L) - d_l * dL
tau_joint = J^T * [F_l, Tp_side]^T
```

## 修改

- 主平衡层默认切到整体 LQR。
- `--lqr-test` 在未手动给 `--lqr-k` 时默认执行局部有限差分线性化和 Riccati 求解，不再默认使用旧手填占位矩阵。
- `_lqr_middle_control()` 删除旧 `wheel-pd` 分支，只保留整体 LQR：

```text
[T, Tp]^T = U0 - gain_scale * K * (X - X0)
```

- LQR 模式下主 `theta` PD 无条件关闭，`Tp` 作为 VMC 角向输入。
- 腿长支撑从 PI 改为 PD + 前馈：

```text
F_l = F_l0 + k_l * (L_ref - L) - d_l * dL
```

- 腿长积分参数从主路径移除，不再用积分掩盖错误工作点。
- 旧 wheel-PD 相关 CLI 参数从主帮助中隐藏，避免把回退/历史诊断参数误认为当前主控制架构。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

默认整体 LQR + VMC 短 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 800 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

3 秒 smoke：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 3000 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

可视化命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --visualize --visualize-seconds 10
```

## 验证结果

- 语法检查通过。
- 默认 `--lqr-test` 会自动线性化求 `K`。
- 800 步 smoke finite。
- `theta_pd_main_disabled: True`。
- 腿长律打印为：

```text
F_l = F_l0 + k_l * (L_ref - L) - d_l * dL
```

- 本次短 smoke：
  - `final_left_theta ≈ -0.0030 rad`
  - `final pitch ≈ 0.0028 rad`
  - `saturated_steps = 0`
  - `final_left_length ≈ 0.325 m`
- 3 秒 smoke：
  - `final_left_theta ≈ -0.0200 rad`
  - `final pitch ≈ 0.0198 rad`
  - `final_x ≈ -0.431 m`
  - `final_x_rate ≈ -0.236 m/s`
  - `saturated_steps = 0`
  - `final_left_branch_violation = 0`

## 结论

主路径已经从历史分层 PD/VMC 清理为整体 LQR + VMC。当前剩余问题不是继续加 PI/PD，而是继续寻找更真实的 `X0/U0/F_l0` 和接触工作点，使腿长工作点与 LQR 线性化点一致。

3 秒结果说明姿态已经能保持小角度，但水平位置仍有慢漂。下一步应在文章框架内处理：先确认工作点和 `U0/F_l0`，再使用位置外环 `x -> v_ref` 或重新线性化增益，不回退到旧 wheel-PD。
