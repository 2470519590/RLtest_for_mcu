# 任务：修正 `L0_slice` upright-IK 初态构造

## 目标

继续按文章式流程推进：

```text
给定 L0_slice -> upright 正常分支初态 -> true contact equilibrium -> linearization -> K(L0)
```

本轮只处理第一步：让给定 `L0_slice` 时的静态构型真的落在对应腿长截面，而不是被局部 IK 或 reset base 锁定污染。

## 修改

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- `_settled_leg_shape_for_drive_targets()` 和 `_settle_ik_candidate()` 不再锁回 `model.qpos0`，而是锁定 source data 的当前 base 位姿。
- 新增对称五连杆解析 IK：
  - 根据前/后上铰点和目标 carrier 位置做两圆交点；
  - 前腿选择 `elbow.x > carrier.x`、`elbow.z > carrier.z`；
  - 后腿选择 `elbow.x < carrier.x`、`elbow.z > carrier.z`；
  - 用 reset 上杆向量到目标上杆向量的旋转角得到驱动关节目标。
- `upright-ik` equilibrium 初始化优先使用解析 IK；解析不可达时才回退到动态全局搜索。

更新 [docs/CONTROL_THEORY.md](E:/STM32_PROJ/RL_training/docs/CONTROL_THEORY.md)：

- 记录解析 IK 的几何公式和正常分支选择。

更新 [docs/ERROR_CATALOG.md](E:/STM32_PROJ/RL_training/docs/ERROR_CATALOG.md)：

- 记录“非 reset 工作点 IK 候选不能锁回 reset base”的错误。

## 验证结果

语法检查通过：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

静态截面检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-static-pose-check --diagnostics-only --equilibrium-l-slices 0.35 0.38 0.395 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 0.2 --equilibrium-init-drop-steps 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

关键输出：

```text
L0_slice=0.35  -> measured L0≈0.350075, theta≈-0.000116, contact_min≈8.06479
L0_slice=0.38  -> measured L0≈0.379986, theta≈-0.0000328, contact_min≈12.9832
L0_slice=0.395 -> measured L0≈0.395010, theta≈-0.0000101, contact_min≈12.4057
```

锁 base VMC 腿长方向检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --lock-base --virtual-rod-length-force-ff 0 --virtual-rod-steps 500 --left-rod-length 0.35 --right-rod-length 0.35 --left-rod-theta 0 --right-rod-theta 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1 --print-static-operating-point
```

关键输出：

```text
target L=0.35
final L≈0.363924
F_l_cmd≈-6.93 N
branch violation=0
```

这说明锁 base 且不加重力前馈时，腿长通道方向正确，可以从 reset 方向收向 `0.35`。

自由 equilibrium 短扫：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 --equilibrium-fl0-scales 0.8 1.0 1.2 1.5 --equilibrium-theta-refs 0 --equilibrium-steps 1200 --equilibrium-eval-steps 400 --equilibrium-wheel-com-kps 80 --equilibrium-wheel-dampings 0.12 --equilibrium-wheel-pitch-kps 0 --equilibrium-wheel-pitch-kds 0 --equilibrium-wheel-world-theta-kps 0 --equilibrium-wheel-world-theta-kds 0 --equilibrium-wheel-base-dx-kds 0 --equilibrium-init-drop-steps 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

结果仍不合格：

```text
best: L0_slice=0.35, F_l0_scale=0.8
L_mean≈0.278
dL_RMS≈0.171
joint_sat_ratio≈0.113
contact_force_min=0
```

## 当前结论

- `L0_slice` 静态 upright 初态构造已修复。
- 锁 base 腿长 VMC 方向正确。
- 自由接触 equilibrium 仍失败，主要不是 `K` 问题，而是给定 `L0` 下的腿长支撑/contact 工作点还没找到；自由阶段会塌到短腿构型并出现接触切换或关节力矩长期介入。

## 下一步

不要进入线性化和 LQR 求 K。下一步应继续做 true equilibrium search 的底层支撑部分：

1. 在解析 upright 初态上，先只放开垂向/接触，检查 `F_l0` 与 `L` 的静态力平衡。
2. 单独扫描 `length_kp / length_kd / joint_tau_limit` 对 `L_mean` 和 `dL_RMS` 的影响。
3. 确认腿长保持不塌后，再加入 `T = k_com * (x_com - x_wheel) - d_wheel * dx_wheel`。
4. 只有满足 `dX≈0`、`dL≈0`、contact 稳定、输入不长期饱和后，才线性化求 `K(L0)`。
