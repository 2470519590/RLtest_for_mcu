# 任务：五连杆运动学门禁与文章式力学搜索种子

## 目标

用户要求先确保五连杆运动学解算准确，并且 equilibrium search 应按文章的经典力学公式算出中心值，再在附近小范围搜索，不应过度依赖优化算法。

参考链接：

- `https://zhuanlan.zhihu.com/p/613007726`
- `https://zhuanlan.zhihu.com/p/563048952`

说明：当前环境下第一个链接未能直接读取正文，因此本轮采用平面五连杆通用解析法：两圆交点 + 显式分支选择 + MuJoCo 闭链 settle 反验。

## 修改

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- 新增 `--fivebar-kinematics-check` 诊断模式。
- 诊断流程：
  1. 从 XML body 局部位置读取前/后上铰点、上杆长度、下杆长度。
  2. 对给定 `L0_slice/theta_ref` 用两圆交点解析 IK 求前/后 elbow。
  3. 选择正常分支。
  4. 把解析关节目标送入 MuJoCo 闭链模型，锁 base settle。
  5. 输出实测 `L/theta/branch violation` 与目标误差。
- equilibrium search 内部新增力学中心值：

```text
F_l0_center = m_total * g / (2 * cos(theta_ref))
Tp_center   = -m_total * g * (x_com - x_wheel)
T_center    = 0
```

- `F_l0_scale` 现在是围绕 `F_l0_center` 的比例，`Tp_bias` 是围绕 `Tp_center` 的偏置。

更新 [docs/CONTROL_THEORY.md](E:/STM32_PROJ/RL_training/docs/CONTROL_THEORY.md)：

- 记录力学中心值公式。

更新 [docs/ERROR_CATALOG.md](E:/STM32_PROJ/RL_training/docs/ERROR_CATALOG.md)：

- 记录“equilibrium search 不能用大范围优化替代力学中心值”。

## 验证结果

语法检查通过：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

五连杆运动学检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --fivebar-kinematics-check --diagnostics-only --fivebar-kinematics-l-slices 0.35 0.38 0.395 --fivebar-kinematics-theta-refs 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

关键结果：

```text
front_upper_length = rear_upper_length ≈ 0.211896 m
front_lower_length = rear_lower_length ≈ 0.264008 m
L0_slice=0.35  -> max length error ≈ 1.77e-6 m
L0_slice=0.38  -> max length error ≈ 2.32e-5 m
L0_slice=0.395 -> max length error ≈ 9.37e-6 m
max_abs_theta_error ≈ 1.27e-5 rad
max_branch_violation = 0
```

力学种子静态检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-static-pose-check --diagnostics-only --equilibrium-l-slices 0.35 0.38 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 1.0 --equilibrium-init-drop-steps 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

关键结果：

```text
F_l0_center≈50.7738 N
Tp_center≈0 N*m
```

自由 equilibrium 小范围短扫：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 --equilibrium-fl0-scales 0.8 1.0 1.2 --equilibrium-theta-refs 0 --equilibrium-steps 1000 --equilibrium-eval-steps 300 --equilibrium-wheel-com-kps 80 --equilibrium-wheel-dampings 0.12 --equilibrium-tp-biases 0 --equilibrium-wheel-pitch-kps 0 --equilibrium-wheel-pitch-kds 0 --equilibrium-wheel-world-theta-kps 0 --equilibrium-wheel-world-theta-kds 0 --equilibrium-wheel-base-dx-kds 0 --equilibrium-init-drop-steps 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

结果仍不合格：

```text
best: L0_slice=0.35, F_l0_scale=0.8
F_l0_center≈50.7738 N
F_l0≈40.619 N
Tp_center≈-0.003 N*m
L_mean≈0.277
contact_force_min=0
dL_RMS≈0.276
```

支撑-only 检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 --equilibrium-fl0-scales 0.5 0.8 1.0 --equilibrium-theta-refs 0 --equilibrium-steps 800 --equilibrium-eval-steps 250 --equilibrium-wheel-com-kps 0 --equilibrium-wheel-dampings 0 --equilibrium-theta-kp 0 --equilibrium-theta-kd 0 --equilibrium-pitch-kp 0 --equilibrium-pitch-kd 0 --equilibrium-tp-biases 0 --equilibrium-wheel-pitch-kps 0 --equilibrium-wheel-pitch-kds 0 --equilibrium-wheel-world-theta-kps 0 --equilibrium-wheel-world-theta-kds 0 --equilibrium-wheel-base-dx-kds 0 --equilibrium-init-drop-steps 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

结果：

```text
best: F_l0_scale=1.0
phi_mean≈-0.277
theta_mean≈0.283
dx_RMS≈0.554
dL_RMS≈0.181
contact_force0: left≈52.46, right=0
```

说明只靠腿长支撑不能形成轮腿 true equilibrium；轮端倒立摆控制必须参与让轮子跑到重心下方。但加入轮端后又出现腿长塌短，因此下一步应处理“轮端平衡 + 腿长支撑”的耦合静力残差。

## 结论

- 运动学解算已经有独立门禁，当前对称五连杆解析 IK 误差在 `1e-5 m / 1e-5 rad` 量级。
- equilibrium search 已经改为从经典静力中心值出发，而不是大范围优化。
- 当前失败不是“搜索范围不够大”，而是自由接触下腿长支撑工作点不一致，表现为腿长塌短、接触切换、`dL_RMS` 大。
- 支撑-only 模式会倒下，说明不能把腿长支撑和平衡轮端割裂成两个互不耦合的问题。
- 下一步仍不能进入 LQR 线性化，应先处理给定 `L0` 下的支撑/contact 平衡。

## 下一步

1. 做“支撑-only”局部实验：不加轮端 COM 控制，不加 `Tp` 反馈，只检查 `F_l0_center` 附近能否在自由接触下保持 `L0`。
2. 如果仍塌短，检查 VMC 的 `J_l` 在自由接触构型下是否与解析 IK 分支一致。
3. 必要时把腿长支撑工作点求解改为静力方程残差检查，而不是 rollout 末端点。
