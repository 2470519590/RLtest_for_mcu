# CONTROL_THEORY.md

本文只定义当前控制算法、公式与物理量。实验过程写入 `tasks/CONTROL_FRAMEWORK.md`，错误原因写入 `docs/ERROR_CATALOG.md`。

## 平衡主链路

```text
MuJoCo state
  -> virtual leg: L, theta_world, dL, dtheta_world
  -> X = [theta, dtheta, x, dx, phi, dphi]^T
  -> U = [T, Tp]^T = U0 - K (X - X0)
  -> F_l = F_l0 + k_l (L0 - L) - d_l dL
  -> tau_leg = J(q)^T [F_l, Tp_side]^T
  -> tau_left_wheel = tau_right_wheel = T / 2
```

`x/dx` 使用轮子在世界系中的运动换算得到的模拟里程计；不以 `base_x/base_x_dot` 作为主平衡状态。`L0=0.35 m` 是当前固定实验截面，不代表唯一可用腿长。

LQR 工作点形式为：

```text
U = U0 - K (X - X0)
```

当前执行符号为 `lqr_wheel_sign=+1`、`lqr_pitch_sign=-1`。线性化输入映射必须使用同一执行符号。

## 虚拟腿与 VMC

每条腿以髋部参考点到 carrier 公共铰点的向量 `r` 定义：

```text
h = normalize(project_xy(R_base [1, 0, 0]^T))
r_f = h^T r
L = sqrt(r_f^2 + r_z^2)
theta_world = atan2(r_f, -r_z)
```

`theta_world=0` 表示虚拟腿在世界系竖直；`theta_world>0` 表示轮侧参考点位于髋部前方。`F_l>0` 的初始响应为增大 `L`。

五连杆关节力矩由解析 Jacobian 映射：

```text
tau_joint = J(q)^T [F_l, Tp_side]^T
```

分支保护只检查五连杆几何，不承担平衡：

```text
front_dx, rear_dx, below_front, below_rear, elbow_span > 0
```

## 转向与双腿协调

转向在公共平衡轮端力矩 `T` 上叠加差动轮端力矩：

```text
tau_left_wheel  = T / 2 - tau_turn
tau_right_wheel = T / 2 + tau_turn
```

三档航向角速度幅值为：

```text
low    = pi / 2 rad/s
medium = pi     rad/s
high   = 2 pi   rad/s
```

`--turn-speed` 选择档位；`--turn-test` 使用选中档位的单一梯形参考，1.0 s 开始、0.15 s 升速、5.0 s 开始、0.15 s 降为零。

航向 PD：

```text
e_yaw = psi_dot_ref - psi_dot
tau_turn = clip(Kp_yaw e_yaw + Kd_yaw de_yaw, -2, 2)
```

双腿协调 PD：

```text
e_sync = theta_right - theta_left
Tp_sync = Kp_sync e_sync + Kd_sync de_sync
Tp_left = Tp + Tp_sync
Tp_right = Tp - Tp_sync
```

两个 PD 都使用三段测量链路：

```text
raw measurement -> INPUT LPF
raw error       -> ERROR LPF -> P
raw derivative  -> DERIVATIVE LPF -> D
```

六个截止频率位于 `core/constants.py`：

```text
YAW_TURN_INPUT_LOWPASS_HZ
YAW_TURN_ERROR_LOWPASS_HZ
YAW_TURN_DERIVATIVE_LOWPASS_HZ
LEG_SYNC_INPUT_LOWPASS_HZ
LEG_SYNC_ERROR_LOWPASS_HZ
LEG_SYNC_DERIVATIVE_LOWPASS_HZ
```

当前不启用 roll 支撑补偿，也不包含斜坡地形测试。

## 执行器映射

```text
left_front_motor, left_rear_motor   -> 左腿两驱动关节
right_front_motor, right_rear_motor -> 右腿两驱动关节
left_wheel_motor, right_wheel_motor -> 左右轮关节

tau = gear * ctrl
ctrl = clip(tau / gear, ctrlrange)
```

腿部电机 `gear=45`，轮电机 `gear=12`。

## 观察门禁

平衡、摔倒、扰动恢复和运动性能只由 MuJoCo viewer 中的人工观察确认。曲线用于定位控制通路、符号、尺度和饱和，不构成物理成功结论。文章已给出解析 IK、Jacobian 或力学关系时，正式控制优先使用解析实现。
