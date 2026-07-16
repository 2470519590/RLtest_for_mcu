# 控制理论

本文只记录当前有效控制算法、公式和物理量定义；实验过程与结果记录在 `tasks/CONTROL_FRAMEWORK.md`。

## 纵向平衡

每条虚拟腿测量 `L, dL, theta_world, dtheta_world`，整体状态和输入为：

```text
X = [theta, dtheta, x, dx, phi, dphi]^T
U = [T, Tp]^T
U = U0 - K (X - X0)
```

当前已恢复的平衡入口使用固定 `L0=0.35 m` 的 locked 工作点：

```text
L0 = 0.35 m
X0 = LOCKED_LQR_X0
U0 = LOCKED_LQR_U0
K  = LOCKED_LQR_K
F_l0 = LOCKED_EQUILIBRIUM_FL0 或显式配置的腿长前馈
```

多腿长工作点、Jacobian/前馈/LQR 调度尚未作为正式控制入口启用。

`T` 是左右驱动轮公共平衡力矩，`Tp` 是虚拟腿中心轴的整体力矩。轮端速度与位置来自轮子相对惯性系的模拟里程计，不使用 `base_x/base_x_dot` 直接替代。

## 虚拟腿与 VMC

```text
theta_world = atan2(r_f, -r_z)
tau_joint = J(q)^T [F_l, Tp_side]^T
```

`theta_world=0` 表示虚拟腿在世界系竖直方向；`F_l>0` 的初始响应为增大虚拟腿长度。五连杆使用解析运动学和解析 Jacobian。正常分支满足：

```text
front_dx > 0, rear_dx > 0, below_front > 0,
below_rear > 0, elbow_span > 0
```

## 腿长与横滚

左右腿基础腿长控制为：

```text
F_l,base = F_l0 + Kp,L (L_d - L)
           + Ki,L integral(L_d - L) dt - Kd,L dL
```

在平地 smoke 中，地面倾角为零，期望横滚姿态产生几何腿长参考：

```text
e_h = w / 2 * sin(gamma_d)
L_d,left  = L0 + e_h
L_d,right = L0 - e_h
```

所有送入 VMC 的左右腿长参考均满足 `L_d,left/right >= 0.16 m`；该下限在横滚几何偏置之后施加。

论文 2.2.2 的动态横滚补偿不修改 `L_d`。它直接叠加到腿长 PID 加前馈后的沿腿推力：

```text
F_roll  = K_gamma (gamma_d - gamma)
F_left  = F_l,base,left  + F_roll
F_right = F_l,base,right - F_roll
tau_left/right = J_left/right(q)^T [F_left/right, Tp_left/right]^T
```

因此 `L_d` 的几何参考与 `F_roll` 的动态抗扰通道必须分离。斜坡地面倾角估计尚未接入。

## 转向与双腿协调

```text
tau_turn = PD(psi_dot_ref - psi_dot)
tau_left_wheel  = T / 2 - tau_turn
tau_right_wheel = T / 2 + tau_turn

e_sync = theta_right - theta_left
Tp_sync = PD(e_sync)
Tp_left  = Tp + Tp_sync
Tp_right = Tp - Tp_sync
```

## 物理语义门禁

下列语义未经重新实验不得修改：`theta_world` 的定义和正方向、`F_l` 正方向、`J^T[F_l,Tp]` 映射、actuator `gear/ctrlrange` 与力矩换算、正常腿型分支，以及工作点控制律 `U=U0-K(X-X0)`。

平衡、摔倒和抗扰恢复只由 MuJoCo viewer 人工确认；曲线只用于定位控制通道、符号、尺度和饱和。
