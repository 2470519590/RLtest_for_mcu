# 控制理论

本文只记录当前有效控制算法、公式和物理量定义；实验过程与结果记录在 `tasks/CONTROL_FRAMEWORK.md`。

## 纵向平衡

每条虚拟腿测量 `L, dL, theta_world, dtheta_world`，整体状态和输入为：

```text
X = [theta, dtheta, x, dx, phi, dphi]^T
U = [T, Tp]^T
U = U0 - K (X - X0)
```

当前默认腿长和工作点以 `config/smoke.yaml` 为准。当前默认：

```text
L_ref = 0.24 m
length_schedule = true
X0, U0, K, F_l0 = length_schedule.yaml 按实际腿长插值
```

`LOCKED_*` 仅作为调度关闭时的 fallback / 诊断基线，不是当前默认运行语义。

`T` 是左右驱动轮公共平衡力矩，`Tp` 是虚拟腿中心轴的整体力矩。轮端速度与位置来自轮子相对惯性系的模拟里程计，不使用 `base_x/base_x_dot` 直接替代。

`x` 通道表示局部线性化坐标，不是全局站位任务。所有地面模式默认用当前自身里程计位置作为 `x_ref`，使 `x-X0` 中性化；前进、冲击、转向等任务只通过 `dx_ref`、姿态和腿角产生恢复动作，不追踪或回正世界系绝对位置。

## 控制器接口模式

运行时中层控制器支持两种接口模式：

```text
rl_controller_mode = lqr
U_mid = U_lqr

rl_controller_mode = lqr_residual
U_mid = clip(U_lqr + U_residual, limits)
```

其中 `U_lqr=[T,Tp]` 仍由原始 LQR 控制律生成；`U_residual` 是后续 RL 策略的有界增量接口。当前没有加载训练策略，默认 residual 为零。接口模式只允许在 LQR 输出之后叠加残差，不改写 LQR 的状态定义、工作点、符号、VMC 映射或底层控制逻辑。

Residual RL 的动作通道为：

```text
delta_T, delta_Tp, delta_F_l_common,
delta_L_ref,left, delta_L_ref,right
```

`delta_L_ref,left/right` 只改变送入现有腿长/IK/VMC 链路的参考值，并且总是经过 `minimum_leg_length..maximum_leg_length` 限幅。离地时，论文第 3 节门控仍作为名义 LQR 基线：纯 `lqr` 模式保持 `T=0`、只保留 `Tp(theta,dtheta)`；`lqr_residual` 模式允许策略在该基线之上通过受限 residual 学习起跳、空中姿态、收腿/伸腿和落地平衡。

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
## 机器人离地检测

论文第 3 节用轮端向上支撑力 `F_N` 判断双轮离地。MuJoCo 测试中直接用左右轮接触法向力作为 `F_N` 的等价值：

```text
airborne = (F_N,left < 20 N) and (F_N,right < 20 N)
```

为避免 reset 后接触尚未解析时的 `F_N=0` 误判，检测器必须先看到任一轮 `F_N >= 20 N` 后才允许进入离地状态。纯 LQR 离地基线执行论文第 3 节的增益门控：反馈增益矩阵除 `K21,K22` 外全部置零，等价于轮端公共力矩 `T=0`，只保留 `Tp` 对 `theta,dtheta` 的反馈，即仅保留 `K[1,0]` 和 `K[1,1]`，用于在空中保持虚拟腿姿态接近竖直。`lqr_residual` 模式下，该门控是 residual RL 的名义基线，策略仍可通过受限残差优化离地、空中和落地动作。

离地检测不得额外接管腿长参考、速度参考或落地刹车；重新接触后直接恢复完整 LQR。腿长缓冲应由正常腿长弹簧阻尼和机械行程承担，不在离地检测状态机里冻结或改写 `L_ref/dx_ref/x_ref`。
