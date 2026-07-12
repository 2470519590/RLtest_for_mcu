# CONTROL_THEORY.md

本文件只记录当前有效控制框架、公式和物理量定义。历史排查过程写入 `docs/ERROR_CATALOG.md`，实验记录写入 `tasks/CONTROL_FRAMEWORK.md`。

## 当前控制框架

当前入口：

```text
run_smoke.py --lqr-true-equilibrium
```

控制链路：

```text
MuJoCo state
  -> 虚拟腿几何 L, theta_world, dL, dtheta_world
  -> LQR 状态 X = [theta, dtheta, x, dx, phi, dphi]^T
  -> U = U0 - K * (X - X0)
  -> 腿长支撑 F_l = F_l0 + k_l * (L0 - L) - d_l * dL
  -> VMC: tau_leg = J(q)^T * [F_l, Tp_side]^T
  -> wheel torque: tau_left = tau_right = T / 2
  -> actuator ctrl = tau / gear
```

当前阶段不使用强化学习、PPO、行为克隆或 residual RL。

## 观察与结论门禁

涉及平衡、摔倒、扰动恢复和前进运动的物理结论，必须以 MuJoCo 可视化中人类观察到的现象为准。程序输出的数据指标只能用于调试，不得单独作为“已经平衡”或“不会摔倒”的结论。

短时 smoke 只证明程序有限运行。若机器人可能在数秒后才摔倒，`200 step` 不能判断平衡；长时行为必须通过可视化观察确认。

参考文章中的轮腿 LQR + VMC 控制策略视为现实已验证可行的目标框架。若本项目不能复现，应优先检查当前实现的符号、尺度、工作点、线性化、VMC 映射和执行路径，而不是否定文章控制框架。

文章或理论推导已给出解析解时，必须优先使用解析公式。数值方法只允许作为解析解附近的局部工作点搜索或诊断对照，不得替代解析 IK、解析 Jacobian 或解析力学关系。

## LQR 状态与输入

LQR 状态：

```text
X = [theta, dtheta, x, dx, phi, dphi]^T
```

定义：

```text
theta  = 0.5 * (theta_world_left + theta_world_right)
dtheta = 0.5 * (dtheta_world_left + dtheta_world_right)
x      = r_wheel * 0.5 * (q_left_wheel + q_right_wheel) - x_ref
dx     = r_wheel * 0.5 * (dq_left_wheel + dq_right_wheel) - dx_ref
phi    = body pitch
dphi   = body pitch rate
```

`x/dx` 默认来自轮子角度和轮速，不使用 `base_x/base_x_dot` 作为主平衡状态。`base_x/base_x_dot` 只作为漂移或打滑诊断量。

LQR 输入：

```text
U = [T, Tp]^T
```

含义：

- `T`：左右轮总力矩。
- `Tp`：每条腿的虚拟腿角向等效力矩；左右腿各执行同一 `Tp`，再叠加反相同步项。

当前默认执行符号：

```text
lqr_wheel_sign = +1
lqr_pitch_sign = -1
```

其中 `lqr_pitch_sign = -1` 表示 LQR 第二输入通道的正控制量在运行时按反号送入 VMC 角向力矩通道。当前主路径下若保持 `+1`，可视化会出现车体后倒时虚拟腿反向前摆。

有限差分线性化的输入仿真和工作点 `U0` 必须使用同一执行映射；不得用未乘 `lqr_pitch_sign` 的 `B` 矩阵，配合已乘该符号的运行输出。

腿长 `L/dL` 不进入当前有效 LQR 状态；腿长只由 VMC 支撑环控制。曾经尝试的 `X_aug=[theta,dtheta,x,dx,phi,dphi,L,dL]` 和 `U_aug=[T,Tp,delta_F_l]` 不再作为有效平衡控制器。

## 虚拟腿几何

每条腿用髋部参考点到轮侧公共铰点的虚拟杆表示：

```text
p_hip   = 0.5 * (p_front_upper + p_rear_upper)
p_wheel = p_carrier_site
r       = p_wheel - p_hip
L       = sqrt(r_x^2 + r_z^2)
theta_world = atan2(r_x, -r_z)
```

速度：

```text
dL           = (r_x v_x + r_z v_z) / L
dtheta_world = (-r_z v_x + r_x v_z) / L^2
```

符号：

```text
theta_world = 0  表示虚拟腿在世界系近似竖直
theta_world > 0  表示轮侧参考点在髋部前方
F_l > 0          初始增大虚拟腿长度
```

在当前 `L0=0.35 m` 接触工作点的短脉冲辨识中，实际 `T>0` 与实际
`Tp>0` 都使 `theta_world` 初始增大；实际 `Tp>0` 同时使
`theta_rel = theta_world - phi` 初始减小。故 `theta_rel` 与
`theta_world` 不能混作同一个 `Tp` 反馈目标。

当前 LQR 按参考文章使用 `theta = theta_world` 与
`dtheta = dtheta_world`；`phi/dphi` 作为独立的机身世界系姿态状态。

## 五连杆解析 Jacobian

VMC 主路径使用解析五连杆 Jacobian：

```text
J(q) = d[L, theta_world] / d[q_front, q_rear]
```

关节力矩映射：

```text
tau_leg = J(q)^T * [F_l, Tp_side]^T
```

在自由机身下，`Tp` 的关节反作用会同时改变车身 pitch。故若目标是让
虚拟腿相对世界系转动而机身保持近似稳定，`Tp` 不能单独执行，必须由
轮端 `T` 同步提供反作用补偿；该补偿应与 LQR 的有限差分输入映射一致。

在当前模型的前倾单通道识别中，`T+` 同时使 `theta_world` 增大、`pitch` 减小；`Tp+` 使两者都增大。因此 `T` 是当前世界系腿角/车身 pitch 恢复的主执行通道，`Tp` 只能视为身-腿内部力矩通道，不得在未经广义力学分配推导的情况下作为世界系腿角的主控制量。

禁止把带轮地接触的 rollout 数值差分作为 VMC 主 Jacobian。该差分会把地面接触约束混入纯几何微分，使 `dL/dq` 过小。

当前 `L≈0.35 m, theta≈0` 附近，解析量级约为：

```text
J ≈ [
  [ +0.16, -0.16 ],
  [ -0.39, -0.39 ],
]
```

该矩阵只表示局部量级，运行时必须按当前 `q` 重新计算。

## 腿长支撑

腿长控制只负责支撑高度：

```text
F_l = F_l0 + k_l * (L0 - L) - d_l * dL
```

当前锁定工作点：

```text
L0      = 0.35 m
theta0  = 0 rad
F_l0    ≈ 34.5262 N
k_l     = 400
d_l     = 80
```

`F_l0` 是虚拟腿长广义力前馈，不等于单轮地面法向力。改变机构、腿长截面或状态定义后，必须重新辨识 `X0/U0/F_l0/J0/contact_force0`。

## True Equilibrium 门禁

进入线性化和 LQR 前必须有真实接触平衡点：

```text
dX ≈ 0
dL ≈ 0
contact normal force > 0
T/Tp/joint torque 不长期饱和
phi 和 theta 为小角度
接触模式不切换
```

全局 `x` 不要求为 0，因为平地上存在平移对称性。真正重要的是 `dx≈0` 和姿态/接触稳定。

## 左右差模同步

当前 LQR 使用左右平均状态，不显式控制左右腿差模。三维双轮模型在 VMC 分配前使用弱同步阻尼：

```text
e_diff  = theta_left - theta_right
de_diff = dtheta_left - dtheta_right
Tp_sync = -k_sync * e_diff - d_sync * de_diff
Tp_left  = Tp + Tp_sync
Tp_right = Tp - Tp_sync
```

该项只抑制左右反相腿角，不改变整体 LQR 的共模 `Tp`。

## 正常腿型分支

分支判断在 base 机体系中计算：

```text
front_dx    = front_elbow.x - carrier.x
rear_dx     = carrier.x - rear_elbow.x
below_front = front_elbow.z - carrier.z
below_rear  = rear_elbow.z - carrier.z
elbow_span  = front_elbow.x - rear_elbow.x
```

正常分支：

```text
front_dx > 0
rear_dx > 0
below_front > 0
below_rear > 0
elbow_span > 0
```

分支保护只作为几何安全保护，不是主平衡控制器。

## Actuator 映射

当前 actuator：

```text
left_front_motor  -> left_front_drive
left_rear_motor   -> left_rear_drive
right_front_motor -> right_front_drive
right_rear_motor  -> right_rear_drive
left_wheel_motor  -> left_wheel_joint
right_wheel_motor -> right_wheel_joint
```

控制量：

```text
tau  = gear * ctrl
ctrl = clip(tau / gear, ctrlrange)
```

当前 gear：

```text
leg motor gear   = 45
wheel motor gear = 12
```

## 已知限制

- 当前 `--lqr-true-equilibrium` 只是实验入口；是否真正平衡必须看可视化，不能由短时数据门禁判定。
- 小扰动测试数据只说明某些量在阈值内，不代表视觉上已经恢复，也不代表不会在更长时间后摔倒。
- 任何新几何、新腿长截面、新状态定义都必须重新走：解析 IK/Jacobian 检查、true equilibrium、线性化、短 smoke、可视化。
