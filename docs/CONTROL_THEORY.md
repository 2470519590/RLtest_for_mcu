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
  -> 虚拟腿几何 L, theta, dL, dtheta
  -> LQR 状态 X
  -> U_aug = U0 - K (X_aug - X0)
  -> 腿长支撑 F_l = F_l0 + k_l (L0 - L) - d_l dL + delta_F_l
  -> VMC: tau_leg = J(q)^T [F_l, Tp_side]^T
  -> wheel torque: tau_left = tau_right = T / 2
  -> actuator ctrl = tau / gear
```

当前阶段不使用强化学习、PPO、行为克隆或 residual RL。

## 状态与输入

LQR 状态：

```text
X_aug = [theta, dtheta, x, dx, phi, dphi, L, dL]^T
```

定义：

```text
theta  = 0.5 * (theta_world_left + theta_world_right) - phi
dtheta = 0.5 * (dtheta_world_left + dtheta_world_right) - dphi
x      = r_wheel * 0.5 * (q_left_wheel + q_right_wheel) - x_ref
dx     = r_wheel * 0.5 * (dq_left_wheel + dq_right_wheel) - dx_ref
phi    = body pitch
dphi   = body pitch rate
L      = 0.5 * (L_left + L_right)
dL     = 0.5 * (dL_left + dL_right)
```

`x/dx` 默认来自轮子角度和轮速，不使用 `base_x/base_x_dot` 作为主平衡状态。`base_x` 只作为漂移或打滑诊断量。

LQR 输入：

```text
U_aug = [T, Tp, delta_F_l]^T
```

含义：

- `T`：左右轮总力矩。
- `Tp`：虚拟腿角向等效力矩，由左右腿分配执行。
- `delta_F_l`：整体反馈给两条腿的共同支撑力修正，限幅后叠加到腿长支撑环。

控制律：

```text
U_aug = U0 - K * (X_aug - X0)
```

`X0/U0` 必须来自 true equilibrium，不允许默认写成零点。

## 虚拟腿几何

每条腿用髋部参考点到轮侧公共铰点的虚拟杆表示：

```text
p_hip   = 0.5 * (p_front_upper + p_rear_upper)
p_wheel = p_carrier_site
r       = p_wheel - p_hip
L       = sqrt(r_x^2 + r_z^2)
theta   = atan2(r_x, -r_z)
```

速度：

```text
dL     = (r_x v_x + r_z v_z) / L
dtheta = (-r_z v_x + r_x v_z) / L^2
```

符号：

```text
theta = 0  表示虚拟腿在世界系近似竖直
theta > 0  表示轮侧参考点在髋部前方
F_l > 0    初始增大虚拟腿长度
```

## 五连杆解析 Jacobian

VMC 主路径使用解析五连杆 Jacobian：

```text
J(q) = d[L, theta] / d[q_front, q_rear]
```

关节力矩映射：

```text
tau_leg = J(q)^T * [F_l, Tp_side]^T
```

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
F_l = F_l0 + k_l * (L0 - L) - d_l * dL + delta_F_l
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

## 外部扰动测试

风、碰撞和踢击使用施加到 `base` 的世界系外力脉冲：

```text
F_ext(t) = [F_x, 0, 0]
J_ext = integral(F_ext dt)
```

外力通过 MuJoCo `xfrc_applied` 注入，作用结束后清零。禁止直接修改腿角、腿长或瞬移 freejoint 姿态来代替外部扰动。

当前有效工作点量级：

```text
X0 ≈ [0, 0, 0.0280, 0.0035, -0.00163, -0.000092]
U0 ≈ [-0.00136, 0.00213]
L0_actual ≈ 0.35019 m
contact_force_per_wheel ≈ 50.77 N

左右平均状态不包含差模。三维双轮模型在 VMC 分配前使用弱同步阻尼：

```text
e_diff  = theta_left - theta_right
de_diff = dtheta_left - dtheta_right
Tp_sync = -k_sync * e_diff - d_sync * de_diff
Tp_left  = Tp/2 + Tp_sync
Tp_right = Tp/2 - Tp_sync
```

该项只抑制左右反相腿角，不改变整体 LQR 的共模 `Tp`。

轮速离开线性小范围时，限幅前加入安全恢复阻尼：

```text
v_excess = sign(dx) * max(0, abs(dx) - 0.01)
T_used   = clip(T_LQR - 20 * v_excess, -T_max, T_max)
```

该项只用于阻止受扰或减速时轮速继续增大，不能替代 LQR 姿态恢复。
```

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

- 当前 LQR 使用左右平均 `theta`，没有显式控制左右腿差模。
- 当前 `--lqr-true-equilibrium` 是本地 smoke 入口，不是最终训练平台。
- 任何新几何、新腿长截面、新状态定义都必须重新走：解析 IK/Jacobian 检查、true equilibrium、线性化、短 smoke、可视化。
