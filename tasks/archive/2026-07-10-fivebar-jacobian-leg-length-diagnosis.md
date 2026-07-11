# 任务：解析 Jacobian 对照并修复腿长支撑映射

## 目标

当前现象是：解析 IK 可以到达 `L=0.35 m`，但自由接触和重量作用下腿长达不到目标；车体倾倒后腿长反而可能回到目标附近。任务目标是先排查五连杆解析 IK、解析 Jacobian、数值 Jacobian 和 VMC `J^T F_l` 映射，不继续盲目调 LQR 或 PD 参数。

## 修改

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- 新增五连杆解析 Jacobian：
  - 由上杆驱动角得到前后 elbow；
  - 由两条下杆闭链约束隐式求导得到 `dC/dq_front`、`dC/dq_rear`；
  - 再链式得到 `J=d[L,theta]/d[q_front,q_rear]`。
- VMC 主路径改为优先使用解析 Jacobian。
- 原接触参与的 settle 数值差分改名为 `_settled_numeric_leg_shape_jacobian()`，只用于诊断或解析失败回退。
- 新增 `--fivebar-jacobian-check`：
  - 输出解析 IK、解析 `L/theta`、MuJoCo 实测 `L/theta`；
  - 输出解析 Jacobian、旧数值 Jacobian、差值；
  - 输出同一 `F_l` 下解析/数值 `tau_support`；
  - 可选做短 support-only load 段。
- 将默认 `--virtual-rod-length-force-ff` 从 `60` 改为 `33`。
- 将默认 `--fl-pulse-base-force` 从 `60` 改为 `33`。

更新 [docs/CONTROL_THEORY.md](E:/STM32_PROJ/RL_training/docs/CONTROL_THEORY.md)：

- 记录解析 Jacobian 公式。
- 说明 `F_l` 是虚拟腿长坐标的广义力，不等同于地面法向力。
- 记录 `L≈0.35, theta≈0` 附近解析 Jacobian 量级。

更新 [docs/ERROR_CATALOG.md](E:/STM32_PROJ/RL_training/docs/ERROR_CATALOG.md)：

- 记录“地面接触污染的数值 Jacobian 会把腿长支撑力矩压小”。

## 关键验证结果

Jacobian 对照命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --fivebar-jacobian-check --diagnostics-only --fivebar-jacobian-l-slices 0.35 --fivebar-jacobian-theta-refs 0 --fivebar-jacobian-force-scale 1.0 --fivebar-jacobian-load-steps 200 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

修复前诊断暴露：

```text
analytic_J length row ≈ [0.162771, -0.162770]
numeric_J length row  ≈ [0.00363, -0.00269] 到 [0.01550, -0.01456]
tau_support_analytic_Fl ≈ [8.264, -8.264] N*m
tau_support_numeric_Fl  ≈ [0.184, -0.136] 到 [0.787, -0.739] N*m
```

说明旧数值 Jacobian 被轮地接触严重污染，导致 VMC 支撑力矩过小。

修复后同一 support-only load：

```text
initial_L: left=0.350002, right=0.350002
final_L: left≈0.4045, right≈0.4045
saturated_steps=0
max_abs_joint_tau≈8.264
```

这说明力映射已经恢复到足够尺度，旧“重量下塌腿”不是 `F_l` 符号反，也不是前馈没有进入 VMC。

短 equilibrium 扫描：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 0.45 0.55 0.65 0.75 0.85 0.95 --equilibrium-steps 600 --equilibrium-eval-steps 180 --equilibrium-wheel-com-kps 80 --equilibrium-wheel-dampings 0.12 --equilibrium-tp-biases 0 --equilibrium-init-drop-steps 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

最佳候选：

```text
F_l0_scale=0.65
L_mean≈0.347304
dL_RMS≈0.00203
contact_force_min≈50.7066
joint_sat_ratio=0
qualified=True
```

## 结论

- 五连杆解析 IK 本身没有问题。
- 旧运行时数值 Jacobian 的长度行被地面接触压小，是重量下腿长支撑失败的核心原因。
- VMC 主路径应使用解析 Jacobian，不能使用带接触 rollout 的差分 Jacobian。
- 对当前 `L0=0.35, theta=0`，`F_l0≈33N` 比 `m*g/2≈50.77N` 更接近真实广义静态前馈；`m*g/2` 只能作为尺度初值，不能直接当 VMC `F_l0` 工作点。

## 下一步

1. 用解析 Jacobian 后重新做较长 equilibrium search，确认 `L≈0.35` 能持续 3~5 秒。
2. 重新构造 `X0/U0/F_l0/J0/contact_force0`。
3. 只有 true equilibrium 稳定后，再围绕该点做线性化和 LQR。

## 追加：true equilibrium 门禁与 LQR 工作点接入

### 修改

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- `EquilibriumSearchResult` 保留最终 `MjData`，用于后续局部线性化和局部 LQR rollout。
- 新增 `--lqr-use-equilibrium-operating-point`：
  - 仅在显式启用时，把通过门禁的 best equilibrium 作为自动 LQR 的操作点。
  - 同步设置运行时 `lqr_x0/lqr_u0`。
  - 同步把 `virtual_rod_length_force_ff` 设置为该工作点的 `F_l0`。
- 自动 LQR 有限差分改为围绕 `U0`：

```text
U = U0 + dU
U = U0 - dU
```

而不是旧的围绕零输入差分。

- `virtual_rod_test` 支持从外部初态启动；当使用 equilibrium 工作点时，局部 LQR smoke 从同一个 true equilibrium 接触状态起步，不再重新使用旧锁 base 操作点。

### 5 秒 true equilibrium 验证命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 0.6 --equilibrium-steps 5000 --equilibrium-eval-steps 2000 --equilibrium-length-kp 400 --equilibrium-length-kd 80 --equilibrium-theta-kp 3 --equilibrium-theta-kd 2.0 --equilibrium-pitch-kp 3 --equilibrium-pitch-kd 2.0 --equilibrium-wheel-com-kps 80 --equilibrium-wheel-dampings 0.5 --equilibrium-tp-biases 0 --equilibrium-init-drop-steps 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

关键结果：

```text
qualified=True
L_mean=0.349399
com_offset_mean=3.39686e-05
phi_mean=6.17212e-05
theta_mean=-0.000120395
dx_RMS=0.00249105
dphi_RMS=0.00930719
dtheta_RMS=0.00611926
dL_RMS=0.00250027
T_sat_ratio=0
Tp_sat_ratio=0
joint_sat_ratio=0
contact_force_mean=50.7394
contact_force_min=45.0991
slip_indicator=0.000895703
```

记录的新工作点：

```text
X0 = [-0.000151483, -0.00112651, -0.0200273, 0.00345884, 8.78245e-05, -0.00052178]
U0 = [0.00168131, -3.75119e-05]
F_l0_config = 30.4643 N
F_l_cmd = left 30.5 N, right 30.5557 N
L0 = left 0.349368 m, right 0.349344 m
dL0 = left 0.00282829 m/s, right 0.00218779 m/s
contact_force0 = left 48.0876 N, right 47.8452 N
J0_left =
  [0.165257, -0.147732]
  [-0.335172, -0.44289]
J0_right =
  [0.147756, -0.165281]
  [-0.442876, -0.33521]
tau_support left  = [5.04112, -4.50741] N*m
tau_support right = [4.51628, -5.05097] N*m
```

### LQR 线性化与局部闭环验证

命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 0.6 --equilibrium-steps 5000 --equilibrium-eval-steps 2000 --equilibrium-length-kp 400 --equilibrium-length-kd 80 --equilibrium-theta-kp 3 --equilibrium-theta-kd 2.0 --equilibrium-pitch-kp 3 --equilibrium-pitch-kd 2.0 --equilibrium-wheel-com-kps 80 --equilibrium-wheel-dampings 0.5 --equilibrium-tp-biases 0 --equilibrium-init-drop-steps 0 --virtual-rod-test --virtual-rod-steps 5000 --left-rod-length 0.35 --right-rod-length 0.35 --left-rod-theta 0 --right-rod-theta 0 --virtual-rod-length-kp 400 --virtual-rod-length-kd 80 --lqr-test --lqr-auto-design --lqr-use-equilibrium-operating-point --lqr-design-steps 1 --lqr-control-period-steps 1 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

关键结果：

```text
lqr_design_operating_state = X0
lqr_u0 = [0.00168131, -3.75119e-05]
vmc_length_force_ff_or_F_l0 = 30.4643
lqr_design_closed_loop_max_abs_eig = 0.99971
finite PASS
saturated_steps = 0
final_left_branch_violation = 0
final_right_branch_violation = 0
final_lqr_state:
  theta=-0.00103414
  theta_rate=0.00027128
  x=0.06548
  x_rate=0.0510507
  pitch=0.000445576
  pitch_rate=-0.0177424
```

观察到的新问题：

```text
left_final theta  ≈ -0.352 rad
right_final theta ≈ +0.350 rad
平均 theta 仍接近 0
```

这说明当前文章式平均状态 LQR 已经进入正确工作点并能维持对称平均模态，但没有显式控制左右差模。下一步不要继续盲目调平均 LQR 权重，应增加左右差模诊断或阻尼，再做更长时间可视化。

### 追加：锁定 true-equilibrium 入口并压缩参数

修改 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- 新增短入口 `--lqr-true-equilibrium`。
- 该入口强制使用当前已验证参数：

```text
L0_slice=0.35
theta_ref=0
F_l0_scale=0.6
equilibrium_steps=5000
equilibrium_eval_steps=2000
length_kp=400
length_kd=80
theta_kp=3
theta_kd=2
pitch_kp=3
pitch_kd=2
wheel_com_kp=80
wheel_damping=0.5
Tp_bias=0
lqr_design_steps=1
lqr_control_period_steps=1
```

- 常用 `--help` 不再显示 equilibrium 网格、LQR 细项、VMC 低层调参、F_l 诊断和五连杆诊断参数；这些参数仅作为隐藏诊断兼容入口保留。
- `--lqr-true-equilibrium` 下 `zero/probe/pd_hold` 预检缩短为 1 步，避免输出淹没 true equilibrium 和 LQR 结果。

可视化命令已运行：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 0.6 --equilibrium-steps 5000 --equilibrium-eval-steps 2000 --equilibrium-length-kp 400 --equilibrium-length-kd 80 --equilibrium-theta-kp 3 --equilibrium-theta-kd 2.0 --equilibrium-pitch-kp 3 --equilibrium-pitch-kd 2.0 --equilibrium-wheel-com-kps 80 --equilibrium-wheel-dampings 0.5 --equilibrium-tp-biases 0 --equilibrium-init-drop-steps 0 --virtual-rod-test --virtual-rod-steps 5000 --left-rod-length 0.35 --right-rod-length 0.35 --left-rod-theta 0 --right-rod-theta 0 --virtual-rod-length-kp 400 --virtual-rod-length-kd 80 --lqr-test --lqr-auto-design --lqr-use-equilibrium-operating-point --lqr-design-steps 1 --lqr-control-period-steps 1 --visualize --visualize-seconds 8 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

同等短命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --visualize --visualize-seconds 8
```

短命令 smoke 已验证：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --virtual-rod-steps 1200
```

关键结果：

```text
qualified=True
F_l0=30.4643
lqr_x0 与 true equilibrium operating_state 一致
lqr_u0=[0.00168131, -3.75119e-05]
max_length_error=0.00172821
saturated_steps=0
final_left_branch_violation=0
final_right_branch_violation=0
```

### 追加：拆分 `model_smoke.py` 中的非控制职责

用户要求不能把所有代码继续堆在 [model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)，也不能继续暴露大量 parser 参数。本轮做结构性拆分，不改变物理公式、控制符号或 XML。

新增文件：

- [constants.py](E:/STM32_PROJ/RL_training/src/robot_smoke/constants.py)：项目路径、默认模型、锁定 true-equilibrium 参数、默认 LQR 矩阵。
- [types.py](E:/STM32_PROJ/RL_training/src/robot_smoke/types.py)：所有 dataclass 结果结构，包括 LQR、VMC、trace、equilibrium、五连杆诊断结果。
- [cli.py](E:/STM32_PROJ/RL_training/src/robot_smoke/cli.py)：唯一命令行 parser。只暴露稳定入口，其余内部参数使用锁定默认值。
- [output.py](E:/STM32_PROJ/RL_training/src/robot_smoke/output.py)：CSV、history plot、motor torque plot、control trace plot 输出函数。

当前职责边界：

```text
run_smoke.py          根目录可运行入口
cli.py                CLI 参数，只保留短入口
constants.py          当前锁定工作点和常量
types.py              数据结构
output.py             文件输出和画图
model_smoke.py        MuJoCo 状态、五连杆、VMC、equilibrium、LQR 主实现
```

`run_smoke.py --help` 当前只显示：

```text
--model
--visualize
--visualize-seconds
--virtual-rod-steps
--lqr-true-equilibrium
--history-csv
--history-plot
--motor-torque-plot
--diagnostics-only
--no-realtime
```

验证命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\constants.py src\robot_smoke\types.py src\robot_smoke\cli.py src\robot_smoke\output.py src\robot_smoke\model_smoke.py
```

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --virtual-rod-steps 200
```

结果：

```text
py_compile 通过
help 只显示短入口
200 步 true-equilibrium LQR smoke 通过 finite
qualified=True
saturated_steps=0
final_left_branch_violation=0
final_right_branch_violation=0
```

后续还需要继续拆：

```text
fivebar.py       五连杆解析 IK/Jacobian/分支
vmc.py           VMC 和支撑优先分配
lqr.py           LQR 状态、线性化、Riccati
equilibrium.py   true equilibrium 搜索和门禁
viewer.py        MuJoCo viewer overlay
```

本轮先拆走无物理耦合的部分，避免一次性大搬迁引入控制行为变化。

### 追加：继续拆出 MuJoCo 通用工具，并记录立起来的排查原因

新增 [mujoco_utils.py](E:/STM32_PROJ/RL_training/src/robot_smoke/mujoco_utils.py)：

- MuJoCo import 环境处理。
- name / id 查询。
- body/site 位置速度读取。
- finite 检查。
- `MjData` copy。
- base lock 工具。
- 简单 ctrl step。

`model_smoke.py` 继续保留控制主逻辑，但不再直接定义这些通用 MuJoCo helper。

本次“可以立起来”的严格原因已写入 [docs/ERROR_CATALOG.md](E:/STM32_PROJ/RL_training/docs/ERROR_CATALOG.md)。结论不是单个参数调优成功，而是：

```text
解析 Jacobian 修复支撑力矩尺度
true equilibrium 门禁找到自由接触工作点
X0/U0/F_l0/J0/contact_force0 同步
有限差分围绕 U0
LQR 控制律围绕同一工作点运行
```

验证命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\constants.py src\robot_smoke\types.py src\robot_smoke\cli.py src\robot_smoke\output.py src\robot_smoke\mujoco_utils.py src\robot_smoke\model_smoke.py
```

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --virtual-rod-steps 200
```

结果：

```text
py_compile 通过
200 步 true-equilibrium LQR smoke 通过
qualified=True
saturated_steps=0
final_left_branch_violation=0
final_right_branch_violation=0
```

### 追加：拆出五连杆几何模块

本轮继续按“只拆结构，不改物理语义”的方式拆分 [model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)。

新增 [fivebar.py](E:/STM32_PROJ/RL_training/src/robot_smoke/fivebar.py)：

- 五连杆 base 平面解析几何工具：
  - `body_local_xz`
  - 两圆交点
  - 上杆驱动角和上杆向量互算
  - 驱动关节角读取
- 当前已验证的解析 IK：
  - `equilibrium_analytic_ik_targets`
- 当前已验证的解析 Jacobian：
  - `analytic_fivebar_kinematics_from_q`
- 正常腿型分支指标：
  - `compute_leg_branch_metrics`
  - `leg_branch_guard_error`

[model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py) 保留原下划线函数名别名导入，调用点不改，避免在拆分时改变控制行为。

本次“能立起来”的排查结论再次确认：

```text
1. 不是 XML 机构优先错误：解析 IK 能到 L=0.35, theta=0。
2. 不是 F_l 符号反：+F_l 初始会让虚拟腿伸长。
3. 核心旧错是接触污染数值 Jacobian，把 dL/dq 压得过小，导致 J^T F_l 支撑力矩不足。
4. 仅有解析 Jacobian 还不够，必须用 true equilibrium 作为 X0/U0/F_l0/J0/contact_force0 来源。
5. LQR 线性化和运行时控制律必须围绕同一个 U0，而不是围绕零输入。
6. L0=0.35 是局部冻结腿长截面/调度量，不是“机器人只能搜索出的固定腿长”。
7. 当前平均状态 LQR 只稳定左右对称模态，左右腿 theta 差模仍需要后续单独处理。
```

验证命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\constants.py src\robot_smoke\types.py src\robot_smoke\cli.py src\robot_smoke\output.py src\robot_smoke\mujoco_utils.py src\robot_smoke\fivebar.py src\robot_smoke\model_smoke.py
```

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --virtual-rod-steps 200
```

结果：

```text
py_compile 通过
200 步 true-equilibrium LQR smoke 通过
qualified=True
saturated_steps=0
final_left_branch_violation=0
final_right_branch_violation=0
```
