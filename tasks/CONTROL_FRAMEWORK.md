# CONTROL_FRAMEWORK.md

本文件记录当前轮腿控制框架的大任务进展。后续同类小实验只更新本文件，不再新增零散任务文档。

## 目标

把 MuJoCo 轮腿实验脚本整理成可维护的本地 smoke 项目：

- 控制框架清楚。
- 物理语义稳定。
- 文档短而准。
- 单个 Python 文件目标小于 2000 行。
- 当前只做本地 smoke，不做训练。

## 当前控制框架

当前有效入口：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --virtual-rod-steps 200
```

可视化入口：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --visualize --visualize-seconds 8
```

控制链路：

```text
解析五连杆 IK/Jacobian
  -> true equilibrium
  -> X0/U0/F_l0/J0/contact_force0
  -> finite-difference LQR
  -> VMC: tau = J^T [F_l, Tp]^T
  -> wheel torque T/2
```

## 已确认关键事实

- XML 机构不是当前优先问题，解析 IK 能到 `L=0.35, theta=0`。
- `F_l > 0` 初始增大虚拟腿长度，符号没有反。
- 旧主错是接触污染数值 Jacobian 把 `dL/dq` 压小，导致支撑力矩不足。
- VMC 主路径必须使用解析五连杆 Jacobian。
- LQR 必须围绕 true equilibrium：`U = U0 - K(X-X0)`。
- `x/dx` 主状态来自轮子角度和轮速，不来自 base 速度。
- 当前平均 LQR 没有显式控制左右腿差模。

## 当前锁定工作点

```text
L0 = 0.35 m
theta0 = 0 rad
F_l0 ≈ 30.4643 N
X0 ≈ [-0.00015, -0.00113, -0.02003, 0.00346, 0.000088, -0.000522]
U0 ≈ [0.00168, -0.000038]
contact_force_per_wheel ≈ 48 N
```

## 本轮结构整理

当前分包：

- `core/`：常量、数据结构、MuJoCo 通用工具。
- `model/`：actuator 映射、五连杆解析几何、运行时运动学、接触/静态采样。
- `control/`：IK、VMC、LQR 状态和有限差分 LQR 设计。
- `experiments/`：equilibrium search、五连杆检查、`F_l` 测试、trace、virtual rod smoke。
- `io/`：CLI 和 CSV/绘图输出。
- `runner.py`：顶层 smoke 编排。
- `model_smoke.py`：模型 smoke、扫描和 viewer overlay；后续只保留入口相关轻量逻辑。

已整理：

- 旧小任务文件已移动到 `tasks/archive/`。
- 主任务目录只保留 `tasks/CONTROL_FRAMEWORK.md`。
- `runner.py` 已从临时 `globals().update` 桥接改为显式导入。
- `equilibrium.py`、`virtual_rod.py`、`trace.py`、`fl_tests.py`、`fivebar_checks.py` 已去掉临时 `from model_smoke import *` 和 `globals().update` 桥接。
- `equilibrium.py`、`virtual_rod.py`、`fivebar_checks.py`、`fl_tests.py`、`lqr_design.py` 不再从 `model_smoke.py` 拉公共 mechanics/IK/VMC helper。
- `model_smoke.py` 已从杂糅实现降到约 621 行，只保留模型 smoke、扫描、viewer overlay 和少量入口相关流程。
- `src/robot_smoke/` 已从单层平铺改为 `core/model/control/experiments/io` 分包。
- `PROJECT_ROOT` 在 `core/constants.py` 中按新层级修正，避免默认模型路径指向 `src/assets`。
- 修复分包迁移后 `model_smoke.py` 可视化路径漏导入 `_prepare_lqr_operating_point` / `_lqr_middle_control` 的问题。

待拆：

- `viewer.py`：MuJoCo viewer overlay 后续可继续从 `model_smoke.py` 拆出。

当前 Python 文件行数已满足单文件小于 2000 行：

```text
runner.py                          1029
model_smoke.py                      621
experiments/equilibrium.py          591
control/lqr_design.py               579
experiments/virtual_rod.py          556
```

## 验证记录

最近通过：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\core\__init__.py src\robot_smoke\core\constants.py src\robot_smoke\core\types.py src\robot_smoke\core\mujoco_utils.py src\robot_smoke\model\__init__.py src\robot_smoke\model\actuators.py src\robot_smoke\model\fivebar.py src\robot_smoke\model\kinematics.py src\robot_smoke\model\mechanics.py src\robot_smoke\control\__init__.py src\robot_smoke\control\ik.py src\robot_smoke\control\lqr.py src\robot_smoke\control\lqr_design.py src\robot_smoke\control\vmc.py src\robot_smoke\experiments\__init__.py src\robot_smoke\experiments\equilibrium.py src\robot_smoke\experiments\fivebar_checks.py src\robot_smoke\experiments\fl_tests.py src\robot_smoke\experiments\trace.py src\robot_smoke\experiments\virtual_rod.py src\robot_smoke\io\__init__.py src\robot_smoke\io\cli.py src\robot_smoke\io\output.py src\robot_smoke\model_smoke.py src\robot_smoke\runner.py
```

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --virtual-rod-steps 200
```

结果：

```text
L0_slices: [0.35]
qualified=True
saturated_steps=0
final_left_branch_violation=0
final_right_branch_violation=0
result: PASS finite model/load/step smoke
```

## 当前问题

- 旧小任务文档已归档到 `tasks/archive/`，尚未物理删除。
- `model_smoke.py` 仍保留 viewer overlay 和部分 smoke 扫描流程；后续可继续拆 `viewer.py`，但不应在整理时改控制公式。
- `model/mechanics.py` 当前包含静态工作点采样输出，依赖 `control.lqr` 和 `control.vmc`；后续若继续清架构，可考虑把采样输出再拆到 `experiments/`，让 `model/` 更纯。
- 后续不能再把控制理论文档写成流水账。

## 腿长、差模与运动测试

- 默认腿长恢复为 `0.35 m`。
- equilibrium 新增腿长目标误差和左右腿差模门禁。
- 0.35 m 工作点：`L_mean=0.350189 m`，`F_l0≈34.5262 N`，无饱和，左右差模接近数值噪声。
- 扰动测试已改为站稳后向 `base` 施加世界系水平外力，不再直接修改 pitch 或腿状态。
- 外力测试当前从 `2 N × 0.1 s` 小冲量开始。DARE 上限由 800 提高到 10000，当前约 7475 次收敛；未收敛 K 是此前长期发散的重要原因。
- LQR 状态 `theta` 已改为相对车体角：`theta_world-phi`，临时 equilibrium 控制仍使用世界腿角。
- 增加轮速越界恢复阻尼后，外力后的轮速峰值明显下降，但 `Tp` 姿态通道仍会把机体带出线性域并摔倒；`Tp` 暂限 2 N·m，结果仍未通过。
- runner 现在输出 `behavior_qualified`，摔倒或恢复不合格时返回失败，不再打印伪 PASS。
- 修正 theta 扰动后重新投影 `x/dx`，`A[x,theta]` 从约 `0.296` 降到约 `2e-5`，确认旧线性化存在状态串扰。
- NumPy Hamiltonian CARE 也无法让真实零扰动 rollout 稳定，说明当前六状态 A/B 未包含腿长/接触内部动态。后续应扩展线性化状态或严格约束隐藏状态一致，不能继续叠加经验恢复项。
- 已加入移动位置/速度参考入口 `--target-speed` 和加速度斜坡。
- 当前非零速度跟踪未通过：即使 `0.005 m/s` 也出现参考误差持续增长。下一步先做 `T` 脉冲和 `x_ref` 小阶跃，确认 wheel state、base `+X` 和 `T` 的局部 DC 增益，不继续放大速度指令。
- 抗扰结构已增广为 `X_aug=[theta,dtheta,x,dx,phi,dphi,L,dL]`、`U_aug=[T,Tp,delta_F_l]`。双输入增广模型可控秩仅 `6/8`；加入共同支撑力修正并使用 5 ms 辨识窗后，标准 DARE 成功，短时闭环特征值 `max|eig|≈0.999636`。
- viewer 保持 1 ms 控制/物理步，只约每 16 步同步一次画面，目标刷新率约 60 Hz，避免逐步 OpenGL 同步造成慢放。
- 已删除线性化模型外的经验轮速恢复阻尼。3 秒无扰动测试不再立即倒地，腿长约 `0.3476 m` 且无饱和，但轮速仍持续到约 `-9.25 rad/s`，`behavior_qualified=False`；小外力测试同样未通过。下一步必须做 `theta_rel -> T`、`phi -> T` 小脉冲闭环符号辨识，当前不得宣称抗扰完成。
