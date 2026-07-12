# CONTROL_FRAMEWORK.md

> 2026-07-11 更正：主路径恢复文章状态 `X=[theta_world,dtheta_world,x,dx,phi,dphi]`。文章摆杆向量为“轮轴到机体”，代码为“机体到轮轴”，所以 `theta_article=-theta_world`；结合单通道脉冲，文章输入应执行为 `lqr_wheel_sign=+1`、`lqr_pitch_sign=-1`。此前的相对模态坐标及增益硬投影使局部闭环特征值超过 1，已完整撤回。

> 轮端/Tp 通道试验：从 true equilibrium 施加 `phi=+0.04 rad` 后，当前闭环在 0.3 s 内已令轮角由 `-0.330` 变为 `-0.022 rad`、`theta_world` 由 `-0.041` 变为 `-0.003 rad`，证明轮端与 VMC 执行存在且方向可用；同期 `Tp` 初值约 `+1.23`，按文章方程会增大 `phi`，是“只抬头”的直接来源。为先验证轮端倒立摆恢复，当前把 LQR 代价设为 `R=[0.1,10000]`，使 `Tp` 成为近似禁用的辅助输入，后续只在完成受约束分配推导后再恢复其公共平衡职责。

> 输出平滑：前倾局部记录中 `T` 在 25 ms 内由 `+8` 切至 `-1.2`，来源是 `theta/dtheta` 项瞬时换向，不是轮电机覆盖或 `x/dx` 位置回正。启用已有的 `lqr_output_rate_limit=1000 N*m/s`；按当前实现，每个 5 ms 控制周期最多变化 `1 N*m`，限制突发前冲而不改变 LQR、VMC、模型或符号定义。

本文件记录当前轮腿控制框架的大任务进展。后续同类小实验只更新本文件，不再新增零散任务文档。

- 转向时不得把世界 X 直接作为前进状态。`x/dx` 现由轮心世界速度沿当前车头前向 `h` 投影并积分得到；虚拟腿的前后分量也使用 `r_f=h^T r`。这保持世界竖直定义不变，并避免 yaw 到 `+/-90 deg` 时固定世界 X 投影退化、同步环误判劈叉。

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

最近语法检查通过：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\core\__init__.py src\robot_smoke\core\constants.py src\robot_smoke\core\types.py src\robot_smoke\core\mujoco_utils.py src\robot_smoke\model\__init__.py src\robot_smoke\model\actuators.py src\robot_smoke\model\fivebar.py src\robot_smoke\model\kinematics.py src\robot_smoke\model\mechanics.py src\robot_smoke\control\__init__.py src\robot_smoke\control\ik.py src\robot_smoke\control\lqr.py src\robot_smoke\control\lqr_design.py src\robot_smoke\control\vmc.py src\robot_smoke\experiments\__init__.py src\robot_smoke\experiments\equilibrium.py src\robot_smoke\experiments\fivebar_checks.py src\robot_smoke\experiments\fl_tests.py src\robot_smoke\experiments\trace.py src\robot_smoke\experiments\virtual_rod.py src\robot_smoke\io\__init__.py src\robot_smoke\io\cli.py src\robot_smoke\io\output.py src\robot_smoke\model_smoke.py src\robot_smoke\runner.py
```

旧短时 `--virtual-rod-steps 200` 结果只允许理解为 finite smoke 记录，不能作为平衡证明。该结论已在 2026-07-11 撤销。

## 2026-07-11 无扰动 12 秒控制记录

- 新增 history 诊断字段：`Tp_left = Tp/2 + Tp_sync`、`Tp_right = Tp/2 - Tp_sync`；它们只记录 VMC 前的左右腿角向输入，不改变控制律。
- 使用 `--lqr-true-equilibrium --virtual-rod-steps 12000` 采样。可视化判定：约 5 s 的第三次后倒未恢复，6 s 后已经摔倒；5 s 只是本次失稳时刻，不是后续实验的固定诊断上限。`Tp` 自 0.236 s 起长期位于总限幅 ±2 N*m；`T` 自 6.446 s 起首次达到 ±8 N*m。
- 发现并修复控制离散周期不一致：线性化和 K 使用 `5 ms` 离散步长，而运行时旧代码每 `1 ms` 重算 K。锁定运行控制周期为 5 个物理步（200 Hz），使运行采样与离散模型一致；不改变状态、输入、几何或 VMC 物理语义。
- 发现线性化输入坐标错误：旧 `B` 和 `U0` 未包含运行时的 `lqr_pitch_sign=-1`，但运行输出包含该符号，导致 K 对应的 `Tp` 方向与真实执行方向相反。现在线性化仿真和 `U0` 使用同一符号映射；不改变 `theta`、`F_l`、Jacobian 或已确认的运行符号。
- 修复后 12 秒无 viewer rollout 仍未通过，不能宣称已平衡；同时已运行 12 秒 viewer，后续平衡结论必须由该可视化中的实际首次动作确认。注意 viewer 前的默认 800-step smoke 输出不能替代 12 秒可视化结论。
- 在 true equilibrium 上做 50 ms 单通道脉冲：实际 `T+/-1 N*m` 分别使 `theta_world` 初始变化约 `+/-0.00866 rad`；实际 `Tp+/-0.5 N*m` 分别使 `theta_world` 变化约 `+/-0.00159 rad`，但 `Tp+` 同时使 `theta_rel` 减小、pitch 增大。该事实解释了当前“后倒时腿反而向前”：LQR 对 `theta_rel` 的负反馈与世界系竖直腿目标发生冲突。
- 用户确认参考文章的 `theta` 是摆杆相对世界竖直方向的角。LQR 状态已改为 `theta_world/dtheta_world`，并同步修改状态扰动与 equilibrium 临时姿态环；`phi/dphi` 仍为独立机身状态，必须重新线性化，不可复用旧 K。
- 修复 viewer 与 rollout 初态不一致：旧 viewer 从锁机身 IK 姿态启动，而 LQR 从 true equilibrium 启动。viewer 现复制同一 `lqr_operating_data`，并复用左右 `Tp_sync` 分配；可视化才可用于判断当前 K 的实际动作。
- 世界系状态重线性化后，局部 `T` 脉冲方向正确，但最大闭环特征值约 `0.999549`（5 ms 控制周期），小信号恢复时间常数约 11 s。曾试将 `R_T` 从 `2.0` 降为 `0.1`，主特征值几乎不变且 rollout 仍失败，故已恢复 `R_T=2.0`，不将该试验当作有效修复。
- 发现 `x/dx` 线性化扰动不满足无滑滚动：旧代码只改 wheel q，不同步平移 free base，导致位置列混入错误接触几何，`K_T,x` 会把后向位移继续放大。现改为每个 `x/dx` 扰动同时平移 base x 并增加对应 wheel q，再重新线性化。
- 滚动一致 A/B 下，当前 `R_T=2.0` 使 LQR 过度依赖总限幅 ±2 N*m 的 `Tp`，画面表现为机体 pitch 变化而世界系虚拟腿几乎不动。曾以 `R_T=0.1` 做轮端优先试验，但在修复 `Tp` 尺度前不保留该耦合变更，现已恢复 `R_T=2.0`。
- 按文章输入语义修复 `Tp` 分配：`T` 是总轮力矩，仍按左右各 `T/2`；`Tp` 是每条腿的角向等效力矩，旧代码错误地按 `Tp/2` 分给每条腿。现改为 `Tp_left=Tp+Tp_sync`、`Tp_right=Tp-Tp_sync`，并同步更新 rollout、viewer、equilibrium 和有限差分线性化。
- 以修复后的 B 矩阵重新分配 LQR 输入代价：`Tp` 对车身 pitch 的局部作用强于对世界系腿角的作用，旧 `R=[2.0,1.2]` 促使控制器优先打满 `Tp`。现试验 `R=[0.1,12.0]`，将轮端 T 设为主恢复通道、Tp 作为昂贵辅助通道。
- VMC 单通道辨识：锁 base 时每腿 `Tp=+1` 使 `theta_world` 增加约 `0.01379 rad`、pitch 近零；释放 base 时 `theta_world` 仅增加约 `0.00636 rad`，而 pitch 增加约 `0.00918 rad`。因此 VMC `J^T` 映射有效，当前“只转车身”来自 `Tp` 的自由机身反作用未被 T 同步补偿。
- viewer 显示旧 `Q_theta=Q_phi` 会把 `theta_world` 锁在 0，车身则绕竖直虚拟杆摔倒。现将性能目标改为机身/轮端优先：`Q=[40,8,10,300,2500,250]`，`R=[0.1,100]`。这允许世界系腿角在瞬态偏转，以轮端先追到重心下方；不改变状态或 VMC 语义。
- 产物：`tasks/lqr-state-trace-2026-07-11.csv`、`tasks/lqr-state-trace-2026-07-11.png`、`tasks/lqr-motor-torque-2026-07-11.png`。

## 当前问题

## 独立运动场景

- 原地转向：`--turn-test` 使用航向角速度五段轨迹 `0.15 -> 0.35 -> 0.60 -> 0.35 -> 0.15 rad/s`，相邻档位以 `0.5 s` 线性升降沿连接，避免 PD 微分项被参考阶跃激发；仅叠加左右轮反相差动力矩，不与直线速度或冲击混用。
- 转向 viewer 同时显示 `yaw_rate_ref`、实际 `yaw_rate`、`tau_turn`、左右轮实际力矩、`theta_right-theta_left` 与 `Tp_sync`；公共 `T/Tp` 不能替代这些差模量。
- `--turn-test --history-plot <png>` 输出四分图：左右虚拟腿竖直角和角差、请求/实际 `Tp`、航向角速度、左右轮实际力矩，用于确认劈叉是否被检测且协调 PD 是否真的穿过 VMC。
- 协调误差统一为同一世界系虚拟腿角的差模：`e_sync=theta_right-theta_left`、`de_sync=dtheta_right-dtheta_left`，并采用 `Tp_sync=Kp_sync*e_sync+Kd_sync*de_sync`、`Tp_left=Tp+Tp_sync`、`Tp_right=Tp-Tp_sync`。锁 base 的单腿方向测试确认 `+Tp` 初始增大 `theta_world`；因此该分配减小差模。反号试验会使左腿持续后摆、右腿持续前摆，并造成关节饱和，已撤回。此前的和式定义已撤回。
- 同一诊断中，轮地法向接触存在；对 `(-1,+1) N m` 反向轮矩施加 100 ms，轮子相对转速约为 `-10/+10 rad/s` 而 yaw rate 仅约 `0.0085 rad/s`。后续应辨识轮端差矩到 yaw 的接触力/惯量尺度，不能通过增大腿角 PD 代替。
- 航向速度环和横滚补偿为常态控制：默认参考均为零，`--turn-test` 只写入航向速度轨迹，`--ramp-test` 只写入后退速度轨迹。三角坡顶点使用以 geom 原点为中心的局部坐标：左坡范围 `x=[-1.40,-0.60]`、`y=+0.255`；右坡范围 `x=[-2.20,-1.40]`、`y=-0.255`。
- 单侧斜坡：`--ramp-test left|right` 从冻结 `L0=0.35 m` 平衡初态立即执行后退速度轨迹；运行时关闭非目标侧斜坡的碰撞掩码，因此一次仅检验一侧车轮的斜坡接触。测试地形为后退方向上先上升、越过顶点后下降的三角棱柱，长 `0.80 m`、高 `0.18 m`、宽 `0.24 m`。物理通过与否由 MuJoCo viewer 观察确认。

- 已按文章加入航向角速度 PD 差动力矩和左右腿角差 PD 协调；`--turn left|right` 将差动力矩反相叠加到左右轮，现有 `Tp_sync` 反相叠加到左右腿。模型后方为两条独立单轮轨迹三角斜坡，中心在 `y=±0.255 m`，宽 `0.24 m`、长 `0.80 m`、高 `0.18 m`；后退时从斜坡低端上升至顶点再下降。左右测试通过碰撞掩码分别运行，而非同时接触。
- 当前运动测试使用冻结的 `L0=0.35 m` 工作点和固定 `K/X0/U0/F_l0`，不在每次运行时重新进行 equilibrium search、有限差分或 Riccati 求解。`--speed-profile low|medium|high` 只跟踪梯形 `v_ref`，不使用绝对位置回拉；后续前进能力仍须以 viewer 观察为准。

- `--turn-test --history-plot` 的转向诊断图固定只输出两个 PD 环：yaw-rate PD 的 `e_yaw`、`de_yaw/dt`、`tau_turn`，以及双腿同步 PD 的 `e_sync`、`de_sync/dt`、`Tp_sync`。同一 8 s 正常差速测试中，yaw 误差接近参考而 `e_sync` 持续增大；该图用于定位两环的因果关系，不作为转向成功与否的最终结论。

- 正常差速与支撑控制下的局部同步脉冲：在 `t=1.566 s`、`e_sync>0` 时，向现有 `Tp_sync` 额外叠加 `+0.5 N*m`、持续 `30 ms`。实测 `de_sync/dt` 从 `+0.24294 rad/s` 降至 `+0.19117 rad/s`（20 ms），50 ms 为 `+0.23064 rad/s`。该局部响应表明当前 `Tp_sync` 增量初始降低差模增长率；持续非零差模仍需另行解释为通道尺度不足或差轮矩持续扰动，不能仅据稳态误差反转同步符号。

- 同步控制权辨识显示与当前误差同量级的正 `Tp_sync` 增量可压回差模，因此将默认同步参数由 `Kp=8, Kd=2` 调为 `Kp=20, Kd=0.8`，并加入 `--leg-sync-kp` 覆盖入口。`Kp=20` 的 8 s 正常差速 smoke 无关节饱和，`max_theta_error=0.18757 rad`、末端左右腿角约 `-0.00435/+0.00789 rad`，有限行为阈值返回 PASS；该结果不替代 MuJoCo viewer 对实际转向和稳定性的观察。

- 航向速率 PD 的比例项也开放为 `--yaw-turn-kp`，默认 `1.8`；双腿同步比例项为 `--leg-sync-kp`，默认 `20`。两项在同一 normal turn rollout 中独立传递到轮端差速和 `Tp_sync`，KD 均保持固定。

- 本轮已定位 VMC 的自由机身反作用：在 true equilibrium 上，每腿 `Tp=+1` 的 50 ms 脉冲在锁定 base 时使 `theta_world` 增加约 `0.013793 rad`；释放 base 时只增加约 `0.006359 rad`，而 pitch 增加约 `0.009176 rad`。几何 Jacobian 映射有效，症状来自 `Tp` 没有由轮端 `T` 同步抵消机身反作用。
- 尝试过以有限差分 `B` 构造 `Tp -> T` 一阶反作用补偿，但可视化显示它使系统更早原地摔倒。该近似补偿已完整撤销，不作为有效控制策略。后续需将 VMC 的世界系虚拟力矩与机身反作用的广义力学映射分开验证，而不能继续在 `B` 上叠加经验补偿。
- 前倾单通道脉冲（`phi=+0.06 rad`、30 ms）：相对零输入，`T+` 使 `theta_world` 增大且 `pitch` 减小，是轮端向重心下方移动的正确方向；`Tp+` 使 `theta_world` 增大但同时显著增大 `pitch`。因此当前 VMC 的 `Tp` 是内部身-腿力矩通道，不可作为世界系虚拟腿角的主恢复通道；“只抬头”正是该通道主导时的物理结果。
- 纠正：之前手工撤回失败 `Tp->T` 补偿时，误把 CLI 的 LQR 代价恢复为旧 `R=[2.0,1.2]`。该设置会使内部 `Tp` 通道比轮端 `T` 更便宜，与“只抬头”相符。现在恢复为该主路径已记录的轮端主导代价：`Q=[40,8,10,300,2500,250]`、`R=[0.1,100]`。
- 控制顺序已逐行确认：`LQR -> VMC 写入腿电机 ctrl -> T/2 写入轮电机 ctrl -> mj_step`，viewer 与 rollout 一致，轮端不会被 `data.ctrl[:]=0` 覆盖。当前前倾时 `theta_world` 和 `phi` 同时为正：旧 `Qtheta=40,Qphi=2500` 仍使 `theta` 通道过早把轮端往反方向拉回，与 pitch 恢复所需的轮端方向冲突。因此将当前试验代价更新为 `Q=[1,0.5,10,300,2500,250]`，让 `phi/dphi` 主导首段轮端恢复，`theta/dtheta` 只保留弱约束。
- 独立模态线性化后仍暴露实际因果符号冲突：纯 `phi` 模态下，实测要求 `phi>0 -> T>0`，但未约束的 DARE 给出 `K[T,phi]>0`，即 `U=-KX` 会输出错向的 `T<0`。现对增益施加基于通道职责的结构投影：`T` 不再直接响应身-腿相对模态，且强制 `K[T,phi]<0`；`Tp` 不再直接响应公共 `phi/dphi`。这使 `T` 专责轮端/pitch 恢复，`Tp` 只留给身-腿内部模态。

- 旧小任务文档已归档到 `tasks/archive/`，尚未物理删除。
- `model_smoke.py` 仍保留 viewer overlay 和部分 smoke 扫描流程；后续可继续拆 `viewer.py`，但不应在整理时改控制公式。
- `model/mechanics.py` 当前包含静态工作点采样输出，依赖 `control.lqr` 和 `control.vmc`；后续若继续清架构，可考虑把采样输出再拆到 `experiments/`，让 `model/` 更纯。
- 后续不能再把控制理论文档写成流水账。

## 腿长、差模与运动测试

- 默认腿长恢复为 `0.35 m`。
- equilibrium 新增腿长目标误差和左右腿差模门禁。
- 0.35 m 工作点：`L_mean=0.350189 m`，`F_l0≈34.5262 N`，无饱和，左右差模接近数值噪声。
- 已撤销扰动入口作为当前主路径；恢复无扰动平衡前，不再把 `base` 外力脉冲或速度指令混入默认控制链。
- LQR 状态 `theta` 已改为相对车体角：`theta_world-phi`，临时 equilibrium 控制仍使用世界腿角。
- 增加轮速越界恢复阻尼后，外力后的轮速峰值明显下降，但 `Tp` 姿态通道仍会把机体带出线性域并摔倒；`Tp` 暂限 2 N·m，结果仍未通过。
- runner 现在输出 `behavior_qualified`，摔倒或恢复不合格时返回失败，不再打印伪 PASS。
- 修正 theta 扰动后重新投影 `x/dx`，`A[x,theta]` 从约 `0.296` 降到约 `2e-5`，确认旧线性化存在状态串扰。
- NumPy Hamiltonian CARE 也无法让真实零扰动 rollout 稳定，说明当前六状态 A/B 未包含腿长/接触内部动态。后续应扩展线性化状态或严格约束隐藏状态一致，不能继续叠加经验恢复项。
- 已撤销移动位置/速度参考入口 `--target-speed`，避免在恢复无扰动版本前引入额外运动指令。
- 2026-07-11 恢复无扰动正常平衡主路径：撤下增广 `X_aug/U_aug` 作为有效控制器，当前 LQR 回到 `X=[theta,dtheta,x,dx,phi,dphi]`、`U=[T,Tp]`，腿长只由 VMC 的 `F_l0 + k_l(L0-L)-d_l dL` 支撑。
- viewer 保持 1 ms 控制/物理步，只约每 16 步同步一次画面，目标刷新率约 60 Hz，避免逐步 OpenGL 同步造成慢放。
- 2026-07-11 纠正：撤销“`200 step` 无扰动通过”和“`1500 step` 小扰动通过”作为平衡结论。实际长时间可视化中仍会腿向反方向摆动、加速翻车；后续不得再用短时数据门禁宣称平衡。
- 后续涉及平衡、扰动恢复或前进运动时，必须以用户观看 MuJoCo 可视化为准。数据指标只用于定位问题，不能代替物理观察。
- 下一步控制实现必须严格对齐参考文章的解析控制策略。文章已给解析解或解析公式的部分优先解析实现；仅允许在解析解附近做局部工作点搜索，不允许用大范围数值方法替代。
- 2026-07-11 手工恢复扰动测试前主路径：删除 CLI/runner/rollout/viewer 中的 `--disturbance`、`--target-speed`、`xfrc_applied` 和移动 `x_ref` 残留；LQR 状态扰动恢复为两个闭链姿态经 `mj_differentiatePos` 求速度，不再手动覆盖五连杆驱动关节速度。
- 2026-07-11 根据最新可视化现象修正 `Tp` 默认执行方向：当前主路径下 `lqr_pitch_sign` 改为 `-1`。保持 `+1` 时，车体后倒会出现虚拟腿反向前摆，因此先做最小符号恢复，不改 `theta_world`、`F_l` 或状态维度语义。
