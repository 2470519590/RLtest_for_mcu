# ERROR_CATALOG.md

本文件记录已经确认的调试错误和修复经验，避免重复踩坑。

## 模板

```markdown
### YYYY-MM-DD：问题标题

- 现象：
- 根因：
- 修复：
- 验证：
- 下次避免：
```

## 已解决或已确认的问题

### 2026-07-10：这次能立起来不是单一参数调优，而是工作点链路修正

- 现象：此前可视化中会出现腿长塌短、低频摆动、前后加速、慢漂，或者倒下后腿长才回到目标；当前 `--lqr-true-equilibrium` 路径已经能在局部工作点附近立起来，腿长接近 `0.35 m`，分支不凹陷，执行器不饱和。
- 根因：问题不是一个单独的 `K` 或 PD 参数不够猛，而是多处语义链路不一致叠加：
  1. 旧 VMC 主路径使用了被轮地接触污染的数值 Jacobian，导致 `dL/dq` 过小，`F_l` 映射成的关节支撑力矩严重不足。
  2. 曾把 LQR-settle 末端或锁 base 几何点当成工作点，未满足自由接触下的 `dX≈0/dL≈0/contact>0/无饱和` 门禁。
  3. 自动 LQR 设计得到的工作点没有完整同步到运行时 `X0/U0/F_l0`。
  4. 有限差分输入曾围绕零输入而不是围绕真实 `U0`，使局部线性化和控制律 `U=U0-K(X-X0)` 不一致。
  5. 腿长 `L0` 一度被误解成需要大范围搜索的状态，而不是局部冻结截面/调度量。
- 修复：VMC 主路径改用五连杆解析 Jacobian；先找 true equilibrium，记录 `X0/U0/F_l0/J0/contact_force0`；自动 LQR 的操作点、运行时 `X0/U0/F_l0` 和有限差分输入全部围绕同一个 true equilibrium；锁定 `--lqr-true-equilibrium` 入口，避免继续暴露大量临时调参入口。
- 验证：`run_smoke.py --lqr-true-equilibrium --virtual-rod-steps 200` 通过 finite smoke，`qualified=True`，`F_l0=30.4643`，`saturated_steps=0`，左右腿分支 violation 均为 0。
- 下次避免：不要把“终于能立起来”归因于某个单一增益。任何后续几何、腿长截面、状态定义或 LQR 权重变化，都必须重新走：解析 IK/Jacobian 门禁 → true equilibrium 门禁 → 同步 `X0/U0/F_l0/J0/contact_force0` → 围绕 `U0` 线性化 → 局部 smoke/可视化。

### 2026-07-10：地面接触污染的数值 Jacobian 会把腿长支撑力矩压小

- 现象：`L0=0.35, theta=0` 的解析 IK 能准确到达目标，但自由接触和重量作用下腿长会塌短；旧 VMC 打印显示 `F_l` 已进入控制链，却无法提供足够支撑。
- 根因：运行时使用“锁机身 + 目标角 settle + 差分”的数值 Jacobian 时，样本已经处在轮地接触中，微扰会被地面接触约束抵消。结果 `dL/dq` 被严重压小：解析长度行约 `[+0.163, -0.163]`，接触污染数值差分只有约 `[+0.004, -0.003]` 到 `[+0.016, -0.015]`。同样 `F_l≈50.77N` 下，解析支撑力矩约 `[+8.26,-8.26] N*m`，污染数值力矩只有约 `0.18~0.79 N*m`。
- 修复：VMC 主路径改为优先使用五连杆解析 Jacobian `J=d[L,theta]/d[q_front,q_rear]`；旧 settle 数值差分只保留为诊断对照或解析分支不可用时回退。
- 验证：改用解析 Jacobian 后，support-only 200 步不再从 `0.35m` 塌到约 `0.23m`，而是在 `F_l≈50.77N` 下伸到约 `0.404m`，说明力映射恢复到足够尺度；进一步扫 `F_l0_scale` 得到 `0.65` 附近可把平均腿长保持在约 `0.347m`。
- 下次避免：不要在有轮地接触的 MuJoCo 状态上用接触参与的 rollout 差分估计纯机构 Jacobian。VMC 的 `J^T F` 必须使用纯五连杆几何 Jacobian，接触力只能作为工作点验证量，不能进入几何微分。

### 2026-07-09：倾倒后的腿长回到目标值不能作为 upright 控制成功证据

- 现象：旧的可视化命令 `run_smoke.py --virtual-rod-test --visualize --left-rod-length 0.35 --right-rod-length 0.35 --left-rod-theta 0 --right-rod-theta 0 --visualize-seconds 10` 中，受冲击后画面可能出现腿长逐渐回到 `0.35 m` 附近。
- 根因：自由接触下车体倾倒、pitch 改变、轮地接触模式变化和五连杆几何重构都会改变实际虚拟杆长度。若机器人不是 upright 接触平衡，腿长回到命令值可能只是倾倒后的几何结果，不代表腿长支撑控制在正确工作点托住了车身。
- 修复：不再把该可视化现象作为 `L0=0.35` 工作点、腿长控制成功或 LQR/VMC 正确的证据。判断 true equilibrium 必须同时检查 `dx≈0`、`dtheta≈0`、`dphi≈0`、`dL≈0`、`phi/theta` 小角度、contact normal force 为正且稳定、`T/Tp/joint torque` 不长期饱和。
- 验证：该命令只能用于画面定性观察，不能替代五连杆解析 IK 门禁、静态接触平衡门禁和短步数值输出。
- 下次避免：凡是“倒下以后某个几何量恢复目标”的现象，都只能记为 transient/fall artifact；不得写入 `docs/CONTROL_THEORY.md` 作为物理结论，也不得用来推进线性化或 LQR 求 `K`。

### 2026-07-09：闭链五连杆不能直接用单关节 qpos 微扰 + mj_forward 估任务空间 Jacobian

- 现象：模型改成前后对称后，直接对 `q_front / q_rear` 做 `qpos += epsilon` 再 `mj_forward`，得到的 `J = d[l, theta] / d[q_front, q_rear]` 会出现一列接近零，表现成 rear 通道“像是没作用”，进而导致 IK / VMC 基于错误 Jacobian 失效。
- 根因：这是闭链约束一致性问题，不是 rear 驱动真的失效。对连接约束系统只改一个驱动角再 `mj_forward`，并不会自动得到与闭链一致的局部平衡构型，因此数值差分方向错了。
- 修复：改为“锁机身 + 小幅目标角扰动 + 短步 settle + 再差分”的 Jacobian 估计方法，让微扰样本先回到闭链一致构型。
- 验证：修复后对称几何下 `J0_left / J0_right` 能得到前后驱动都参与的非奇异矩阵，虚拟腿控制恢复 finite。
- 下次避免：只要是 `equality/connect` 闭链，不要默认“直接改 qpos 再 mj_forward”就能得到可用任务空间 Jacobian；先确认微扰样本是否满足约束一致性。

### 2026-07-09：LQR pitch 扰动夹带虚假 theta

- 现象：`LQR + VMC` 短时看似能稳住，但长时会持续加速；虚拟腿没有保持世界竖直，出现 `theta` 与 `pitch` 反相的大幅模态。
- 根因：`theta = atan2(r_x, -r_z)` 是世界系虚拟腿角。旧线性化构造 `pitch` 扰动时只改 base pitch，没有用 IK 把世界系 `theta` 补回目标值，导致 `A_d` 的 pitch 列夹带接近 `-1` 的虚假 `theta` 位移。
- 修复：构造 LQR 状态扰动时，先设置 `x / dx / pitch / pitch_rate`，再通过腿部 IK 把世界系 `theta` 调回目标值；`_apply_lqr_theta_state_perturbation()` 锁住当前 base 位姿，而不是固定锁回 reset 位姿。
- 验证：修复后同类 10 秒 smoke 中，`theta` 和 `pitch` 降到小角度，轮速不再高速增长，腿型分支最终 violation 为 0。
- 下次避免：任务空间状态量做有限差分线性化时，必须构造“纯状态扰动”，不能只改某个 `qpos` 后假定其他任务空间量不变。

### 2026-07-09：轮关节速度阻尼不能替代车体平移阻尼

- 现象：小车不再加速摔倒后，仍出现慢速匀速漂移；增大轮速阻尼时，漂移和姿态误差反而变大。
- 根因：轮关节速度包含车体平移、轮地接触和姿态修正共同作用，不是干净的固定点速度误差信号。直接对平均轮速加大阻尼会与 LQR 的 `T` 输出和姿态通道竞争。
- 修复：优先用状态向量中的 `x_rate = base_x_dot` 分析固定点漂移；默认 LQR 权重增强平移速度项，并降低过激输出限幅。
- 验证：默认 10 秒 LQR smoke 中，调整后轮速和 `x_rate` 明显下降，分支 violation 为 0。
- 下次避免：分析固定点漂移时先看 `x / x_rate`，不要先把轮速当作车体速度。

说明：上述经验只适用于旧阶段的“漂移诊断”，不再作为当前文章对齐后的平衡状态定义。当前主平衡状态已回到 wheel source，`base_x_dot` 只保留为 slip / 漂移诊断量。

### 2026-07-09：LQR 工作点不能只改高度而不重做输入尺度验证

- 现象：把自动 LQR 工作点从 reset 高度改为按 `L_target = 0.35 m` 计算的接触高度后，3 秒内腿长可接近目标，但 10 秒会进入大角度模态，轮速升高并摔倒。
- 根因：接触工作点改变了有限差分矩阵的尺度；`lqr_design_steps = 1` 时 `Tp` 输入列退化，`lqr_design_steps = 8` 虽恢复 `Tp` 作用，但得到的 `K` 对长时非线性 rollout 过激。
- 修复：本轮没有把接触工作点作为默认启用，默认回退到稳定 smoke 路径；保留腿长通道诊断、前馈、条件积分和外层速度参考接口。
- 验证：接触工作点加 `lqr_design_steps = 8` 的 3 秒 smoke 腿长可到约 `0.35 m`，但 10 秒 smoke 会发散；默认稳定路径 10 秒 finite、分支 violation 为 0、执行器不饱和，但腿长仍约 `0.326 m`。
- 下次避免：修改 LQR 工作点后必须同时检查 `A/B/K`、输入列尺度、3 秒和 10 秒以上 rollout；不能只凭短时腿长达到目标就认为控制器已完成。

### 2026-07-09：F_l 前馈有效不等于腿长工作点正确

- 现象：`gravity_comp_scale = 1.0` 后，`F_l` 前馈和 `tau_support` 都变化了，但最终腿长仍约 `0.325~0.326 m`，没有靠近 `0.35 m`。
- 根因：`F_l` 通道符号和接线是有效的；当前静差来自整体接触/LQR/VMC 闭环工作点不一致，末端状态仍带有非零 `X0`、非零 `U0`，且 `Tp` 常到限幅。
- 修复：加入 `F_l` sweep、pulse、`tau_support` 诊断和静态工作点打印；LQR 控制律接口改为 `U = U0 - K*(X-X0)`，腿长前馈按 `F_l0` 语义处理。
- 验证：F_l sweep 中 `+F_l` 初始 `dL > 0`；gravity scale 改变时 `F_l_cmd` 和 `tau_support` 同步改变；默认 10 秒末端工作点显示 `F_l_cmd≈78.6N`、`tau_support≈[11.24,-11.24]`、`Tp≈8`。
- 下次避免：腿长静差先辨识 `F_l -> J_l^T F_l -> tau -> contact/L0` 链路；如果链路有效，不要继续盲目加 `k_l` 或前馈，应重构真实 `X0/U0/F_l0` 工作点和线性化模型。

### 2026-07-09：把腿长截面误写成唯一平衡腿长

- 现象：equilibrium search 被描述成搜索 `L_ref / F_l0` 工作区，容易把 `0.38~0.395 m` 这类静态构型观察误解成机器人真实只能在该腿长范围平衡。
- 根因：没有严格按文章区分主平衡状态和腿长调度量。文章式主状态为 `X=[theta,dtheta,x,dx,phi,dphi]^T`，腿长 `L` 不进入该状态，只作为局部线性化冻结截面 `L0` 或增益调度变量。
- 修复：代码输出和推荐 CLI 改为 `L0_slice` / `--equilibrium-l-slices`，旧 `--equilibrium-l-refs` 只保留为兼容别名；`docs/CONTROL_THEORY.md` 明确 `L_cmd(t)` 可连续变化。
- 验证：语义层验证通过；后续数值验证必须在给定 `L0_slice` 下寻找满足 `dX≈0`、`dL≈0`、contact 稳定、输入不长期饱和的 true equilibrium。
- 下次避免：不要把某个静态构型检查结果写成“合理腿长范围”或“默认腿长工作点”；只能写成“该 `L0` 截面可用于局部建模检查”。

### 2026-07-09：文章平衡状态的速度源不能偷换成 base 速度

- 现象：控制文档和任务记录一度同时出现 wheel source 与 `base_x/base_x_dot` 两种状态定义，导致 LQR 状态语义冲突。
- 根因：把 MuJoCo freejoint 的 base 平移速度当成轮式倒立摆文章中的轮端位移/速度，混淆了主平衡状态和 slip / 漂移诊断量。
- 修复：主平衡状态统一为 `x=r_wheel*(q_left_wheel+q_right_wheel)/2`，`dx=r_wheel*(dq_left_wheel+dq_right_wheel)/2`；`base_x/base_x_dot` 只保留为诊断。
- 验证：`src/robot_smoke/model_smoke.py` 中 `_compute_lqr_state(..., x_source="wheel")` 默认使用 wheel source。
- 下次避免：凡是实现文章中的 `x/dx`，先检查是否来自驱动轮转角和轮速；不要为了短期稳定把 base 速度塞回主状态。

### 2026-07-09：非 reset 工作点 IK 候选不能锁回 reset base

- 现象：`equilibrium-static-pose-check` 中 `L0_slice=0.35` 一度测得实际 `L0≈0.316`，随后改 base 锁定后又可能掉到更短腿构型，说明 IK 初始化并没有真实构造目标截面。
- 根因：闭链候选 settle 和局部 Jacobian settle 使用了 `_lock_base_to_initial()`，会把候选评估锁回 `model.qpos0` 的 base 位姿；这对非 reset 的 upright `L0` 截面是错误的。同时，单纯局部 Jacobian IK 可能落入五连杆多解短腿分支。
- 修复：候选 settle 和 Jacobian settle 改为锁定 source data 的当前 base 位姿；equilibrium upright 初态优先使用对称五连杆两圆交点解析 IK，并选择正常分支。
- 验证：修复后静态截面检查得到 `L0_slice=0.35 -> measured L0≈0.350075`，`L0_slice=0.38 -> measured L0≈0.379986`，`L0_slice=0.395 -> measured L0≈0.395010`。
- 下次避免：凡是围绕某个 `X0/L0` 工作点做 IK、Jacobian 或有限差分，必须锁当前工作点 base 位姿，不能默认锁回 reset；短腿/长腿多解问题优先用解析几何或显式分支条件排除。

### 2026-07-09：equilibrium search 不能用大范围优化替代力学中心值

- 现象：在 `L0_slice` 初态已经准确后，自由 equilibrium search 仍容易退到短腿构型；继续扩大 `F_l0 / wheel damping / gain` 网格只会增加输出量，不会解释问题。
- 根因：搜索中心没有显式绑定文章力学模型。对于给定 `L0/theta0`，支撑力中心应先由 `F_l0_center=m_total*g/(2*cos(theta0))` 给出，角向力矩中心应由 `Tp_center=-m_total*g*(x_com-x_wheel)` 给出。若从这些中心附近仍失败，说明接触/支撑工作点不一致，而不是“搜索范围还不够大”。
- 修复：equilibrium search 输出和内部计算加入 `F_l0_center`、`Tp_center`，`F_l0_scale` 解释为围绕力学中心值的小范围比例。
- 验证：当前 `theta0=0` 时 `F_l0_center≈50.77N`，`Tp_center≈0`；围绕 `0.8~1.2` 倍支撑力短扫仍出现 `contact_force_min=0` 和腿长塌短，说明下一步应处理接触一致性和腿长支撑，而不是进入 LQR 线性化。
- 下次避免：先写清楚力学中心值和物理单位，再做局部扫描；不得把大范围优化结果直接当成工作点。
# 已确认错误

## 平均腿角掩盖左右反相差模

只检查 `0.5*(theta_left+theta_right)` 会把 `theta_left≈-theta_right` 误判为竖直平衡。equilibrium 门禁必须同时检查腿角差、角速度差和腿长差；VMC 需维持二维模型隐含的左右同相条件。

## 腿长合格判据遗漏目标误差

旧 equilibrium 只检查 `dL`，没有检查 `L_mean-L_ref`，因此会把稳定但存在厘米级静差的点判为合格。当前必须满足 `abs(L_mean-L_ref)<3 mm`。

## 轮速符号修改未同步线性化扰动器

修改轮角状态符号后，有限差分状态扰动的轮角/轮速反解也必须同时修改。不同步会使离散矩阵出现 `A[x,x]≈-1`，由此得到的 K 无效。

当前模型轮轴为 `+Y`。无滑滚动关系为 `v_x=r*dq_wheel`，因此 LQR 平移状态使用 `x=r*q_wheel`、`dx=r*dq_wheel`。反转该符号会使水平外力恢复力矩与扰动同向。

## Riccati 迭代未收敛

旧离散 Riccati 求解上限为 800 次，实际每次都恰好达到上限，得到的是未收敛中间矩阵。当前工作点需要约 7475 次才达到容差。未收敛时打印的闭环特征值不能作为长期 rollout 稳定证明。

## LQR 状态扰动混入轮位移

旧 theta 状态扰动通过 IK settle 生成，但 settle 会同时滚动车轮。若不在扰动后重新投影目标 `x/dx`，线性化会出现不可能的 `A[x,theta]≈0.296`。修正隔离后该项降至约 `2e-5`。

## 六状态线性模型未闭合

隔离状态扰动并使用 Hamiltonian CARE 后，线性闭环特征值仍不能预测真实 rollout。当前 `X=[theta,dtheta,x,dx,phi,dphi]` 省略了会演化的腿长、腿长速度和接触内部状态；若扰动样本的这些状态不一致，A/B 不是闭合 Markov 模型。不得继续仅凭 `max|eig|<1` 宣称抗扰稳定。

## 增广状态不能继续沿用双输入模型

把 `L/dL` 加入状态后若仍只保留 `[T,Tp]`，局部可控矩阵只有 `6/8` 阶。腿长由 `F_l` 通道驱动，因此增广模型必须显式包含共同支撑力修正 `delta_F_l`，或只把腿长作为调度量而不进入 Riccati 状态。当前采用 `[T,Tp,delta_F_l]`，并用 5 ms 辨识窗提高力输入列的数值可辨识度；控制执行周期仍为 1 ms。

## 设计模型外叠加轮速阻尼会改变闭环

在 LQR 输出后额外叠加经验式 `-k*dx`，但在线性化采样中不包含该项，会使真实闭环与 `A-BK` 不一致。该历史恢复阻尼已删除；轮速恢复必须由统一状态反馈产生。

## 物理步进频率不等于 viewer 刷新频率

每个 1 ms 物理步都调用 `viewer.sync()` 会让 OpenGL 开销拖慢墙钟时间，看起来像慢放。控制和 MuJoCo 仍按 1 kHz 步进，viewer 只需约 60 Hz 批量刷新，并按累计仿真时间做实时节拍。

## 自由基座姿态瞬移不是有效扰动

直接修改 freejoint pitch 会改变整套子机构的接触几何并产生非物理接触冲击。姿态鲁棒性测试应使用角速度冲量或外力脉冲。

对于模拟风、碰撞或踢击，必须优先使用施加到 `base` 的有限时长外力或外力矩，不能用初始角速度扰动冒充机身受力。
