# 控制框架

## 当前目标

恢复固定 `L0=0.35 m` 的 locked true-equilibrium 平衡入口，维护解析五连杆 VMC、腿长控制、轮端转向控制；最终运动与稳定性由 viewer 人工确认。

## 当前有效控制

- 平衡：`U=U0-K(X-X0)`，状态为 `[theta,dtheta,x,dx,phi,dphi]`。
- 支撑：`F_l=F_l0+k_l(L0-L)-d_l*dL`。
- 五连杆：`tau_joint=J(q)^T[F_l,Tp_side]^T`。
- 转向：轮端差动 `tau_left=T/2-tau_turn`、`tau_right=T/2+tau_turn`。
- 协调：`e_sync=theta_right-theta_left`，左右腿施加反相 `Tp_sync`。
- 转向档位：`low=pi/2`、`medium=pi`、`high=10 rad/s`。
- 旋转前进：`low` 固定为低速前进加 low 旋转；`high` 固定为高速前进加 medium 旋转。
- 双腿长度采用 PID + 重力前馈；`L_d` 只表示几何高度参考。横滚动态补偿为 `F_roll=K_gamma(gamma_d-gamma)`，以反号直接叠加到左右沿腿推力。斜坡地面倾角估计当前未启用。
- 当前为长度通路辨识，默认暂时锁定 `Ki,L=Kd,L=0`，仅保留前馈加比例项；`L_d` 固定为 `0.35 m`。
- 控制结构按文章框图组合：整体反馈输出 `T/Tp`，yaw PD 输出轮差，腿同步 PD 输出反向 `Tp_sync`，长度 PID 输出 `F_base`，roll 几何通道输出左右 `L_d`，roll P 通道输出 `F_roll`，两路在 VMC 前汇合。

## 入口

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed high --visualize --visualize-seconds 6 --turn-pd-plot
```

`--turn-speed` 可取 `low`、`medium`、`high`。转向诊断图只输出到 `output\HHMMSS.png`，用于查看原始量、滤波量以及 P/D 力矩分量。
`--turn-length-sine-test` 为增强腿长旋转测试：默认 high 原地旋转，腿长参考在 `minimum_leg_length..maximum_leg_length` 内按 `1.5 s` 周期正弦变化；自转和腿长参考同时开始、同时结束；未显式给 `--visualize-seconds` 时默认运行 10 秒。

`--roll-length-plot` 输出文章 2.2 长度/roll 控制图，记录左右 `L_d/L/dL`、roll、几何项、`F_roll`、`F_base +/- F_roll` 与最终 `F_l`；只作控制通路诊断。

## 最近整理

- 删除三角斜坡模型、`ramp_test` 和 roll 支撑补偿路径。
- 删除通用 history CSV、LQR 图、电机图和 control trace 输出；保留转向 PD 图。
- 删除任务目录下旧 CSV/PNG 实验产物。

## 验证

- 2026-07-12：`py_compile` 通过。
- 2026-07-12：`--turn-test --turn-speed medium --visualize-seconds 0.02 --turn-pd-plot` 完成有限步运行；仅证明入口和接口可执行，不构成平衡结论。
- 2026-07-12：`--lqr-true-equilibrium --virtual-rod-steps 20 --roll-length-plot` 生成 `output\HHMMSS.png`；仅证明文章 2.2 观测与绘图链路可执行，不构成 roll 补偿有效结论。
- 2026-07-13：公式 (13)-(15) 的 `L_d` 解算已接入；`--turn-drive-test low --visualize-seconds 2 --roll-length-plot` 显示正 roll 时左 `L_d` 下调、右 `L_d` 上调。该短 rollout 未满足既有动态行为判据，不作为稳定结论。
- 2026-07-13：动态 `L_d` 曾使 VMC 每步重新执行候选分支 IK settle，导致 1-3 秒稳定卡顿；安全分支姿态改为首次计算后复用。同一 2 秒绘图 rollout 的墙钟时间由约 22 秒降至约 4.4 秒，后者包含初始化和绘图。
- 2026-07-13：腿长 `Kd/Ki/积分限幅/前馈` 支持显式命令行覆盖；未显式传入时保持原 locked 基线。短 smoke 已确认四项覆盖值原样进入 VMC。
## 横滚通道与统一可视化

- 按本地论文 2.2.2 修正横滚通道：`L_d` 仅保留几何参考；`F_roll=K_gamma(gamma_d-gamma)` 以反号叠加到左右沿腿推力后进入 VMC。
- 绘图和 viewer 改为观察同一个 `virtual_rod` rollout；`--visualize-seconds` 同时决定该 rollout 的步数。
- 验证：`--turn-drive-test low --roll-length-plot --visualize-seconds 2` 已生成包含 `F_roll`、`F_base+/-F_roll` 与左右最终 `F_l` 的图。该短测试出现关节饱和和不合格动态，不构成转向稳定结论。
- 可视化验证：同一命令加 `--visualize` 已能打开 viewer 并完成同一 rollout；物理表现仍需人工观察。

## 单轮坡横滚测试

- 默认 `roll_reference=0`；`roll_force_kp` 是论文横滚比例增益 `K_gamma`。
- `--roll-test` 固定 locked equilibrium 和 medium 前进；横滚参考与增益直接读取 YAML，不再在入口覆盖。赛道仅保留 `x=2.5 m` 的左轮三角坡，坡体只在该入口启用。
- 接触检查：左坡只与 `left_wheel_geom` 接触，右坡只与 `right_wheel_geom` 接触；普通模式下坡体接触数为 0。
- 10 秒可视化与绘图已运行。曲线显示两次单侧腿长/横滚响应；是否真实通过扰动与恢复由 viewer 人工判断，当前暂停调参。

## 坡体尺寸扩展

- 单轮三角坡扩展为 `0.84 m` 长、`0.24 m` 宽、`0.06 m` 高，仍各自只覆盖对应轮轨。
- 三座单轮坡体横向全宽均为 `0.60 m`，左右坡中心仍为 `y=+/-0.255 m`，轮间保持横向间隙，避免横滚时轮子从坡侧跌落。紧随小坡的两座梯形高度场坡为 `1.70 m` 长、`0.60 m` 宽、`0.18 m` 顶高；左、右坡中心分别位于 `x=4.5 m`、`x=6.7 m`，按左后右顺序通过。
- 接触检查：小坡和大坡均各自只接触对应左/右轮。10 秒 viewer 已运行，物理表现等待人工观察。

## 腿长调度范围

- 命令腿长范围改为 `0.16..0.30 m`；`minimum_leg_length` 与 `maximum_leg_length` 同时限制普通腿高目标和 roll 几何后的左右 `L_d`。
- `config/length_schedule.yaml` 已按当前五连杆几何、`Q=[1,1,50,100,5000,10]`、`R=[1,0.25]` 重新生成 `0.16:0.01:0.30 m` 的 runtime 调度表；`B(L)` 来自各腿长的 runtime finite-difference，不使用固定 measured B。
- 入口 smoke：`--lqr-true-equilibrium --visualize-seconds 10` 可执行并加载新表；是否真实稳定仍以 viewer 人工观察为准。
- 默认测试腿长改为 `0.24 m`，启动 ramp 默认关闭；调度表模式下 LQR 初始姿态由目标腿长 IK 生成，不再先显示 0.35 m 腿型再慢慢收缩。
