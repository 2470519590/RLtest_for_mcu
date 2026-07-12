# 控制框架

## 当前目标

在固定 `L0=0.35 m` 的本地 MuJoCo 模型上维护解析五连杆 VMC、true-equilibrium LQR 和轮端转向控制；最终运动与稳定性由 viewer 人工确认。

## 当前有效控制

- 平衡：`U=U0-K(X-X0)`，状态为 `[theta,dtheta,x,dx,phi,dphi]`。
- 支撑：`F_l=F_l0+k_l(L0-L)-d_l*dL`。
- 五连杆：`tau_joint=J(q)^T[F_l,Tp_side]^T`。
- 转向：轮端差动 `tau_left=T/2-tau_turn`、`tau_right=T/2+tau_turn`。
- 协调：`e_sync=theta_right-theta_left`，左右腿施加反相 `Tp_sync`。
- 转向档位：`low=pi/2`、`medium=pi`、`high=2*pi rad/s`。
- 旋转前进：`low` 固定为低速前进加 low 旋转；`high` 固定为高速前进加 medium 旋转。
- roll 支撑补偿和斜坡测试当前未启用。

## 入口

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed high --visualize --visualize-seconds 6 --turn-pd-plot
```

`--turn-speed` 可取 `low`、`medium`、`high`。转向诊断图只输出到 `output\HHMMSS.png`，用于查看原始量、滤波量以及 P/D 力矩分量。

## 最近整理

- 删除三角斜坡模型、`ramp_test` 和 roll 支撑补偿路径。
- 删除通用 history CSV、LQR 图、电机图和 control trace 输出；保留转向 PD 图。
- 删除任务目录下旧 CSV/PNG 实验产物。

## 验证

- 2026-07-12：`py_compile` 通过。
- 2026-07-12：`--turn-test --turn-speed medium --visualize-seconds 0.02 --turn-pd-plot` 完成有限步运行；仅证明入口和接口可执行，不构成平衡结论。
