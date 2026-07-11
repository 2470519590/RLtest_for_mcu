# DECISIONS.md

本文件只记录会影响后续实验方向的重要决策。公式和物理量写入 `CONTROL_THEORY.md`，错误经验写入 `ERROR_CATALOG.md`，实验过程写入 `tasks/CONTROL_FRAMEWORK.md`。

## 决策

### 2026-07-10：当前主入口锁定为 true-equilibrium LQR + VMC

- 决策：常用入口使用 `run_smoke.py --lqr-true-equilibrium`。
- 原因：旧的分层 PD/VMC 和 transient endpoint 不能作为可靠工作点。
- 影响：默认围绕 `X0/U0/F_l0` 工作点运行，不暴露大批调参入口。
- 验证：短 smoke finite，`saturated_steps=0`，分支 violation 为 0。

### 2026-07-10：VMC 主路径使用解析五连杆 Jacobian

- 决策：`J=d[L,theta]/d[q_front,q_rear]` 使用解析几何计算。
- 原因：接触参与的数值差分会污染 `dL/dq`，导致支撑力矩严重偏小。
- 影响：数值 Jacobian 只作为诊断或解析失败回退，不作为主控制映射。
- 验证：解析映射后 `F_l` 支撑力矩尺度恢复，`L0=0.35` 工作点可站立。

### 2026-07-10：LQR 状态的 x/dx 使用轮源

- 决策：主平衡状态中 `x/dx` 来自左右轮角度和轮速。
- 原因：参考文章的轮腿倒立摆状态使用轮端运动；`base_x_dot` 会混入浮动基和接触诊断语义。
- 影响：`base_x/base_x_dot` 只作为漂移或打滑诊断。
- 验证：当前代码默认 `x_source="wheel"`。

### 2026-07-10：任务记录合并为大任务文档

- 决策：后续同类小实验只更新 `tasks/CONTROL_FRAMEWORK.md`。
- 原因：大量小任务文档已经变成流水账，难以追踪控制框架。
- 影响：不再为每个小测试新增单独任务文件。
- 验证：本轮开始清理旧小任务记录。

### 2026-07-10：单个 Python 文件目标不超过 2000 行

- 决策：`src/robot_smoke/` 下 Python 文件按职责拆分，目标单文件小于 2000 行。
- 原因：`model_smoke.py` 已经超过可维护规模。
- 影响：拆分方向为 `actuators.py`、`fivebar.py`、`vmc.py`、`lqr.py`、`equilibrium.py`、`viewer.py`、`runner.py`。
- 验证：每次拆分后必须跑 `py_compile` 和短 smoke。
