# RL 说明

本文档用于开始强化学习入口前的交接。当前仓库仍以 MuJoCo 本地控制验证为主，RL 只能在这些底层物理语义和测试入口之上继续做，不应反过来掩盖 VMC、LQR、接触、腿长或转向通道的问题。

## 当前状态

- 默认模型：`assets/biped_wheel_leg.xml`
- 主配置：`config/smoke.yaml`
- 腿长调度表：`config/length_schedule.yaml`
- 主入口：`run_smoke.py`
- 默认腿长：当前由 `config/smoke.yaml` 控制，最近使用的是 `0.24 m`
- 当前主要控制链路：五连杆解析 IK/FK/Jacobian、VMC、腿长 PID+前馈、LQR 平衡、速度跟踪、yaw 差速、双腿同步、roll 补偿、离地检测

所有“是否站稳、是否摔倒、动作是否合理”的结论以 MuJoCo viewer 人工观察为准。曲线只用于定位状态、参考、输入、饱和和接触问题。

## 项目结构

```text
.
├─ assets/
│  └─ biped_wheel_leg.xml          # MuJoCo 轮腿模型
├─ config/
│  ├─ smoke.yaml                   # 本地仿真与控制参数
│  └─ length_schedule.yaml         # 多腿长 F_l0 / X0 / U0 / K 调度表
├─ docs/
│  ├─ CONTROL_THEORY.md            # 当前有效控制公式和物理语义
│  ├─ CODING_RULES.md              # 代码与验证规则
│  ├─ DEBUG_GUIDE.md               # 调试提示
│  ├─ DECISIONS.md                 # 重要决策
│  └─ ERROR_CATALOG.md             # 已知错误和排查经验
├─ ref/
│  ├─ j.cnki.xk.2023.2533.pdf      # 论文原文
│  ├─ j.cnki.xk.2023.md            # 论文文字提取版
│  └─ jump.txt                     # 跳跃参考说明
├─ src/robot_smoke/
│  ├─ core/                        # 配置、类型、MuJoCo 通用工具
│  ├─ model/                       # 执行器、五连杆、运动学、接触与力学
│  ├─ control/                     # IK、VMC、LQR、转向、roll、轨迹
│  ├─ experiments/                 # smoke、诊断、工作点和可视化实验
│  ├─ io/                          # CLI 与绘图输出
│  └─ runner.py                    # run_smoke.py 的实际编排
├─ tasks/
│  └─ CONTROL_FRAMEWORK.md         # 当前控制框架任务记录
├─ analyze_length_workpoints.py    # 多腿长工作点/调度表分析入口
├─ run_smoke.py                    # 本地仿真主入口
└─ RL说明.md                       # 本文件
```

## 仿真入口

以下命令默认使用：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
```

### 1. 静止平衡

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --visualize --visualize-seconds 10
```

### 2. 直线速度跟踪

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile low --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile medium --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile high --visualize --visualize-seconds 10
```

### 3. 平衡冲击

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --impact small --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --impact medium --visualize --visualize-seconds 10
```

### 4. 原地旋转

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed low --visualize --visualize-seconds 6
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed medium --visualize --visualize-seconds 6
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed high --visualize --visualize-seconds 6
```

当前约定：

- `low = pi/2 rad/s`
- `medium = pi rad/s`
- `high = 10 rad/s`

### 5. 旋转前进

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-drive-test low --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-drive-test high --visualize --visualize-seconds 10
```

含义：

- `low`：低速前进 + 低速旋转
- `high`：高速前进 + 中速旋转

### 6. 变腿长高速旋转

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-length-sine-test --visualize --visualize-seconds 10 --roll-length-plot
```

该入口用于观察高速原地旋转时，腿长在 `minimum_leg_length..maximum_leg_length` 范围内做低频正弦跟踪的效果。

### 7. Roll 坡道测试

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --roll-test --roll-length-plot --visualize --visualize-seconds 10
```

该入口注入专用坡道，用于观察论文 2.2 双腿长度控制和 roll 补偿通道。

### 8. 飞坡 / 离地检测

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --flight-test --visualize --visualize-seconds 10 --lqr-debug-plot
```

该入口注入全宽飞坡，并启用论文第 3 节离地检测。当前飞坡与跳跃仍是诊断入口，不是合格 RL 任务结论。

### 9. 斜坡 Roll 原地旋转

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --slope-roll-turn-test --visualize --visualize-seconds 10 --lqr-debug-plot
```

该入口复用飞坡场地但不启用离地检测：先中速前进到坡上，停车 1 秒，再低速原地旋转。停车开始时间可调：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --slope-roll-turn-test --slope-roll-turn-start-time 2.3 --visualize --visualize-seconds 10 --lqr-debug-plot
```

当前已暂停继续做坡上静止平衡，不把该场景作为 RL 前置合格项。

### 10. 原地跳跃

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --jump-test --visualize --visualize-seconds 10 --lqr-debug-plot
```

### 11. 前进跳跃

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --forward-jump-test low --visualize --visualize-seconds 10 --lqr-debug-plot
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --forward-jump-test medium --visualize --visualize-seconds 10 --lqr-debug-plot
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --forward-jump-test high --visualize --visualize-seconds 10 --lqr-debug-plot
```

跳跃相关入口目前仍不合格，后续可交给 RL 优化落地姿态，但训练前必须明确状态机、接触检测和参考命令语义。

## 绘图入口

### 转向 PD 图

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed high --visualize-seconds 6 --turn-pd-plot
```

### 腿长 / Roll 图

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --visualize-seconds 10 --roll-length-plot
```

### LQR debug 图

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile high --visualize-seconds 10 --lqr-debug-plot
```

图片默认输出到 `output/HHMMSS.png`，该目录已加入 `.gitignore`。

## RL 前置注意事项

- 不要把 RL 作为修复物理语义的工具。VMC、腿长、LQR、接触和执行器方向必须先保持可解释。
- RL 环境应复用 `src/robot_smoke/` 中已经拆分好的模型、控制和观测逻辑，不要复制一份新的物理语义。
- 训练日志、checkpoint、tensorboard、wandb、临时 CSV/PNG 不进 git。
- `output/`、`tmp/`、`checkpoints/`、`runs/`、`wandb/`、`trained_results/` 已加入忽略。
- 飞坡和跳跃目前适合做诊断或后续 RL 任务原型，不应写成“已通过基准”。
