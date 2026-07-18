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
│  └─ jump.txt                     # 临时参考文本；不作为当前 RL/控制语义依据
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
```

含义：

- `low`：低速前进 + 低速旋转


### 6. 变腿长高速旋转

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-length-sine-test --visualize --visualize-seconds 10
```

该入口用于观察高速原地旋转时，腿长在 `minimum_leg_length..maximum_leg_length` 范围内做低频正弦跟踪的效果。

### 7. Roll 坡道测试

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --roll-test --visualize --visualize-seconds 10
```

该入口注入专用坡道，用于观察论文 2.2 双腿长度控制和 roll 补偿通道。

### 8. 飞坡 / 离地检测

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --flight-test --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --flight-test --flight-test-speed medium --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --flight-test --flight-test-speed high --visualize --visualize-seconds 10
```

该入口注入全宽飞坡，并启用论文第 3 节离地检测。当前飞坡与跳跃仍是诊断入口，不是合格 RL 任务结论。

### 9. 斜坡 Roll 原地旋转(不作为任务，没有调好)

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --slope-roll-turn-test --visualize --visualize-seconds 10
```

该入口复用飞坡场地但不启用离地检测：先中速前进到坡上，停车 1 秒，再低速原地旋转。停车开始时间可调：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --slope-roll-turn-test --slope-roll-turn-start-time 2.3 --visualize --visualize-seconds 10 --lqr-debug-plot
```

当前已暂停继续做坡上静止平衡，不把该场景作为 RL 前置合格项。

### 10. 原地跳跃

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --jump-test --visualize --visualize-seconds 10
```

### 11. 前进跳跃

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --forward-jump-test medium --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --forward-jump-test high --visualize --visualize-seconds 10
```

跳跃相关入口目前仍不合格，后续可交给 RL 优化落地姿态，但训练前必须明确状态机、接触检测和参考命令语义。

## Residual RL 控制框架

当前方向是保留可移植传统控制器，在其上训练一个小 residual policy：

```text
遥控器 / 任务命令
  -> 简单 phase: GROUND / TAKEOFF / AIRBORNE / LANDING
  -> LQR + VMC + 腿长 PID + yaw/sync/roll PD/P
  -> 小 MLP residual policy
  -> 限幅与安全腿长 clamp
  -> 电机命令
```

MCU 部署目标：

- 传统控制环约 `1 kHz`。
- residual NN 约 `100 Hz`，两层 `32x32`，`tanh`，输出保持到下一次推理。
- 最终导出 ONNX，再转成 STM32F407 可用的 float32 权重/前向代码；默认启用硬浮点。
- LQR 调度表 `length_schedule.yaml` 可离线转成 C 数组并线性插值，不在 MCU 上求 Riccati。

Residual action 第一版固定 5 个通道，不继续堆参数：

```text
delta_T
delta_Tp
delta_F_l_common
delta_L_ref_left
delta_L_ref_right
```

策略不允许输出 phase/mode。phase 由遥控器命令和仿真观测触发；起跳 phase 允许切到 position 模式，RL 输出左右腿长参考，再由 IK 转成 position target。飞坡不是主动起跳任务，只作为带速度的空中/落地平衡场景。

训练任务清单只保留在 `server_training/residual_rl_tasks.yaml`，用于服务器并行采样和日志分桶。任务分组原则：

- forward jump：`medium / high`，共用 jump、airborne、landing 策略；`low` 低速带速度跳不作为训练任务。
- flight ramp：`medium / high`，只训练空中姿态和落地平衡；`low` 低速飞坡不作为训练任务。
- inplace jump：原地起跳、空中姿态、落地平衡。
- landing 策略共用，但 landing command 分为 `quick_stop` 和 `keep_speed`。

服务器训练方向：Gymnasium-style env + Stable-Baselines3 PPO；训练产物、日志、checkpoint 不进仓库。完成训练后导出 ONNX，再单独做本地 MuJoCo 可视化验证。

当前最小 Env 原型入口：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_residual_env_smoke.py --task-key inplace_jump --steps 5
& 'E:\miniconda\envs\py310\python.exe' run_residual_env_smoke.py --task-key inplace_jump --controller-mode lqr --visualize --visualize-seconds 5
& 'E:\miniconda\envs\py310\python.exe' run_residual_env_smoke.py --task-key inplace_jump --controller-mode lqr_residual --visualize --visualize-seconds 5 --action 0.2 0.1 0.0 0.1 -0.1
& 'E:\miniconda\envs\py310\python.exe' run_residual_env_smoke.py --task-key flight_ramp_medium --controller-mode lqr --visualize --visualize-seconds 8 --viewer-sync-hz 20
& 'E:\miniconda\envs\py310\python.exe' run_residual_env_smoke.py --task-key flight_ramp_medium --compare-zero-residual --visualize-seconds 8
```

`--task-key` 使用 `server_training/residual_rl_tasks.yaml` 内的 5 个训练 key。`--action` 是归一化 residual 测试动作，顺序为 `[delta_T, delta_Tp, delta_F_l_common, delta_L_ref_left, delta_L_ref_right]`，每个值在 `[-1,1]` 内，再由 Env 内部固定限幅缩放到真实单位。`--controller-mode lqr` 用来对照原始 LQR/VMC 基线；`--controller-mode lqr_residual` 才会把 `--action` 叠加到名义控制上。

当前并行 PPO 训练入口：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_train_residual_ppo.py --tasks all --vec-env subproc --n-envs 5 --total-timesteps 200000 --n-steps 64 --batch-size 256 --run-name residual_ppo_mh_jump_ramp
```

训练任务来自 `server_training/residual_rl_tasks.yaml`，当前为 `forward_jump_medium/high`、`flight_ramp_medium/high`、`inplace_jump`。训练 episode 默认 `10 s` 是 MuJoCo 仿真时间，不是墙钟时间；训练脚本不打开 viewer、不按 realtime sleep，会尽可能快地把完整任务仿真完。不要把 episode 裁剪到 2 s 来判断行为改善。产物默认写入 `runs/residual_ppo/<run-name>/`，不进仓库。若服务器想先做最小连通性检查，可用：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_train_residual_ppo.py --tasks inplace_jump --vec-env dummy --n-envs 1 --total-timesteps 16 --n-steps 8 --batch-size 8 --episode-sim-seconds 0.05 --run-name smoke
```

上面的 `--episode-sim-seconds 0.05` 只用于检查训练管线能否启动和保存模型，不是训练设置。

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
- 控制器接口已按两种模式预留：`rl_controller_mode=lqr` 表示纯原始 LQR；`rl_controller_mode=lqr_residual` 表示在原始 LQR 中层输出后叠加有界 residual RL。当前默认 residual 策略为 0，且 residual 限幅默认为 0，因此该模式现在只用于接口 smoke，不代表已经开始训练。
- residual RL 接口的动作语义为公共轮端力矩 `T`、虚拟腿俯仰力矩 `Tp`、公共腿长力增量 `length_force_delta`、左右腿长参考增量的有界增量；它不改写 LQR 状态定义、VMC/Jacobian 映射、actuator 符号或底层控制器逻辑。离地 `airborne` 阶段同样属于 RL 优化范围：论文第 3 节门控只是名义基线，residual 可用于起跳、空中姿态、收腿/伸腿和落地恢复。
- 训练日志、checkpoint、tensorboard、wandb、临时 CSV/PNG 不进 git。
- `output/`、`tmp/`、`checkpoints/`、`runs/`、`wandb/`、`trained_results/` 已加入忽略。
- 飞坡和跳跃目前适合做诊断或后续 RL 任务原型，不应写成“已通过基准”。
