# MuJoCo 轮腿机器人实验脚本

本仓库当前处于 MuJoCo 本地实验阶段，目标是先把模型物理语义、五连杆 VMC、腿长控制、平衡 LQR、转向和基础运动测试跑清楚。底层物理问题不能交给 LQR、强化学习或参数硬调来掩盖。

所有“是否站稳、是否摔倒、是否动作合理”的结论，以 MuJoCo viewer 人工观察为准；曲线和数据只用于定位通道、符号、尺度、饱和和接触问题。

## 主要文件

- `assets/biped_wheel_leg.xml`：MuJoCo 轮腿模型。
- `run_smoke.py`：本地 smoke 和可视化测试入口。
- `config/smoke.yaml`：当前主要参数配置。
- `config/length_schedule.yaml`：腿长调度表，包含不同腿长下的 `F_l0 / X0 / U0 / K`。
- `src/robot_smoke/`：模型、控制、实验和绘图代码。
- `docs/CONTROL_THEORY.md`：当前有效控制公式和物理语义。
- `tasks/CONTROL_FRAMEWORK.md`：同一控制框架下的实验记录。
- `RL说明.md`：Residual RL Env、任务清单、PPO 训练和本地验证命令。
- `run_residual_env_smoke.py`：Residual RL Env smoke / 可视化 / 零残差对照入口。
- `run_train_residual_ppo.py`：最小 Stable-Baselines3 PPO 训练入口，输出默认写入忽略目录 `runs/`。
- `run_residual_policy_eval.py`：加载 PPO `.zip` 策略后的 headless / viewer 评估入口。
- `run_export_residual_policy_onnx.py`：把 PPO `.zip` residual actor 导出为 ONNX 的入口。

## 环境

默认使用 `py310` conda 环境：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py
```

基础可视化：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --visualize
```

## 当前测试入口

默认腿长来自 `config/smoke.yaml`，当前为 `0.24 m`；腿长调度表默认启用。`--visualize-seconds` 控制可视化运行时长。

### 平衡测试

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --visualize --visualize-seconds 10
```

### 直线速度测试

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile low --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile medium --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile high --visualize --visualize-seconds 10
```

### 平衡冲击测试

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --impact small --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --impact medium --visualize --visualize-seconds 10
```

### 原地旋转测试

`--turn-speed` 可取 `low`、`medium`、`high`。当前约定为：

- `low = pi/2 rad/s`
- `medium = pi rad/s`
- `high = 10 rad/s`

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed low --visualize --visualize-seconds 6
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed medium --visualize --visualize-seconds 6
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed high --visualize --visualize-seconds 6
```

### 变腿长高速旋转测试

该测试会高速原地旋转，同时让腿长参考在 `minimum_leg_length..maximum_leg_length` 范围内做低频正弦跟踪。当前腿长周期为 `1.5 s`，自转和腿长变化同时开始、同时结束。

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-length-sine-test --visualize --visualize-seconds 10 --roll-length-plot
```

### 旋转前进测试

`low` 表示低速前进加低速旋转；`high` 表示高速前进加中速旋转。

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-drive-test low --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-drive-test high --visualize --visualize-seconds 10
```

### Roll 坡道测试

`--roll-test` 会注入专用临时坡道场地，普通测试不带这些坡。该入口用于观察文章 2.2 的双腿长度和横滚补偿通道。

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --roll-test --roll-length-plot --visualize --visualize-seconds 10
```

### 飞坡 / 离地检测测试

该入口会注入全宽飞坡，并启用论文第 3 节离地检测。离地检测使用左右轮接触法向力；双轮法向力低于阈值后进入离地模式。

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --flight-test --visualize --visualize-seconds 10 --lqr-debug-plot
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --flight-test --flight-test-speed medium --visualize --visualize-seconds 10 --lqr-debug-plot
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --flight-test --flight-test-speed high --visualize --visualize-seconds 10 --lqr-debug-plot
```

### 斜坡 ROLL 原地旋转测试

该入口复用飞坡场地，但不启用离地检测。小车先中速前进到全宽斜坡上，随后停止速度参考并停 1 秒，再启动低速原地旋转，用于观察斜坡上 roll/腿长补偿和双腿协调。

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --slope-roll-turn-test --visualize --visualize-seconds 10 --lqr-debug-plot
```

如果停车开始时小车还没有到坡上，或已经越过坡面，可调整：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --slope-roll-turn-test --slope-roll-turn-start-time 2.3 --visualize --visualize-seconds 10 --lqr-debug-plot
```

### 原地跳跃测试

该入口用于诊断跳跃流程：默认工作点支撑，随后下蹲到最小腿长，再快速伸腿；离地检测和落地逻辑保持启用。

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --jump-test --visualize --visualize-seconds 10 --lqr-debug-plot
```

### 前进跳跃测试

该入口先执行指定速度档位的前进测试，只在匀速阶段且双腿世界竖直角满足触发条件时启动跳跃流程。

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --forward-jump-test low --visualize --visualize-seconds 10 --lqr-debug-plot
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --forward-jump-test medium --visualize --visualize-seconds 10 --lqr-debug-plot
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --forward-jump-test high --visualize --visualize-seconds 10 --lqr-debug-plot
```

## 绘图诊断接口

这些接口只用于调试，不默认开启。

转向 PD 图：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed high --visualize-seconds 6 --turn-pd-plot
```

腿长 / Roll 图：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --visualize-seconds 10 --roll-length-plot
```

LQR 调试图：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile high --visualize-seconds 10 --lqr-debug-plot
```

图片默认输出到 `output\HHMMSS.png`。

## Residual RL / PPO 当前入口

当前已有最小 residual RL 训练原型，但它仍服务于本地验证和服务器训练交接，不表示飞坡、跳跃或落地行为已经合格。

训练任务只保留 5 个 key：

- `forward_jump_medium`
- `forward_jump_high`
- `flight_ramp_medium`
- `flight_ramp_high`
- `inplace_jump`

低速飞坡和低速带速度跳不作为 RL 训练任务，因为低速场景会先绊倒，不能提供有效起跳/落地样本。

Env smoke：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_residual_env_smoke.py --task-key flight_ramp_medium --steps 3 --episode-seconds 10 --step-seconds 0.02
```

零残差对照：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_residual_env_smoke.py --task-key flight_ramp_medium --compare-zero-residual --episode-seconds 10 --visualize-seconds 10 --step-seconds 0.02
```

本地单回合测速：

```powershell
Measure-Command { & 'E:\miniconda\envs\py310\python.exe' run_residual_env_smoke.py --task-key flight_ramp_medium --controller-mode lqr_residual --steps 500 --episode-seconds 10 --step-seconds 0.02 --action 0 0 0 0 0 | Out-Null }
```

短 PPO 连通性检查：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_train_residual_ppo.py --tasks flight_ramp_medium --total-timesteps 500 --n-envs 1 --n-steps 500 --batch-size 250 --episode-sim-seconds 10 --step-seconds 0.02 --run-name local_single_episode_50hz_speed_check --device cpu
```

`--episode-sim-seconds 10` 是 MuJoCo 仿真时间，不能为了训练速度裁剪成 2 秒；训练加速应来自 headless、并行 Env 和降低 residual policy 推理频率。`--step-seconds 0.02` 表示 50 Hz residual policy 更新，MuJoCo 仍推进完整 10000 个 1 ms 物理步，底层 LQR/PID/VMC 默认保持每个 1 ms 物理步更新。若显式传参，使用 `--control-decimation-steps 1` 保持该语义。

服务器并行训练建议：

```bash
python run_train_residual_ppo.py \
  --tasks all \
  --vec-env subproc \
  --subproc-start-method forkserver \
  --n-envs 5 \
  --total-timesteps 1000000 \
  --n-steps 500 \
  --batch-size 1000 \
  --episode-sim-seconds 10 \
  --step-seconds 0.02 \
  --control-decimation-steps 1 \
  --checkpoint-freq 50000 \
  --run-name residual_ppo_all_1m_50hz
```

训练后加载模型评估：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_residual_policy_eval.py --model runs\residual_ppo\residual_ppo_all_1m_50hz\models\final_model.zip --task-key flight_ramp_medium --episode-sim-seconds 10 --step-seconds 0.02 --control-decimation-steps 1 --print-every 25 --device cpu
```

本地打开 viewer 看动作：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_residual_policy_eval.py --model runs\residual_ppo\residual_ppo_all_1m_50hz\models\final_model.zip --task-key flight_ramp_medium --episode-sim-seconds 10 --step-seconds 0.02 --control-decimation-steps 1 --visualize --viewer-sync-hz 30 --device cpu
```

## 脚本规则

- 根目录 `.py` 只保留可直接运行入口。
- 可复用代码放到 `src/robot_smoke/`。
- 不保留无用临时脚本、日志、缓存、checkpoint 或大体积输出。
- 实验过程和验证结果写入 `tasks/CONTROL_FRAMEWORK.md`。
- 控制公式、稳定物理量定义和当前有效物理语义写入 `docs/CONTROL_THEORY.md`。
- 涉及机器人动作的判断必须优先给出可视化命令，由 viewer 人工确认。

## 训练交接状态

当前已有最小 PPO/Env 入口，可以用于服务器并行训练前的连通性验证。仓库仍不是完整工程化训练平台：日志、checkpoint、tensorboard、wandb、临时输出和训练结果不进仓库；训练结果后续再下载到本地并转换为 ONNX 做 MuJoCo 可视化验证。
