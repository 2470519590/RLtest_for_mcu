# INDEX.md

本文件是当前实验脚本阶段的目录索引。

## 当前目录

- `assets/`：MuJoCo 模型。
- `src/robot_smoke/`：本地 smoke 和物理语义检查代码。
- `run_smoke.py`：根目录可运行 smoke 入口。
- `docs/`：少量项目记录。
- `tasks/`：只保留大任务记录；同类小实验更新同一个任务文件。
- `templates/`：轻量任务模板。
- `prompts/`：轻量提示词。
- `server_training/`：服务器 residual RL 任务清单；当前只放训练配置语义，不放 checkpoint 或日志。

以后需要时再新增：

- `trained_results/`：本地下载服务器训练压缩包后使用。
- `tools/`：后续可放转 ONNX 或可视化测试脚本。

## 关键文档

- `docs/CONTROL_THEORY.md`：当前控制框架、公式和有效物理语义。
- `docs/DECISIONS.md`：影响后续方向的重要决策。
- `docs/CODING_RULES.md`：实验脚本规则。
- `docs/DEBUG_GUIDE.md`：调试提示。
- `docs/ERROR_CATALOG.md`：已经确认的调试错误、根因和避免重复踩坑的经验。
- `tasks/CONTROL_FRAMEWORK.md`：当前控制框架大任务记录。后续同类小实验不要新建零散任务文件。
- `RL说明.md`：residual RL 多任务接口、任务时间和服务器训练指令。
- `server_training/residual_rl_tasks.yaml`：服务器训练任务 key、速度档和入口参数清单。
- `server_training/residual_env.py`：最小 residual RL Env 原型，复用现有 MuJoCo smoke rollout。
- `run_residual_env_smoke.py`：根目录可直接运行的最小 Env smoke 入口。
- `run_train_residual_ppo.py`：根目录可直接运行的 residual PPO 训练入口，输出默认写入 `runs/`。

## 默认不要读取或提交

- `__pycache__/`
- `.pytest_cache/`
- 日志和缓存
- checkpoint
- 大体积训练输出
- 临时实验垃圾文件

## 当前已知入口

- `run_smoke.py`

当前还没有正式 PPO/SAC 训练代码；已有 residual RL 控制接口和服务器任务清单。

## 代码分包

- `src/robot_smoke/core/`：常量、类型和 MuJoCo 通用工具。
- `src/robot_smoke/model/`：模型语义、actuator、五连杆、运动学和接触采样。
- `src/robot_smoke/control/`：IK、VMC、LQR 和 LQR 设计。
- `src/robot_smoke/experiments/`：本地 smoke、equilibrium search、诊断和 trace。
- `src/robot_smoke/io/`：CLI 和转向 PD 图输出。
- `src/robot_smoke/runner.py`：入口编排。
