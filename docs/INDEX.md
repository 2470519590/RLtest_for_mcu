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

以后需要时再新增：

- `server_training/`：只差正式训练时放服务器训练代码和配置。
- `trained_results/`：本地下载服务器训练压缩包后使用。
- `tools/`：后续可放转 ONNX 或可视化测试脚本。

## 关键文档

- `docs/CONTROL_THEORY.md`：当前控制框架、公式和有效物理语义。
- `docs/DECISIONS.md`：影响后续方向的重要决策。
- `docs/CODING_RULES.md`：实验脚本规则。
- `docs/DEBUG_GUIDE.md`：调试提示。
- `docs/ERROR_CATALOG.md`：已经确认的调试错误、根因和避免重复踩坑的经验。
- `tasks/CONTROL_FRAMEWORK.md`：当前控制框架大任务记录。后续同类小实验不要新建零散任务文件。

## 默认不要读取或提交

- `__pycache__/`
- `.pytest_cache/`
- 日志和缓存
- checkpoint
- 大体积训练输出
- 临时实验垃圾文件

## 当前已知入口

- `run_smoke.py`

当前还没有正式环境、LQR、行为克隆或 PPO 代码。

## 代码分包

- `src/robot_smoke/core/`：常量、类型和 MuJoCo 通用工具。
- `src/robot_smoke/model/`：模型语义、actuator、五连杆、运动学和接触采样。
- `src/robot_smoke/control/`：IK、VMC、LQR 和 LQR 设计。
- `src/robot_smoke/experiments/`：本地 smoke、equilibrium search、诊断和 trace。
- `src/robot_smoke/io/`：CLI 和转向 PD 图输出。
- `src/robot_smoke/runner.py`：入口编排。
