# AI_README.md

本仓库使用轻量 AI 协作流程。当前不是完整工程化训练平台，而是 MuJoCo 机器人实验脚本阶段；仓库里已经有最小 residual RL Env 和 PPO 入口，用于本地 smoke、服务器训练连通性检查和后续 ONNX 交接。

AI 的主要任务是帮助：

- 读清楚当前模型和脚本。
- 维护物理语义记录。
- 编写小而可运行的实验脚本。
- 运行本地 smoke 检查。
- 把实验过程和验证结果写入 `tasks/`。
- 把控制算法、公式推导和稳定物理量定义写入 `docs/CONTROL_THEORY.md`。

## 当前优先级

先完成底层物理语义和简单底层控制。

在以下内容没有确认前，不推进正式 LQR：

- joint / actuator 命名和映射。
- 单位、正方向、坐标系。
- reset 初态。
- 接触和限幅。
- 简单底层控制检查。

LQR 不能用来补偿底层物理语义错误。

## 已确认物理语义保护

`docs/CONTROL_THEORY.md` 中“已确认物理语义”是当前底层控制的长期约束，不是任务日志。

后续 AI 不得在没有重新实验的情况下修改或反转：

- 虚拟杆 `theta` 的定义和正方向。
- `F_l` 的正方向。
- `J_l^T * F_l` 到关节力矩的映射语义。
- actuator gear、ctrlrange 和力矩换算关系。
- 正常腿型分支的几何判据。
- LQR 的工作点控制律 `U = U0 - K*(X-X0)`。

当前已确认：`F_l` 通道有效，重力补偿能进入 VMC；腿长静差主要指向接触/LQR/VMC 工作点不一致，不能先假设机械 XML 错、`F_l` 符号反或前馈没接线。

## AI 开始任务前读取

每次实现任务先读：

1. `README.md`
2. `AGENTS.md`
3. `docs/INDEX.md`
4. `docs/CODING_RULES.md`
5. `docs/CONTROL_THEORY.md`
6. 任务相关脚本或模型

非平凡修改前，AI 简短说明范围、风险和验证方式。

## 本地验证

默认只跑本地轻量检查：

- Python 语法或 import。
- MuJoCo 模型加载。
- smoke 脚本。
- 很短的 rollout 或小输入探测。

不要默认做大规模训练。当前也不需要完整工程化流程。

## 以后服务器训练怎么接

只保留轻量规则：

- `server_training/` 保存最小 Env、任务清单和 PPO 入口；不要把它扩成臃肿平台。
- 训练任务以 `server_training/residual_rl_tasks.yaml` 为准，当前只包含 `forward_jump_medium/high`、`flight_ramp_medium/high`、`inplace_jump`。
- episode 时长必须使用任务文档中的完整 MuJoCo 仿真时间；不要为了提速裁剪任务。提速应通过 headless、并行 Env 和策略/控制更新频率实现。
- 服务器训练完成后，用户会下载训练结果压缩包到本地结果文件夹。
- 后续需要本地脚本把结果转成 ONNX。
- 再用 ONNX 在本地 MuJoCo 可视化测试小车。

当前阶段不要提前写复杂服务器训练流程，也不要把训练日志、checkpoint、tensorboard、wandb 或临时输出提交进仓库。
