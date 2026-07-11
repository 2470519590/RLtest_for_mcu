# AGENTS.md

本文件定义当前 MuJoCo 实验脚本阶段的 AI 工作规则。

## 当前目标

先完成物理语义和底层控制，不把底层问题带到上层 LQR。

当前阶段只需要轻量流程：

- 看清模型。
- 写小脚本。
- 跑本地 smoke。
- 记录结果。

不需要完整工程化流程；服务器训练只保留远期交接规则。

## 开始任务前

AI 修改代码或文档前必须读取：

1. `README.md`
2. `AI_README.md`
3. `docs/INDEX.md`
4. `docs/CODING_RULES.md`
5. `docs/CONTROL_THEORY.md`
6. 任务相关脚本、模型或任务记录

如果任务会改变物理语义，先说明影响并等用户确认。

## 预编辑摘要

非平凡修改前，输出：

- `Task`：要做什么。
- `Context read`：读了哪些文件。
- `In scope`：会改哪些文件。
- `Out of scope`：不会改什么。
- `Risk`：是否影响物理语义或底层控制。
- `Verification`：准备怎么检查。

## 实验脚本规则

- 根目录 `.py` 只能是可直接运行入口。
- 可复用代码放到 `src/`。
- 不留下无用临时脚本。
- 不提交 `__pycache__`、日志、缓存、checkpoint 或大体积输出。
- 实验过程、验证命令和验证结果写进 `tasks/`。
- 控制算法、公式推导、稳定物理量定义和当前有效物理语义写进 `docs/CONTROL_THEORY.md`。
- `docs/CONTROL_THEORY.md` 禁止写时间线、修改流水账、smoke 输出和任务完成记录。
- 每完成一段调试，最终回复必须给出用户可本地运行的命令；如果需要判断物理行为，优先给可视化命令。

## 物理语义门禁

以下内容没确认前，不推进正式 LQR：

- joint / actuator 映射。
- 单位和正方向。
- reset 初态。
- 接触和限幅。
- 简单底层控制检查。

LQR、行为克隆、PPO 不能用来掩盖这些问题。

## 验证

优先使用最小检查：

- `py310` 下 Python 语法检查。
- MuJoCo 模型加载。
- `run_smoke.py`。
- 小输入或短 rollout。

没有运行的检查不要说通过。

## 最终回复

任务完成后简短说明：

- `Summary`
- `Files changed`
- `Verification`
- `Risks`
- `Next`
- 本地运行命令
