# TASK_TEMPLATE.md

用于描述一个小实验任务。

## 任务

标题：

目标：

## 范围

允许修改：

-

不修改：

-

## 必读

- `README.md`
- `AGENTS.md`
- `docs/CODING_RULES.md`
- `docs/CONTROL_THEORY.md`
- 相关脚本或模型

## 物理语义检查

本任务是否影响：

- [ ] joint / actuator 映射
- [ ] 单位或正方向
- [ ] reset 初态
- [ ] 接触或限幅
- [ ] observation/action 草案
- [ ] 底层控制
- [ ] 不影响

如果影响，必须更新 `docs/CONTROL_THEORY.md`。

## LQR 门禁

底层物理语义和简单底层控制未通过前，不做正式 LQR。

## 验证

计划运行：

- [ ] Python 语法/import
- [ ] `run_smoke.py`
- [ ] 小输入或短 rollout
- [ ] 人工审查

## 记录

重要实验结果写入：

- [ ] `docs/CONTROL_THEORY.md`
- [ ] `tasks/YYYY-MM-DD-HHMMSS-topic.md`
