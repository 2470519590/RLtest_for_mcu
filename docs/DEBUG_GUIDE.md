# DEBUG_GUIDE.md

当前只需要轻量调试流程。

## 调试顺序

1. 记录现象。
2. 确认使用 `py310`。
3. 运行最小复现脚本。
4. 如果涉及模型，先跑 `run_smoke.py`。
5. 区分事实和猜测。
6. 修复后把过程和验证结果写到 `tasks/`。
7. 只有当产生稳定的控制公式、物理量定义或当前有效物理语义时，才更新 `docs/CONTROL_THEORY.md`。

## 常见检查

- MuJoCo 是否能 import。
- XML 是否能加载。
- joint / actuator 名称是否符合脚本预期。
- action 到 actuator 映射是否符合预期。
- qpos/qvel 是否出现 NaN 或 Inf。
- reset 初态是否合理。

不要为了调试直接启动长训练。
