# 调试提示词

请按轻量实验脚本流程调试。

先读：

1. `README.md`
2. `AGENTS.md`
3. `docs/CODING_RULES.md`
4. `docs/CONTROL_THEORY.md`
5. 相关脚本、模型和任务记录

调试时先区分：

- 观察到的事实
- 猜测
- 下一步最小验证

如果问题涉及 LQR，先检查底层物理语义和底层控制，不要让 LQR 补偿底层错误。

修复后把重要结论写到 `docs/CONTROL_THEORY.md` 或 `tasks/`。
