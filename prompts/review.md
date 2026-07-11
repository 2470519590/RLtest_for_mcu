# 评审提示词

只 review，不编辑。

重点看：

- 是否改坏 joint / actuator 映射。
- 是否改了单位、方向、reset、接触或限幅。
- 是否在底层未通过前推进 LQR。
- 根目录 `.py` 是否都是可运行入口。
- 是否留下缓存、日志、checkpoint 或无用脚本。
- 重要结果是否写入 `docs/CONTROL_THEORY.md` 或 `tasks/`。

输出 findings，按严重程度排序。
