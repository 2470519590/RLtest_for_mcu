# 2026-07-09：控制输出溯源曲线

## 任务

把控制输出溯源从逐步刷屏改成 CSV 和 PNG 曲线，便于观察电机输出是否真的在跳，以及跳变主要来自 LQR 中层、VMC、限幅裁剪还是 viewer 显示。

## 修改

- 新增 `--trace-control-csv PATH`。
- 新增 `--trace-control-plot PATH`。
- `--trace-control-mode summary` 可只打印摘要和文件路径，不再打印逐步 trace。
- 曲线包含：
  - `max_ctrl_delta`。
  - `dT`、`dTp`。
  - 左右腿 `dF_l`、`dF_theta`。
  - 左右腿 `F_l`、`F_theta`。
  - `max_raw_tau_delta` 和 `max_abs_ctrl`。
  - `LQR/middle`、`VMC`、`clip`、`viewer_possible` 来源标记。

## 验证命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --trace-control-output --trace-control-mode summary --trace-control-max-steps 800 --trace-control-plot tasks\2026-07-09-control-trace.png --trace-control-csv tasks\2026-07-09-control-trace.csv --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 验证结果

- 语法检查通过。
- 已生成 `tasks\2026-07-09-control-trace.png`。
- 已生成 `tasks\2026-07-09-control-trace.csv`。
- 终端不再打印逐步 trace，只打印 `control_trace_summary` 和文件路径。
- 本次样例 `max_ctrl_delta=0.13137`，最大步在 `step=359`。
- `source_clip_steps=0`，说明该样例的输出跳变不是限幅裁剪主导。
- 曲线显示最大 `max_ctrl_delta` 峰值与 `max_raw_tau_delta` 峰值同步，后续优先看 VMC 映射前后的腿部力矩差分。

## 本地查看

曲线文件：

```text
E:\STM32_PROJ\RL_training\tasks\2026-07-09-control-trace.png
```

表格文件：

```text
E:\STM32_PROJ\RL_training\tasks\2026-07-09-control-trace.csv
```
