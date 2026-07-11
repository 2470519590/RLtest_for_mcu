# 2026-07-09：LQR 符号、尺度和曲线实验

## 任务

继续优化 LQR 中层控制，确认轮端输出 `T` 和虚拟腿角向输出 `Tp` 的符号与尺度，并生成本地实验曲线。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 新增 LQR 输出符号参数：
  - `--lqr-wheel-sign`
  - `--lqr-pitch-sign`
- 新增历史记录参数：
  - `--history-csv`
  - `--history-plot`
  - `--history-sample-interval`
- `--history-csv/--history-plot` 只允许配合 `--lqr-test` 使用。
- 默认 `--lqr-gain-scale` 从 `0.02` 调整为 `0.01`。
- 更新 `docs/CONTROL_THEORY.md` 中的 LQR 输出符号和默认尺度定义。

## 符号实验

命令模板：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 800 --lqr-gain-scale 0.02 --lqr-wheel-sign 1 --lqr-pitch-sign 1 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

结果摘要：

| wheel sign | pitch sign | final theta | final pitch | final x | max branch violation | saturated steps |
|---:|---:|---:|---:|---:|---:|---:|
| +1 | +1 | 0.748932 | -0.755866 | -0.045855 | 0 | 213 |
| -1 | +1 | 0.765148 | -0.772650 | -0.107680 | 3.69464e-05 | 271 |
| +1 | -1 | 0.751892 | -0.759515 | -0.046708 | 0 | 217 |
| -1 | -1 | 0.796248 | -0.804915 | -0.120094 | 9.26281e-05 | 273 |

结论：

- 轮端 `T` 使用 `+1` 明显优于 `-1`，反向后车体前后位移误差变大，并出现轻微分支 violation。
- 虚拟腿角向 `Tp` 使用 `+1` 略优于 `-1`。
- 当前继续保留默认符号 `(+1, +1)`。

## 尺度实验

命令模板：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 800 --lqr-gain-scale 0.01 --lqr-wheel-sign 1 --lqr-pitch-sign 1 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

结果摘要：

| gain scale | final theta | final pitch | final x | wheel ctrl max | saturated steps |
|---:|---:|---:|---:|---:|---:|
| 0 | 0.708982 | -0.716269 | -0.044192 | 0 | 256 |
| 0.01 | 0.700063 | -0.706998 | -0.037573 | 0.034976 | 230 |
| 0.02 | 0.748932 | -0.755866 | -0.045855 | 0.0743241 | 213 |
| 0.04 | 0.844274 | -0.852240 | -0.062032 | 0.164041 | 196 |
| 0.08 | 0.963210 | -0.972063 | -0.081876 | 0.383952 | 161 |

结论：

- `0.01` 相比无 LQR 有轻微改善，final theta、final pitch 和 final x 都更小。
- 大于 `0.02` 后姿态误差明显变大，说明当前增益尺度偏大时会把系统推向更大的俯仰/腿角偏差。
- 默认尺度调整为 `0.01`。

## 曲线输出

生成命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 800 --constraint-steps 800 --check-constraints --history-csv tasks\lqr_default_history.csv --history-plot tasks\lqr_default_history.png --history-sample-interval 5 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

输出文件：

- `tasks/lqr_default_history.csv`
- `tasks/lqr_default_history.png`

验证结果：

- 语法检查通过。
- 800 step 默认 LQR smoke 状态 finite。
- 闭链最大误差约 `2.50816e-05 m`。
- 左右 `max_branch_violation = 0`。
- `max_abs_lqr_wheel_torque = 0.839424`。
- `max_abs_lqr_pitch_torque = 1.2417`。
- wheel motor 未饱和，最大 wheel ctrl 约 `0.034976`。
- 腿部 VMC 仍有饱和，至少一个 actuator 饱和 `230` step。

## 结论

当前符号采用 `T=+1`、`Tp=+1`，默认尺度采用 `gain_scale=0.01`。

曲线显示中层 LQR 输出没有打满 wheel motor，但腿部 VMC 在落地和姿态快速变化阶段长时间顶到 `ctrl=1`。后续优化重点不应继续盲目放大 LQR，而应检查腿端力矩分配、VMC 饱和策略和姿态/腿角耦合。

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --visualize --visualize-seconds 10
```
