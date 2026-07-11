# 2026-07-09：控制输出抖动溯源

## 任务

新增控制输出逐步溯源诊断，用来判断 MuJoCo viewer 中六个电机输出跳变来自控制软件输出、VMC 映射、限幅裁剪，还是可能只是 viewer 显示现象。

## 修改

- 新增 `--trace-control-output`。
- 新增 `--trace-control-start-step`，默认 `0`。
- 新增 `--trace-control-max-steps`，默认 `200`；设为 `0` 表示不限制打印步数。
- 新增 `--trace-control-mode`：
  - `events`：默认，只打印超过阈值的事件步和最后摘要。
  - `summary`：只打印最后摘要，不打印逐步 trace。
  - `all`：打印所有 trace 步，用于极短 rollout。
- 新增 `--trace-control-event-delta`，默认 `0.01`，用于 `events` 模式过滤小跳变。
- 每个 trace 步打印：
  - `T`, `Tp`
  - 左右腿 `F_l` 和 `F_l_raw`
  - 左右腿 `tau_joint_before_clip`
  - 左右腿 `tau_joint_after_clip`
  - 六个 actuator 的 `ctrl`
  - `clip_flags`
  - `source_hint`

`source_hint` 当前只作为诊断提示：

- `LQR/middle`：`T` 或 `Tp` 相邻步变化明显。
- `VMC`：`F_l` 或关节原始力矩相邻步变化明显。
- `clip`：腿长力、角向通道缩放、关节力矩或 actuator 控制量发生限幅。
- `viewer_possible`：控制输出几乎不变，此时若 viewer 仍显示跳变，才优先怀疑显示层。

## 验证

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

短步 VMC trace：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --trace-control-output --trace-control-max-steps 5 --virtual-rod-steps 8 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

短步 LQR trace：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --trace-control-output --trace-control-max-steps 5 --virtual-rod-steps 8 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 初步结果

- VMC 短步 trace 中，`T=0`、`Tp=0`，但 `F_l`、`tau_joint_before_clip` 和 `ctrl` 逐步变化，`source_hint=VMC`。
- LQR 短步 trace 中，从第 1 步开始 `T` 以 rate limit 变化，轮电机 `ctrl` 同步变化，`source_hint=LQR/middle+VMC`。
- 两组短步验证中 `clip_flags` 均为 False，说明短步样例里的跳变不是由限幅裁剪导致。
- 用户反馈实际长测中：
  - 初期 `max_ctrl_delta` 约 `0.0002`。
  - 中期曾出现 `0.02`，且 `source_hint=LQR/middle`。
  - 也有一段 `1e-4` 量级。
  - 后期常见 `0.005`，偶发 `0.009`，且出现 `source_hint=VMC`。
  - 没有出现 `source_hint=clip`。

这说明当前可观测抖动不是限幅裁剪主导；更像是中层先激发低频运动，后期 VMC 受运动状态影响继续调节腿部输出。

## 下一步

推荐先用摘要模式看全局：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --trace-control-output --trace-control-mode summary --trace-control-max-steps 0
```

如果要抓中期 `0.02` 这种事件：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --trace-control-output --trace-control-mode events --trace-control-event-delta 0.01 --trace-control-max-steps 0
```

## 中期日志补充判断

用户提供 `C:\Users\24705\Desktop\log.txt` 后，检查 `step=358~365` 和 `step=441~445`：

- `T` 每步按 rate limit 变化 `0.12 N*m`，对应轮电机 `ctrl` 变化约 `0.005`。
- `Tp=0`，没有角向中层输出。
- `F_l` 在最大跳变附近几乎不变，例如 `step=358~360` 约 `64.15 -> 64.07 -> 64.07 N`。
- 但腿部 `tau_joint_before_clip` 和腿部 actuator `ctrl` 大幅跳变，例如 `step=359`：
  - `left_front_motor: 0.304332 -> 0.422273`
  - `left_rear_motor: -0.101151 -> 0.0302191`
- `clip_flags` 仍全为 False。

因此 `step=359` 的 `max_ctrl_delta=0.13137` 不是轮端 `T` 直接造成，也不是 `F_l` 腿长力或限幅造成，而是 VMC 角向通道 `F_theta` / `theta` 相关项在跳。已新增 `F_theta_left/right`、`dF_theta_left/right` 和 `vmc_jump_channel` 打印，后续可直接确认。
