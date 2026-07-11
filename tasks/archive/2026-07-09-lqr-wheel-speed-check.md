# 2026-07-09：LQR 轮速检查

## 任务

用户反馈当前仍然无法平衡，并询问腿部轮子是否有速度。检查 wheel joint 速度是否真实存在，并把轮速写入 smoke 输出和历史曲线。

## 修改

- 更新 `src/robot_smoke/model_smoke.py`。
- 新增 wheel joint 速度读取：
  - `left_wheel_joint`
  - `right_wheel_joint`
- `virtual_rod_test` 输出新增：
  - `max_abs_left_wheel_speed_rad_s`
  - `max_abs_right_wheel_speed_rad_s`
  - `final_left_wheel_speed_rad_s`
  - `final_right_wheel_speed_rad_s`
- 历史 CSV 新增：
  - `left_wheel_speed`
  - `right_wheel_speed`
- 历史 PNG 新增轮速曲线子图。
- 更新 `docs/CONTROL_THEORY.md` 中的轮速观测定义。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

轮速检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --virtual-rod-steps 800 --constraint-steps 800 --check-constraints --history-csv tasks\lqr_wheel_speed_history.csv --history-plot tasks\lqr_wheel_speed_history.png --history-sample-interval 5 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

## 验证结果

- 语法检查通过。
- wheel actuator 小输入探测显示 wheel joint 正负输入速度方向相反：
  - `left_wheel_motor`: `qvel(+)=0.213119`, `qvel(-)=-0.213119`
  - `right_wheel_motor`: `qvel(+)=0.213119`, `qvel(-)=-0.213119`
- 默认 LQR 800 step：
  - `max_abs_left_wheel_speed_rad_s = 16.2387`
  - `max_abs_right_wheel_speed_rad_s = 16.2387`
  - `final_left_wheel_speed_rad_s = 5.8726`
  - `final_right_wheel_speed_rad_s = 5.8726`
  - wheel motor ctrl 最大约 `0.034976`，没有饱和。
  - 腿部 VMC 仍然饱和，至少一个 actuator 饱和 `230` step。
  - 左右 `max_branch_violation = 0`。

## 结论

轮子有速度，且左右轮速度一致。当前无法平衡不是因为 wheel joint 没有转动。

当前更像是：

- LQR 轮端力矩尺度仍然很保守，wheel ctrl 很小。
- 车身姿态快速下落时，腿部 VMC 已经长时间饱和。
- 轮速起来时，车体已经明显倾倒，平衡补偿太晚。

下一步应优先检查：

- 是否需要提高轮端 `T` 的作用而不是继续加大腿端 `Tp`。
- 是否需要把 LQR 中的 pitch / pitch_rate 增益拆开单独调。
- 是否需要在 VMC 中降低腿部饱和阶段的姿态耦合，避免腿端已经满幅时继续把控制压力堆到腿上。

## 输出文件

- `tasks/lqr_wheel_speed_history.csv`
- `tasks/lqr_wheel_speed_history.png`

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-test --visualize --visualize-seconds 10
```
