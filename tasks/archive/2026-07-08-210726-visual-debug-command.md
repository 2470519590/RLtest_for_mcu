# 2026-07-08 21:07:26：每段调试提供本地可视化命令

## 任务

加入规则：每完成一段调试，必须给出用户可本地运行的命令；涉及机器人运动判断时，优先给可视化命令。

## 修改

- 更新 `AGENTS.md`。
- 更新 `docs/CODING_RULES.md`。
- 更新 `README.md`。
- 更新 `docs/CONTROL_THEORY.md`。
- 更新 `src/robot_smoke/model_smoke.py`，新增 `--visualize` 参数。

## 当前本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --visualize
```

## 验证

已运行：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --pd-hold-steps 100
```

结果：

- 语法检查通过。
- 非可视化 smoke 通过。
- `mujoco.viewer` 可 import。

## 说明

没有在自动流程中打开 viewer。可视化需要用户在本地桌面运行并根据画面反馈。
