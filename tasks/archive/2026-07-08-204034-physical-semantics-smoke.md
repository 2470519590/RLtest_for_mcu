# 2026-07-08 20:40:34：物理语义与底层 actuator smoke 骨架

## 任务

为当前 MuJoCo 轮腿模型建立最小物理语义和底层 actuator smoke 检查，确保在推进 LQR 前先有可重复的底层验证入口。

## 背景

项目目标是先完成物理语义和底层控制，不把底层物理问题带到上层 LQR。

当前仓库在本任务前只有 MuJoCo XML 和文档，没有 Python smoke 入口。

## 修改

- 新增 `run_smoke.py`：根目录可直接运行的 smoke 入口。
- 新增 `src/robot_smoke/__init__.py`：smoke 包入口。
- 新增 `src/robot_smoke/model_smoke.py`：模型加载、joint/actuator 摘要、零输入 step、小幅正负 actuator 探测。
- 更新 `README.md`：记录 smoke 命令和边界。
- 更新 `docs/INDEX.md`：登记 `run_smoke.py` 和 `src/robot_smoke/`。
- 更新 `docs/CONTROL_THEORY.md`：记录模型摘要和本次 smoke 结果。

## 验证命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py
```

## 验证结果

- Python 环境：`E:\miniconda\envs\py310\python.exe`
- Python 版本：3.10.20
- MuJoCo 版本：3.8.0
- 模型加载：通过
- 模型规模：`nq=19`, `nv=18`, `nu=6`, `njnt=13`
- timestep：0.001
- 零输入 step：200 step，状态保持 finite
- actuator 探测：6 个 actuator 的 `+/-0.05` 输入在对应 joint dof 上均得到相反速度符号

## 重要观察

- 默认 `python` 是 Python 3.13.12，不是项目要求的 `py310`。
- 当前进程如果带有 `MUJOCO_GL=osmesa`，Windows 下 `import mujoco` 会失败；本地 smoke 命令临时清除了该环境变量。
- 零输入 200 step 后 `qvel` norm 为 27.286，说明自由落体/接触等动力学仍需要进一步解释，不能把该结果当成控制稳定。

## 未完成

- 尚未定义正式 observation/action 合约。
- 尚未定义 reset 目标姿态和稳定初态。
- 尚未实现低层闭环控制器。
- 尚未做接触、摩擦、关节方向和真实机器人语义的人工确认。
- 尚未具备推进 LQR 正式实现的前置条件。

## 下一步

1. 明确 reset 初态、坐标系、正方向和单位。
2. 建立最小低层控制器检查，例如固定姿态或简单 PD hold。
3. 将 observation/action 合约写入 `docs/CONTROL_THEORY.md`。
