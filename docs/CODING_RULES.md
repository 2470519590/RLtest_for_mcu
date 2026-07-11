# CODING_RULES.md

当前项目是 MuJoCo 实验脚本阶段。规则保持轻量，重点是物理语义清楚、脚本可运行、结果有记录。

## 基本规则

- 默认使用 `py310` conda 环境。
- 根目录 `.py` 必须是可直接运行入口。
- 可复用逻辑放到 `src/`。
- 不保留废弃实验脚本。
- 不提交 `__pycache__`、日志、缓存、checkpoint 或大体积输出。
- 不做无关重构。
- `src/robot_smoke/` 下单个 Python 文件目标不超过 2000 行。

## 物理语义规则

修改 MuJoCo 模型、控制脚本或 smoke 脚本时，先检查：

- joint / actuator 名称。
- action 到 actuator 的映射。
- 单位、正方向、坐标系。
- reset 初态。
- 接触、摩擦和限幅。
- 是否会影响后续 LQR 的状态或输入定义。

不能把物理语义变化伪装成脚本整理。

已写入 `docs/CONTROL_THEORY.md` 的“已确认物理语义”属于门禁内容。以下内容不得在没有重新实验和文档记录的情况下改写：

- `theta = atan2(r_x, -r_z)` 的世界系虚拟腿角定义。
- `theta > 0` 表示轮侧参考点在髋部前方。
- `F_l > 0` 表示初始增大虚拟腿长度。
- `tau_support = J_l^T * F_l` 的支撑力矩语义。
- actuator 的 `gear`、`ctrlrange` 和 `tau = gear * ctrl` 关系。
- 正常腿型分支的 base 机体系几何指标。
- LQR 工作点形式 `U = U0 - K * (X - X0)`。

如果修改上述任一语义，必须同时更新：

- `docs/CONTROL_THEORY.md`
- `docs/ERROR_CATALOG.md`，若修改来自错误修复
- `tasks/` 中对应实验记录

## LQR 前置门禁

在以下内容没有记录和验证前，不实现正式 LQR：

- 模型可加载。
- actuator 小输入方向可解释。
- reset 初态明确。
- 简单底层控制检查通过。
- observation/action 草案明确。

LQR 只应建立在干净的底层语义上。

## 验证规则

优先跑最小检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py
```

如果新增脚本，只检查相关脚本，不做大规模训练。

实验过程、验证命令和验证结果写入 `tasks/CONTROL_FRAMEWORK.md`。同一控制框架内的小实验不得新增零散任务文件；旧零散任务只放 `tasks/archive/`。

控制算法、公式推导、稳定物理量定义和当前有效物理语义写入 `docs/CONTROL_THEORY.md`。

`docs/CONTROL_THEORY.md` 禁止写时间线、修改流水账、smoke 输出和任务完成记录。

每完成一段调试，必须给出一条用户可直接本地运行的命令。涉及机器人运动判断时，必须给出可视化命令，让用户看画面后反馈。

## 以后服务器训练

当前不做服务器训练。等本地都准备好、只差正式训练时：

- 单独文件夹放训练需要的代码和配置。
- 服务器训练完成后产出压缩包。
- 本地下载压缩包到结果文件夹。
- 本地脚本转换为 ONNX。
- 用 ONNX 在 MuJoCo 里可视化测试小车。

不要提前加入完整工程化流程。
