# MuJoCo 机器人实验脚本项目

本仓库当前处于实验脚本阶段，目标是先把 MuJoCo 模型的物理语义和简单底层控制跑通。

当前路线：

1. 明确 joint、actuator、单位、方向、reset、接触和限幅。
2. 跑通本地 smoke 检查。
3. 做简单底层控制，例如小输入探测、零输入检查、后续 PD hold。
4. 底层语义确认后，再考虑 LQR 专家。

底层物理问题不能推给 LQR、行为克隆或 PPO。

## 当前文件

- `assets/biped_wheel_leg.xml`：MuJoCo 轮腿模型。
- `run_smoke.py`：本地 smoke 入口。
- `src/robot_smoke/`：模型加载和 actuator 探测代码。
- `docs/CONTROL_THEORY.md`：控制算法、公式推导和稳定物理量定义。
- `tasks/`：带时间的任务记录。

## 环境

默认使用 `py310` conda 环境。

当前 smoke 命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py
```

当前可视化命令：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --visualize
```

如果只是实验脚本，不需要引入完整工程化流程。

## 以后服务器训练怎么接

现在不做服务器训练。等本地物理语义、底层控制和训练代码都准备好，只差正式训练时：

1. 把训练需要的代码和配置放到单独文件夹。
2. 上传服务器训练。
3. 服务器训练完成后应产出一个压缩包。
4. 把压缩包下载到本地结果文件夹。
5. 用本地脚本把结果转换成 ONNX。
6. 用 ONNX 模型在本地 MuJoCo 可视化测试小车。

## 脚本规则

- 根目录 `.py` 必须是可直接运行入口。
- 可复用逻辑放到 `src/`。
- 不保留无用临时脚本。
- 不提交缓存、日志、checkpoint 或大体积输出。
- 实验过程和验证结果写到 `tasks/`。
- 控制算法、公式推导和稳定物理量定义写到 `docs/CONTROL_THEORY.md`。
