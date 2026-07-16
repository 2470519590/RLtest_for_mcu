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

## 当前测试接口

所有物理行为以 MuJoCo viewer 人工观察为准。当前 `--lqr-true-equilibrium` 已恢复为固定 `L0=0.35 m` 的 locked 平衡入口，不自动重算多腿长工作点。

启动平衡测试：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --visualize --visualize-seconds 10
```

直线速度测试：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile low --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile medium --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --speed-profile high --visualize --visualize-seconds 10
```

原地旋转测试：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed low --visualize --visualize-seconds 6
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed medium --visualize --visualize-seconds 6
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed high --visualize --visualize-seconds 6
```

Enhanced leg-length turn test:
```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-length-sine-test --visualize --visualize-seconds 10 --roll-length-plot
```

This runs high-rate in-place turning while the leg-length reference follows a 1.5 s sine over `minimum_leg_length..maximum_leg_length`.

转向档位为 `low=pi/2`、`medium=pi`、`high=10 rad/s`。`--turn left|right --turn-speed <档位>` 保留为连续手动转向接口。

旋转前进测试：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-drive-test low --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-drive-test high --visualize --visualize-seconds 10
```

`--turn-drive-test low` 固定为低速前进加 low 旋转；`--turn-drive-test high` 固定为高速前进加 medium 旋转。`--turn-speed` 只用于单独转向测试。

平衡冲击测试：

```powershell
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --impact small --visualize --visualize-seconds 10
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --lqr-true-equilibrium --impact medium --visualize --visualize-seconds 10
```

`--turn-pd-plot` 保留为按需诊断接口；正常转向和其他测试默认不启动绘图。仅在检查 yaw PD 与双腿同步 PD 的输入、P/D 分量时追加：

```powershell
--turn-pd-plot
```

文章 2.2 双腿长度与 roll 控制的诊断图：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-drive-test low --visualize-seconds 10 --roll-length-plot
```

图输出到 `output\HHMMSS.png`，包含左右 `L_d/L/dL`、roll、几何高度差、直接横滚补偿 `F_roll`、左右 `F_base +/- F_roll` 和最终沿腿推力。

单轮三角坡 roll 测试：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --roll-test --roll-length-plot --visualize --visualize-seconds 10
```

该测试按 `config/smoke.yaml` 的 `roll_reference` 与 `roll_force_kp` 运行，并以中速前进；左轮依次通过 `x=2.5 m` 的加宽三角坡和 `x=4.5 m` 的 `0.18 m` 高梯形坡，随后右轮通过 `x=6.7 m` 的同规格梯形坡。坡体只在 `--roll-test` 时显示并参与碰撞。

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
