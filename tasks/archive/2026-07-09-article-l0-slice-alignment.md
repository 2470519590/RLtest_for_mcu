# 任务：按文章修正 LQR/VMC 工作点语义

## 目标

停止把腿长当成要搜索出来的唯一平衡状态，改成文章式语义：

- 主平衡状态为 `X = [theta, dtheta, x, dx, phi, dphi]^T`。
- `x / dx` 默认来自左右驱动轮转角和轮速。
- 输入为 `U = [T, Tp]^T`。
- 腿长 `L` 不进入主平衡状态，只作为局部线性化冻结截面 `L0` 或后续增益调度变量。
- VMC 只负责把 `[F_l, Tp]` 通过 `J^T` 映射到腿关节力矩。

## 修改

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- 新增推荐 CLI 名称 `--equilibrium-l-slices`。
- 保留旧 `--equilibrium-l-refs` 作为兼容别名。
- equilibrium 输出从 `L_ref` 改为 `L0_slice`。
- 默认 diagnostic 截面改为 `0.35, 0.38, 0.395, 0.41`，这些值只表示待检查的局部截面。

更新 [docs/CONTROL_THEORY.md](E:/STM32_PROJ/RL_training/docs/CONTROL_THEORY.md)：

- 修正 LQR 状态定义中的 `x / dx`，明确使用 wheel source。
- 明确 `base_x / base_x_dot` 只作为 slip / 漂移诊断。
- 明确 `L0` 是冻结腿长截面，不是被搜索出的唯一真实腿长。
- 把腿长控制写为 `F_l = F_l0 + k_l * (L_cmd - L) - d_l * dL`。

更新 [tasks/2026-07-09-wheel-source-equilibrium-search.md](E:/STM32_PROJ/RL_training/tasks/2026-07-09-wheel-source-equilibrium-search.md)：

- 降级旧的 `L_ref≈0.38~0.395` 结论，避免误读成真实腿长范围。
- 验证命令改用 `--equilibrium-l-slices`。

## 文章对齐结论

下一步 equilibrium search 应按以下问题定义推进：

```text
给定 L0_slice：
    1. 用 upright-IK 构造正常分支初态；
    2. 腿长环只保持 L 接近 L0_slice；
    3. 轮端临时控制按 x_com - x_wheel 让轮子跑到重心下方；
    4. 找 dx≈0, dtheta≈0, dphi≈0, dL≈0, contact 稳定且输入不长期饱和的 X0/U0/F_l0/J0；
    5. 合格后再围绕该截面线性化求 K(L0)。
```

不能把 LQR-settle 末端点、非零漂移点或长期饱和点直接当作线性化工作点。

## 验证命令

语法检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

轻量静态截面检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-static-pose-check --diagnostics-only --equilibrium-l-slices 0.35 0.38 0.395 0.41 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 0.2 0.4
```

可视化检查：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --visualize --left-rod-length 0.35 --right-rod-length 0.35 --left-rod-theta 0 --right-rod-theta 0 --visualize-seconds 10
```

## 本轮验证结果

语法检查通过：

```text
py_compile run_smoke.py src/robot_smoke/__init__.py src/robot_smoke/model_smoke.py
```

轻量静态截面检查通过，终端输出已使用 `L0_slice` 语义：

```text
L0_slice=0.35, measured L0≈0.316547, contact_min≈21.0948
L0_slice=0.38, measured L0≈0.380018, contact_min≈20.6035
```

注意：`L0_slice=0.35` 下测得实际 `L0` 没有等于命令值，说明该截面需要继续检查 IK / 接触构造，不能把目标腿长字面值直接当作线性化工作点。
