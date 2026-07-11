# 任务：五连杆改为前后对称并重算局部几何解算

## 目标

把轮腿五连杆从前后不对称几何改为前后镜像、下肢长于上肢的几何，并同步去掉控制代码中的旧手填几何系数，让 IK / VMC 按当前 MuJoCo 几何实时估计局部 Jacobian。

## 修改

### 1. XML 几何

更新 [assets/biped_wheel_leg.xml](E:/STM32_PROJ/RL_training/assets/biped_wheel_leg.xml)：

- 保持上方驱动关节在 `x = +/-0.09 m`
- 前上肢改为局部向量 `[+0.07, 0, -0.20]`
- 后上肢改为局部向量 `[-0.07, 0, -0.20]`
- 前下肢改为局部向量 `[-0.16, 0, -0.21]`
- 后下肢改为局部向量 `[+0.16, 0, -0.21]`
- 因此：
  - 上肢长度约 `0.212 m`
  - 下肢长度约 `0.264 m`
- reset 下 carrier 回到髋部中点正下方，`theta_reset ≈ 0`

### 2. IK / VMC 局部几何

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- 删除旧固定系数：
  - `length_per_diff_rad = 0.143`
  - `theta_per_negative_sum_rad = 0.424`
- `J = d[l, theta] / d[q_front, q_rear]` 改为：
  - 锁机身
  - 对前/后驱动目标角做小扰动
  - 短步 settle
  - 再做数值差分
- 线性 IK 基候选改为：

```text
dq = pinv(J_current) * [l_target - l, wrap(theta_target - theta)]^T
```

## 物理结论

### 已确认

1. 新几何 reset 下左右对称：
   - `l_reset ≈ 0.41 m`
   - `theta_reset = 0`
2. 闭链仍成立：
   - `max_left_error_m ≈ 6.67e-7`
   - `max_right_error_m ≈ 6.67e-7`
3. branch 语义正常：
   - `max_branch_violation = 0`
   - `final_branch_violation = 0`
4. 当前工作点附近的局部 Jacobian 量级约为：

```text
J ≈ [
  [ +0.055, -0.055],
  [ -0.262, -0.262],
]
```

### 仍未解决

当前控制参数还沿用旧工作点语义，因此新几何下默认 `--virtual-rod-test` 会把腿长推到约 `0.43 ~ 0.44 m`，而不是收敛到 `0.35 m`。

这说明下一步主要问题已经从“几何前后不对称”切换为：

- `F_l0`
- `L_ref`
- equilibrium / static operating point
- 新几何下的线性化工作点

不能继续沿用旧几何时整定出来的腿长支撑参数。

## 验证命令

### 1. 语法检查

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

### 2. reset 几何与对称性

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --print-virtual-leg --scan-virtual-rod --scan-virtual-rod-sample 0.15 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

### 3. 闭链与 branch

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --lock-base --check-constraints --constraint-steps 300 --virtual-rod-steps 300 --left-rod-length 0.35 --right-rod-length 0.35 --left-rod-theta 0 --right-rod-theta 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

### 4. 新几何下默认虚拟腿控制现状

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --virtual-rod-steps 120 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1 --print-static-operating-point
```

## 验证结果

- 语法检查通过
- 对称 reset 成立：`l_reset ≈ 0.41 m`，`theta_reset = 0`
- 闭链误差和 branch violation 都正常
- 当前参数下 `l_target = 0.35 m` 仍未达到，最终腿长约 `0.431 ~ 0.444 m`

## 可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --visualize --left-rod-length 0.35 --right-rod-length 0.35 --left-rod-theta 0 --right-rod-theta 0 --visualize-seconds 10
```

## 下一步

基于新对称几何重新做：

1. `F_l0 / L_ref` 工作点扫描
2. equilibrium search
3. 新几何下的 `X0 / U0 / J0`
4. 再做新的 LQR 线性化
