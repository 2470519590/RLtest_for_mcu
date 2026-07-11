# 任务：修复 viewer 启动卡顿并确认速度状态来源

## 现象

执行：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --visualize --left-rod-length 0.35 --right-rod-length 0.35 --left-rod-theta 0 --right-rod-theta 0 --visualize-seconds 10
```

窗口长时间不出现，看起来像卡死。

## 根因

新对称几何后，运行时 Jacobian 改成了“闭链一致的小扰动 settle 差分”。如果每个 step 都重算，第一步控制会非常重，而 MuJoCo passive viewer 通常要到第一次 `viewer.sync()` 后窗口才明显刷新，所以表现成“长时间没有窗口”。

## 修改

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

1. viewer 进入后先执行一次 `viewer.sync()`，保证窗口先出现。
2. 为 `VmcSideMemory` 增加运行时 Jacobian 缓存：
   - `shape_jacobian`
   - `shape_jacobian_age`
3. 增加 `_runtime_leg_shape_jacobian(...)`
   - 默认每 `25` 个 step 刷新一次闭链一致 Jacobian
   - 中间 step 复用缓存

## 速度状态检查结论

当前 `LQR` 的 `x/x_rate` 有两种来源：

### 默认

```text
x_source = base
```

对应：

```text
x      = data.qpos[0]
x_rate = data.qvel[0]
```

也就是当前默认用的是车体基座在世界系 `x` 方向的位置和速度，不是轮速。

### 可选轮速来源

如果显式指定：

```text
--lqr-x-source wheel
```

则：

```text
x      = r * 0.5 * (q_left_wheel + q_right_wheel)
x_rate = r * 0.5 * (dq_left_wheel + dq_right_wheel)
```

当前 `r = 0.085 m`。

## 轮电机语义

当前轮电机仍是 MuJoCo `motor` actuator，也就是纯力矩控制：

```text
tau_wheel = gear * ctrl
gear = 12
ctrlrange = [-1, 1]
```

因此当前没有单独的轮速内环，只有：

- 轮端力矩命令
- 可选地在状态层使用轮速构造 `x_rate`

## 验证

### 1. 语法检查

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

### 2. 短运行耗时

```powershell
$sw = [System.Diagnostics.Stopwatch]::StartNew()
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --virtual-rod-steps 40 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1 > $null
$sw.Stop()
$sw.Elapsed.TotalSeconds
```

本次测得：

```text
elapsed_s ≈ 1.844
```

### 3. 速度来源最小检查

```powershell
@'
from src.robot_smoke import model_smoke as ms
mujoco = ms._load_mujoco()
model = mujoco.MjModel.from_xml_path(str(ms.DEFAULT_MODEL))
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)
base_state = ms._compute_lqr_state(mujoco, model, data, 0.0, 'base')
wheel_state = ms._compute_lqr_state(mujoco, model, data, 0.0, 'wheel')
print('base_x_rate=', base_state.x_rate)
print('wheel_x_rate=', wheel_state.x_rate)
print('left_wheel_speed=', ms._wheel_speeds(mujoco, model, data)[0])
print('wheel_radius=', ms._wheel_radius(mujoco, model))
'@ | & 'E:\miniconda\envs\py310\python.exe' -
```

## 下一步

viewer 启动问题先压住后，再继续：

1. 新对称几何下 equilibrium search
2. `F_l0 / L_ref` 重构
3. 判断 `x_rate` 应继续用 `base`，还是改成 `wheel`，或者做 slip-aware 组合量
