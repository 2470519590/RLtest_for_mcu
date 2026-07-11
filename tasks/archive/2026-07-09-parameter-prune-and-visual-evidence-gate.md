# 任务：清理历史控制参数并修正可视化证据门禁

## 目标

暂停继续调 LQR 权重和旧分层 PD 参数，先处理两个问题：

- 代码入口里旧 `wheel_balance_*` / parking / direct-x / world-theta 参数过多，容易把当前文章式 LQR+VMC 路径重新污染成历史分层 PD。
- 旧可视化命令中“倒下后腿长回到 0.35 m”不能作为 upright 支撑或 true equilibrium 成功证据。

## 修改

更新 [src/robot_smoke/model_smoke.py](E:/STM32_PROJ/RL_training/src/robot_smoke/model_smoke.py)：

- 删除隐藏的旧轮端分层 PD 参数入口和透传：
  - `--balance-control`
  - `--wheel-balance-pitch-kp/kd`
  - `--wheel-balance-x-kp/kd`
  - `--wheel-balance-direct-x-kp/kd`
  - `--wheel-balance-wheel-speed-kd`
  - `--wheel-balance-parking-*`
  - `--wheel-balance-world-theta-kp/kd`
  - `--wheel-balance-max-pitch-target`
- 将当前仍实际表示 LQR 输出限幅/限速的接口重命名为文章语义：
  - `--lqr-t-limit`
  - `--lqr-tp-limit`
  - `--lqr-output-rate-limit`
- 精简 equilibrium search 的轮端临时控制扫描，只保留：
  - `wheel_com_kp`
  - `wheel_damping`
  - `Tp_bias`
  - 弱 `theta/pitch` damping
- 删除 equilibrium search 中旧的 wheel pitch / world theta / base dx 扫描项。

更新 [docs/CONTROL_THEORY.md](E:/STM32_PROJ/RL_training/docs/CONTROL_THEORY.md)：

- 记录物理判据：倾倒或接触切换后腿长回到 `L_cmd` 附近，不构成 true equilibrium 证据。

更新 [docs/ERROR_CATALOG.md](E:/STM32_PROJ/RL_training/docs/ERROR_CATALOG.md)：

- 记录错误经验：旧 `--virtual-rod-test --visualize --left/right-rod-length 0.35` 画面不能证明腿长控制成功。

## 验证

语法检查通过：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

五连杆运动学门禁通过：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --fivebar-kinematics-check --diagnostics-only --fivebar-kinematics-l-slices 0.35 0.38 0.395 --fivebar-kinematics-theta-refs 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

关键输出：

```text
max_abs_length_error: 2.32473e-05
max_abs_theta_error: 1.26795e-05
max_branch_violation: 0
```

短 equilibrium search 参数链检查通过，但结果不合格：

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --equilibrium-search --diagnostics-only --equilibrium-init-modes upright-ik --equilibrium-l-slices 0.35 --equilibrium-theta-refs 0 --equilibrium-fl0-scales 1.0 --equilibrium-steps 80 --equilibrium-eval-steps 20 --equilibrium-wheel-com-kps 80 --equilibrium-wheel-dampings 0.12 --equilibrium-tp-biases 0 --equilibrium-init-drop-steps 0 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1
```

关键输出：

```text
qualified=False
L_mean=0.296132
dL_RMS=0.606623
joint_sat_ratio=0.3125
contact_force_min=0
```

该结果只证明参数链可运行，并再次说明当前还没有 true equilibrium，不能进入 LQR 线性化。

## 结论

- 历史旧分层 PD 参数已从主入口和 equilibrium search 扫描中删除。
- 当前五连杆解析 IK 仍通过门禁。
- 旧可视化中的腿长恢复现象不可信，不能作为后续控制结论。
- 下一步仍应继续做 true contact equilibrium：给定 `L0_slice`，用解析 IK 初始化，轮端只负责追 COM 下方，腿长环只保高度，然后找 `dX≈0/dL≈0/contact` 稳定且输入不长期饱和的工作点。
