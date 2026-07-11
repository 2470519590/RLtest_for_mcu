# 任务：MuJoCo viewer 显示线性化工作点与实时状态

## 目标

在 `--visualize --virtual-rod-test --lqr-test` 画面里直接显示当前线性化工作点和实时状态，避免只看终端猜测 `LQR` 是否围绕错误工作点在控制。

## 本次修改

1. 在 `src/robot_smoke/model_smoke.py` 中为 `LqrDesignResult` 增加工作点样本透传：
   - 保存自动线性化时实际使用的 `StaticOperatingPointSample`。
2. 在 MuJoCo passive viewer 中增加 HUD：
   - `LQR op L(avg)`
   - `LQR op theta/x/phi`
   - `L target L/R`
   - `Actual L/R`
   - `Actual theta/x/phi`
   - `Delta(theta,phi)`
   - `Branch vio L/R`
   - `U0(T,Tp)`
   - `U(T,Tp)`
   - `VMC F_l cmd/raw`
   - `Theta scale L/R`
   - `LPF / deadzone`
3. 清理 `tasks/` 目录下此前残留的测试 `.png` 与 `.csv`，只保留文字记录。

## 结论

- 现在 viewer 可以直接对比“线性化工作点腿长/姿态”和“当前实际腿长/姿态”。
- 这一步是诊断层增强，不改变主控制律，也不改变机械 XML 语义。
- 如果后续看到 `Actual L` 长期偏离 `LQR op L(avg)`，且 `Branch vio = 0`，就更支持“当前问题是工作点/VMC/机构几何不一致”，而不是单纯死区或高频抖动。

## 验证

### 1. 语法检查

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -m py_compile run_smoke.py src\robot_smoke\__init__.py src\robot_smoke\model_smoke.py
```

### 2. overlay helper 无 GUI 检查

```powershell
@'
import numpy as np
from src.robot_smoke import model_smoke as ms
mujoco = ms._load_mujoco()
model = mujoco.MjModel.from_xml_path(str(ms.DEFAULT_MODEL))
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)
class FakeViewer:
    def __init__(self):
        self.payload = None
    def set_texts(self, texts):
        self.payload = texts
viewer = FakeViewer()
left_target, right_target = ms._build_virtual_rod_targets(mujoco, model, None, 0.0, None, None, None, None)
ms._set_viewer_status_overlay(
    viewer, mujoco, model, data, 0, 0.0, 0.0,
    left_target, right_target,
    np.zeros(6), np.zeros(2),
    None, {}, 20.0, 0.002,
)
print(viewer.payload[2].splitlines()[0])
print(viewer.payload[3].splitlines()[0])
'@ | & 'E:\miniconda\envs\py310\python.exe' -
```

预期输出包含：

```text
step
0 (0.000s)
```

### 3. 短 smoke

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --lqr-test --virtual-rod-steps 50 --zero-steps 1 --probe-steps 1 --pd-hold-steps 1 --history-sample-interval 10
```

结果：

- 运行通过，`PASS finite model/load/step smoke`
- `final_left_branch_violation = 0`
- `final_right_branch_violation = 0`

## 可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --virtual-rod-test --lqr-test --lqr-output-lowpass-hz 20 --visualize --visualize-seconds 10
```

## 下一步

- 先用 HUD 确认 `LQR op L(avg)` 与 `Actual L/R` 的长期偏差。
- 如果确认工作点长期错配，再决定是先重做 equilibrium / 线性化工作点，还是进入机械对称性与 VMC 几何语义调整。
