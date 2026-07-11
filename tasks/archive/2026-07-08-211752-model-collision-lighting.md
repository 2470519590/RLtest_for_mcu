# 2026-07-08 21:17:52：补中间关节/轮腿碰撞与柔化光照

## 任务

根据可视化反馈修 MuJoCo 模型：

- 轮腿和中间自由/被动关节附近缺少碰撞。
- 光照不够平滑。

## 修改

- 更新 `assets/biped_wheel_leg.xml`。
- 新增 `hidden_collision` 默认类。
- 给 front/rear elbow、hub、carrier 增加隐藏碰撞几何。
- 隐藏碰撞只与地面碰撞，不参与机器人内部自碰撞。
- 调整 headlight 参数。
- 增加侧向 fill light。
- 更新 `docs/CONTROL_THEORY.md`。

## 重要过程

第一次给 carrier/hub 添加普通隐藏碰撞后，轮电机方向 probe 被内部接触干扰，出现 `not-confirmed`。

修正方式：

- 新增隐藏碰撞使用 `contype="2"`、`conaffinity="0"`。
- 地面使用 `conaffinity="3"`。
- 这样新增碰撞可以和地面接触，但不和机器人内部几何互相碰撞。

## 验证命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' -c "import mujoco; m=mujoco.MjModel.from_xml_path('assets/biped_wheel_leg.xml'); print(m.ngeom, m.nlight, m.njnt, m.nu)"
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --pd-hold-steps 100
```

## 验证结果

- XML 加载通过。
- `ngeom=48`
- `nlight=2`
- `njnt=13`
- `nu=6`
- 6 个 actuator 的正负小输入方向检查通过。
- `run_smoke.py --pd-hold-steps 100` 通过。

## 本地可视化命令

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --visualize
```

## 需要用户反馈

- 中间关节和 wheel carrier 是否还穿地或穿模。
- 轮胎接触是否合理。
- 光照是否足够平滑。
- 是否有明显内部碰撞、抖动或卡死。
