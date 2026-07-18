# 控制框架

## 当前目标

维护以 `config/smoke.yaml` 为准的本地 MuJoCo 控制链：默认腿长 `0.24 m`、启用腿长调度表、使用模拟 wheel odometry 的 LQR `x/dx`。继续保持解析五连杆 VMC、腿长控制、轮端转向控制；最终运动与稳定性由 viewer 人工确认。

## 当前有效控制

- 平衡：`U=U0-K(X-X0)`，状态为 `[theta,dtheta,x,dx,phi,dphi]`。
- 支撑：`F_l=F_l0+k_l(L0-L)-d_l*dL`。
- 五连杆：`tau_joint=J(q)^T[F_l,Tp_side]^T`。
- 转向：轮端差动 `tau_left=T/2-tau_turn`、`tau_right=T/2+tau_turn`。
- 协调：`e_sync=theta_right-theta_left`，左右腿施加反相 `Tp_sync`。
- 转向档位：`low=pi/2`、`medium=pi`、`high=10 rad/s`。
- 旋转前进：`low` 固定为低速前进加 low 旋转；`high` 固定为高速前进加 medium 旋转。
- 双腿长度采用 PID + 重力前馈；`L_d` 只表示几何高度参考。横滚动态补偿为 `F_roll=K_gamma(gamma_d-gamma)`，以反号直接叠加到左右沿腿推力。斜坡地面倾角估计当前未启用。
- 当前默认腿长目标来自 `config/smoke.yaml`，最近为 `0.24 m`；`Ki,L/Kd,L/F_l0` 等底层参数也以 YAML 为准，不在入口里做隐藏覆盖。
- 控制结构按文章框图组合：整体反馈输出 `T/Tp`，yaw PD 输出轮差，腿同步 PD 输出反向 `Tp_sync`，长度 PID 输出 `F_base`，roll 几何通道输出左右 `L_d`，roll P 通道输出 `F_roll`，两路在 VMC 前汇合。

## 入口

```powershell
Remove-Item Env:MUJOCO_GL -ErrorAction SilentlyContinue
& 'E:\miniconda\envs\py310\python.exe' run_smoke.py --turn-test --turn-speed high --visualize --visualize-seconds 6 --turn-pd-plot
```

`--turn-speed` 可取 `low`、`medium`、`high`。转向诊断图只输出到 `output\HHMMSS.png`，用于查看原始量、滤波量以及 P/D 力矩分量。
`--turn-length-sine-test` 为增强腿长旋转测试：默认 high 原地旋转，腿长参考在 `minimum_leg_length..maximum_leg_length` 内按 `1.5 s` 周期正弦变化；自转和腿长参考同时开始、同时结束；未显式给 `--visualize-seconds` 时默认运行 10 秒。

`--roll-length-plot` 输出文章 2.2 长度/roll 控制图，记录左右 `L_d/L/dL`、roll、几何项、`F_roll`、`F_base +/- F_roll` 与最终 `F_l`；只作控制通路诊断。

## 最近整理

- 删除三角斜坡模型、`ramp_test` 和 roll 支撑补偿路径。
- 删除通用 history CSV、LQR 图、电机图和 control trace 输出；保留转向 PD 图。
- 删除任务目录下旧 CSV/PNG 实验产物。

## 验证

- 2026-07-12：`py_compile` 通过。
- 2026-07-12：`--turn-test --turn-speed medium --visualize-seconds 0.02 --turn-pd-plot` 完成有限步运行；仅证明入口和接口可执行，不构成平衡结论。
- 2026-07-12：`--lqr-true-equilibrium --virtual-rod-steps 20 --roll-length-plot` 生成 `output\HHMMSS.png`；仅证明文章 2.2 观测与绘图链路可执行，不构成 roll 补偿有效结论。
- 2026-07-13：公式 (13)-(15) 的 `L_d` 解算已接入；`--turn-drive-test low --visualize-seconds 2 --roll-length-plot` 显示正 roll 时左 `L_d` 下调、右 `L_d` 上调。该短 rollout 未满足既有动态行为判据，不作为稳定结论。
- 2026-07-13：动态 `L_d` 曾使 VMC 每步重新执行候选分支 IK settle，导致 1-3 秒稳定卡顿；安全分支姿态改为首次计算后复用。同一 2 秒绘图 rollout 的墙钟时间由约 22 秒降至约 4.4 秒，后者包含初始化和绘图。
- 2026-07-13：腿长 `Kd/Ki/积分限幅/前馈` 支持显式命令行覆盖；未显式传入时保持原 locked 基线。短 smoke 已确认四项覆盖值原样进入 VMC。
## 横滚通道与统一可视化

- 按本地论文 2.2.2 修正横滚通道：`L_d` 仅保留几何参考；`F_roll=K_gamma(gamma_d-gamma)` 以反号叠加到左右沿腿推力后进入 VMC。
- 绘图和 viewer 改为观察同一个 `virtual_rod` rollout；`--visualize-seconds` 同时决定该 rollout 的步数。
- 验证：`--turn-drive-test low --roll-length-plot --visualize-seconds 2` 已生成包含 `F_roll`、`F_base+/-F_roll` 与左右最终 `F_l` 的图。该短测试出现关节饱和和不合格动态，不构成转向稳定结论。
- 可视化验证：同一命令加 `--visualize` 已能打开 viewer 并完成同一 rollout；物理表现仍需人工观察。

## 单轮坡横滚测试

- 默认 `roll_reference=0`；`roll_force_kp` 是论文横滚比例增益 `K_gamma`。
- `--roll-test` 固定 locked equilibrium 和 medium 前进；横滚参考与增益直接读取 YAML，不再在入口覆盖。赛道仅保留 `x=2.5 m` 的左轮三角坡，坡体只在该入口启用。
- 接触检查：左坡只与 `left_wheel_geom` 接触，右坡只与 `right_wheel_geom` 接触；普通模式下坡体接触数为 0。
- 10 秒可视化与绘图已运行。曲线显示两次单侧腿长/横滚响应；是否真实通过扰动与恢复由 viewer 人工判断，当前暂停调参。

## 坡体尺寸扩展

- 单轮三角坡扩展为 `0.84 m` 长、`0.24 m` 宽、`0.06 m` 高，仍各自只覆盖对应轮轨。
- 三座单轮坡体横向全宽均为 `0.60 m`，左右坡中心仍为 `y=+/-0.255 m`，轮间保持横向间隙，避免横滚时轮子从坡侧跌落。紧随小坡的两座梯形高度场坡为 `1.70 m` 长、`0.60 m` 宽、`0.18 m` 顶高；左、右坡中心分别位于 `x=4.5 m`、`x=6.7 m`，按左后右顺序通过。
- 接触检查：小坡和大坡均各自只接触对应左/右轮。10 秒 viewer 已运行，物理表现等待人工观察。

## 腿长调度范围

- 命令腿长范围改为 `0.16..0.30 m`；`minimum_leg_length` 与 `maximum_leg_length` 同时限制普通腿高目标和 roll 几何后的左右 `L_d`。
- `config/length_schedule.yaml` 已按当前五连杆几何、`Q=[1,1,50,100,5000,10]`、`R=[1,0.25]` 重新生成 `0.16:0.01:0.30 m` 的 runtime 调度表；`B(L)` 来自各腿长的 runtime finite-difference，不使用固定 measured B。
- 入口 smoke：`--lqr-true-equilibrium --visualize-seconds 10` 可执行并加载新表；是否真实稳定仍以 viewer 人工观察为准。
- 默认测试腿长改为 `0.24 m`，启动 ramp 默认关闭；调度表模式下 LQR 初始姿态由目标腿长 IK 生成，不再先显示 0.35 m 腿型再慢慢收缩。

## Roll-test 场地恢复

- `--roll-test` 现在使用专用临时 MuJoCo XML 注入坡场，不污染普通测试模型。
- 坡场包含：左轮 `x=2.5 m` 小三角坡、左轮 `x=4.5 m` 大梯形坡、右轮 `x=6.7 m` 大梯形坡；坡体按左右轮轨横向放置。
- 该入口作为地形/roll 诊断测试，不使用平地最终姿态判据决定退出码；是否真实恢复和通过坡场仍由 viewer 人工观察。
- 当前坡场版本：左轮 `x=2.5 m` 低三角坡、右轮 `x=3.7 m` 低三角坡；两轮共同通过 `x=5.1 m` 全宽飞坡，飞坡长 `1.20 m`、宽 `1.20 m`、高 `0.12 m`、斜坡段 `0.32 m`。
# Flight test / airborne detection

- Added `--flight-test` as a diagnostic entry: high-speed forward motion over a temporary full-width launch ramp at `x=3.8 m`, `length=1.80 m`, `width=1.40 m`, `height=0.32 m`.
- Added paper section 3 airborne detection using wheel contact normal force as `F_N`; the detector is armed only after a valid support force is first observed, then declares airborne when both wheels are below `20 N`.
- In airborne mode, the LQR middle layer gates out wheel torque `T` and keeps only `Tp` feedback from `theta,dtheta`, matching the paper instruction to keep only `K21,K22`.
- Verification: `py_compile` passed; the temporary MuJoCo XML loaded; `run_smoke.py --flight-test --visualize-seconds 10 --lqr-debug-plot` ran. This is a diagnostic run only; flight posture and landing must be judged in viewer.
- Deprecated landing stop path: `--flight-test` must not use the old direct-zero `dx_ref=0` velocity-only parking logic after recontact. That path caused the robot to keep correcting speed without a proper landing recovery mode and could produce repeated forward/backward swings. The flight diagnostic plot remains the LQR debug plot with `x` and `dx`, but landing recovery must be treated as a separate contact-switching problem.
- Added `--jump-test` as an in-place jump diagnostic: initialize at the nominal `0.24 m` work point, crouch to `minimum_leg_length`, then extend both leg direct IK/position targets to `maximum_leg_length`. The jump does not actively retract in the air; only after recontact does it return to the nominal work point, matching the flight-test landing logic. The extension stroke bypasses the VMC leg-length PID/force channel, keeps length force rate limiting disabled, and keeps the paper section 3 airborne detector active.
- Verification: `run_smoke.py --jump-test --visualize-seconds 10 --lqr-debug-plot` produced `output\144308.png`. The reference step executed and final leg length reached the maximum-length region, but the current PID/VMC gains did not produce takeoff in headless data (`airborne_steps=0`, min wheel normal force about `31.8 N`). Viewer judgment remains required.
- Revised jump actuation: before 1 s the test keeps normal VMC support at the nominal work point; after 1 s it switches the leg channel to direct IK/position servo for crouch/extension/retraction, then returns to normal VMC after recontact.
- Corrected jump order: `0..1 s` holds nominal `0.24 m` with VMC, `1.0..1.2 s` crouches to `0.16 m` with VMC, then direct IK/position extension goes to `0.30 m`; recontact switches the target back to `0.24 m` using the same landing detector as flight-test. Verification produced `output\153301.png`, with first airborne detection at about `1.255 s`.
- Added `--forward-jump-test low|medium|high`: the selected forward speed profile runs first, and the jump sequence is triggered only during the constant-speed segment when both virtual-leg world angles satisfy `abs(theta) < 3 deg`. After triggering, it reuses the existing jump flow: VMC crouch, direct extension, airborne LQR gating, and return to the nominal work point after recontact. This is a viewer diagnostic entry; physical jump quality is still judged manually in MuJoCo.
- Jump viewer stutter fix: the first direct-extension step previously computed branch-aware IK during the visible rollout, which could block the viewer at jump start. Jump extension IK is now prewarmed before the viewer opens. Passive viewer `sync()` is also decimated to about 60 Hz, while the simulation and controller still step at the original timestep.

## Jump landing mode-order fix

- Jump direct-extension IK prewarm now first uses the existing analytic five-bar IK. The old branch-aware numerical path remains only as fallback when the analytic target is unreachable.
- Landing detection now uses explicit phases: `AIRBORNE -> TOUCHDOWN_CANDIDATE -> LANDING_ABSORB -> BALANCE_RECOVERY -> BRAKE`.
- A first single-step contact no longer immediately becomes final landing. Touchdown must keep both wheel normal forces above the airborne threshold for a short debounce window.
- If the robot bounces and both wheels lose contact again, the landing latch is cleared and the phase returns to `AIRBORNE`.
- During `TOUCHDOWN_CANDIDATE` and `LANDING_ABSORB`, wheel torque `T` stays gated like airborne mode. In `BALANCE_RECOVERY`, wheel torque is ramped back before `BRAKE` sets the speed profile to direct zero.
- Jump leg retraction is delayed: after stable contact, the controller first holds the touchdown leg length, then ramps back to the nominal work point. The high-gain position stroke is limited to the short thrust window; after takeoff or timeout, the leg returns to compliant VMC hold.
- Verification: `py_compile` passed for `run_smoke.py`, `src/robot_smoke/experiments/virtual_rod.py`, `src/robot_smoke/control/ik.py`, and `src/robot_smoke/model/fivebar.py`. Viewer validation is still required for physical landing quality.
- Correction after viewer feedback: the analytic IK prewarm path was removed from jump position control because it could select a wrong practical branch for the current runtime posture. Flight-test landing detection was restored to the previous non-jump path; the new debounce state machine now only applies to `--jump-test` / `--forward-jump-test`. Airborne LQR gating again keys directly off `airborne` for all flight modes. The jump extension high-gain position window was lengthened to `0.50 s` maximum, while still exiting immediately after takeoff.
- Jump touchdown pitch correction: `TOUCHDOWN_CANDIDATE` and `LANDING_ABSORB` are no longer treated as airborne `Tp-only(theta,dtheta)` phases. They still gate wheel torque `T` and length-force delta, but keep the full LQR `Tp` output so the body pitch can be corrected immediately after contact. Pure airborne mode still uses the paper section 3 `Tp-only` gate.
- Jump touchdown balance correction: `TOUCHDOWN_CANDIDATE` and `LANDING_ABSORB` must not gate wheel torque `T` to zero. After contact, the robot is again a wheel inverted-pendulum system and needs wheel torque to move the support point under the body. Only pure airborne mode gates `T=0`; landing speed-stop is still delayed until `BRAKE`.
- Jump airborne tuck: while `landing_phase == AIRBORNE`, jump tests command both legs to `minimum_leg_length` instead of holding the maximum extension. The touchdown/absorb path still captures the actual contact length and then ramps back to the nominal work point, so the tuck command does not remain active after contact.
- Moving-jump airborne/landing correction: airborne tuck ramps to `minimum_leg_length` over `0.15 s` instead of stepping immediately. A trial airborne pitch-hold term was removed after viewer feedback because it did not change the visible airborne posture. The main landing impulse fix is to capture the actual leg length at first `TOUCHDOWN_CANDIDATE` contact and hold that target through candidate/absorb, preventing the command from jumping from tucked length back to `maximum_leg_length` before stable contact is confirmed.

## Body height reduction

- The base body was too tall above the leg motor mounting plane. The main `base_box` is now centered at the motor height `z=-0.05 m`, with vertical bounds `[-0.11, 0.01] m`, so the motor installation height is the midpoint between body bottom and top.
- The visual battery box and front marker were lowered to stay within the slimmer body envelope.
- The base inertial center was moved to `z=-0.05 m`, and diagonal inertia was reduced to match the shorter body envelope. This changes model mass distribution, so balance and jump behavior must be rechecked in viewer before drawing conclusions.
- Verification: `run_smoke.py --virtual-rod-steps 10` loaded the MuJoCo XML and passed finite smoke. This is only a load/finite-step check, not a stability claim.

## Five-bar link layering

- Same-side five-bar upper and lower link geoms were visually layered in the lateral `y` direction so they no longer occupy the same plane. Left upper links use the outward layer and left lower links use the inward layer; right side mirrors this.
- Only link `geom fromto` coordinates were offset. Joint bodies, sites, equality constraints, actuator mappings, and xz link geometry were not moved, so analytic IK/FK/Jacobian semantics remain unchanged.
- Verification: `run_smoke.py --virtual-rod-steps 10` loaded and stepped the model. Viewer inspection is still required to confirm the layered mechanism looks and moves correctly.
- Added zero-mass visual spacers at motor shafts, elbow shafts, front/rear hub layers, and wheel axle extensions so the layered links visually connect to motors and wheel carriers. Original carrier axle dimensions were restored; only zero-mass extensions bridge the visual gap to the wheel. Support-force estimate returned to the gimbal/counterweight baseline after this correction.

## Gimbal and counterweight body mockup

- Added a low-profile RM-style visual gimbal on top of the base: yaw pedestal, pitch block, and short barrel. These are visual-only and do not participate in contacts.
- Added a flat lower counterweight box under the base. Its footprint is smaller than the main body projection and its height is shallow.
- The base explicit inertial mass was increased to `7.2 kg`, with inertial center kept at the motor installation height `z=-0.05 m`. This represents the combined body, gimbal, and counterweight mass while keeping the target COM height near the motor layer.
- Verification: `run_smoke.py --virtual-rod-steps 10` loaded and stepped the model. The estimated support force changed to about `53.35 N` per leg, so leg feedforward and scheduled workpoints may need to be refreshed before judging dynamics.
- Revised launcher visual detail after reference-image feedback: the simple block/barrel was replaced by side carbon plates, a top plate, green top cover, black launcher body, front muzzle, upper/lower rollers, blue rail, and rear support posts. These remain visual-only.
- Enlarged the lower counterweight to `0.33 x 0.25 x 0.048 m`, slightly smaller than the main body projection and about two fifths of the current body height. Base mass is now `7.6 kg`, with COM still explicitly kept at `z=-0.05 m`.
- Verification: `run_smoke.py --virtual-rod-steps 10` passed after the revision. Estimated support force is now about `55.32 N` per leg.
- Removed the red front marker, shifted the launcher visual group rearward to center its projection over the body, and added a rear closing plate. Verification smoke still reports the same support-force estimate, so this was a visual/layout-only adjustment.
- Correction: the lower `counterweight_box` is now `visual_only`. Its mass is represented by the base explicit inertial, and it must not create a second body-ground/ramp contact below the chassis. Contact-enabled geoms now exclude the counterweight.

## Workpoint refresh after body model changes

- Regenerated `config/length_schedule.yaml` for `L0=0.16:0.01:0.30 m` using the current XML, `Q=[1,1,50,100,5000,10]`, `R=[1,0.25]`, `schedule_source=runtime`, and `--no-anchor-locked-035`.
- Current reduced-model parameters from XML: `R=0.085 m`, `mw=0.7 kg`, `Iw=0.00544731`, `mp=2.97734 kg`, `M=7.6 kg`, `Ip=0.0949108`, `IM=0.108`, `body_com_to_hip=0`, `leg_com_ratio=0.442203`.
- Analytic five-bar IK/Jacobian diagnostics are reachable for every table point; `cond(J)` is about `2.89..3.51`, finite-difference Jacobian error is about `1e-11..1e-10`, and controllability rank is `6/6`.
- The theory support force is `55.315 N` per leg. The scheduled length feedforward after the existing support-force scale is about `33.19 N` from `0.16..0.25 m`, then rises to `35.21 N` at `0.30 m`.
- Verification: `py_compile` passed for the schedule generator and loader. `run_smoke.py --virtual-rod-steps 10` loaded the new schedule and passed finite smoke. `run_smoke.py --lqr-true-equilibrium --visualize-seconds 10` completed a 10 s non-visual rollout with schedule enabled. Viewer validation is still required for physical stability.

## Flight/jump diagnostic status

- Current flight and jump entries are diagnostics, not qualified recovery tests. The stance `length_schedule` and LQR table are for wheel-ground contact; airborne and touchdown are contact-mode transitions and cannot be validated by the same stance workpoint alone.
- After the counterweight contact fix, a 10 s non-visual `--flight-test --lqr-debug-plot` still reported airborne from about `2.553 s` through the end of the rollout. That means the landing recovery logic did not get a stable recontact window in that run, so the result must not be used as a landing-balance conclusion.
- Speed-state correction: while the airborne detector is active, simulated wheel-center odometry is now frozen and its `previous_time` is cleared. This prevents in-air wheel-center/body motion from being integrated as ground forward speed before touchdown.
- Next landing work should be separated from ordinary balance/speed tests: first ensure only the wheels contact during ramp/touchdown, then create a controlled recontact scenario with stable touchdown timing before tuning or replacing the landing controller.

- Landing recovery update: the old `velocity-only` implementation was removed from the LQR helper and from the flight/jump control path. Speed profiles now feed both `x_ref` and `dx_ref` into LQR. After confirmed recontact, the landing path starts a separate finite-deceleration brake trajectory from the current odometry position and speed; it does not re-enable the deprecated direct-zero parking logic.
- Airborne motion-reference gate: while `airborne=True`, the ground speed profile is suspended, `dx_ref` is forced to zero, the LQR position reference is frozen at the last ground odometry position, and yaw differential wheel torque is also gated to zero. This prevents flight/jump entries from continuing a ground forward/turn command while both wheels have no contact.
- Unified LQR x-reference rule: all modes now neutralize the absolute `x` channel by setting the LQR position reference from current odometry position and `X0[2]`. Speed profiles keep only their `dx_ref`; the integrated position reference is not used as a global tracking target.
- Airborne length-reference rule: flight detection now has priority over ordinary leg-length references. During upward flight (`qvel[2] > 0`) both legs rapidly tuck; during downward flight they extend toward `maximum_leg_length` for landing preparation. The old `touchdown_candidate` and `landing_absorb` phases were removed because they froze the touchdown leg length and added false buffering behavior. Recontact now goes directly to normal balance recovery and the spring-damper length controller. Jump crouch is a VMC target of `0.18 m`; jump extension waits for measured leg length to reach that neighborhood before commanding the extension stroke. The jump entries no longer override the configured leg-length integral gain.
- LQR debug plot now includes left/right wheel-center world `z` curves with a wheel-radius reference line, so airborne/contact state transitions can be checked against actual wheel height instead of trusting the state machine alone.

## YAML parameter override audit

- Removed hidden post-YAML overrides in `runner.py`: jump entries no longer force `virtual_rod_length_force_rate_limit=0`, `--lqr-true-equilibrium` no longer replaces configured leg-length `Kp/Kd/F_l0` with locked constants, and LQR auto-design no longer silently changes `lqr_t_limit` from `16` to `8`.
- Remaining `LOCKED_*` use in normal LQR entries is limited to the locked pose / fallback LQR workpoint and equilibrium-search diagnostic defaults. Parser defaults in `cli.py` are still overwritten by `config/smoke.yaml` before parsing normal command-line arguments.
- Verification: `py_compile` passed. Ten-second non-visual startup checks for `--lqr-true-equilibrium` and `--jump-test` both printed `vmc_length_kp=200`, `vmc_length_kd=50`, `vmc_length_ki=120`, and `vmc_length_force_rate_limit=0`, matching current `config/smoke.yaml`. These are parameter-chain checks only, not viewer stability conclusions.

## Paper-style airborne gate

- Reworked flight detection back to the paper section 3 strategy: when both wheel normal forces are below the configured `F_N` threshold after contact arming, the controller only keeps the `K21/K22` feedback path, so wheel torque `T=0` and `Tp` is computed from `theta,dtheta` only.
- Removed the extra non-paper paths from the airborne branch: no odometry freeze, no forced `dx_ref=0`, no landing brake trajectory, and no vertical-speed-based airborne leg-length target. Recontact now immediately returns to the normal full LQR and normal compliant leg-length controller.
- Verification: `py_compile` passed for `virtual_rod.py`. Viewer validation is still required for landing behavior.
- Landing leg-length hold: after the first recontact, the effective airborne flag is latched off until the non-position LQR states except `x` stay near zero for `0.5 s`. During this hold, left/right leg-length references follow the measured leg lengths, so the length spring does not force the legs back to the nominal `L_ref` while the body is still recovering. The support-force curves remain visible in the LQR debug plot, but airborne start/end markers use the effective control state rather than raw threshold chatter.
- Landing speed reference: during `landing_hold`, the speed reference is explicitly set to `dx_ref=0`. After a rollout has actually entered airborne mode, the old ground speed trapezoid is suppressed for the rest of that rollout, so leaving `landing_hold` cannot reconnect a residual `dx_ref` from the original high-speed command. The LQR debug plot draws phase transition lines on every subplot so the landing phase can be compared against `x/dx`, pitch, theta, torque, leg length, wheel height, and support force.
- Landing wheel-torque limit: `config/smoke.yaml` now exposes `landing_hold_t_limit`. During `landing_hold`, the final wheel common torque `T` is clipped after the normal LQR low-pass/rate limiting stage, so the landing recovery cannot command the full `lqr_t_limit` impulse. Current value is read from YAML; as of this record it is `7.0 N*m`.
- Airborne re-trigger filtering: `config/smoke.yaml` exposes `flight_airborne_confirm_seconds` and `flight_airborne_rearm_seconds`. Both wheel support forces must stay below the threshold for the confirm time before `airborne` is entered, and after `landing_hold` exits the detector ignores short re-triggers for the rearm window. This prevents support-force chatter after balance recovery from creating extra airborne/landing phases.

## Slope roll-turn diagnostic

- Added `--slope-roll-turn-test`: it reuses the full-width flight-ramp scene without enabling airborne detection, drives forward at medium speed until `--slope-roll-turn-start-time` seconds, holds `dx_ref=0` for one second, then ramps to a low in-place yaw-rate command. This entry is diagnostic only; ramp placement and roll behavior must be judged in viewer.

## RL residual interface preparation

- Corrected the current semantics record to trust `config/smoke.yaml`: default `leg_length=0.24 m`, `length_schedule=true`, and LQR `x/dx` use simulated wheel odometry rather than `base_x/base_x_dot`. The old `0.35 m` locked workpoint remains only as fallback / diagnostic wording when scheduling is disabled.
- Added a controller interface mode switch: `rl_controller_mode=lqr` keeps the nominal LQR middle-layer output unchanged; `rl_controller_mode=lqr_residual` adds a bounded residual-RL action after the nominal LQR output and before the existing output low-pass/rate-limit path.
- Current residual policy is a zero placeholder and all residual limits default to `0`, so the residual mode is an interface smoke only. It does not start training and does not alter VMC, Jacobian, actuator signs, or the original LQR control law.
- Residual actions now cover `delta_T`, `delta_Tp`, common `delta_F_l`, and left/right leg-length reference deltas. Leg-length residuals still pass through the configured safe length clamp.
- Airborne mode is also part of the RL optimization surface: pure `lqr` keeps the paper section 3 LQR gate; `lqr_residual` treats that gate as the nominal baseline and can add bounded residuals for takeoff, aerial posture, tuck/extension and landing recovery.
- Added `--flight-test-speed low|medium|high` so flight-ramp training can be sampled by speed just like forward-jump training. `RL说明.md` now records the compact MCU-oriented residual RL framework; `server_training/residual_rl_tasks.yaml` stores the task keys for server-side samplers.
- Direction correction: keep LQR/VMC/PID/PD as MCU-portable nominal control and train only a small residual MLP. Target runtime is roughly 1 kHz traditional control and 100 Hz residual NN. The RL documentation was trimmed to framework semantics; detailed PPO/env implementation remains future work.
- Added the first minimal residual Env prototype: `server_training/residual_env.py` wraps the existing virtual-rod MuJoCo rollout with Gymnasium-style `reset/step`, normalized 5-D residual action, and task conditioning from `server_training/residual_rl_tasks.yaml`. `run_residual_env_smoke.py` is the root runnable smoke entry.
- Verification: `py_compile` passed for the new Env/script and touched rollout/types files. Short 2-step env smoke passed for `inplace_jump`, `forward_jump_low`, and `flight_ramp_low`; a nonzero normalized residual action also reached the control path without crashing. This only verifies the interface path, not physical jump/landing quality or PPO training performance.
- Added `run_residual_env_smoke.py --visualize`: visual smoke opens the MuJoCo viewer through the same task-conditioned residual Env config and prints `residual_mode`, normalized action and action limits before rollout, so the user can directly see whether the residual-control path is being exercised.
- Fixed residual zero-action semantics: airborne paper-style nominal gating is applied before residual injection in both `lqr` and `lqr_residual` modes, and airborne wheel torque in residual mode comes only from residual `delta_T`. Therefore a zero residual no longer bypasses the nominal airborne gate. Added `--controller-mode lqr|lqr_residual` and `--viewer-sync-hz` to the Env smoke entry for visual baseline comparison and lower render load.
- Removed low-speed forward-jump and flight-ramp keys from the residual-RL training task list. Low speed is kept only as an ordinary diagnostic capability where applicable; it is not a valid RL training task for speed jump or ramp landing because it predictably trips before a useful landing/balance sample.
- Added `run_residual_env_smoke.py --compare-zero-residual` for headless baseline checks: it runs the same task once as pure `lqr` and once as `lqr_residual` with zero residual action, then prints key metric differences. A short `flight_ramp_medium` 0.1 s comparison produced zero difference before airborne/contact transitions.
- Added `run_train_residual_ppo.py` and `server_training/train_residual_ppo.py` as the first Stable-Baselines3 PPO training entry. It samples the 5 residual task keys in parallel with `SubprocVecEnv` by default, uses a small `32x32 tanh` actor/critic MLP, and saves training outputs under ignored `runs/`. Training episode horizon defaults to `10 s` of MuJoCo simulation time, matching README/RL task visualization duration; training runs headless/as-fast-as-possible, and shorter horizons are only for pipeline smoke checks. Verification: `py_compile` passed, `--help` listed the 5 tasks, and a 16-timestep dummy PPO smoke saved `runs/residual_ppo/local_smoke_cpu/models/final_model.zip`.
- Fixed the residual Env training path after a 10 s / 1000-step episode was observed to take more than 60 s with no output. The Env now keeps per-episode rollout context across `step()` calls, including IK target cache, VMC memory, odometry, filters, jump/airborne/landing state, and previous controller outputs. The Env passes MuJoCo data by reference inside the episode instead of copying at every step. The training Env also runs the full 10 s MuJoCo horizon while updating the Python controller at a lower default rate of about 100 Hz and holding the previous control between updates; ordinary `run_smoke.py` diagnostics still use the original full-rate path unless explicitly changed through the Env path.
- Training Env result collection now skips repeated per-step diagnostic work that is not needed by PPO rewards, such as branch-violation summaries, history samples, finite assertions, wheel-speed maxima, and final static operating-point reports. This does not change the nominal controller formulas; it only removes repeated diagnostics from the Env step path.
- Verification: `py_compile` passed for `server_training/residual_env.py`, `src/robot_smoke/experiments/virtual_rod.py`, and `run_residual_env_smoke.py`. A full 10 s `flight_ramp_medium` Env episode with zero residual ran in about `5.88 s` wall-clock using `Measure-Command`. `run_residual_env_smoke.py --task-key flight_ramp_medium --compare-zero-residual --episode-seconds 10 --visualize-seconds 10` produced zero metric differences between pure `lqr` and `lqr_residual` with zero action for the printed comparison fields. A one-episode PPO smoke with `total_timesteps=1000` completed in about `19.39 s`, including Stable-Baselines3 initialization, PPO update, and model saving.
- Added explicit training-rate control for the residual Env. `ResidualCommandEnv` now defaults the controller update decimation to the policy step duration, and `run_train_residual_ppo.py` / `run_residual_env_smoke.py` expose `--control-decimation-steps` for overrides. With `--step-seconds 0.02`, a complete 10 s episode uses 500 policy/controller updates while MuJoCo still advances the full 10000 physics steps.
- Reused MuJoCo translational Jacobian buffers in the shared MuJoCo utility helpers and stopped requesting unused rotational Jacobians for body/site linear velocity calls. This is an allocation/overhead reduction only and does not change kinematic definitions.
- Verification: `py_compile` passed for the touched Env, training, smoke, MuJoCo utility, kinematics, and virtual-rod files. A full 10 s `flight_ramp_medium` zero-residual Env episode at `--step-seconds 0.02` ran in about `3.22 s` wall-clock. The same 50 Hz path passed `--compare-zero-residual`, with zero printed metric differences between `lqr` and `lqr_residual`. A one-episode PPO smoke with `total_timesteps=500`, `n_steps=500`, and `step_seconds=0.02` completed in about `14.14 s`, including Stable-Baselines3 initialization, PPO update, and model saving.
