"""MuJoCo model and low-level actuator smoke checks.

This module verifies that the model loads, exposes the expected low-level
semantics, and remains finite under small local smoke controllers.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np

from .model.actuators import (
    actuator_ctrl_map as _actuator_ctrl_map,
    apply_additive_wheel_torque_ctrl as _apply_additive_wheel_torque_ctrl,
    apply_joint_torque_ctrl as _apply_joint_torque_ctrl,
    leg_drive_actuator_ids as _leg_drive_actuator_ids,
    leg_drive_joint_ids as _leg_drive_joint_ids,
    wheel_actuator_ids as _wheel_actuator_ids,
)
from .core.constants import (
    DEFAULT_LQR_K,
    DEFAULT_MODEL,
    DEFAULT_VIRTUAL_ROD_LENGTH,
    LOCKED_EQUILIBRIUM_EVAL_STEPS,
    LOCKED_EQUILIBRIUM_FL0,
    LOCKED_EQUILIBRIUM_FL0_SCALE,
    LOCKED_EQUILIBRIUM_L0,
    LOCKED_EQUILIBRIUM_LENGTH_KD,
    LOCKED_EQUILIBRIUM_LENGTH_KP,
    LOCKED_EQUILIBRIUM_PITCH_KD,
    LOCKED_EQUILIBRIUM_PITCH_KP,
    LOCKED_EQUILIBRIUM_STEPS,
    LOCKED_EQUILIBRIUM_THETA,
    LOCKED_EQUILIBRIUM_THETA_KD,
    LOCKED_EQUILIBRIUM_THETA_KP,
    LOCKED_EQUILIBRIUM_TP_BIAS,
    LOCKED_EQUILIBRIUM_WHEEL_COM_KP,
    LOCKED_EQUILIBRIUM_WHEEL_DAMPING,
    LOCKED_LQR_CONTROL_PERIOD_STEPS,
    LOCKED_LQR_DESIGN_STEPS,
    PROJECT_ROOT,
)
from .io.cli import build_parser
from .model.fivebar import (
    analytic_fivebar_kinematics_from_q as _analytic_fivebar_kinematics_from_q,
    body_local_xz as _body_local_xz,
    compute_leg_branch_metrics as _compute_leg_branch_metrics,
    drive_qpos_for_side as _drive_qpos_for_side,
    equilibrium_analytic_ik_targets as _equilibrium_analytic_ik_targets,
    leg_branch_guard_error as _leg_branch_guard_error,
)
from .control.lqr import (
    apply_theta_pitch_feedforward as _apply_theta_pitch_feedforward,
    base_pitch_from_qpos as _base_pitch_from_qpos,
    compute_lqr_state as _compute_lqr_state,
    lowpass_value as _lowpass_value,
    lqr_middle_control_from_state as _lqr_middle_control_from_state,
    lqr_state_from_measurements as _lqr_state_from_measurements,
    lqr_state_vector as _lqr_state_vector,
    set_base_pitch as _set_base_pitch,
    smooth_deadzone_gate as _smooth_deadzone_gate,
    solve_discrete_are as _solve_discrete_are,
)
from .control.lqr_design import (
    _lqr_middle_control,
    _prepare_lqr_operating_point,
)
from .core.constants import LEG_THETA_SYNC_KD, LEG_THETA_SYNC_KP
from .model.kinematics import (
    compute_virtual_leg_shape as _compute_virtual_leg_shape,
    compute_virtual_leg_state as _compute_virtual_leg_state,
    lower_loop_error as _lower_loop_error,
    wheel_positions as _wheel_positions,
    wheel_radius as _wheel_radius,
    wheel_speeds as _wheel_speeds,
)
from .control.ik import (
    _branch_aware_ik_targets,
    _equilibrium_global_ik_targets,
    _virtual_rod_ik_ctrl,
)
from .model.mechanics import (
    _collect_static_operating_point_sample,
    _estimated_support_force_per_leg,
    _format_matrix_rows,
    _mechanics_equilibrium_seed,
    _print_static_operating_point_sample,
    _set_base_height_for_wheel_contact,
    _standing_base_qpos_for_virtual_leg_target,
)
from .core.mujoco_utils import (
    assert_finite as _assert_finite,
    body_pos as _body_pos,
    body_pos_vel as _body_pos_vel,
    copy_data as _copy_data,
    id_by_name as _id_by_name,
    iter_actuator_names as _iter_actuator_names,
    iter_joint_names as _iter_joint_names,
    joint_for_actuator as _joint_for_actuator,
    load_mujoco as _load_mujoco,
    lock_base_to_initial as _lock_base_to_initial,
    lock_base_to_qpos as _lock_base_to_qpos,
    name as _name,
    site_pos as _site_pos,
    site_pos_vel as _site_pos_vel,
    step_with_ctrl as _step_with_ctrl,
)
from .io.output import (
    plot_control_trace as _plot_control_trace,
    plot_lqr_history as _plot_lqr_history,
    plot_motor_torque_history as _plot_motor_torque_history,
    resolve_output_path as _resolve_output_path,
    write_control_trace_csv as _write_control_trace_csv,
    write_lqr_history_csv as _write_lqr_history_csv,
)
from .core.types import (
    ActuatorCtrlStats,
    ActuatorProbe,
    ConstraintCheckResult,
    ControlTracePrevious,
    ControlTraceSample,
    ControlTraceStats,
    EquilibriumSearchResult,
    LegBranchMetrics,
    LqrControlDebug,
    LqrDesignResult,
    LqrHistorySample,
    LqrState,
    MechanicsEquilibriumSeed,
    PdHoldResult,
    StaticOperatingPointSample,
    VirtualLegState,
    VirtualRodResult,
    VmcDiagnostics,
    VmcSideMemory,
)


def _set_viewer_status_overlay(
    viewer,
    mujoco,
    model,
    data,
    step: int,
    wheel_torque: float,
    pitch_torque: float,
    left_target: tuple[float, float],
    right_target: tuple[float, float],
    lqr_x0: np.ndarray,
    lqr_u0: np.ndarray,
    lqr_design_result: LqrDesignResult | None,
    x_source: str,
    vmc_diagnostics: dict[str, VmcDiagnostics],
    lqr_output_lowpass_hz: float,
    wheel_ctrl_deadzone: float,
) -> None:
    lqr_state = _compute_lqr_state(mujoco, model, data, 0.0, x_source)
    labels = [
        "step / time",
        "STATE theta, dtheta",
        "STATE x, dx",
        "STATE phi, dphi",
        "CONTROL T, Tp",
    ]
    values = [
        f"{step:d} ({step * float(model.opt.timestep):.3f}s)",
        f"{lqr_state.theta:+.5f} rad, {lqr_state.theta_rate:+.5f} rad/s",
        f"{lqr_state.x:+.5f} m, {lqr_state.x_rate:+.5f} m/s",
        f"{lqr_state.pitch:+.5f} rad, {lqr_state.pitch_rate:+.5f} rad/s",
        f"{wheel_torque:+.4f}, {pitch_torque:+.4f}",
    ]
    viewer.set_texts(
        (
            mujoco.mjtFontScale.mjFONTSCALE_150,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "\n".join(labels),
            "\n".join(values),
        )
    )


def _probe_actuators(mujoco, model, steps: int, ctrl_value: float) -> list[ActuatorProbe]:
    probes: list[ActuatorProbe] = []
    for actuator_id, actuator_name in enumerate(_iter_actuator_names(mujoco, model)):
        joint_id, joint_name = _joint_for_actuator(mujoco, model, actuator_id)
        ctrl_abs = min(abs(ctrl_value), float(model.actuator_ctrlrange[actuator_id, 1]))
        ctrl_abs = max(ctrl_abs, 1e-6)

        positive = np.zeros(model.nu)
        negative = np.zeros(model.nu)
        positive[actuator_id] = ctrl_abs
        negative[actuator_id] = -ctrl_abs

        _, qvel_pos = _step_with_ctrl(mujoco, model, steps, positive)
        _, qvel_neg = _step_with_ctrl(mujoco, model, steps, negative)

        if joint_id >= 0:
            dof_id = int(model.jnt_dofadr[joint_id])
            pos_vel = float(qvel_pos[dof_id])
            neg_vel = float(qvel_neg[dof_id])
        else:
            pos_vel = math.nan
            neg_vel = math.nan

        probes.append(
            ActuatorProbe(
                actuator=actuator_name,
                joint=joint_name,
                ctrl=ctrl_abs,
                positive_velocity=pos_vel,
                negative_velocity=neg_vel,
                opposite_sign=bool(pos_vel * neg_vel < 0.0),
            )
        )
    return probes


def _run_pd_hold(mujoco, model, steps: int, kp: float, kd: float) -> PdHoldResult:
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    max_abs_ctrl = 0.0
    max_abs_joint_error = 0.0
    saturated_steps = 0
    for _ in range(steps):
        max_abs_joint_error = max(max_abs_joint_error, _max_abs_actuated_joint_error(model, data))
        ctrl_abs, saturated_this_step = _pd_hold_ctrl(model, data, kp, kd)
        max_abs_ctrl = max(max_abs_ctrl, ctrl_abs)
        if saturated_this_step:
            saturated_steps += 1
        mujoco.mj_step(model, data)
        _assert_finite("qpos", data.qpos)
        _assert_finite("qvel", data.qvel)

    base_height = float(data.qpos[2]) if model.nq >= 3 else math.nan
    final_abs_joint_error = _max_abs_actuated_joint_error(model, data)
    return PdHoldResult(
        steps=steps,
        kp=kp,
        kd=kd,
        max_abs_ctrl=max_abs_ctrl,
        final_qpos_norm=float(np.linalg.norm(data.qpos)),
        final_qvel_norm=float(np.linalg.norm(data.qvel)),
        final_base_height=base_height,
        saturated_steps=saturated_steps,
        max_abs_joint_error=max_abs_joint_error,
        final_abs_joint_error=final_abs_joint_error,
    )


def _max_abs_actuated_joint_error(model, data) -> float:
    max_error = 0.0
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            continue
        qpos_id = int(model.jnt_qposadr[joint_id])
        target = float(model.qpos0[qpos_id])
        max_error = max(max_error, abs(target - float(data.qpos[qpos_id])))
    return max_error


def _pd_hold_ctrl(model, data, kp: float, kd: float) -> tuple[float, bool]:
    data.ctrl[:] = 0.0
    saturated = False
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            continue
        qpos_id = int(model.jnt_qposadr[joint_id])
        dof_id = int(model.jnt_dofadr[joint_id])
        target = float(model.qpos0[qpos_id])
        pos_err = target - float(data.qpos[qpos_id])
        vel = float(data.qvel[dof_id])
        low, high = model.actuator_ctrlrange[actuator_id]
        raw = kp * pos_err - kd * vel
        clipped = float(np.clip(raw, low, high))
        if abs(raw - clipped) > 1e-12:
            saturated = True
        data.ctrl[actuator_id] = clipped
    return float(np.max(np.abs(data.ctrl))), saturated


def _visualize_pd_hold(mujoco, model, steps: int, kp: float, kd: float, realtime: bool) -> None:
    import mujoco.viewer  # pylint: disable=import-outside-toplevel

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    timestep = float(model.opt.timestep)
    render_interval_steps = max(1, round(1.0 / (60.0 * timestep)))
    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall_start = time.perf_counter()
        for step in range(steps):
            _pd_hold_ctrl(model, data, kp, kd)
            mujoco.mj_step(model, data)
            _assert_finite("qpos", data.qpos)
            _assert_finite("qvel", data.qvel)
            render_due = (step + 1) % render_interval_steps == 0 or step + 1 == steps
            if render_due:
                viewer.sync()
                if realtime:
                    target_wall_time = (step + 1) * timestep
                    delay = target_wall_time - (time.perf_counter() - wall_start)
                    if delay > 0.0:
                        time.sleep(delay)
            if not viewer.is_running():
                break


def _visualize_virtual_rod_test(
    mujoco,
    model,
    steps: int,
    lock_base: bool,
    left_target: tuple[float, float],
    right_target: tuple[float, float],
    virtual_rod_control: str,
    leg_branch: str,
    ik_search_radius: float,
    ik_search_samples: int,
    length_kp: float,
    length_kd: float,
    length_ki: float,
    length_force_ff: float,
    length_integral_limit: float,
    length_force_rate_limit: float,
    theta_kp: float,
    theta_kd: float,
    joint_kd: float,
    theta_pitch_ff: float,
    lqr_test: bool,
    lqr_gain_scale: float,
    lqr_k: np.ndarray,
    lqr_x0: np.ndarray,
    lqr_u0: np.ndarray,
    lqr_design_result: LqrDesignResult | None,
    lqr_auto_design: bool,
    lqr_control_period_steps: int,
    lqr_x_reference: float,
    lqr_x_source: str,
    lqr_x_outer_kp: float,
    lqr_x_outer_max_v: float,
    lqr_wheel_sign: float,
    lqr_pitch_sign: float,
    lqr_t_limit: float,
    lqr_tp_limit: float,
    lqr_output_rate_limit: float,
    lqr_output_lowpass_hz: float,
    wheel_ctrl_deadzone: float,
    initial_data: object | None,
    realtime: bool,
) -> None:
    import mujoco.viewer  # pylint: disable=import-outside-toplevel

    if initial_data is not None:
        data = _copy_data(mujoco, model, initial_data)
    else:
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
    if initial_data is None and lqr_test and lqr_auto_design:
        data = _prepare_lqr_operating_point(
            mujoco,
            model,
            left_target,
            right_target,
            leg_branch,
            ik_search_radius,
            ik_search_samples,
        )
    timestep = float(model.opt.timestep)
    render_interval_steps = max(1, round(1.0 / (60.0 * timestep)))
    previous_wheel_torque = 0.0
    previous_pitch_torque = 0.0
    length_force_delta = 0.0
    ik_target_cache: dict[tuple[str, float, float, str, float, int], tuple[float, float]] = {}
    vmc_memory: dict[str, VmcSideMemory] = {}
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.sync()
        wall_start = time.perf_counter()
        for step in range(steps):
            if lock_base:
                _lock_base_to_initial(mujoco, model, data)
            wheel_torque = previous_wheel_torque
            pitch_torque = previous_pitch_torque
            if lqr_test and step % lqr_control_period_steps == 0:
                wheel_torque, pitch_torque, length_force_delta, _, _ = _lqr_middle_control(
                    mujoco,
                    model,
                    data,
                    lqr_x_reference,
                    lqr_x_source,
                    lqr_gain_scale,
                    lqr_k,
                    lqr_x0,
                    lqr_u0,
                    lqr_wheel_sign,
                    lqr_pitch_sign,
                    lqr_t_limit,
                    lqr_tp_limit,
                    lqr_x_outer_kp,
                    lqr_x_outer_max_v,
                )
                control_dt = timestep * lqr_control_period_steps
                wheel_torque = _lowpass_value(
                    wheel_torque,
                    previous_wheel_torque,
                    lqr_output_lowpass_hz,
                    control_dt,
                )
                pitch_torque = _lowpass_value(
                    pitch_torque,
                    previous_pitch_torque,
                    lqr_output_lowpass_hz,
                    control_dt,
                )
                if lqr_output_rate_limit > 0.0:
                    max_delta = lqr_output_rate_limit * timestep
                    wheel_torque = float(
                        np.clip(
                            wheel_torque,
                            previous_wheel_torque - max_delta,
                            previous_wheel_torque + max_delta,
                        )
                    )
                    pitch_torque = float(
                        np.clip(
                            pitch_torque,
                            previous_pitch_torque - max_delta,
                            previous_pitch_torque + max_delta,
                        )
                    )
                previous_wheel_torque = wheel_torque
                previous_pitch_torque = pitch_torque
            pitch_for_theta_ff = _base_pitch_from_qpos(data.qpos)
            step_left_target = _apply_theta_pitch_feedforward(left_target, pitch_for_theta_ff, theta_pitch_ff)
            step_right_target = _apply_theta_pitch_feedforward(right_target, pitch_for_theta_ff, theta_pitch_ff)
            effective_theta_kp = 0.0 if lqr_test else theta_kp
            effective_theta_kd = 0.0 if lqr_test else theta_kd
            left_leg = _compute_virtual_leg_state(mujoco, model, data, "left")
            right_leg = _compute_virtual_leg_state(mujoco, model, data, "right")
            sync_torque = -LEG_THETA_SYNC_KP * (left_leg.theta - right_leg.theta) - LEG_THETA_SYNC_KD * (
                left_leg.theta_rate - right_leg.theta_rate
            )
            vmc_diagnostics: dict[str, VmcDiagnostics] = {}
            _virtual_rod_ik_ctrl(
                mujoco,
                model,
                data,
                step_left_target,
                step_right_target,
                virtual_rod_control,
                leg_branch,
                ik_search_radius,
                ik_search_samples,
                length_kp,
                length_kd,
                effective_theta_kp,
                effective_theta_kd,
                joint_kd,
                ik_target_cache,
                theta_force_offset=(
                    pitch_torque + sync_torque,
                    pitch_torque - sync_torque,
                ),
                length_force_ff=length_force_ff + length_force_delta,
                length_ki=length_ki,
                length_integral_limit=length_integral_limit,
                length_force_rate_limit=length_force_rate_limit,
                vmc_memory=vmc_memory,
                vmc_diagnostics=vmc_diagnostics,
            )
            if lqr_test:
                _apply_additive_wheel_torque_ctrl(
                    mujoco,
                    model,
                    data,
                    wheel_torque,
                    wheel_ctrl_deadzone,
                )
            mujoco.mj_step(model, data)
            _assert_finite("qpos", data.qpos)
            _assert_finite("qvel", data.qvel)
            render_due = (step + 1) % render_interval_steps == 0 or step + 1 == steps
            if render_due:
                _set_viewer_status_overlay(
                    viewer,
                    mujoco,
                    model,
                    data,
                    step,
                    wheel_torque,
                    pitch_torque,
                    left_target,
                    right_target,
                    lqr_x0,
                    lqr_u0,
                    lqr_design_result,
                    lqr_x_source,
                    vmc_diagnostics,
                    lqr_output_lowpass_hz,
                    wheel_ctrl_deadzone,
                )
                viewer.sync()
                if realtime:
                    target_wall_time = (step + 1) * timestep
                    delay = target_wall_time - (time.perf_counter() - wall_start)
                    if delay > 0.0:
                        time.sleep(delay)
            if not viewer.is_running():
                break


def _print_virtual_leg_state(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    print("virtual_leg_state_at_reset:")
    for side in ("left", "right"):
        state = _compute_virtual_leg_state(mujoco, model, data, side)
        print(
            f"  {side}: l={state.length:.6g} m, dl={state.length_rate:.6g} m/s, "
            f"theta={state.theta:.6g} rad, dtheta={state.theta_rate:.6g} rad/s"
        )


def _scan_virtual_rod_geometry(mujoco, model, sample: float) -> None:
    data = mujoco.MjData(model)
    print("virtual_rod_geometry_scan:")
    print("  note: q_front/q_rear are set directly, then mj_forward is called")
    for side in ("left", "right"):
        front_joint_id, rear_joint_id = _leg_drive_joint_ids(mujoco, model, side)
        front_qpos_id = int(model.jnt_qposadr[front_joint_id])
        rear_qpos_id = int(model.jnt_qposadr[rear_joint_id])
        print(f"  {side}:")
        for q_front in (-sample, 0.0, sample):
            for q_rear in (-sample, 0.0, sample):
                mujoco.mj_resetData(model, data)
                data.qpos[front_qpos_id] = q_front
                data.qpos[rear_qpos_id] = q_rear
                mujoco.mj_forward(model, data)
                state = _compute_virtual_leg_state(mujoco, model, data, side)
                print(
                    f"    q_front={q_front:+.3f}, q_rear={q_rear:+.3f} -> "
                    f"l={state.length:.6g}, theta={state.theta:.6g}"
                )


def _scan_virtual_rod_dynamic(mujoco, model, sample: float, steps: int, kp: float, kd: float) -> None:
    data = mujoco.MjData(model)
    print("virtual_rod_dynamic_scan:")
    print("  note: base is locked each step; front/rear drive targets are tracked by motor PD")
    for side in ("left", "right"):
        print(f"  {side}:")
        for q_front in (-sample, 0.0, sample):
            for q_rear in (-sample, 0.0, sample):
                mujoco.mj_resetData(model, data)
                mujoco.mj_forward(model, data)
                for _ in range(steps):
                    _lock_base_to_initial(mujoco, model, data)
                    data.ctrl[:] = 0.0
                    _drive_joint_position_ctrl(mujoco, model, data, side, q_front, q_rear, kp, kd)
                    mujoco.mj_step(model, data)
                    _assert_finite("qpos", data.qpos)
                    _assert_finite("qvel", data.qvel)
                _lock_base_to_initial(mujoco, model, data)
                state = _compute_virtual_leg_state(mujoco, model, data, side)
                print(
                    f"    q_front={q_front:+.3f}, q_rear={q_rear:+.3f} -> "
                    f"l={state.length:.6g}, theta={state.theta:.6g}"
                )


def _build_virtual_rod_targets(
    mujoco,
    model,
    virtual_rod_length_delta: float | None,
    virtual_rod_theta_target: float,
    left_rod_length: float | None,
    right_rod_length: float | None,
    left_rod_theta: float | None,
    right_rod_theta: float | None,
) -> tuple[tuple[float, float], tuple[float, float]]:
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    left_reset = _compute_virtual_leg_state(mujoco, model, data, "left")
    right_reset = _compute_virtual_leg_state(mujoco, model, data, "right")
    left_target = (
        left_rod_length
        if left_rod_length is not None
        else (
            left_reset.length + virtual_rod_length_delta
            if virtual_rod_length_delta is not None
            else DEFAULT_VIRTUAL_ROD_LENGTH
        ),
        left_rod_theta if left_rod_theta is not None else virtual_rod_theta_target,
    )
    right_target = (
        right_rod_length
        if right_rod_length is not None
        else (
            right_reset.length + virtual_rod_length_delta
            if virtual_rod_length_delta is not None
            else DEFAULT_VIRTUAL_ROD_LENGTH
        ),
        right_rod_theta if right_rod_theta is not None else virtual_rod_theta_target,
    )
    return left_target, right_target


