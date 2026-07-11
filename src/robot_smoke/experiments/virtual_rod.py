"""Virtual-rod smoke and constraint-check flows."""

from __future__ import annotations

import math

import numpy as np

from ..model.actuators import apply_additive_wheel_torque_ctrl as _apply_additive_wheel_torque_ctrl
from ..model.fivebar import compute_leg_branch_metrics as _compute_leg_branch_metrics
from ..control.lqr import (
    apply_theta_pitch_feedforward as _apply_theta_pitch_feedforward,
    base_pitch_from_qpos as _base_pitch_from_qpos,
    lowpass_value as _lowpass_value,
)
from ..control.lqr_design import (
    _collect_lqr_history_sample,
    _lqr_middle_control,
    _prepare_lqr_operating_point,
)
from ..model.kinematics import (
    compute_virtual_leg_state as _compute_virtual_leg_state,
    lower_loop_error as _lower_loop_error,
    wheel_speeds as _wheel_speeds,
)
from ..control.ik import _virtual_rod_ik_ctrl
from ..model.mechanics import _collect_static_operating_point_sample
from ..core.mujoco_utils import assert_finite as _assert_finite
from ..core.mujoco_utils import copy_data as _copy_data
from ..core.mujoco_utils import id_by_name as _id_by_name
from ..core.mujoco_utils import lock_base_to_initial as _lock_base_to_initial
from ..core.mujoco_utils import name as _name
from ..io.output import (
    plot_control_trace as _plot_control_trace,
    resolve_output_path as _resolve_output_path,
    write_control_trace_csv as _write_control_trace_csv,
)
from ..experiments.trace import _print_control_trace_summary, _trace_control_output
from ..core.types import (
    ActuatorCtrlStats,
    ConstraintCheckResult,
    ControlTracePrevious,
    ControlTraceSample,
    ControlTraceStats,
    LqrHistorySample,
    LqrState,
    VirtualRodResult,
    VmcDiagnostics,
    VmcSideMemory,
)
from ..core.constants import (
    LEG_THETA_SYNC_KD,
    LEG_THETA_SYNC_KP,
)

def _run_virtual_rod_test(
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
    history_sample_interval: int,
    trace_control_output: bool,
    trace_control_start_step: int,
    trace_control_max_steps: int,
    trace_control_mode: str,
    trace_control_event_delta: float,
    trace_control_csv: Path | None,
    trace_control_plot: Path | None,
    initial_data: object | None = None,
    disturbance_force_x: float = 0.0,
    target_speed: float = 0.0,
) -> VirtualRodResult:
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
    if lock_base:
        _lock_base_to_initial(mujoco, model, data)

    max_abs_ctrl = 0.0
    max_length_error = 0.0
    max_theta_error = 0.0
    max_left_branch_violation = 0.0
    max_right_branch_violation = 0.0
    saturated_steps = 0
    min_ctrl = np.full(model.nu, math.inf)
    max_ctrl = np.full(model.nu, -math.inf)
    negative_saturated_steps = np.zeros(model.nu, dtype=int)
    positive_saturated_steps = np.zeros(model.nu, dtype=int)
    final_lqr_state: LqrState | None = None
    max_abs_lqr_wheel_torque = 0.0
    max_abs_lqr_pitch_torque = 0.0
    max_abs_left_wheel_speed = 0.0
    max_abs_right_wheel_speed = 0.0
    previous_wheel_torque = 0.0
    previous_pitch_torque = 0.0
    length_force_delta = 0.0
    x_velocity_reference = 0.0
    history: list[LqrHistorySample] = []
    ik_target_cache: dict[tuple[str, float, float, str, float, int], tuple[float, float]] = {}
    vmc_memory: dict[str, VmcSideMemory] = {}
    vmc_diagnostics: dict[str, VmcDiagnostics] = {}
    trace_previous: ControlTracePrevious | None = None
    trace_stats = ControlTraceStats()
    trace_samples: list[ControlTraceSample] = []
    moving_x_reference = lqr_x_reference
    commanded_speed = 0.0
    base_body_id = _id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, "base")
    disturbance_start = min(1000, max(1, steps // 3))
    disturbance_stop = min(steps, disturbance_start + 100)
    for step in range(steps):
        data.xfrc_applied[base_body_id, :] = 0.0
        if disturbance_force_x != 0.0 and disturbance_start <= step < disturbance_stop:
            data.xfrc_applied[base_body_id, 0] = disturbance_force_x
        timestep = float(model.opt.timestep)
        commanded_speed += float(np.clip(target_speed - commanded_speed, -0.1 * timestep, 0.1 * timestep))
        state_reference_speed = commanded_speed
        moving_x_reference += state_reference_speed * timestep
        if lock_base:
            _lock_base_to_initial(mujoco, model, data)
        wheel_torque = previous_wheel_torque
        pitch_torque = previous_pitch_torque
        if lqr_test and step % lqr_control_period_steps == 0:
            wheel_torque, pitch_torque, length_force_delta, final_lqr_state, x_velocity_reference = _lqr_middle_control(
                mujoco,
                model,
                data,
                moving_x_reference,
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
                state_reference_speed,
            )
            control_dt = float(model.opt.timestep) * lqr_control_period_steps
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
                max_delta = lqr_output_rate_limit * float(model.opt.timestep)
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
            max_abs_lqr_wheel_torque = max(max_abs_lqr_wheel_torque, abs(wheel_torque))
            max_abs_lqr_pitch_torque = max(max_abs_lqr_pitch_torque, abs(pitch_torque))
        elif lqr_test:
            max_abs_lqr_wheel_torque = max(max_abs_lqr_wheel_torque, abs(wheel_torque))
            max_abs_lqr_pitch_torque = max(max_abs_lqr_pitch_torque, abs(pitch_torque))
        pitch_for_theta_ff = final_lqr_state.pitch if final_lqr_state is not None else _base_pitch_from_qpos(data.qpos)
        step_left_target = _apply_theta_pitch_feedforward(left_target, pitch_for_theta_ff, theta_pitch_ff)
        step_right_target = _apply_theta_pitch_feedforward(right_target, pitch_for_theta_ff, theta_pitch_ff)
        effective_theta_kp = 0.0 if lqr_test else theta_kp
        effective_theta_kd = 0.0 if lqr_test else theta_kd
        left_leg = _compute_virtual_leg_state(mujoco, model, data, "left")
        right_leg = _compute_virtual_leg_state(mujoco, model, data, "right")
        sync_torque = -LEG_THETA_SYNC_KP * (left_leg.theta - right_leg.theta) - LEG_THETA_SYNC_KD * (
            left_leg.theta_rate - right_leg.theta_rate
        )
        ctrl_abs, saturated, length_error, theta_error = _virtual_rod_ik_ctrl(
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
            theta_force_offset=(0.5 * pitch_torque + sync_torque, 0.5 * pitch_torque - sync_torque),
            length_force_ff=length_force_ff + length_force_delta,
            length_ki=length_ki,
            length_integral_limit=length_integral_limit,
            length_force_rate_limit=length_force_rate_limit,
            vmc_memory=vmc_memory,
            vmc_diagnostics=vmc_diagnostics,
        )
        if lqr_test:
            saturated = (
                _apply_additive_wheel_torque_ctrl(
                    mujoco,
                    model,
                    data,
                    wheel_torque,
                    wheel_ctrl_deadzone,
                )
                or saturated
            )
            ctrl_abs = max(ctrl_abs, float(np.max(np.abs(data.ctrl))))
        if (
            trace_control_output
            and step >= trace_control_start_step
            and (trace_control_max_steps <= 0 or trace_stats.traced_steps < trace_control_max_steps)
        ):
            trace_previous, trace_sample = _trace_control_output(
                mujoco,
                model,
                data,
                step,
                wheel_torque,
                pitch_torque,
                vmc_diagnostics,
                trace_previous,
                trace_stats,
                trace_control_mode,
                trace_control_event_delta,
            )
            trace_samples.append(trace_sample)
        max_abs_ctrl = max(max_abs_ctrl, ctrl_abs)
        max_length_error = max(max_length_error, length_error)
        max_theta_error = max(max_theta_error, theta_error)
        left_branch = _compute_leg_branch_metrics(mujoco, model, data, "left")
        right_branch = _compute_leg_branch_metrics(mujoco, model, data, "right")
        max_left_branch_violation = max(max_left_branch_violation, left_branch.violation)
        max_right_branch_violation = max(max_right_branch_violation, right_branch.violation)
        if saturated:
            saturated_steps += 1
        min_ctrl = np.minimum(min_ctrl, data.ctrl)
        max_ctrl = np.maximum(max_ctrl, data.ctrl)
        for actuator_id in range(model.nu):
            low, high = model.actuator_ctrlrange[actuator_id]
            if abs(float(data.ctrl[actuator_id]) - low) < 1e-12:
                negative_saturated_steps[actuator_id] += 1
            if abs(float(data.ctrl[actuator_id]) - high) < 1e-12:
                positive_saturated_steps[actuator_id] += 1
        mujoco.mj_step(model, data)
        _assert_finite("qpos", data.qpos)
        _assert_finite("qvel", data.qvel)
        left_wheel_speed, right_wheel_speed = _wheel_speeds(mujoco, model, data)
        max_abs_left_wheel_speed = max(max_abs_left_wheel_speed, abs(left_wheel_speed))
        max_abs_right_wheel_speed = max(max_abs_right_wheel_speed, abs(right_wheel_speed))
        if lqr_test and (step % history_sample_interval == 0 or step == steps - 1):
            history.append(
                _collect_lqr_history_sample(
                    mujoco,
                    model,
                    data,
                    step + 1,
                    wheel_torque,
                    pitch_torque,
                    lqr_x_reference,
                    lqr_x_source,
                    left_target,
                    right_target,
                    vmc_diagnostics,
                    x_velocity_reference,
                )
            )

    if trace_control_output:
        _print_control_trace_summary(trace_stats)
        if trace_control_csv is not None:
            csv_path = _resolve_output_path(trace_control_csv)
            _write_control_trace_csv(csv_path, tuple(trace_samples))
            print(f"  trace_control_csv: {csv_path}")
        if trace_control_plot is not None:
            plot_path = _resolve_output_path(trace_control_plot)
            _plot_control_trace(plot_path, tuple(trace_samples))
            print(f"  trace_control_plot: {plot_path}")

    if lock_base:
        _lock_base_to_initial(mujoco, model, data)
    left_state = _compute_virtual_leg_state(mujoco, model, data, "left")
    right_state = _compute_virtual_leg_state(mujoco, model, data, "right")
    final_left_wheel_speed, final_right_wheel_speed = _wheel_speeds(mujoco, model, data)
    left_branch = _compute_leg_branch_metrics(mujoco, model, data, "left")
    right_branch = _compute_leg_branch_metrics(mujoco, model, data, "right")
    final_operating_point = _collect_static_operating_point_sample(
        mujoco,
        model,
        data,
        "final_virtual_rod_state",
        previous_wheel_torque,
        previous_pitch_torque,
        x_source=lqr_x_source,
        left_diag=vmc_diagnostics.get("left"),
        right_diag=vmc_diagnostics.get("right"),
    )
    actuator_ctrl_stats = tuple(
        ActuatorCtrlStats(
            actuator=_name(mujoco, model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id),
            min_ctrl=float(min_ctrl[actuator_id]),
            max_ctrl=float(max_ctrl[actuator_id]),
            negative_saturated_steps=int(negative_saturated_steps[actuator_id]),
            positive_saturated_steps=int(positive_saturated_steps[actuator_id]),
        )
        for actuator_id in range(model.nu)
    )
    return VirtualRodResult(
        steps=steps,
        lock_base=lock_base,
        left_target_length=left_target[0],
        left_target_theta=left_target[1],
        right_target_length=right_target[0],
        right_target_theta=right_target[1],
        final_left_length=left_state.length,
        final_left_theta=left_state.theta,
        final_right_length=right_state.length,
        final_right_theta=right_state.theta,
        max_length_error=max_length_error,
        max_theta_error=max_theta_error,
        max_left_branch_violation=max_left_branch_violation,
        max_right_branch_violation=max_right_branch_violation,
        final_left_branch_violation=left_branch.violation,
        final_right_branch_violation=right_branch.violation,
        max_abs_ctrl=max_abs_ctrl,
        saturated_steps=saturated_steps,
        actuator_ctrl_stats=actuator_ctrl_stats,
        final_lqr_state=final_lqr_state,
        history=tuple(history),
        max_abs_lqr_wheel_torque=max_abs_lqr_wheel_torque,
        max_abs_lqr_pitch_torque=max_abs_lqr_pitch_torque,
        max_abs_left_wheel_speed=max_abs_left_wheel_speed,
        max_abs_right_wheel_speed=max_abs_right_wheel_speed,
        final_left_wheel_speed=final_left_wheel_speed,
        final_right_wheel_speed=final_right_wheel_speed,
        final_base_height=float(data.qpos[2]) if model.nq >= 3 else math.nan,
        final_operating_point=final_operating_point,
        control_trace=tuple(trace_samples),
    )


def _check_lower_loop_constraints(
    mujoco,
    model,
    steps: int,
    use_virtual_rod: bool,
    lock_base: bool,
    left_target: tuple[float, float],
    right_target: tuple[float, float],
    virtual_rod_control: str,
    leg_branch: str,
    ik_search_radius: float,
    ik_search_samples: int,
    motor_kp: float,
    motor_kd: float,
    motor_ki: float,
    motor_length_force_ff: float,
    motor_length_integral_limit: float,
    motor_length_force_rate_limit: float,
    theta_kp: float,
    theta_kd: float,
    joint_kd: float,
    theta_pitch_ff: float,
    lqr_test: bool,
    lqr_gain_scale: float,
    lqr_k: np.ndarray,
    lqr_x0: np.ndarray,
    lqr_u0: np.ndarray,
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
) -> ConstraintCheckResult:
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    if lqr_test and lqr_auto_design:
        data = _prepare_lqr_operating_point(
            mujoco,
            model,
            left_target,
            right_target,
            leg_branch,
            ik_search_radius,
            ik_search_samples,
        )
    max_left_error = 0.0
    max_right_error = 0.0
    max_left_branch_violation = 0.0
    max_right_branch_violation = 0.0
    previous_wheel_torque = 0.0
    previous_pitch_torque = 0.0
    length_force_delta = 0.0
    ik_target_cache: dict[tuple[str, float, float, str, float, int], tuple[float, float]] = {}
    vmc_memory: dict[str, VmcSideMemory] = {}
    for step in range(steps):
        if lock_base:
            _lock_base_to_initial(mujoco, model, data)
        if use_virtual_rod:
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
                control_dt = float(model.opt.timestep) * lqr_control_period_steps
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
                    max_delta = lqr_output_rate_limit * float(model.opt.timestep)
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
                motor_kp,
                motor_kd,
                effective_theta_kp,
                effective_theta_kd,
                joint_kd,
                ik_target_cache,
                theta_force_offset=0.5 * pitch_torque,
                length_force_ff=motor_length_force_ff + length_force_delta,
                length_ki=motor_ki,
                length_integral_limit=motor_length_integral_limit,
                length_force_rate_limit=motor_length_force_rate_limit,
                vmc_memory=vmc_memory,
            )
            if lqr_test:
                _apply_additive_wheel_torque_ctrl(
                    mujoco,
                    model,
                    data,
                    wheel_torque,
                    wheel_ctrl_deadzone,
                )
        else:
            data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        _assert_finite("qpos", data.qpos)
        _assert_finite("qvel", data.qvel)
        max_left_error = max(max_left_error, _lower_loop_error(mujoco, model, data, "left"))
        max_right_error = max(max_right_error, _lower_loop_error(mujoco, model, data, "right"))
        left_branch = _compute_leg_branch_metrics(mujoco, model, data, "left")
        right_branch = _compute_leg_branch_metrics(mujoco, model, data, "right")
        max_left_branch_violation = max(max_left_branch_violation, left_branch.violation)
        max_right_branch_violation = max(max_right_branch_violation, right_branch.violation)
    final_left_branch = _compute_leg_branch_metrics(mujoco, model, data, "left")
    final_right_branch = _compute_leg_branch_metrics(mujoco, model, data, "right")
    return ConstraintCheckResult(
        steps=steps,
        max_left_error=max_left_error,
        max_right_error=max_right_error,
        final_left_error=_lower_loop_error(mujoco, model, data, "left"),
        final_right_error=_lower_loop_error(mujoco, model, data, "right"),
        max_left_branch_violation=max_left_branch_violation,
        max_right_branch_violation=max_right_branch_violation,
        final_left_branch_violation=final_left_branch.violation,
        final_right_branch_violation=final_right_branch.violation,
        final_base_height=float(data.qpos[2]) if model.nq >= 3 else math.nan,
    )


