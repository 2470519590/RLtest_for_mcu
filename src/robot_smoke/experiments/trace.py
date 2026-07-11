"""Control trace diagnostic helpers."""

from __future__ import annotations

from ..model.actuators import (
    actuator_ctrl_map as _actuator_ctrl_map,
    leg_drive_actuator_ids as _leg_drive_actuator_ids,
    wheel_actuator_ids as _wheel_actuator_ids,
)
from ..core.types import ControlTracePrevious, ControlTraceSample, ControlTraceStats, VmcDiagnostics

def _trace_control_output(
    mujoco,
    model,
    data,
    step: int,
    wheel_torque: float,
    pitch_torque: float,
    vmc_diagnostics: dict[str, VmcDiagnostics],
    previous: ControlTracePrevious | None,
    stats: ControlTraceStats,
    mode: str,
    event_ctrl_delta: float,
) -> tuple[ControlTracePrevious, ControlTraceSample]:
    left = vmc_diagnostics.get("left", VmcDiagnostics())
    right = vmc_diagnostics.get("right", VmcDiagnostics())
    ctrl_values = tuple(float(value) for value in data.ctrl)
    if previous is None:
        previous = ControlTracePrevious(ctrl=ctrl_values)

    d_t = wheel_torque - previous.wheel_torque
    d_tp = pitch_torque - previous.pitch_torque
    d_fl_left = left.length_force - previous.left_length_force
    d_fl_right = right.length_force - previous.right_length_force
    d_ftheta_left = left.theta_force - previous.left_theta_force
    d_ftheta_right = right.theta_force - previous.right_theta_force
    raw_tau_values = (
        left.joint_tau_front_raw,
        left.joint_tau_rear_raw,
        right.joint_tau_front_raw,
        right.joint_tau_rear_raw,
    )
    previous_raw_tau_values = (
        previous.left_front_tau_raw,
        previous.left_rear_tau_raw,
        previous.right_front_tau_raw,
        previous.right_rear_tau_raw,
    )
    max_raw_tau_delta = max(
        abs(current - old) for current, old in zip(raw_tau_values, previous_raw_tau_values)
    )
    max_ctrl_delta = max(
        abs(current - old) for current, old in zip(ctrl_values, previous.ctrl or ctrl_values)
    )

    lqr_jump = max(abs(d_t), abs(d_tp)) > 0.05
    vmc_jump = max(abs(d_fl_left), abs(d_fl_right), abs(d_ftheta_left), abs(d_ftheta_right), max_raw_tau_delta) > 0.1
    length_clip = (
        abs(left.length_force - left.length_force_raw) > 1e-9
        or abs(right.length_force - right.length_force_raw) > 1e-9
    )
    theta_scale_clip = left.theta_force_scale < 0.999 or right.theta_force_scale < 0.999
    joint_clip = (
        abs(left.joint_tau_front - left.joint_tau_front_raw) > 1e-9
        or abs(left.joint_tau_rear - left.joint_tau_rear_raw) > 1e-9
        or abs(right.joint_tau_front - right.joint_tau_front_raw) > 1e-9
        or abs(right.joint_tau_rear - right.joint_tau_rear_raw) > 1e-9
    )
    actuator_clip = False
    for actuator_id, ctrl in enumerate(ctrl_values):
        low, high = model.actuator_ctrlrange[actuator_id]
        actuator_clip = actuator_clip or abs(ctrl - float(low)) < 1e-12 or abs(ctrl - float(high)) < 1e-12
    clip_active = length_clip or theta_scale_clip or joint_clip or actuator_clip
    vmc_channel = "theta" if max(abs(d_ftheta_left), abs(d_ftheta_right)) > max(abs(d_fl_left), abs(d_fl_right)) else "length"

    source_parts: list[str] = []
    if lqr_jump:
        source_parts.append("LQR/middle")
    if vmc_jump:
        source_parts.append("VMC")
    if clip_active:
        source_parts.append("clip")
    if not source_parts:
        source_parts.append("viewer_possible" if max_ctrl_delta < 1e-4 else "small_ctrl_change")
    source_hint = "+".join(source_parts)

    stats.traced_steps += 1
    stats.sum_ctrl_delta += max_ctrl_delta
    stats.max_abs_dT = max(stats.max_abs_dT, abs(d_t))
    stats.max_abs_dTp = max(stats.max_abs_dTp, abs(d_tp))
    stats.max_abs_dF_l = max(stats.max_abs_dF_l, abs(d_fl_left), abs(d_fl_right))
    stats.max_abs_dF_theta = max(stats.max_abs_dF_theta, abs(d_ftheta_left), abs(d_ftheta_right))
    if max_ctrl_delta > stats.max_ctrl_delta:
        stats.max_ctrl_delta = max_ctrl_delta
        stats.max_ctrl_delta_step = step
    if lqr_jump:
        stats.lqr_steps += 1
    if vmc_jump:
        stats.vmc_steps += 1
    if clip_active:
        stats.clip_steps += 1
    if "viewer_possible" in source_parts:
        stats.viewer_possible_steps += 1
    if "small_ctrl_change" in source_parts:
        stats.small_change_steps += 1

    should_print = mode == "all" or (
        mode == "events" and (max_ctrl_delta >= event_ctrl_delta or clip_active)
    )
    if should_print:
        stats.event_steps += 1
        ctrl_map = _actuator_ctrl_map(mujoco, model, data)
        print(
            "control_trace: "
            f"step={step}, time_s={step * float(model.opt.timestep):.6g}, "
            f"source_hint={source_hint}, "
            f"T={wheel_torque:.6g}, dT={d_t:.6g}, "
            f"Tp={pitch_torque:.6g}, dTp={d_tp:.6g}, "
            f"F_l_left={left.length_force:.6g}, F_l_left_raw={left.length_force_raw:.6g}, "
            f"dF_l_left={d_fl_left:.6g}, "
            f"F_l_right={right.length_force:.6g}, F_l_right_raw={right.length_force_raw:.6g}, "
            f"dF_l_right={d_fl_right:.6g}, "
            f"F_theta_left={left.theta_force:.6g}, F_theta_left_raw={left.theta_force_raw:.6g}, "
            f"dF_theta_left={d_ftheta_left:.6g}, "
            f"F_theta_right={right.theta_force:.6g}, F_theta_right_raw={right.theta_force_raw:.6g}, "
            f"dF_theta_right={d_ftheta_right:.6g}, "
            f"vmc_jump_channel={vmc_channel}, "
            f"tau_joint_before_clip_left=[{left.joint_tau_front_raw:.6g}, {left.joint_tau_rear_raw:.6g}], "
            f"tau_joint_after_clip_left=[{left.joint_tau_front:.6g}, {left.joint_tau_rear:.6g}], "
            f"tau_joint_before_clip_right=[{right.joint_tau_front_raw:.6g}, {right.joint_tau_rear_raw:.6g}], "
            f"tau_joint_after_clip_right=[{right.joint_tau_front:.6g}, {right.joint_tau_rear:.6g}], "
            f"theta_scale_left={left.theta_force_scale:.6g}, "
            f"theta_scale_right={right.theta_force_scale:.6g}, "
            f"max_ctrl_delta={max_ctrl_delta:.6g}, "
            "ctrl={"
            + ", ".join(f"{name}={value:.6g}" for name, value in ctrl_map.items())
            + "}, "
            f"clip_flags=length:{length_clip},theta_scale:{theta_scale_clip},"
            f"joint:{joint_clip},actuator:{actuator_clip}"
        )

    next_previous = ControlTracePrevious(
        wheel_torque=wheel_torque,
        pitch_torque=pitch_torque,
        left_length_force=left.length_force,
        right_length_force=right.length_force,
        left_theta_force=left.theta_force,
        right_theta_force=right.theta_force,
        left_front_tau_raw=left.joint_tau_front_raw,
        left_rear_tau_raw=left.joint_tau_rear_raw,
        right_front_tau_raw=right.joint_tau_front_raw,
        right_rear_tau_raw=right.joint_tau_rear_raw,
        ctrl=ctrl_values,
    )
    left_front_id, left_rear_id = _leg_drive_actuator_ids(mujoco, model, "left")
    right_front_id, right_rear_id = _leg_drive_actuator_ids(mujoco, model, "right")
    left_wheel_id, right_wheel_id = _wheel_actuator_ids(mujoco, model)
    sample = ControlTraceSample(
        step=step,
        time_s=step * float(model.opt.timestep),
        source_lqr=1 if lqr_jump else 0,
        source_vmc=1 if vmc_jump else 0,
        source_clip=1 if clip_active else 0,
        source_viewer_possible=1 if "viewer_possible" in source_parts else 0,
        vmc_jump_channel=vmc_channel if vmc_jump else "",
        max_ctrl_delta=max_ctrl_delta,
        dT=d_t,
        dTp=d_tp,
        dF_l_left=d_fl_left,
        dF_l_right=d_fl_right,
        dF_theta_left=d_ftheta_left,
        dF_theta_right=d_ftheta_right,
        max_raw_tau_delta=max_raw_tau_delta,
        T=wheel_torque,
        Tp=pitch_torque,
        F_l_left=left.length_force,
        F_l_right=right.length_force,
        F_theta_left=left.theta_force,
        F_theta_right=right.theta_force,
        left_front_tau_raw=left.joint_tau_front_raw,
        left_rear_tau_raw=left.joint_tau_rear_raw,
        right_front_tau_raw=right.joint_tau_front_raw,
        right_rear_tau_raw=right.joint_tau_rear_raw,
        left_front_tau=left.joint_tau_front,
        left_rear_tau=left.joint_tau_rear,
        right_front_tau=right.joint_tau_front,
        right_rear_tau=right.joint_tau_rear,
        left_theta_force_scale=left.theta_force_scale,
        right_theta_force_scale=right.theta_force_scale,
        max_abs_ctrl=max(abs(value) for value in ctrl_values) if ctrl_values else 0.0,
        left_front_ctrl=float(data.ctrl[left_front_id]),
        left_rear_ctrl=float(data.ctrl[left_rear_id]),
        left_wheel_ctrl=float(data.ctrl[left_wheel_id]),
        right_front_ctrl=float(data.ctrl[right_front_id]),
        right_rear_ctrl=float(data.ctrl[right_rear_id]),
        right_wheel_ctrl=float(data.ctrl[right_wheel_id]),
    )
    return next_previous, sample


def _print_control_trace_summary(stats: ControlTraceStats) -> None:
    if stats.traced_steps <= 0:
        return
    mean_ctrl_delta = stats.sum_ctrl_delta / stats.traced_steps
    print("control_trace_summary:")
    print(f"  traced_steps: {stats.traced_steps}")
    print(f"  printed_event_steps: {stats.event_steps}")
    print(f"  max_ctrl_delta: {stats.max_ctrl_delta:.6g}")
    print(f"  max_ctrl_delta_step: {stats.max_ctrl_delta_step}")
    print(f"  mean_ctrl_delta: {mean_ctrl_delta:.6g}")
    print(f"  max_abs_dT: {stats.max_abs_dT:.6g}")
    print(f"  max_abs_dTp: {stats.max_abs_dTp:.6g}")
    print(f"  max_abs_dF_l: {stats.max_abs_dF_l:.6g}")
    print(f"  max_abs_dF_theta: {stats.max_abs_dF_theta:.6g}")
    print(f"  source_LQR_middle_steps: {stats.lqr_steps}")
    print(f"  source_VMC_steps: {stats.vmc_steps}")
    print(f"  source_clip_steps: {stats.clip_steps}")
    print(f"  source_viewer_possible_steps: {stats.viewer_possible_steps}")
    print(f"  source_small_ctrl_change_steps: {stats.small_change_steps}")


