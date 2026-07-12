"""Top-level smoke runner orchestration."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .io.cli import build_parser
from .core.constants import (
    DEFAULT_LQR_K,
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
from .experiments.equilibrium import (
    _equilibrium_score,
    _print_static_operating_point_sample,
    _run_equilibrium_search,
    _run_equilibrium_static_pose_check,
)
from .experiments.fivebar_checks import _run_fivebar_jacobian_check, _run_fivebar_kinematics_check
from .experiments.fl_tests import _run_fl_channel_test, _run_fl_pulse_test
from .control.lqr import lqr_state_vector as _lqr_state_vector
from .control.lqr_design import _compute_lqr_state, _design_lqr_gain
from .model_smoke import (
    _build_virtual_rod_targets,
    _estimated_support_force_per_leg,
    _format_matrix_rows,
    _print_virtual_leg_state,
    _probe_actuators,
    _run_pd_hold,
    _scan_virtual_rod_dynamic,
    _scan_virtual_rod_geometry,
    _visualize_pd_hold,
    _visualize_virtual_rod_test,
)
from .core.mujoco_utils import (
    iter_actuator_names as _iter_actuator_names,
    iter_joint_names as _iter_joint_names,
    joint_for_actuator as _joint_for_actuator,
    load_mujoco as _load_mujoco,
    step_with_ctrl as _step_with_ctrl,
)
from .io.output import (
    plot_lqr_history as _plot_lqr_history,
    plot_motor_torque_history as _plot_motor_torque_history,
    resolve_output_path as _resolve_output_path,
    write_lqr_history_csv as _write_lqr_history_csv,
)
from .core.types import EquilibriumSearchResult, LqrDesignResult
from .experiments.virtual_rod import _check_lower_loop_constraints, _run_virtual_rod_test
from .model.mechanics import support_force_scale_for_length

def run_smoke(
    model_path: Path,
    zero_steps: int,
    probe_steps: int,
    ctrl: float,
    pd_hold_steps: int,
    pd_kp: float,
    pd_kd: float,
    visualize: bool,
    print_virtual_leg: bool,
    scan_virtual_rod: bool,
    scan_virtual_rod_dynamic: bool,
    scan_virtual_rod_sample: float,
    check_constraints: bool,
    constraint_steps: int,
    virtual_rod_test: bool,
    virtual_rod_lock_base: bool,
    virtual_rod_steps: int,
    virtual_rod_length_delta: float | None,
    virtual_rod_theta_target: float,
    left_rod_length: float | None,
    right_rod_length: float | None,
    left_rod_theta: float | None,
    right_rod_theta: float | None,
    virtual_rod_control: str,
    leg_branch: str,
    ik_search_radius: float,
    ik_search_samples: int,
    virtual_rod_length_kp: float,
    virtual_rod_length_kd: float,
    virtual_rod_length_ki: float,
    virtual_rod_length_force_ff: float,
    virtual_rod_gravity_comp_scale: float | None,
    virtual_rod_length_integral_limit: float,
    virtual_rod_length_force_rate_limit: float,
    virtual_rod_theta_kp: float,
    virtual_rod_theta_kd: float,
    virtual_rod_joint_kd: float,
    virtual_rod_theta_pitch_ff: float,
    lqr_test: bool,
    lqr_gain_scale: float,
    lqr_k: np.ndarray,
    lqr_x0: np.ndarray,
    lqr_u0: np.ndarray,
    lqr_auto_design: bool,
    lqr_q_diag: np.ndarray,
    lqr_r_diag: np.ndarray,
    lqr_state_eps: np.ndarray,
    lqr_input_eps: np.ndarray,
    lqr_design_steps: int,
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
    history_csv: Path | None,
    history_plot: Path | None,
    motor_torque_plot: Path | None,
    trace_control_output: bool,
    trace_control_start_step: int,
    trace_control_max_steps: int,
    trace_control_mode: str,
    trace_control_event_delta: float,
    trace_control_csv: Path | None,
    trace_control_plot: Path | None,
    visualize_seconds: float | None,
    fl_channel_test: bool,
    fl_channel_forces: tuple[float, ...],
    fl_channel_steps: int,
    fl_pulse_test: bool,
    fl_pulse_base_force: float,
    fl_pulse_delta_force: float,
    fl_pulse_settle_steps: int,
    fl_pulse_steps: int,
    fivebar_kinematics_check: bool,
    fivebar_kinematics_l_slices: tuple[float, ...],
    fivebar_kinematics_theta_refs: tuple[float, ...],
    fivebar_jacobian_check: bool,
    fivebar_jacobian_l_slices: tuple[float, ...],
    fivebar_jacobian_theta_refs: tuple[float, ...],
    fivebar_jacobian_force_scale: float,
    fivebar_jacobian_load_steps: int,
    equilibrium_static_pose_check: bool,
    equilibrium_search: bool,
    equilibrium_init_modes: tuple[str, ...],
    equilibrium_l_slices: tuple[float, ...],
    equilibrium_theta_refs: tuple[float, ...],
    equilibrium_fl0_scales: tuple[float, ...],
    equilibrium_tp_biases: tuple[float, ...],
    equilibrium_wheel_com_kps: tuple[float, ...],
    equilibrium_wheel_dampings: tuple[float, ...],
    equilibrium_init_drop_steps: int,
    equilibrium_steps: int,
    equilibrium_eval_steps: int,
    equilibrium_length_kp: float,
    equilibrium_length_kd: float,
    equilibrium_theta_kp: float,
    equilibrium_theta_kd: float,
    equilibrium_pitch_kp: float,
    equilibrium_pitch_kd: float,
    lqr_use_equilibrium_operating_point: bool,
    print_static_operating_point: bool,
    diagnostics_only: bool,
    realtime: bool,
) -> int:
    mujoco = _load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    behavior_passed = True
    estimated_support_force_per_leg = _estimated_support_force_per_leg(model)
    if virtual_rod_gravity_comp_scale is not None:
        virtual_rod_length_force_ff = virtual_rod_gravity_comp_scale * estimated_support_force_per_leg

    print("MuJoCo smoke check")
    print(f"model_path: {model_path}")
    print(f"mujoco_version: {mujoco.__version__}")
    print(f"nq: {model.nq}")
    print(f"nv: {model.nv}")
    print(f"nu: {model.nu}")
    print(f"njnt: {model.njnt}")
    print(f"timestep: {model.opt.timestep}")
    print(f"estimated_support_force_per_leg: {estimated_support_force_per_leg:.6g}")
    print()

    if print_virtual_leg or virtual_rod_test:
        _print_virtual_leg_state(mujoco, model)
        print()

    if scan_virtual_rod:
        _scan_virtual_rod_geometry(mujoco, model, scan_virtual_rod_sample)
        print()

    if scan_virtual_rod_dynamic:
        _scan_virtual_rod_dynamic(mujoco, model, scan_virtual_rod_sample, 250, 30.0, 1.2)
        print()

    if fl_channel_test:
        _run_fl_channel_test(mujoco, model, fl_channel_forces, fl_channel_steps)
        print()

    if fl_pulse_test:
        _run_fl_pulse_test(
            mujoco,
            model,
            fl_pulse_base_force,
            fl_pulse_delta_force,
            fl_pulse_settle_steps,
            fl_pulse_steps,
        )
        print()

    if fivebar_kinematics_check:
        _run_fivebar_kinematics_check(
            mujoco,
            model,
            fivebar_kinematics_l_slices,
            fivebar_kinematics_theta_refs,
        )
        print()

    if fivebar_jacobian_check:
        _run_fivebar_jacobian_check(
            mujoco,
            model,
            fivebar_jacobian_l_slices,
            fivebar_jacobian_theta_refs,
            fivebar_jacobian_force_scale,
            fivebar_jacobian_load_steps,
        )
        print()

    if equilibrium_static_pose_check:
        _run_equilibrium_static_pose_check(
            mujoco,
            model,
            equilibrium_l_slices,
            equilibrium_theta_refs,
            equilibrium_fl0_scales,
            leg_branch,
            ik_search_radius,
            ik_search_samples,
        )
        print()

    best_equilibrium: EquilibriumSearchResult | None = None
    if equilibrium_search:
        equilibrium_results = _run_equilibrium_search(
            mujoco,
            model,
            equilibrium_init_modes,
            equilibrium_l_slices,
            equilibrium_theta_refs,
            equilibrium_fl0_scales,
            equilibrium_tp_biases,
            equilibrium_wheel_com_kps,
            equilibrium_wheel_dampings,
            equilibrium_init_drop_steps,
            equilibrium_steps,
            equilibrium_eval_steps,
            equilibrium_length_kp,
            equilibrium_length_kd,
            equilibrium_theta_kp,
            equilibrium_theta_kd,
            equilibrium_pitch_kp,
            equilibrium_pitch_kd,
            lqr_t_limit,
            lqr_tp_limit,
            45.0,
            leg_branch,
            ik_search_radius,
            ik_search_samples,
        )
        best_equilibrium = min(equilibrium_results, key=_equilibrium_score) if equilibrium_results else None
        print()

    if diagnostics_only:
        return 0

    left_target, right_target = _build_virtual_rod_targets(
        mujoco,
        model,
        virtual_rod_length_delta,
        virtual_rod_theta_target,
        left_rod_length,
        right_rod_length,
        left_rod_theta,
        right_rod_theta,
    )
    lqr_design_result: LqrDesignResult | None = None
    lqr_operating_data = None
    lqr_operating_u0 = None
    if lqr_test and lqr_auto_design:
        if lqr_use_equilibrium_operating_point:
            if best_equilibrium is None:
                print("warning: --lqr-use-equilibrium-operating-point requested without --equilibrium-search")
            elif not best_equilibrium.qualified:
                print("error: equilibrium candidate is not qualified; LQR linearization aborted")
                return 2
            elif best_equilibrium.final_data is None:
                print("warning: best equilibrium candidate has no final_data; using default LQR operating point")
            else:
                lqr_operating_data = best_equilibrium.final_data
                lqr_operating_data.qvel[:] = 0.0
                lqr_operating_data.ctrl[:] = 0.0
                mujoco.mj_forward(model, lqr_operating_data)
                lqr_operating_u0 = np.array(
                    [
                        best_equilibrium.final_sample.wheel_torque / lqr_wheel_sign,
                        best_equilibrium.final_sample.pitch_torque / lqr_pitch_sign,
                    ],
                    dtype=float,
                )
                lqr_x0 = _lqr_state_vector(
                    _compute_lqr_state(mujoco, model, lqr_operating_data, 0.0, lqr_x_source)
                )
                lqr_u0 = lqr_operating_u0.copy()
                virtual_rod_length_force_ff = best_equilibrium.fl0
        lqr_design_result = _design_lqr_gain(
            mujoco,
            model,
            left_target,
            right_target,
            leg_branch,
            ik_search_radius,
            ik_search_samples,
            virtual_rod_length_kp,
            virtual_rod_length_kd,
            virtual_rod_length_ki,
            virtual_rod_length_force_ff,
            virtual_rod_length_integral_limit,
            virtual_rod_length_force_rate_limit,
            virtual_rod_joint_kd,
            lqr_q_diag,
            lqr_r_diag,
            lqr_state_eps,
            lqr_input_eps,
            lqr_design_steps,
            lqr_x_source,
            lqr_wheel_sign,
            lqr_pitch_sign,
            leg_control_enabled=virtual_rod_control != "off",
            operating_data=lqr_operating_data,
            operating_u0=lqr_operating_u0,
        )
        lqr_k = lqr_design_result.k_matrix
        lqr_x0 = _lqr_state_vector(lqr_design_result.operating_state)
        lqr_u0 = lqr_design_result.operating_u0.copy()

    print("joints:")
    for joint_name in _iter_joint_names(mujoco, model):
        print(f"  - {joint_name}")
    print()

    print("actuators:")
    for actuator_id, actuator_name in enumerate(_iter_actuator_names(mujoco, model)):
        _, joint_name = _joint_for_actuator(mujoco, model, actuator_id)
        low, high = model.actuator_ctrlrange[actuator_id]
        gear = model.actuator_gear[actuator_id, 0]
        print(
            f"  - {actuator_name}: joint={joint_name}, "
            f"ctrlrange=[{low:.3g}, {high:.3g}], gear={gear:.3g}"
        )
    print()

    zero_ctrl = np.zeros(model.nu)
    qpos, qvel = _step_with_ctrl(mujoco, model, zero_steps, zero_ctrl)
    print(f"zero_input_steps: {zero_steps}")
    print(f"zero_input_qpos_norm: {np.linalg.norm(qpos):.6g}")
    print(f"zero_input_qvel_norm: {np.linalg.norm(qvel):.6g}")
    print()

    probes = _probe_actuators(mujoco, model, probe_steps, ctrl)
    print(f"actuator_probe_steps: {probe_steps}")
    for probe in probes:
        sign_status = "opposite" if probe.opposite_sign else "not-confirmed"
        print(
            f"  - {probe.actuator}: joint={probe.joint}, ctrl=+/-{probe.ctrl:.3g}, "
            f"qvel(+)= {probe.positive_velocity:.6g}, "
            f"qvel(-)= {probe.negative_velocity:.6g}, sign={sign_status}"
        )

    not_confirmed = [probe.actuator for probe in probes if not probe.opposite_sign]
    if not_confirmed:
        print()
        print("warning: some actuator direction probes were finite but not sign-confirmed:")
        for actuator_name in not_confirmed:
            print(f"  - {actuator_name}")

    print()
    pd_result = _run_pd_hold(mujoco, model, pd_hold_steps, pd_kp, pd_kd)
    print("pd_hold:")
    print(f"  steps: {pd_result.steps}")
    print(f"  kp: {pd_result.kp:.6g}")
    print(f"  kd: {pd_result.kd:.6g}")
    print(f"  final_base_height: {pd_result.final_base_height:.6g}")
    print(f"  final_qpos_norm: {pd_result.final_qpos_norm:.6g}")
    print(f"  final_qvel_norm: {pd_result.final_qvel_norm:.6g}")
    print(f"  max_abs_ctrl: {pd_result.max_abs_ctrl:.6g}")
    print(f"  saturated_steps: {pd_result.saturated_steps}")
    print(f"  max_abs_joint_error: {pd_result.max_abs_joint_error:.6g}")
    print(f"  final_abs_joint_error: {pd_result.final_abs_joint_error:.6g}")
    print("  note: finite PD hold smoke only; not a stable-controller claim")

    if check_constraints:
        constraint_result = _check_lower_loop_constraints(
            mujoco,
            model,
            constraint_steps,
            virtual_rod_test,
            virtual_rod_lock_base,
            left_target,
            right_target,
            virtual_rod_control,
            leg_branch,
            ik_search_radius,
            ik_search_samples,
            virtual_rod_length_kp,
            virtual_rod_length_kd,
            virtual_rod_length_ki,
            virtual_rod_length_force_ff,
            virtual_rod_length_integral_limit,
            virtual_rod_length_force_rate_limit,
            virtual_rod_theta_kp,
            virtual_rod_theta_kd,
            virtual_rod_joint_kd,
            virtual_rod_theta_pitch_ff,
            lqr_test,
            lqr_gain_scale,
            lqr_k,
            lqr_x0,
            lqr_u0,
            lqr_auto_design,
            lqr_control_period_steps,
            lqr_x_reference,
            lqr_x_source,
            lqr_x_outer_kp,
            lqr_x_outer_max_v,
            lqr_wheel_sign,
            lqr_pitch_sign,
            lqr_t_limit,
            lqr_tp_limit,
            lqr_output_rate_limit,
            lqr_output_lowpass_hz,
            wheel_ctrl_deadzone,
        )
        print()
        print("lower_loop_constraint_check:")
        print(f"  steps: {constraint_result.steps}")
        print(f"  max_left_error_m: {constraint_result.max_left_error:.6g}")
        print(f"  max_right_error_m: {constraint_result.max_right_error:.6g}")
        print(f"  final_left_error_m: {constraint_result.final_left_error:.6g}")
        print(f"  final_right_error_m: {constraint_result.final_right_error:.6g}")
        print(f"  max_left_branch_violation: {constraint_result.max_left_branch_violation:.6g}")
        print(f"  max_right_branch_violation: {constraint_result.max_right_branch_violation:.6g}")
        print(f"  final_left_branch_violation: {constraint_result.final_left_branch_violation:.6g}")
        print(f"  final_right_branch_violation: {constraint_result.final_right_branch_violation:.6g}")
        print(f"  final_base_height: {constraint_result.final_base_height:.6g}")

    if virtual_rod_test:
        virtual_result = _run_virtual_rod_test(
            mujoco,
            model,
            virtual_rod_steps,
            virtual_rod_lock_base,
            left_target,
            right_target,
            virtual_rod_control,
            leg_branch,
            ik_search_radius,
            ik_search_samples,
            virtual_rod_length_kp,
            virtual_rod_length_kd,
            virtual_rod_length_ki,
            virtual_rod_length_force_ff,
            virtual_rod_length_integral_limit,
            virtual_rod_length_force_rate_limit,
            virtual_rod_theta_kp,
            virtual_rod_theta_kd,
            virtual_rod_joint_kd,
            virtual_rod_theta_pitch_ff,
            lqr_test,
            lqr_gain_scale,
            lqr_k,
            lqr_x0,
            lqr_u0,
            lqr_auto_design,
            lqr_control_period_steps,
            lqr_x_reference,
            lqr_x_source,
            lqr_x_outer_kp,
            lqr_x_outer_max_v,
            lqr_wheel_sign,
            lqr_pitch_sign,
            lqr_t_limit,
            lqr_tp_limit,
            lqr_output_rate_limit,
            lqr_output_lowpass_hz,
            wheel_ctrl_deadzone,
            history_sample_interval,
            trace_control_output,
            trace_control_start_step,
            trace_control_max_steps,
            trace_control_mode,
            trace_control_event_delta,
            trace_control_csv,
            trace_control_plot,
            initial_data=lqr_operating_data,
        )
        print()
        print("virtual_rod_test:")
        print(f"  steps: {virtual_result.steps}")
        print(f"  lock_base: {virtual_result.lock_base}")
        print(f"  control: {virtual_rod_control}")
        print(
            f"  left_target: l={virtual_result.left_target_length:.6g}, "
            f"theta={virtual_result.left_target_theta:.6g}"
        )
        print(
            f"  left_final: l={virtual_result.final_left_length:.6g}, "
            f"theta={virtual_result.final_left_theta:.6g}"
        )
        print(
            f"  right_target: l={virtual_result.right_target_length:.6g}, "
            f"theta={virtual_result.right_target_theta:.6g}"
        )
        print(
            f"  right_final: l={virtual_result.final_right_length:.6g}, "
            f"theta={virtual_result.final_right_theta:.6g}"
        )
        print(f"  max_length_error: {virtual_result.max_length_error:.6g}")
        print(f"  max_theta_error: {virtual_result.max_theta_error:.6g}")
        print(f"  max_left_branch_violation: {virtual_result.max_left_branch_violation:.6g}")
        print(f"  max_right_branch_violation: {virtual_result.max_right_branch_violation:.6g}")
        print(f"  final_left_branch_violation: {virtual_result.final_left_branch_violation:.6g}")
        print(f"  final_right_branch_violation: {virtual_result.final_right_branch_violation:.6g}")
        print(f"  max_abs_ctrl: {virtual_result.max_abs_ctrl:.6g}")
        print(f"  saturated_steps: {virtual_result.saturated_steps}")
        print(f"  max_abs_left_wheel_speed_rad_s: {virtual_result.max_abs_left_wheel_speed:.6g}")
        print(f"  max_abs_right_wheel_speed_rad_s: {virtual_result.max_abs_right_wheel_speed:.6g}")
        print(f"  final_left_wheel_speed_rad_s: {virtual_result.final_left_wheel_speed:.6g}")
        print(f"  final_right_wheel_speed_rad_s: {virtual_result.final_right_wheel_speed:.6g}")
        print(f"  lqr_test: {lqr_test}")
        if virtual_result.final_lqr_state is not None:
            lqr_state = virtual_result.final_lqr_state
            print(f"  lqr_gain_scale: {lqr_gain_scale:.6g}")
            print(f"  lqr_x_reference: {lqr_x_reference:.6g}")
            print(f"  lqr_x_source: {lqr_x_source}")
            print(f"  lqr_x_outer_kp: {lqr_x_outer_kp:.6g}")
            print(f"  lqr_x_outer_max_v: {lqr_x_outer_max_v:.6g}")
            print(f"  lqr_wheel_sign: {lqr_wheel_sign:.6g}")
            print(f"  lqr_pitch_sign: {lqr_pitch_sign:.6g}")
            print(f"  lqr_t_limit: {lqr_t_limit:.6g}")
            print(f"  lqr_tp_limit: {lqr_tp_limit:.6g}")
            print(f"  lqr_output_rate_limit: {lqr_output_rate_limit:.6g}")
            print(f"  lqr_output_lowpass_hz: {lqr_output_lowpass_hz:.6g}")
            print(f"  wheel_ctrl_deadzone: {wheel_ctrl_deadzone:.6g}")
            print("  lqr_k:")
            for row in _format_matrix_rows(lqr_k):
                print(f"    {row}")
            print("  theta_pd_main_disabled: True")
            print("  lqr_state_order: theta_world, dtheta_world, x, dx, phi, dphi")
            print("  lqr_input_order: T, Tp")
            print("  lqr_law: [T, Tp]^T = U0 - lqr_gain_scale * K * (X - X0)")
            print("  lqr_x0: [" + ", ".join(f"{value:.6g}" for value in lqr_x0) + "]")
            print("  lqr_u0: [" + ", ".join(f"{value:.6g}" for value in lqr_u0) + "]")
            print("  lqr_T_limit: +/-lqr_t_limit")
            print("  lqr_Tp_limit: +/-lqr_tp_limit before VMC theta-force split")
            print("  vmc_length_force_limit: 250")
            print("  vmc_theta_force_limit: 80")
            print("  vmc_joint_tau_limit: 45")
            print("  vmc_length_law: F_l = F_l0 + k_l * (L0_slice - L) - d_l * dL")
            print(f"  vmc_length_kp: {virtual_rod_length_kp:.6g}")
            print(f"  vmc_length_kd: {virtual_rod_length_kd:.6g}")
            print(f"  vmc_length_force_ff_or_F_l0: {virtual_rod_length_force_ff:.6g}")
            print(
                "  vmc_gravity_comp_scale: "
                f"{virtual_rod_gravity_comp_scale if virtual_rod_gravity_comp_scale is not None else 'off'}"
            )
            print(f"  vmc_length_force_rate_limit: {virtual_rod_length_force_rate_limit:.6g}")
            if lqr_design_result is not None:
                print("  lqr_design: finite-difference A + measured local B + iterative DARE")
                print(f"  lqr_design_q_diag: {list(lqr_design_result.q_diag)}")
                print(f"  lqr_design_r_diag: {list(lqr_design_result.r_diag)}")
                print(f"  lqr_design_state_eps: {list(lqr_design_result.state_eps)}")
                print(f"  lqr_design_input_eps: {list(lqr_design_result.input_eps)}")
                print(f"  lqr_design_steps: {lqr_design_steps}")
                print(f"  lqr_control_period_steps: {lqr_control_period_steps}")
                print(
                    "  lqr_design_operating_state: "
                    f"theta={lqr_design_result.operating_state.theta:.6g}, "
                    f"theta_rate={lqr_design_result.operating_state.theta_rate:.6g}, "
                    f"x={lqr_design_result.operating_state.x:.6g}, "
                    f"x_rate={lqr_design_result.operating_state.x_rate:.6g}, "
                    f"pitch={lqr_design_result.operating_state.pitch:.6g}, "
                    f"pitch_rate={lqr_design_result.operating_state.pitch_rate:.6g}"
                )
                print(f"  lqr_design_riccati_iterations: {lqr_design_result.riccati_iterations}")
                print(f"  lqr_design_closed_loop_max_abs_eig: {lqr_design_result.closed_loop_max_abs_eig:.6g}")
                print(
                    "  lqr_design_affine_residual: ["
                    + ", ".join(f"{value:.6g}" for value in lqr_design_result.affine_residual)
                    + "]"
                )
                print("  lqr_design_A:")
                for row in _format_matrix_rows(lqr_design_result.a_matrix):
                    print(f"    {row}")
                print("  lqr_design_B:")
                for row in _format_matrix_rows(lqr_design_result.b_matrix):
                    print(f"    {row}")
            print(f"  max_abs_lqr_wheel_torque: {virtual_result.max_abs_lqr_wheel_torque:.6g}")
            print(f"  max_abs_lqr_pitch_torque: {virtual_result.max_abs_lqr_pitch_torque:.6g}")
            print(
                "  final_lqr_state: "
                f"theta={lqr_state.theta:.6g}, "
                f"theta_rate={lqr_state.theta_rate:.6g}, "
                f"x={lqr_state.x:.6g}, "
                f"x_rate={lqr_state.x_rate:.6g}, "
                f"pitch={lqr_state.pitch:.6g}, "
                f"pitch_rate={lqr_state.pitch_rate:.6g}"
            )
        print("  actuator_ctrl_limits:")
        for stats in virtual_result.actuator_ctrl_stats:
            print(
                f"    - {stats.actuator}: min={stats.min_ctrl:.6g}, "
                f"max={stats.max_ctrl:.6g}, "
                f"sat_neg_steps={stats.negative_saturated_steps}, "
                f"sat_pos_steps={stats.positive_saturated_steps}"
            )
        print(f"  final_base_height: {virtual_result.final_base_height:.6g}")
        if lqr_test and virtual_result.final_lqr_state is not None:
            final_state = virtual_result.final_lqr_state
            behavior_passed = (
                abs(final_state.theta) < 0.15
                and abs(final_state.pitch) < 0.10
                and abs(virtual_result.final_left_wheel_speed) < 2.0
                and abs(virtual_result.final_right_wheel_speed) < 2.0
                and abs(virtual_result.final_left_length - virtual_result.left_target_length) < 0.01
                and abs(virtual_result.final_right_length - virtual_result.right_target_length) < 0.01
                and virtual_result.final_left_branch_violation < 1e-9
                and virtual_result.final_right_branch_violation < 1e-9
                and virtual_result.final_base_height > 0.30
            )
            print(f"  behavior_qualified: {behavior_passed}")
        if print_static_operating_point and virtual_result.final_operating_point is not None:
            _print_static_operating_point_sample(
                virtual_result.final_operating_point,
                force_ff=virtual_rod_length_force_ff,
            )
        if history_csv is not None:
            csv_path = _resolve_output_path(history_csv)
            _write_lqr_history_csv(csv_path, virtual_result.history)
            print(f"  history_csv: {csv_path}")
        if history_plot is not None:
            plot_path = _resolve_output_path(history_plot)
            _plot_lqr_history(plot_path, virtual_result.history)
            print(f"  history_plot: {plot_path}")
        if motor_torque_plot is not None:
            plot_path = _resolve_output_path(motor_torque_plot)
            _plot_motor_torque_history(plot_path, virtual_result.history)
            print(f"  motor_torque_plot: {plot_path}")
        print("  note: experimental virtual-rod middle-layer smoke")

    if visualize:
        print()
        visualize_steps = (
            max(1, int(math.ceil(visualize_seconds / float(model.opt.timestep))))
            if visualize_seconds is not None
            else None
        )
        if virtual_rod_test:
            print("visualize: opening MuJoCo viewer for virtual rod test")
            _visualize_virtual_rod_test(
                mujoco,
                model,
                visualize_steps if visualize_steps is not None else virtual_rod_steps,
                virtual_rod_lock_base,
                left_target,
                right_target,
                virtual_rod_control,
                leg_branch,
                ik_search_radius,
                ik_search_samples,
                virtual_rod_length_kp,
                virtual_rod_length_kd,
                virtual_rod_length_ki,
                virtual_rod_length_force_ff,
                virtual_rod_length_integral_limit,
                virtual_rod_length_force_rate_limit,
                virtual_rod_theta_kp,
                virtual_rod_theta_kd,
                virtual_rod_joint_kd,
                virtual_rod_theta_pitch_ff,
                lqr_test,
                lqr_gain_scale,
                lqr_k,
                lqr_x0,
                lqr_u0,
                lqr_design_result,
                lqr_auto_design,
                lqr_control_period_steps,
                lqr_x_reference,
                lqr_x_source,
                lqr_x_outer_kp,
                lqr_x_outer_max_v,
                lqr_wheel_sign,
                lqr_pitch_sign,
                lqr_t_limit,
                lqr_tp_limit,
                lqr_output_rate_limit,
                lqr_output_lowpass_hz,
                wheel_ctrl_deadzone,
                lqr_operating_data,
                realtime,
            )
        else:
            print("visualize: opening MuJoCo viewer for PD hold")
            _visualize_pd_hold(
                mujoco,
                model,
                visualize_steps if visualize_steps is not None else pd_hold_steps,
                pd_kp,
                pd_kd,
                realtime,
            )

    print()
    if behavior_passed:
        print("result: PASS finite model/load/step smoke")
        return 0
    print("result: FAIL behavior recovery criteria")
    return 2


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.lqr_true_equilibrium:
        args.virtual_rod_test = True
        args.equilibrium_search = True
        args.lqr_test = True
        args.lqr_auto_design = True
        args.lqr_use_equilibrium_operating_point = True
        args.zero_steps = 1
        args.probe_steps = 1
        args.pd_hold_steps = 1
        args.equilibrium_init_modes = ("upright-ik",)
        args.equilibrium_l_slices = (args.leg_length,)
        args.equilibrium_theta_refs = (LOCKED_EQUILIBRIUM_THETA,)
        support_scale = support_force_scale_for_length(args.leg_length)
        args.equilibrium_fl0_scales = (support_scale,)
        args.equilibrium_steps = LOCKED_EQUILIBRIUM_STEPS
        args.equilibrium_eval_steps = LOCKED_EQUILIBRIUM_EVAL_STEPS
        args.equilibrium_length_kp = LOCKED_EQUILIBRIUM_LENGTH_KP
        args.equilibrium_length_kd = LOCKED_EQUILIBRIUM_LENGTH_KD
        args.equilibrium_theta_kp = LOCKED_EQUILIBRIUM_THETA_KP
        args.equilibrium_theta_kd = LOCKED_EQUILIBRIUM_THETA_KD
        args.equilibrium_pitch_kp = LOCKED_EQUILIBRIUM_PITCH_KP
        args.equilibrium_pitch_kd = LOCKED_EQUILIBRIUM_PITCH_KD
        args.equilibrium_tp_biases = (LOCKED_EQUILIBRIUM_TP_BIAS,)
        args.equilibrium_wheel_com_kps = (LOCKED_EQUILIBRIUM_WHEEL_COM_KP,)
        args.equilibrium_wheel_dampings = (LOCKED_EQUILIBRIUM_WHEEL_DAMPING,)
        args.equilibrium_init_drop_steps = 0
        args.virtual_rod_length_kp = LOCKED_EQUILIBRIUM_LENGTH_KP
        args.virtual_rod_length_kd = LOCKED_EQUILIBRIUM_LENGTH_KD
        args.virtual_rod_length_force_ff = support_scale * 50.7738
        args.virtual_rod_gravity_comp_scale = None
        args.virtual_rod_length_delta = None
        args.virtual_rod_theta_target = LOCKED_EQUILIBRIUM_THETA
        args.left_rod_length = args.leg_length
        args.right_rod_length = args.leg_length
        args.left_rod_theta = LOCKED_EQUILIBRIUM_THETA
        args.right_rod_theta = LOCKED_EQUILIBRIUM_THETA
        args.lqr_design_steps = LOCKED_LQR_DESIGN_STEPS
        args.lqr_control_period_steps = LOCKED_LQR_CONTROL_PERIOD_STEPS
    if args.wheel_balance_only:
        args.virtual_rod_test = True
        args.lqr_test = True
        args.lqr_auto_design = True
        args.lqr_use_equilibrium_operating_point = True
        args.equilibrium_search = True
        args.virtual_rod_control = "off"
    model_path = args.model
    if not model_path.is_absolute():
        model_path = PROJECT_ROOT / model_path
    if not model_path.exists():
        parser.error(f"model file not found: {model_path}")
    if args.zero_steps <= 0 or args.probe_steps <= 0:
        parser.error("step counts must be positive")
    if args.pd_hold_steps <= 0:
        parser.error("--pd-hold-steps must be positive")
    if args.ctrl <= 0.0:
        parser.error("--ctrl must be positive")
    if args.scan_virtual_rod_sample <= 0.0:
        parser.error("--scan-virtual-rod-sample must be positive")
    if args.pd_kp < 0.0 or args.pd_kd < 0.0:
        parser.error("--pd-kp and --pd-kd must be non-negative")
    if args.visualize_seconds is not None and args.visualize_seconds <= 0.0:
        parser.error("--visualize-seconds must be positive")
    if args.virtual_rod_steps <= 0:
        parser.error("--virtual-rod-steps must be positive")
    if args.leg_length <= 0.0:
        parser.error("--leg-length must be positive")
    if args.constraint_steps <= 0:
        parser.error("--constraint-steps must be positive")
    if args.ik_search_radius < 0.0:
        parser.error("--ik-search-radius must be non-negative")
    if args.ik_search_samples <= 0:
        parser.error("--ik-search-samples must be positive")
    lqr_auto_design = bool(args.lqr_auto_design or (args.lqr_test and args.lqr_k is None))
    lqr_wheel_sign = args.lqr_wheel_sign
    lqr_pitch_sign = args.lqr_pitch_sign
    lqr_output_rate_limit = args.lqr_output_rate_limit
    if lqr_auto_design and args.lqr_t_limit == 16.0:
        args.lqr_t_limit = 8.0
    lqr_tp_limit = args.lqr_tp_limit
    if lqr_tp_limit is None:
        lqr_tp_limit = args.lqr_t_limit
    lqr_gain_scale = args.lqr_gain_scale
    if lqr_gain_scale is None:
        lqr_gain_scale = 1.0
    if lqr_gain_scale < 0.0:
        parser.error("--lqr-gain-scale must be non-negative")
    lqr_k = DEFAULT_LQR_K if args.lqr_k is None else np.array(args.lqr_k, dtype=float).reshape(2, 6)
    lqr_x0 = np.array(args.lqr_x0, dtype=float)
    lqr_u0 = np.array(args.lqr_u0, dtype=float)
    lqr_q_diag = np.array(args.lqr_q_diag, dtype=float)
    lqr_r_diag = np.array(args.lqr_r_diag, dtype=float)
    lqr_state_eps = np.array(args.lqr_state_eps, dtype=float)
    lqr_input_eps = np.array(args.lqr_input_eps, dtype=float)
    lqr_design_steps = args.lqr_design_steps
    if lqr_design_steps is None:
        lqr_design_steps = 1
    lqr_control_period_steps = args.lqr_control_period_steps
    if lqr_control_period_steps is None:
        lqr_control_period_steps = 1
    if lqr_design_steps <= 0 or lqr_control_period_steps <= 0:
        parser.error("--lqr-design-steps and --lqr-control-period-steps must be positive")
    lqr_x_source = args.lqr_x_source
    if lqr_x_source is None:
        lqr_x_source = "wheel"
    if np.any(lqr_q_diag <= 0.0) or np.any(lqr_r_diag <= 0.0):
        parser.error("--lqr-q-diag and --lqr-r-diag must be positive")
    if np.any(lqr_state_eps <= 0.0) or np.any(lqr_input_eps <= 0.0):
        parser.error("--lqr-state-eps and --lqr-input-eps must be positive")
    if not np.all(np.isfinite(lqr_x0)) or not np.all(np.isfinite(lqr_u0)):
        parser.error("--lqr-x0 and --lqr-u0 must be finite")
    if args.lqr_x_outer_kp < 0.0 or args.lqr_x_outer_max_v < 0.0:
        parser.error("--lqr-x-outer-kp and --lqr-x-outer-max-v must be non-negative")
    if args.lqr_output_lowpass_hz < 0.0:
        parser.error("--lqr-output-lowpass-hz must be non-negative")
    if args.wheel_ctrl_deadzone < 0.0:
        parser.error("--wheel-ctrl-deadzone must be non-negative")
    if min(
        args.lqr_t_limit,
        lqr_tp_limit,
        lqr_output_rate_limit,
    ) < 0.0:
        parser.error("LQR torque limits and rate limit must be non-negative")
    if args.history_sample_interval <= 0:
        parser.error("--history-sample-interval must be positive")
    if args.trace_control_start_step < 0:
        parser.error("--trace-control-start-step must be non-negative")
    if args.trace_control_max_steps < 0:
        parser.error("--trace-control-max-steps must be non-negative; use 0 for unlimited")
    if args.trace_control_event_delta < 0.0:
        parser.error("--trace-control-event-delta must be non-negative")
    if (args.trace_control_csv is not None or args.trace_control_plot is not None) and not args.trace_control_output:
        parser.error("--trace-control-csv/--trace-control-plot require --trace-control-output")
    if args.trace_control_output and not (args.virtual_rod_test or args.lqr_test):
        parser.error("--trace-control-output requires --virtual-rod-test or --lqr-test")
    if (args.history_csv is not None or args.history_plot is not None or args.motor_torque_plot is not None) and not args.lqr_test:
        parser.error("--history-csv/--history-plot/--motor-torque-plot require --lqr-test")
    if args.fl_channel_steps <= 0 or args.fl_pulse_settle_steps <= 0 or args.fl_pulse_steps <= 0:
        parser.error("F_l channel/pulse step counts must be positive")
    if args.fl_pulse_delta_force <= 0.0:
        parser.error("--fl-pulse-delta-force must be positive")
    if args.diagnostics_only and not (
        args.fl_channel_test
        or args.fl_pulse_test
        or args.fivebar_kinematics_check
        or args.fivebar_jacobian_check
        or args.equilibrium_static_pose_check
        or args.equilibrium_search
    ):
        parser.error(
            "--diagnostics-only requires --fl-channel-test, --fl-pulse-test, "
            "--fivebar-kinematics-check, --fivebar-jacobian-check, "
            "--equilibrium-static-pose-check, or --equilibrium-search"
        )
    if args.fivebar_kinematics_check and any(value <= 0.0 for value in args.fivebar_kinematics_l_slices):
        parser.error("--fivebar-kinematics-l-slices must be positive")
    if args.fivebar_jacobian_check:
        if any(value <= 0.0 for value in args.fivebar_jacobian_l_slices):
            parser.error("--fivebar-jacobian-l-slices must be positive")
        if args.fivebar_jacobian_force_scale <= 0.0:
            parser.error("--fivebar-jacobian-force-scale must be positive")
        if args.fivebar_jacobian_load_steps < 0:
            parser.error("--fivebar-jacobian-load-steps must be non-negative")
    if args.equilibrium_static_pose_check or args.equilibrium_search:
        if args.equilibrium_steps <= 0 or args.equilibrium_eval_steps <= 0:
            parser.error("--equilibrium-steps and --equilibrium-eval-steps must be positive")
        if args.equilibrium_init_drop_steps < 0:
            parser.error("--equilibrium-init-drop-steps must be non-negative")
        if any(value <= 0.0 for value in args.equilibrium_l_slices):
            parser.error("--equilibrium-l-slices must be positive")
        if any(value <= 0.0 for value in args.equilibrium_fl0_scales):
            parser.error("--equilibrium-fl0-scales must be positive")
        if any(value < 0.0 for value in args.equilibrium_wheel_com_kps):
            parser.error("--equilibrium-wheel-com-kps must be non-negative")
        if any(value < 0.0 for value in args.equilibrium_wheel_dampings):
            parser.error("--equilibrium-wheel-dampings must be non-negative")
        if min(
            args.equilibrium_length_kp,
            args.equilibrium_length_kd,
            args.equilibrium_theta_kp,
            args.equilibrium_theta_kd,
            args.equilibrium_pitch_kp,
            args.equilibrium_pitch_kd,
        ) < 0.0:
            parser.error("equilibrium search gains and damping must be non-negative")
    virtual_rod_length_kp = args.virtual_rod_length_kp
    virtual_rod_length_kd = args.virtual_rod_length_kd
    motor_servo_kp = args.motor_servo_kp if args.motor_servo_kp is not None else virtual_rod_length_kp
    motor_servo_kd = args.motor_servo_kd if args.motor_servo_kd is not None else virtual_rod_length_kd
    if min(
        motor_servo_kp,
        motor_servo_kd,
        args.virtual_rod_length_ki,
        abs(args.virtual_rod_length_force_ff),
        0.0 if args.virtual_rod_gravity_comp_scale is None else args.virtual_rod_gravity_comp_scale,
        args.virtual_rod_length_integral_limit,
        args.virtual_rod_length_force_rate_limit,
        args.virtual_rod_theta_kp,
        args.virtual_rod_theta_kd,
        args.virtual_rod_joint_kd,
        abs(args.virtual_rod_theta_pitch_ff),
    ) < 0.0:
        parser.error("virtual rod gains must be non-negative")
    use_virtual_rod_test = args.virtual_rod_test or args.lqr_test
    return run_smoke(
        model_path,
        args.zero_steps,
        args.probe_steps,
        args.ctrl,
        args.pd_hold_steps,
        args.pd_kp,
        args.pd_kd,
        args.visualize,
        args.print_virtual_leg,
        args.scan_virtual_rod,
        args.scan_virtual_rod_dynamic,
        args.scan_virtual_rod_sample,
        args.check_constraints,
        args.constraint_steps,
        use_virtual_rod_test,
        args.lock_base,
        args.virtual_rod_steps,
        args.virtual_rod_length_delta,
        args.virtual_rod_theta_target,
        args.left_rod_length,
        args.right_rod_length,
        args.left_rod_theta,
        args.right_rod_theta,
        args.virtual_rod_control,
        args.leg_branch,
        args.ik_search_radius,
        args.ik_search_samples,
        motor_servo_kp,
        motor_servo_kd,
        args.virtual_rod_length_ki,
        args.virtual_rod_length_force_ff,
        args.virtual_rod_gravity_comp_scale,
        args.virtual_rod_length_integral_limit,
        args.virtual_rod_length_force_rate_limit,
        args.virtual_rod_theta_kp,
        args.virtual_rod_theta_kd,
        args.virtual_rod_joint_kd,
        args.virtual_rod_theta_pitch_ff,
        args.lqr_test,
        lqr_gain_scale,
        lqr_k,
        lqr_x0,
        lqr_u0,
        lqr_auto_design,
        lqr_q_diag,
        lqr_r_diag,
        lqr_state_eps,
        lqr_input_eps,
        lqr_design_steps,
        lqr_control_period_steps,
        args.lqr_x_reference,
        lqr_x_source,
        args.lqr_x_outer_kp,
        args.lqr_x_outer_max_v,
        lqr_wheel_sign,
        lqr_pitch_sign,
        args.lqr_t_limit,
        lqr_tp_limit,
        lqr_output_rate_limit,
        args.lqr_output_lowpass_hz,
        args.wheel_ctrl_deadzone,
        args.history_sample_interval,
        args.history_csv,
        args.history_plot,
        args.motor_torque_plot,
        args.trace_control_output,
        args.trace_control_start_step,
        args.trace_control_max_steps,
        args.trace_control_mode,
        args.trace_control_event_delta,
        args.trace_control_csv,
        args.trace_control_plot,
        args.visualize_seconds,
        args.fl_channel_test,
        tuple(float(value) for value in args.fl_channel_forces),
        args.fl_channel_steps,
        args.fl_pulse_test,
        args.fl_pulse_base_force,
        args.fl_pulse_delta_force,
        args.fl_pulse_settle_steps,
        args.fl_pulse_steps,
        args.fivebar_kinematics_check,
        tuple(float(value) for value in args.fivebar_kinematics_l_slices),
        tuple(float(value) for value in args.fivebar_kinematics_theta_refs),
        args.fivebar_jacobian_check,
        tuple(float(value) for value in args.fivebar_jacobian_l_slices),
        tuple(float(value) for value in args.fivebar_jacobian_theta_refs),
        args.fivebar_jacobian_force_scale,
        args.fivebar_jacobian_load_steps,
        args.equilibrium_static_pose_check,
        args.equilibrium_search,
        tuple(args.equilibrium_init_modes),
        tuple(float(value) for value in args.equilibrium_l_slices),
        tuple(float(value) for value in args.equilibrium_theta_refs),
        tuple(float(value) for value in args.equilibrium_fl0_scales),
        tuple(float(value) for value in args.equilibrium_tp_biases),
        tuple(float(value) for value in args.equilibrium_wheel_com_kps),
        tuple(float(value) for value in args.equilibrium_wheel_dampings),
        args.equilibrium_init_drop_steps,
        args.equilibrium_steps,
        args.equilibrium_eval_steps,
        args.equilibrium_length_kp,
        args.equilibrium_length_kd,
        args.equilibrium_theta_kp,
        args.equilibrium_theta_kd,
        args.equilibrium_pitch_kp,
        args.equilibrium_pitch_kd,
        args.lqr_use_equilibrium_operating_point,
        args.print_static_operating_point,
        args.diagnostics_only,
        not args.no_realtime,
    )




