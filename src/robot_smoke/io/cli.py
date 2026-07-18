"""Thin command-line interface for the smoke runner.

Only stable user-facing switches are exposed here. Diagnostic and tuning
values are locked as internal defaults so the main smoke path stays readable.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from ..core.constants import (
    DEFAULT_MODEL,
    LOCKED_EQUILIBRIUM_EVAL_STEPS,
    LOCKED_EQUILIBRIUM_FL0,
    LOCKED_EQUILIBRIUM_FL0_SCALE,
    LOCKED_EQUILIBRIUM_L0,
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
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MuJoCo wheel-leg smoke runner.")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--visualize-seconds", type=float)
    parser.add_argument("--virtual-rod-steps", type=int, default=800)
    parser.add_argument(
        "--length-schedule",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="enable/disable length-scheduled K/F_l0 table",
    )
    parser.add_argument("--length-schedule-path", type=Path)
    parser.add_argument("--initial-leg-length", type=float, help="initial commanded leg length for scheduled tests")
    parser.add_argument("--startup-ramp-seconds", type=float, help="seconds reserved for initial leg-length ramp")
    parser.add_argument(
        "--leg-length",
        type=float,
        default=LOCKED_EQUILIBRIUM_L0,
        help="fixed leg-length slice; equilibrium and LQR are rebuilt for this value",
    )
    parser.add_argument(
        "--lqr-true-equilibrium",
        action="store_true",
        help="use the locked 0.35 m true-equilibrium operating point for auto LQR",
    )
    parser.add_argument(
        "--wheel-balance-only",
        action="store_true",
        help="diagnostic mode: disable all leg actuator control and Tp; keep wheel T only",
    )
    parser.add_argument("--diagnostics-only", action="store_true")
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument(
        "--impact",
        dest="impact_level",
        choices=("small", "medium"),
        help="apply one fixed horizontal base impact",
    )
    parser.add_argument("--speed-profile", choices=("low", "medium", "high"))
    parser.add_argument("--turn", dest="turn_direction", choices=("left", "right"))
    parser.add_argument("--turn-speed", choices=("low", "medium", "high"), default="high")
    parser.add_argument("--turn-test", action="store_true")
    parser.add_argument("--turn-length-sine-test", action="store_true", help="高速原地旋转，同时腿长在允许范围内做正弦跟踪")
    parser.add_argument("--turn-drive-test", choices=("low", "high"))
    parser.add_argument("--roll-test", action="store_true", help="中速依次通过左、右单轮三角坡")
    parser.add_argument("--flight-test", action="store_true", help="高速全宽飞坡，并启用论文第 3 节离地检测")
    parser.add_argument("--slope-roll-turn-test", action="store_true", help="复用飞坡场地：先中速前进到坡上，再中速原地旋转")
    parser.add_argument("--slope-roll-turn-start-time", type=float, help="斜坡 ROLL 原地旋转开始时间，单位 s")
    parser.add_argument("--jump-test", action="store_true", help="原地跳跃：腿长从最小值瞬间拉到最大值，并保留离地检测")
    parser.add_argument(
        "--forward-jump-test",
        choices=("low", "medium", "high"),
        help="前进匀速阶段触发跳跃；触发条件为双腿世界竖直角均小于 3 度",
    )
    parser.add_argument("--turn-pd-plot", action="store_true")
    parser.add_argument("--roll-length-plot", action="store_true")
    parser.add_argument("--lqr-debug-plot", action="store_true")
    parser.add_argument(
        "--leg-height-test",
        action="store_true",
        help="平衡模式下按低/中/高三档切换腿长目标，不重新搜索工作点",
    )
    parser.add_argument("--length-kd", type=float, help="覆盖腿长微分增益")
    parser.add_argument("--length-ki", type=float, help="覆盖腿长积分增益")
    parser.add_argument("--length-integral-limit", type=float, help="覆盖腿长积分状态限幅")
    parser.add_argument("--length-force-ff", type=float, help="覆盖每腿沿虚拟腿前馈支撑力，单位 N")
    parser.add_argument("--yaw-turn-kp", type=float)
    parser.add_argument("--yaw-turn-kd", type=float)
    parser.add_argument("--leg-sync-kp", type=float)
    parser.add_argument("--leg-sync-kd", type=float)

    parser.set_defaults(
        zero_steps=200,
        probe_steps=40,
        ctrl=0.05,
        pd_hold_steps=500,
        pd_kp=12.0,
        pd_kd=0.6,
        print_virtual_leg=False,
        scan_virtual_rod=False,
        scan_virtual_rod_dynamic=False,
        scan_virtual_rod_sample=0.4,
        check_constraints=False,
        constraint_steps=800,
        virtual_rod_test=False,
        lock_base=False,
        virtual_rod_length_delta=None,
        virtual_rod_theta_target=0.0,
        left_rod_length=None,
        right_rod_length=None,
        left_rod_theta=None,
        right_rod_theta=None,
        virtual_rod_control="vmc",
        leg_branch="normal",
        ik_search_radius=0.25,
        ik_search_samples=5,
        motor_servo_kp=None,
        motor_servo_kd=None,
        virtual_rod_length_kp=LOCKED_EQUILIBRIUM_LENGTH_KP,
        virtual_rod_length_kd=0.0,
        virtual_rod_length_ki=0.0,
        virtual_rod_length_force_ff=LOCKED_EQUILIBRIUM_FL0,
        virtual_rod_gravity_comp_scale=None,
        virtual_rod_length_integral_limit=0.10,
        virtual_rod_length_force_rate_limit=0.0,
        virtual_rod_theta_kp=50.0,
        virtual_rod_theta_kd=10.0,
        virtual_rod_joint_kd=8.0,
        virtual_rod_theta_pitch_ff=0.0,
        lqr_test=False,
        lqr_gain_scale=None,
        lqr_k=None,
        lqr_x0=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        lqr_u0=(0.0, 0.0),
        lqr_auto_design=False,
        lqr_use_equilibrium_operating_point=False,
        lqr_q_diag=(1.0, 0.5, 10.0, 30.0, 25.0, 25.0),
        # Tp is an internal body-leg torque.  In a free-base contact model it
        # must not replace wheel torque as the primary inverted-pendulum input.
        lqr_r_diag=(0.1, 10.0),
        lqr_state_eps=(0.012, 0.25, 0.01, 0.2, 0.012, 0.25),
        lqr_input_eps=(0.5, 0.5),
        lqr_design_steps=None,
        lqr_control_period_steps=None,
        lqr_x_reference=0.0,
        lqr_x_source=None,
        lqr_x_outer_kp=0.0,
        lqr_x_outer_max_v=0.35,
        lqr_wheel_sign=1.0,
        lqr_pitch_sign=-1.0,
        lqr_t_limit=16.0,
        lqr_tp_limit=2.0,
        landing_hold_t_limit=2.0,
        lqr_output_rate_limit=1000.0,
        lqr_output_lowpass_hz=0.0,
        wheel_ctrl_deadzone=0.0,
        history_sample_interval=5,
        length_schedule=True,
        length_schedule_path=Path("config") / "length_schedule.yaml",
        minimum_leg_length=0.16,
        maximum_leg_length=0.30,
        initial_leg_length=None,
        startup_ramp_seconds=0.0,
        turn_pd_plot_path=None,
        roll_length_plot_path=None,
        lqr_debug_plot_path=None,
        fl_channel_test=False,
        fl_channel_forces=(-100.0, -50.0, 0.0, 50.0, 100.0, 150.0),
        fl_channel_steps=1500,
        fl_pulse_test=False,
        fl_pulse_base_force=LOCKED_EQUILIBRIUM_FL0,
        fl_pulse_delta_force=20.0,
        fl_pulse_settle_steps=1500,
        fl_pulse_steps=100,
        fivebar_kinematics_check=False,
        fivebar_kinematics_l_slices=(0.35, 0.38, 0.395),
        fivebar_kinematics_theta_refs=(0.0,),
        fivebar_jacobian_check=False,
        fivebar_jacobian_l_slices=(0.35, 0.38, 0.395),
        fivebar_jacobian_theta_refs=(0.0,),
        fivebar_jacobian_force_scale=1.0,
        fivebar_jacobian_load_steps=200,
        equilibrium_static_pose_check=False,
        equilibrium_search=False,
        equilibrium_init_modes=("upright-ik",),
        equilibrium_l_slices=(LOCKED_EQUILIBRIUM_L0,),
        equilibrium_theta_refs=(LOCKED_EQUILIBRIUM_THETA,),
        equilibrium_fl0_scales=(LOCKED_EQUILIBRIUM_FL0_SCALE,),
        equilibrium_steps=LOCKED_EQUILIBRIUM_STEPS,
        equilibrium_eval_steps=LOCKED_EQUILIBRIUM_EVAL_STEPS,
        equilibrium_length_kp=LOCKED_EQUILIBRIUM_LENGTH_KP,
        equilibrium_length_kd=0.0,
        equilibrium_theta_kp=LOCKED_EQUILIBRIUM_THETA_KP,
        equilibrium_theta_kd=LOCKED_EQUILIBRIUM_THETA_KD,
        equilibrium_pitch_kp=LOCKED_EQUILIBRIUM_PITCH_KP,
        equilibrium_pitch_kd=LOCKED_EQUILIBRIUM_PITCH_KD,
        equilibrium_tp_biases=(LOCKED_EQUILIBRIUM_TP_BIAS,),
        equilibrium_wheel_com_kps=(LOCKED_EQUILIBRIUM_WHEEL_COM_KP,),
        equilibrium_wheel_dampings=(LOCKED_EQUILIBRIUM_WHEEL_DAMPING,),
        equilibrium_init_drop_steps=0,
        print_static_operating_point=False,
        impact_level=None,
        use_locked_equilibrium=False,
        speed_profile=None,
        turn_direction=None,
        turn_test=False,
        turn_length_sine_test=False,
        turn_drive_test=None,
        leg_length_sine_test=False,
        leg_length_sine_period=1.5,
        roll_test=False,
        flight_test=False,
        slope_roll_turn_test=False,
        slope_roll_turn_start_time=2.3,
        jump_test=False,
        forward_jump_test=None,
        flight_detection_enabled=False,
        flight_airborne_force_threshold=20.0,
        flight_airborne_confirm_seconds=0.05,
        flight_airborne_rearm_seconds=1.0,
        turn_pd_plot=False,
        lqr_debug_plot=False,
        yaw_turn_kp=None,
        yaw_turn_kd=None,
        leg_sync_kp=None,
        leg_sync_kd=None,
        length_kd=None,
        length_ki=None,
        length_integral_limit=None,
        length_force_ff=None,
    )
    return parser
