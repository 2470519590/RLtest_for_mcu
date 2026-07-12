"""CSV and plot output helpers for smoke diagnostics."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from ..core.constants import PROJECT_ROOT
from ..core.types import ControlTraceSample, LqrHistorySample


def resolve_output_path(path: Path) -> Path:
    return path if path.is_absolute() else PROJECT_ROOT / path


def write_control_trace_csv(path: Path, samples: tuple[ControlTraceSample, ...]) -> None:
    if not samples:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(ControlTraceSample.__dataclass_fields__.keys())
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow({field: getattr(sample, field) for field in fieldnames})


def plot_control_trace(path: Path, samples: tuple[ControlTraceSample, ...]) -> None:
    if not samples:
        return
    import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel

    path.parent.mkdir(parents=True, exist_ok=True)
    time_s = np.array([sample.time_s for sample in samples], dtype=float)
    max_ctrl_delta = np.array([sample.max_ctrl_delta for sample in samples], dtype=float)
    d_t = np.array([sample.dT for sample in samples], dtype=float)
    d_tp = np.array([sample.dTp for sample in samples], dtype=float)
    d_fl_left = np.array([sample.dF_l_left for sample in samples], dtype=float)
    d_fl_right = np.array([sample.dF_l_right for sample in samples], dtype=float)
    d_ftheta_left = np.array([sample.dF_theta_left for sample in samples], dtype=float)
    d_ftheta_right = np.array([sample.dF_theta_right for sample in samples], dtype=float)
    max_raw_tau_delta = np.array([sample.max_raw_tau_delta for sample in samples], dtype=float)
    left_fl = np.array([sample.F_l_left for sample in samples], dtype=float)
    right_fl = np.array([sample.F_l_right for sample in samples], dtype=float)
    left_ftheta = np.array([sample.F_theta_left for sample in samples], dtype=float)
    right_ftheta = np.array([sample.F_theta_right for sample in samples], dtype=float)
    max_abs_ctrl = np.array([sample.max_abs_ctrl for sample in samples], dtype=float)
    source_lqr = np.array([sample.source_lqr for sample in samples], dtype=float)
    source_vmc = np.array([sample.source_vmc for sample in samples], dtype=float)
    source_clip = np.array([sample.source_clip for sample in samples], dtype=float)
    source_viewer = np.array([sample.source_viewer_possible for sample in samples], dtype=float)

    figure, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    axes[0].plot(time_s, max_ctrl_delta, label="max_ctrl_delta", color="tab:red")
    axes[0].set_ylabel("ctrl step")
    axes[0].legend(loc="best")
    axes[0].grid(True)

    axes[1].plot(time_s, d_t, label="dT")
    axes[1].plot(time_s, d_tp, label="dTp")
    axes[1].set_ylabel("N*m/step")
    axes[1].legend(loc="best")
    axes[1].grid(True)

    axes[2].plot(time_s, d_fl_left, label="left dF_l")
    axes[2].plot(time_s, d_fl_right, label="right dF_l")
    axes[2].plot(time_s, d_ftheta_left, label="left dF_theta")
    axes[2].plot(time_s, d_ftheta_right, label="right dF_theta")
    axes[2].set_ylabel("N/step")
    axes[2].legend(loc="best")
    axes[2].grid(True)

    axes[3].plot(time_s, left_fl, label="left F_l")
    axes[3].plot(time_s, right_fl, label="right F_l")
    axes[3].plot(time_s, left_ftheta, label="left F_theta")
    axes[3].plot(time_s, right_ftheta, label="right F_theta")
    axes[3].set_ylabel("N")
    axes[3].legend(loc="best")
    axes[3].grid(True)

    axes[4].plot(time_s, max_raw_tau_delta, label="max_raw_tau_delta")
    axes[4].plot(time_s, max_abs_ctrl, label="max_abs_ctrl")
    axes[4].set_ylabel("tau/ctrl")
    axes[4].legend(loc="best")
    axes[4].grid(True)

    axes[5].step(time_s, source_lqr, where="post", label="LQR/middle")
    axes[5].step(time_s, source_vmc + 1.2, where="post", label="VMC")
    axes[5].step(time_s, source_clip + 2.4, where="post", label="clip")
    axes[5].step(time_s, source_viewer + 3.6, where="post", label="viewer_possible")
    axes[5].set_yticks([0.5, 1.7, 2.9, 4.1])
    axes[5].set_yticklabels(["LQR", "VMC", "clip", "viewer"])
    axes[5].set_xlabel("time (s)")
    axes[5].legend(loc="best")
    axes[5].grid(True)

    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def write_lqr_history_csv(path: Path, samples: tuple[LqrHistorySample, ...]) -> None:
    if not samples:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = (
        "time_s",
        "theta",
        "theta_rate",
        "x",
        "x_rate",
        "pitch",
        "pitch_rate",
        "wheel_torque",
        "pitch_torque",
    )
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow({field: getattr(sample, field) for field in fieldnames})


def plot_lqr_history(path: Path, samples: tuple[LqrHistorySample, ...]) -> None:
    if not samples:
        return
    import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel

    path.parent.mkdir(parents=True, exist_ok=True)
    time_s = np.array([sample.time_s for sample in samples], dtype=float)
    theta = np.array([sample.theta for sample in samples], dtype=float)
    theta_rate = np.array([sample.theta_rate for sample in samples], dtype=float)
    x = np.array([sample.x for sample in samples], dtype=float)
    x_rate = np.array([sample.x_rate for sample in samples], dtype=float)
    pitch = np.array([sample.pitch for sample in samples], dtype=float)
    pitch_rate = np.array([sample.pitch_rate for sample in samples], dtype=float)
    wheel_torque = np.array([sample.wheel_torque for sample in samples], dtype=float)
    pitch_torque = np.array([sample.pitch_torque for sample in samples], dtype=float)
    left_length = np.array([sample.left_length for sample in samples], dtype=float)
    right_length = np.array([sample.right_length for sample in samples], dtype=float)
    left_length_rate = np.array([sample.left_length_rate for sample in samples], dtype=float)
    right_length_rate = np.array([sample.right_length_rate for sample in samples], dtype=float)
    left_length_force = np.array([sample.left_length_force for sample in samples], dtype=float)
    right_length_force = np.array([sample.right_length_force for sample in samples], dtype=float)

    figure, axes = plt.subplots(8, 1, figsize=(10, 18), sharex=True)
    axes[0].plot(time_s, theta, label="theta")
    axes[0].plot(time_s, pitch, label="phi")
    axes[0].set_ylabel("angle (rad)")
    axes[0].legend(loc="best")
    axes[0].grid(True)

    axes[1].plot(time_s, theta_rate, label="dtheta")
    axes[1].plot(time_s, pitch_rate, label="dphi")
    axes[1].set_ylabel("rate (rad/s)")
    axes[1].legend(loc="best")
    axes[1].grid(True)

    axes[2].plot(time_s, x, label="x")
    axes[2].set_ylabel("position (m)")
    axes[2].legend(loc="best")
    axes[2].grid(True)

    axes[3].plot(time_s, x_rate, label="dx")
    axes[3].set_ylabel("velocity (m/s)")
    axes[3].legend(loc="best")
    axes[3].grid(True)

    axes[4].plot(time_s, wheel_torque, label="T")
    axes[4].plot(time_s, pitch_torque, label="Tp")
    axes[4].set_ylabel("torque (N*m)")
    axes[4].set_xlabel("time (s)")
    axes[4].legend(loc="best")
    axes[4].grid(True)

    axes[5].plot(time_s, left_length_force, label="left F_l")
    axes[5].plot(time_s, right_length_force, label="right F_l")
    axes[5].set_ylabel("support (N)")
    axes[5].legend(loc="best")
    axes[5].grid(True)

    axes[6].plot(time_s, left_length, label="left L")
    axes[6].plot(time_s, right_length, label="right L")
    axes[6].set_ylabel("leg length (m)")
    axes[6].legend(loc="best")
    axes[6].grid(True)

    axes[7].plot(time_s, left_length_rate, label="left dL")
    axes[7].plot(time_s, right_length_rate, label="right dL")
    axes[7].set_ylabel("leg rate (m/s)")
    axes[7].set_xlabel("time (s)")
    axes[7].legend(loc="best")
    axes[7].grid(True)

    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def plot_motor_torque_history(path: Path, samples: tuple[LqrHistorySample, ...]) -> None:
    if not samples:
        return
    import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel

    path.parent.mkdir(parents=True, exist_ok=True)
    time_s = np.array([sample.time_s for sample in samples], dtype=float)
    left_front = np.array([sample.left_front_motor_tau for sample in samples], dtype=float)
    left_rear = np.array([sample.left_rear_motor_tau for sample in samples], dtype=float)
    right_front = np.array([sample.right_front_motor_tau for sample in samples], dtype=float)
    right_rear = np.array([sample.right_rear_motor_tau for sample in samples], dtype=float)
    left_wheel = np.array([sample.left_wheel_motor_tau for sample in samples], dtype=float)
    right_wheel = np.array([sample.right_wheel_motor_tau for sample in samples], dtype=float)
    per_wheel_target = np.array([0.5 * sample.wheel_torque for sample in samples], dtype=float)

    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(time_s, left_front, label="left_front_motor")
    axes[0].plot(time_s, left_rear, label="left_rear_motor")
    axes[0].set_ylabel("leg tau (N*m)")
    axes[0].legend(loc="best")
    axes[0].grid(True)

    axes[1].plot(time_s, right_front, label="right_front_motor")
    axes[1].plot(time_s, right_rear, label="right_rear_motor")
    axes[1].set_ylabel("leg tau (N*m)")
    axes[1].legend(loc="best")
    axes[1].grid(True)

    axes[2].plot(time_s, left_wheel, label="left_wheel_motor")
    axes[2].plot(time_s, right_wheel, label="right_wheel_motor")
    axes[2].plot(time_s, per_wheel_target, "--", label="0.5*T target")
    axes[2].set_ylabel("wheel tau (N*m)")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="best")
    axes[2].grid(True)

    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)
