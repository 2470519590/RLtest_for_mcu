"""Turn-controller plot output."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ..core.constants import PROJECT_ROOT
from ..core.types import LqrHistorySample


def resolve_output_path(path: Path) -> Path:
    return path if path.is_absolute() else PROJECT_ROOT / path


def _series(samples: tuple[LqrHistorySample, ...], name: str, mask: np.ndarray) -> np.ndarray:
    return np.array([getattr(sample, name) for sample in samples], dtype=float)[mask]


def plot_turning_history(
    path: Path,
    samples: tuple[LqrHistorySample, ...],
    yaw_kp: float,
    yaw_kd: float,
    sync_kp: float,
    sync_kd: float,
) -> None:
    """Plot raw/filter signals and P/D contributions for the turn controller."""
    if not samples:
        return
    import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel

    path.parent.mkdir(parents=True, exist_ok=True)
    raw_time_s = np.array([sample.time_s for sample in samples], dtype=float)
    mask = raw_time_s <= 6.0
    time_s = raw_time_s[mask]
    yaw_ref = _series(samples, "yaw_rate_reference", mask)
    yaw_raw = _series(samples, "yaw_rate", mask)
    yaw_filtered = _series(samples, "yaw_rate_filtered", mask)
    yaw_d_raw = _series(samples, "yaw_error_rate_raw", mask)
    yaw_d_filtered = _series(samples, "yaw_error_rate", mask)
    yaw_p = _series(samples, "yaw_p_torque", mask)
    yaw_d = _series(samples, "yaw_d_torque", mask)
    yaw_total = _series(samples, "turn_torque", mask)
    theta_left = _series(samples, "left_theta", mask)
    theta_right = _series(samples, "right_theta", mask)
    sync_raw = _series(samples, "sync_error_raw", mask)
    sync_filtered = _series(samples, "sync_error", mask)
    sync_d_raw = _series(samples, "sync_error_rate_raw", mask)
    sync_d_filtered = _series(samples, "sync_error_rate", mask)
    sync_p = _series(samples, "sync_p_torque", mask)
    sync_d = _series(samples, "sync_d_torque", mask)
    sync_total = _series(samples, "sync_torque", mask)

    figure, axes = plt.subplots(6, 1, figsize=(10, 14), sharex=True)
    axes[0].plot(time_s, yaw_ref, label="yaw_rate ref")
    axes[0].plot(time_s, yaw_raw, label="yaw_rate raw")
    axes[0].plot(time_s, yaw_filtered, "--", label="yaw_rate input LPF")
    axes[0].set_ylabel("rad/s")
    axes[1].plot(time_s, yaw_d_raw, alpha=0.55, label="yaw D raw")
    axes[1].plot(time_s, yaw_d_filtered, label="yaw D LPF")
    axes[1].set_ylabel("rad/s^2")
    axes[2].plot(time_s, yaw_p, label="yaw P")
    axes[2].plot(time_s, yaw_d, label="yaw D")
    axes[2].plot(time_s, yaw_total, linewidth=1.5, label="tau_turn")
    axes[2].set_ylabel("N m")
    axes[3].plot(time_s, theta_left, label="theta_left")
    axes[3].plot(time_s, theta_right, label="theta_right")
    axes[3].plot(time_s, sync_raw, alpha=0.6, label="sync error raw")
    axes[3].plot(time_s, sync_filtered, "--", label="sync error LPF")
    axes[3].set_ylabel("rad")
    axes[4].plot(time_s, sync_d_raw, alpha=0.55, label="sync D raw")
    axes[4].plot(time_s, sync_d_filtered, label="sync D LPF")
    axes[4].set_ylabel("rad/s")
    axes[5].plot(time_s, sync_p, label="sync P")
    axes[5].plot(time_s, sync_d, label="sync D")
    axes[5].plot(time_s, sync_total, linewidth=1.5, label="Tp_sync")
    axes[5].set_ylabel("N m")
    axes[5].set_xlabel("time (s)")
    for axis in axes:
        axis.legend(loc="upper right")
        axis.grid(True)
    figure.suptitle(
        f"Yaw PD: Kp={yaw_kp:g}, Kd={yaw_kd:g} | "
        f"Leg-sync PD: Kp={sync_kp:g}, Kd={sync_kd:g} | 0-6 s"
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    figure.savefig(path, dpi=160)
    plt.close(figure)
