"""Article-style yaw-rate PD and wheel-torque allocation."""

from __future__ import annotations

import numpy as np

from ..core.constants import YAW_TURN_KD, YAW_TURN_KP


def turn_rate_reference(turn_direction: str | None, turn_test: bool, time_s: float) -> float:
    """Return the desired yaw rate for manual or continuous staged turning."""
    if turn_test:
        # Single trapezoid: start at 1 s, ramp for 150 ms, stop at 5 s.
        large_w = np.pi * 2
        time_s = max(time_s, 0.0)
        ramp_time = 0.15
        start_time = 1.0
        stop_time = 5.0
        if time_s <= start_time:
            return 0.0
        if time_s < start_time + ramp_time:
            return large_w * (time_s - start_time) / ramp_time
        if time_s <= stop_time:
            return large_w
        if time_s < stop_time + ramp_time:
            return large_w * (1.0 - (time_s - stop_time) / ramp_time)
        return 0.0
    if turn_direction is None:
        return 0.0
    return 0.35 if turn_direction == "left" else -0.35


def yaw_turn_torque(
    yaw_rate_reference: float,
    yaw_rate: float,
    previous_error: float,
    dt: float,
    kp: float = YAW_TURN_KP,
    kd: float = YAW_TURN_KD,
    error_rate: float | None = None,
    error: float | None = None,
) -> tuple[float, float]:
    """Return differential wheel torque and updated yaw-rate error."""
    error = yaw_rate_reference - yaw_rate if error is None else error
    if error_rate is None:
        error_rate = (error - previous_error) / max(dt, 1e-6)
    torque = float(np.clip(kp * error + kd * error_rate, -2.0, 2.0))
    return torque, error


def split_wheel_torque(total_torque: float, turn_torque: float) -> tuple[float, float]:
    """Article allocation: opposite yaw torque is superposed on total T."""
    return 0.5 * total_torque - turn_torque, 0.5 * total_torque + turn_torque
