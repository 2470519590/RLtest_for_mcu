"""Closed-form motion references for local wheel-leg smoke tests."""

from __future__ import annotations


def trapezoid_speed_reference(profile: str | None, time_s: float) -> tuple[float, float]:
    """Return world-frame wheel position and velocity references."""
    if profile is None:
        return 0.0, 0.0
    peak_speed = {"low": 1.00, "medium": 2.00, "high": 3.00}[profile]
    ramp_s = 1.5
    cruise_s = 4.0
    acceleration = peak_speed / ramp_s
    time_s = max(0.0, time_s)
    if time_s < ramp_s:
        return 0.5 * acceleration * time_s * time_s, acceleration * time_s
    ramp_distance = 0.5 * peak_speed * ramp_s
    if time_s < ramp_s + cruise_s:
        cruise_time = time_s - ramp_s
        return ramp_distance + peak_speed * cruise_time, peak_speed
    if time_s < 2.0 * ramp_s + cruise_s:
        ramp_down_time = time_s - ramp_s - cruise_s
        return (
            ramp_distance + peak_speed * cruise_s + peak_speed * ramp_down_time
            - 0.5 * acceleration * ramp_down_time * ramp_down_time,
            peak_speed - acceleration * ramp_down_time,
        )
    return peak_speed * (ramp_s + cruise_s), 0.0


def rear_ramp_speed_reference(time_s: float) -> float:
    """Immediate reverse profile used by isolated single-wheel ramp tests."""
    peak_speed = -0.50
    ramp_s = 0.8
    time_s = max(0.0, time_s)
    if time_s < ramp_s:
        return peak_speed * time_s / ramp_s
    if time_s < 4.8:
        return peak_speed
    if time_s < 5.6:
        return peak_speed * (1.0 - (time_s - 4.8) / ramp_s)
    return 0.0
