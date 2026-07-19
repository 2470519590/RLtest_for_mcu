"""Task-aware reward for residual RL training.

The reward intentionally stays compact: LQR-like quadratic state costs,
normalized residual-action costs, touchdown impact penalty, and a broad
early-recovery bonus.  It is a training objective, not a physical pass/fail
criterion; viewer inspection remains the final behavior check.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


SPEED_TARGETS = {
    "zero": 0.0,
    "low": 1.0,
    "medium": 2.0,
    "high": 3.0,
}


@dataclass(frozen=True)
class RewardWeights:
    pitch: float = 3.0
    pitch_rate: float = 0.35
    theta: float = 0.45
    theta_rate: float = 0.08
    speed: float = 0.45
    action: float = 0.012
    delta_action: float = 0.018
    saturation: float = 0.015
    impact_force: float = 0.000018
    downward_velocity: float = 0.45
    leg_range: float = 18.0
    leg_diff: float = 2.5
    liftoff_bonus: float = 2.0
    liftoff_delay: float = 0.20
    no_liftoff: float = 0.004
    recovery_bonus: float = 3.0
    fall_penalty: float = 8.0


@dataclass(frozen=True)
class RewardContext:
    task_name: str
    commanded_speed: str
    time_s: float
    episode_seconds: float
    airborne: bool
    was_airborne: bool
    just_liftoff: bool
    just_recontact: bool
    landing_phase: str
    pitch: float
    pitch_rate: float
    theta: float
    theta_rate: float
    x_rate: float
    current_speed_reference: float
    base_z: float
    base_z_vel: float
    roll: float
    left_leg_length: float
    right_leg_length: float
    left_contact_force: float
    right_contact_force: float
    saturated_steps: int
    action: np.ndarray
    previous_action: np.ndarray


def compute_residual_reward(
    context: RewardContext,
    weights: RewardWeights = RewardWeights(),
) -> tuple[float, dict[str, float]]:
    commanded_speed = SPEED_TARGETS.get(context.commanded_speed, 0.0)
    speed_reference = _speed_reference(context, commanded_speed)
    speed_error = context.x_rate - speed_reference

    phase_scale = _phase_scale(context)
    posture = -(
        phase_scale * weights.pitch * context.pitch * context.pitch
        + 0.7 * phase_scale * weights.pitch_rate * context.pitch_rate * context.pitch_rate
        + weights.theta * context.theta * context.theta
        + weights.theta_rate * context.theta_rate * context.theta_rate
    )
    speed = -weights.speed * speed_error * speed_error
    action_cost = -weights.action * float(np.dot(context.action, context.action))
    action_delta = context.action - context.previous_action
    smooth = -weights.delta_action * float(np.dot(action_delta, action_delta))
    saturation = -weights.saturation * float(context.saturated_steps)
    impact = _impact_reward(context, weights)
    leg = _leg_reward(context, weights)
    liftoff = _liftoff_reward(context, weights)
    recovery = _recovery_reward(context, weights, speed_error)
    fall = -weights.fall_penalty if is_fallen(context) else 0.0

    total = posture + speed + action_cost + smooth + saturation + impact + leg + liftoff + recovery + fall
    terms = {
        "posture": posture,
        "speed": speed,
        "action": action_cost,
        "smooth": smooth,
        "saturation": saturation,
        "impact": impact,
        "leg": leg,
        "liftoff": liftoff,
        "recovery": recovery,
        "fall": fall,
        "total": total,
        "phase_airborne": 1.0 if context.airborne else 0.0,
        "phase_landing": 1.0 if _is_landing(context) else 0.0,
    }
    return float(total), {key: float(value) for key, value in terms.items()}


def is_fallen(context: RewardContext) -> bool:
    return abs(context.pitch) > 1.2 or abs(context.roll) > 0.9 or context.base_z < 0.12


def is_balanced(context: RewardContext, speed_error: float | None = None) -> bool:
    if speed_error is None:
        commanded_speed = SPEED_TARGETS.get(context.commanded_speed, 0.0)
        speed_error = context.x_rate - _speed_reference(context, commanded_speed)
    return (
        abs(context.pitch) < 0.16
        and abs(context.pitch_rate) < 1.2
        and abs(speed_error) < 0.45
        and abs(context.theta) < 0.30
        and abs(context.theta_rate) < 2.0
    )


def _phase_scale(context: RewardContext) -> float:
    if _is_landing(context):
        return 2.0
    if context.airborne:
        return 1.4
    return 1.0


def _is_landing(context: RewardContext) -> bool:
    return context.just_recontact or context.landing_phase == "landing_hold"


def _speed_reference(context: RewardContext, target_speed: float) -> float:
    if context.task_name == "inplace_jump":
        return 0.0
    if context.was_airborne and not context.airborne:
        # Let touchdown absorb first, then restore the commanded speed softly.
        # This is deliberately broad rather than a hard landing script.
        landing_elapsed = max(0.0, context.time_s - 0.0)
        ramp = float(np.clip(landing_elapsed / max(context.episode_seconds, 1e-6), 0.0, 1.0))
        return target_speed * max(0.35, ramp)
    return context.current_speed_reference


def _impact_reward(context: RewardContext, weights: RewardWeights) -> float:
    if not (_is_landing(context) or context.just_recontact):
        return 0.0
    force_sum = max(0.0, context.left_contact_force + context.right_contact_force)
    force_excess = max(0.0, force_sum - 180.0)
    downward = max(0.0, -context.base_z_vel)
    return -weights.impact_force * force_excess * force_excess - weights.downward_velocity * downward * downward


def _leg_reward(context: RewardContext, weights: RewardWeights) -> float:
    avg_length = 0.5 * (context.left_leg_length + context.right_leg_length)
    leg_diff = context.left_leg_length - context.right_leg_length
    if context.airborne:
        if context.base_z_vel > 0.0:
            lower, upper = 0.19, 0.24
        else:
            lower, upper = 0.23, 0.29
        range_error = _range_error(avg_length, lower, upper)
        return -weights.leg_range * range_error * range_error - weights.leg_diff * leg_diff * leg_diff
    if _is_landing(context):
        # Avoid forcing a nominal length immediately after contact; low impact
        # is the real objective, so only discourage left/right mismatch.
        return -0.5 * weights.leg_diff * leg_diff * leg_diff
    return 0.0


def _liftoff_reward(context: RewardContext, weights: RewardWeights) -> float:
    if context.task_name == "flight_ramp":
        return 0.0
    if context.just_liftoff:
        return weights.liftoff_bonus - weights.liftoff_delay * context.time_s
    if not context.was_airborne and context.time_s > 0.3:
        return -weights.no_liftoff * context.time_s
    return 0.0


def _recovery_reward(context: RewardContext, weights: RewardWeights, speed_error: float) -> float:
    if not context.was_airborne or context.airborne:
        return 0.0
    if not is_balanced(context, speed_error):
        return 0.0
    remaining = max(0.0, context.episode_seconds - context.time_s) / max(context.episode_seconds, 1e-6)
    return weights.recovery_bonus * remaining


def _range_error(value: float, lower: float, upper: float) -> float:
    if value < lower:
        return lower - value
    if value > upper:
        return value - upper
    return 0.0
