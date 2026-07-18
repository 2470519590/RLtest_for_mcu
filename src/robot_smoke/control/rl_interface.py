"""Controller interface for switching pure LQR and LQR + residual RL modes.

The nominal LQR controller remains the source of truth for the existing
middle-layer command.  Residual RL is represented as an optional, bounded
additive action applied through this small interface, so future policies can
be plugged in without rewriting the original LQR logic.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..core.types import LqrState


RL_CONTROLLER_LQR = "lqr"
RL_CONTROLLER_LQR_RESIDUAL = "lqr_residual"
RL_CONTROLLER_MODES = (RL_CONTROLLER_LQR, RL_CONTROLLER_LQR_RESIDUAL)


@dataclass(frozen=True)
class ResidualRlObservation:
    """Minimal observation exposed to a residual-RL policy.

    ``task_name`` and ``commanded_speed`` make the policy command-conditioned:
    the same policy can serve multiple remote-control tasks while keeping their
    rewards, reset logic and rollout sampling separated by the trainer.
    """

    time_s: float
    task_time_s: float
    task_name: str
    commanded_speed: str
    state: LqrState
    x_velocity_reference: float
    nominal_wheel_torque: float
    nominal_pitch_torque: float
    nominal_length_force_delta: float
    airborne: bool
    landing_phase: str
    left_contact_force: float
    right_contact_force: float


@dataclass(frozen=True)
class ResidualRlAction:
    """Additive residual command in the same units as the control interface."""

    wheel_torque: float = 0.0
    pitch_torque: float = 0.0
    length_force_delta: float = 0.0
    left_length_reference_delta: float = 0.0
    right_length_reference_delta: float = 0.0


class ResidualRlPolicy(Protocol):
    """Callable policy interface used by rollout code and future training."""

    def __call__(self, observation: ResidualRlObservation) -> ResidualRlAction:
        """Return a bounded residual action for the current observation."""


class ZeroResidualRlPolicy:
    """Safe placeholder policy: LQR + residual mode behaves like pure LQR."""

    def __call__(self, observation: ResidualRlObservation) -> ResidualRlAction:
        del observation
        return ResidualRlAction()


def apply_controller_interface(
    *,
    mode: str,
    observation: ResidualRlObservation,
    residual_policy: ResidualRlPolicy | None,
    residual_t_limit: float,
    residual_tp_limit: float,
    residual_length_force_limit: float,
    residual_leg_length_limit: float,
    lqr_t_limit: float,
    lqr_tp_limit: float,
) -> tuple[float, float, float, ResidualRlAction]:
    """Apply the selected controller interface to nominal LQR outputs.

    In ``lqr`` mode the nominal command is returned unchanged.  In
    ``lqr_residual`` mode the policy output is clipped by residual limits and
    added to ``T``, ``Tp`` and the optional common length-force delta.  The
    final middle-layer torques are clipped again by the existing LQR torque
    limits before the original output filters/rate limits run.
    """
    if mode not in RL_CONTROLLER_MODES:
        raise ValueError(f"unknown RL controller mode: {mode}")
    if mode == RL_CONTROLLER_LQR:
        return (
            observation.nominal_wheel_torque,
            observation.nominal_pitch_torque,
            observation.nominal_length_force_delta,
            ResidualRlAction(),
        )
    policy = residual_policy or ZeroResidualRlPolicy()
    raw_action = policy(observation)
    residual = ResidualRlAction(
        wheel_torque=float(np.clip(raw_action.wheel_torque, -residual_t_limit, residual_t_limit)),
        pitch_torque=float(np.clip(raw_action.pitch_torque, -residual_tp_limit, residual_tp_limit)),
        length_force_delta=float(
            np.clip(raw_action.length_force_delta, -residual_length_force_limit, residual_length_force_limit)
        ),
        left_length_reference_delta=float(
            np.clip(raw_action.left_length_reference_delta, -residual_leg_length_limit, residual_leg_length_limit)
        ),
        right_length_reference_delta=float(
            np.clip(raw_action.right_length_reference_delta, -residual_leg_length_limit, residual_leg_length_limit)
        ),
    )
    wheel_torque = float(
        np.clip(observation.nominal_wheel_torque + residual.wheel_torque, -lqr_t_limit, lqr_t_limit)
    )
    pitch_torque = float(
        np.clip(observation.nominal_pitch_torque + residual.pitch_torque, -lqr_tp_limit, lqr_tp_limit)
    )
    length_force_delta = observation.nominal_length_force_delta + residual.length_force_delta
    return wheel_torque, pitch_torque, length_force_delta, residual
