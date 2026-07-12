"""Article-style roll state and differential leg-support compensation."""

from __future__ import annotations

import math

import numpy as np

from ..core.mujoco_utils import id_by_name as _id_by_name


def base_roll_angle(mujoco, model, data) -> float:
    """Return base roll about its forward X axis from the world rotation matrix."""
    base_id = _id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, "base")
    rotation = data.xmat[base_id].reshape(3, 3)
    return math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))


def roll_support_force_offset(roll: float, kp: float, limit: float, deadband: float) -> float:
    """Return the signed left-leg support offset for a zero-roll reference."""
    if abs(roll) <= deadband:
        return 0.0
    return float(np.clip(-kp * roll, -limit, limit))
