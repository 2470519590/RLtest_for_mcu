"""Runtime kinematic measurements from MuJoCo state."""

from __future__ import annotations

import math

import numpy as np

from ..core.mujoco_utils import body_pos_vel as _body_pos_vel
from ..core.mujoco_utils import id_by_name as _id_by_name
from ..core.mujoco_utils import site_pos as _site_pos
from ..core.mujoco_utils import site_pos_vel as _site_pos_vel
from ..core.types import VirtualLegState


def lower_loop_error(mujoco, model, data, side: str) -> float:
    rear_hub = _site_pos(mujoco, model, data, f"{side}_rear_hub_site")
    carrier = _site_pos(mujoco, model, data, f"{side}_carrier_site")
    return float(np.linalg.norm(rear_hub - carrier))


def compute_virtual_leg_state(mujoco, model, data, side: str) -> VirtualLegState:
    front_hip_pos, front_hip_vel = _body_pos_vel(mujoco, model, data, f"{side}_front_upper")
    rear_hip_pos, rear_hip_vel = _body_pos_vel(mujoco, model, data, f"{side}_rear_upper")
    wheel_pos, wheel_vel = _site_pos_vel(mujoco, model, data, f"{side}_carrier_site")
    hip_pos = 0.5 * (front_hip_pos + rear_hip_pos)
    hip_vel = 0.5 * (front_hip_vel + rear_hip_vel)

    rod = wheel_pos - hip_pos
    rod_vel = wheel_vel - hip_vel
    rx = float(rod[0])
    rz = float(rod[2])
    vx = float(rod_vel[0])
    vz = float(rod_vel[2])
    length = max(math.hypot(rx, rz), 1e-9)
    length_rate = (rx * vx + rz * vz) / length
    theta = math.atan2(rx, -rz)
    theta_rate = (-rz * vx + rx * vz) / (length * length)
    return VirtualLegState(
        name=side,
        length=length,
        length_rate=length_rate,
        theta=theta,
        theta_rate=theta_rate,
    )


def compute_virtual_leg_shape(mujoco, model, data, side: str) -> np.ndarray:
    state = compute_virtual_leg_state(mujoco, model, data, side)
    return np.array([state.length, state.theta], dtype=float)


def wheel_speeds(mujoco, model, data) -> tuple[float, float]:
    left_joint = _id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint")
    right_joint = _id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint")
    left_dof = int(model.jnt_dofadr[left_joint])
    right_dof = int(model.jnt_dofadr[right_joint])
    return float(data.qvel[left_dof]), float(data.qvel[right_dof])


def wheel_positions(mujoco, model, data) -> tuple[float, float]:
    left_joint = _id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint")
    right_joint = _id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint")
    left_qpos = int(model.jnt_qposadr[left_joint])
    right_qpos = int(model.jnt_qposadr[right_joint])
    return float(data.qpos[left_qpos]), float(data.qpos[right_qpos])


def wheel_radius(mujoco, model) -> float:
    wheel_geom = _id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, "left_wheel_geom")
    return float(model.geom_size[wheel_geom, 0])
