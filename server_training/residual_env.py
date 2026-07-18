"""Minimal command-conditioned residual-RL environment.

This is deliberately a thin adapter over the existing MuJoCo smoke rollout.
It is meant to verify the RL interface and task entry points before building
the faster PPO training loop.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

try:  # Gymnasium is optional for the local smoke command.
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover - exercised only on minimal machines.
    gym = None

    class _Box:
        def __init__(self, low: float, high: float, shape: tuple[int, ...], dtype: Any) -> None:
            self.low = low
            self.high = high
            self.shape = shape
            self.dtype = dtype

        def sample(self) -> np.ndarray:
            return np.zeros(self.shape, dtype=self.dtype)

    class _Spaces:
        Box = _Box

    spaces = _Spaces()

from src.robot_smoke.control.length_schedule import load_length_schedule
from src.robot_smoke.control.lqr import compute_lqr_state
from src.robot_smoke.control.rl_interface import RL_CONTROLLER_MODES, ResidualRlAction
from src.robot_smoke.core.config import DEFAULT_CONFIG_PATH, RunConfig, load_yaml_defaults, runtime_control_config
from src.robot_smoke.core.constants import (
    DEFAULT_LQR_K,
    LOCKED_EQUILIBRIUM_EVAL_STEPS,
    LOCKED_EQUILIBRIUM_FL0,
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
    LOCKED_LQR_K,
    LOCKED_LQR_U0,
    LOCKED_LQR_X0,
    PROJECT_ROOT,
)
from src.robot_smoke.io.cli import build_parser
from src.robot_smoke.model.kinematics import compute_virtual_leg_state
from src.robot_smoke.model.mechanics import support_force_scale_for_length
from src.robot_smoke.model_smoke import _build_virtual_rod_targets
from src.robot_smoke.runner import _flight_test_model_xml, _scheduled_initial_pose
from src.robot_smoke.experiments.virtual_rod import _run_virtual_rod_test
from src.robot_smoke.experiments.viewer import MujocoViewerObserver
from src.robot_smoke.core.mujoco_utils import load_mujoco


TASKS_PATH = PROJECT_ROOT / "server_training" / "residual_rl_tasks.yaml"
OBSERVATION_SIZE = 20
DEFAULT_EPISODE_SECONDS = 10.0
DEFAULT_ACTION_LIMITS = (2.0, 0.5, 25.0, 0.035, 0.035)
TASK_ONE_HOT = {
    "forward_jump": (1.0, 0.0, 0.0),
    "flight_ramp": (0.0, 1.0, 0.0),
    "inplace_jump": (0.0, 0.0, 1.0),
}
SPEED_ONE_HOT = {
    "zero": (1.0, 0.0, 0.0, 0.0),
    "low": (0.0, 1.0, 0.0, 0.0),
    "medium": (0.0, 0.0, 1.0, 0.0),
    "high": (0.0, 0.0, 0.0, 1.0),
}


@dataclass(frozen=True)
class ResidualEnvTask:
    key: str
    task_name: str
    commanded_speed: str
    run_smoke_args: tuple[str, ...]


class _ConstantResidualPolicy:
    def __init__(self, action: np.ndarray, limits: tuple[float, float, float, float, float]) -> None:
        scaled = np.clip(action.astype(float), -1.0, 1.0) * np.asarray(limits, dtype=float)
        self._action = ResidualRlAction(
            wheel_torque=float(scaled[0]),
            pitch_torque=float(scaled[1]),
            length_force_delta=float(scaled[2]),
            left_length_reference_delta=float(scaled[3]),
            right_length_reference_delta=float(scaled[4]),
        )

    def __call__(self, observation) -> ResidualRlAction:
        del observation
        return self._action


def load_residual_tasks(path: Path = TASKS_PATH) -> dict[str, ResidualEnvTask]:
    with path.open(encoding="utf-8") as file:
        raw = yaml.safe_load(file) or {}
    tasks = raw.get("tasks", {})
    if not isinstance(tasks, dict):
        raise ValueError(f"task file must contain a mapping named tasks: {path}")
    return {
        key: ResidualEnvTask(
            key=key,
            task_name=str(value["task_name"]),
            commanded_speed=str(value["commanded_speed"]),
            run_smoke_args=tuple(str(item) for item in value["run_smoke_args"]),
        )
        for key, value in tasks.items()
    }


def _parse_smoke_config(argv: list[str], config_path: Path) -> RunConfig:
    yaml_defaults = load_yaml_defaults(config_path)
    parser = build_parser()
    parser.add_argument("--config", type=Path, default=config_path)
    parser.set_defaults(**yaml_defaults)
    args = parser.parse_args(argv)
    _apply_task_flags(args)
    _apply_lqr_true_equilibrium(args)
    _normalize_runner_args(args, yaml_defaults, parser)
    return RunConfig.from_namespace(args)


def _apply_task_flags(args: argparse.Namespace) -> None:
    if args.flight_test:
        args.lqr_true_equilibrium = True
        args.speed_profile = args.flight_test_speed
        args.turn_test = False
        args.turn_direction = None
        args.impact_level = None
        args.flight_detection_enabled = True
    if args.jump_test:
        args.lqr_true_equilibrium = True
        args.speed_profile = None
        args.turn_test = False
        args.turn_direction = None
        args.impact_level = None
        args.flight_detection_enabled = True
    if args.forward_jump_test is not None:
        args.lqr_true_equilibrium = True
        args.jump_test = True
        args.speed_profile = args.forward_jump_test
        args.turn_test = False
        args.turn_direction = None
        args.impact_level = None
        args.flight_detection_enabled = True


def _apply_lqr_true_equilibrium(args: argparse.Namespace) -> None:
    if args.initial_leg_length is None:
        args.initial_leg_length = args.leg_length
    if not args.lqr_true_equilibrium:
        return
    target_leg_length = float(args.initial_leg_length)
    if args.left_rod_length is None:
        args.left_rod_length = target_leg_length
    if args.right_rod_length is None:
        args.right_rod_length = target_leg_length
    args.virtual_rod_test = True
    args.equilibrium_search = False
    args.lqr_test = True
    args.lqr_auto_design = False
    args.lqr_use_equilibrium_operating_point = False
    args.use_locked_equilibrium = True
    args.lqr_k = LOCKED_LQR_K.copy()
    args.lqr_x0 = LOCKED_LQR_X0.copy()
    args.lqr_u0 = LOCKED_LQR_U0.copy()
    args.zero_steps = 1
    args.probe_steps = 1
    args.pd_hold_steps = 1
    args.equilibrium_init_modes = ("upright-ik",)
    args.equilibrium_l_slices = (args.leg_length,)
    args.equilibrium_theta_refs = (LOCKED_EQUILIBRIUM_THETA,)
    args.equilibrium_fl0_scales = (support_force_scale_for_length(args.leg_length),)
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
    args.virtual_rod_gravity_comp_scale = None
    args.virtual_rod_length_delta = None
    args.virtual_rod_theta_target = LOCKED_EQUILIBRIUM_THETA
    args.left_rod_length = target_leg_length
    args.right_rod_length = target_leg_length
    args.left_rod_theta = LOCKED_EQUILIBRIUM_THETA
    args.right_rod_theta = LOCKED_EQUILIBRIUM_THETA
    args.lqr_design_steps = LOCKED_LQR_DESIGN_STEPS
    args.lqr_control_period_steps = LOCKED_LQR_CONTROL_PERIOD_STEPS


def _normalize_runner_args(args: argparse.Namespace, yaml_defaults: dict[str, object], parser: argparse.ArgumentParser) -> None:
    if args.length_kd is not None:
        args.virtual_rod_length_kd = args.length_kd
    if args.length_ki is not None:
        args.virtual_rod_length_ki = args.length_ki
    if args.length_integral_limit is not None:
        args.virtual_rod_length_integral_limit = args.length_integral_limit
    if args.length_force_ff is not None:
        args.virtual_rod_length_force_ff = args.length_force_ff
    model_path = args.model
    if not model_path.is_absolute():
        model_path = PROJECT_ROOT / model_path
    if not model_path.exists():
        parser.error(f"model not found: {model_path}")
    args.model_path = model_path
    args.length_force_ff_is_explicit = args.length_force_ff is not None or "virtual_rod_length_force_ff" in yaml_defaults
    args.lqr_gain_scale = 1.0 if args.lqr_gain_scale is None else float(args.lqr_gain_scale)
    args.lqr_k = DEFAULT_LQR_K if args.lqr_k is None else np.array(args.lqr_k, dtype=float).reshape(2, 6)
    args.lqr_x0 = np.array(args.lqr_x0, dtype=float)
    args.lqr_u0 = np.array(args.lqr_u0, dtype=float)
    args.lqr_q_diag = np.array(args.lqr_q_diag, dtype=float)
    args.lqr_r_diag = np.array(args.lqr_r_diag, dtype=float)
    args.lqr_state_eps = np.array(args.lqr_state_eps, dtype=float)
    args.lqr_input_eps = np.array(args.lqr_input_eps, dtype=float)
    args.lqr_design_steps = 1 if args.lqr_design_steps is None else int(args.lqr_design_steps)
    args.lqr_control_period_steps = 1 if args.lqr_control_period_steps is None else int(args.lqr_control_period_steps)
    args.lqr_x_source = "wheel" if args.lqr_x_source is None else str(args.lqr_x_source)
    args.virtual_rod_lock_base = bool(args.lock_base)
    args.leg_sync_kp = float(args.leg_sync_kp if args.leg_sync_kp is not None else 30.0)
    args.leg_sync_kd = float(args.leg_sync_kd if args.leg_sync_kd is not None else 20.0)
    args.yaw_turn_kp = float(args.yaw_turn_kp if args.yaw_turn_kp is not None else 3.0)
    args.yaw_turn_kd = float(args.yaw_turn_kd if args.yaw_turn_kd is not None else 0.1)


class ResidualCommandEnv(gym.Env if gym is not None else object):
    """A minimal Env wrapper around one existing residual-control rollout."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        task_key: str,
        *,
        config_path: Path = DEFAULT_CONFIG_PATH,
        episode_seconds: float = DEFAULT_EPISODE_SECONDS,
        step_seconds: float = 0.01,
        action_limits: tuple[float, float, float, float, float] = DEFAULT_ACTION_LIMITS,
        controller_mode: str = "lqr_residual",
    ) -> None:
        self.tasks = load_residual_tasks()
        if task_key not in self.tasks:
            raise ValueError(f"unknown task_key={task_key!r}; choices={sorted(self.tasks)}")
        if controller_mode not in RL_CONTROLLER_MODES:
            raise ValueError(f"unknown controller_mode={controller_mode!r}; choices={RL_CONTROLLER_MODES}")
        self.task = self.tasks[task_key]
        self.config = _parse_smoke_config(
            [
                *self.task.run_smoke_args,
                "--rl-controller-mode",
                controller_mode,
                "--rl-residual-t-limit",
                str(action_limits[0]),
                "--rl-residual-tp-limit",
                str(action_limits[1]),
                "--rl-residual-length-force-limit",
                str(action_limits[2]),
                "--rl-residual-leg-length-limit",
                str(action_limits[3]),
            ],
            config_path,
        )
        self.action_limits = action_limits
        self.episode_seconds = float(episode_seconds)
        self.step_seconds = float(step_seconds)
        self.mujoco = load_mujoco()
        if self.config.flight_test:
            self.model = self.mujoco.MjModel.from_xml_string(_flight_test_model_xml(Path(self.config.model_path)))
        else:
            self.model = self.mujoco.MjModel.from_xml_path(str(self.config.model_path))
        self.step_mujoco_steps = max(1, int(round(self.step_seconds / float(self.model.opt.timestep))))
        self.left_target, self.right_target = _build_virtual_rod_targets(
            self.mujoco,
            self.model,
            self.config.virtual_rod_length_delta,
            self.config.virtual_rod_theta_target,
            self.config.left_rod_length,
            self.config.right_rod_length,
            self.config.left_rod_theta,
            self.config.right_rod_theta,
        )
        self.length_schedule = None
        if self.config.length_schedule:
            schedule_path = Path(self.config.length_schedule_path)
            if not schedule_path.is_absolute():
                schedule_path = PROJECT_ROOT / schedule_path
            self.length_schedule = load_length_schedule(schedule_path)
        self.runtime_controls = runtime_control_config(self.config)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(5,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(OBSERVATION_SIZE,), dtype=np.float32)
        self._data = None
        self._time_s = 0.0
        self._last_obs = np.zeros(OBSERVATION_SIZE, dtype=np.float32)

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        if gym is not None:
            super().reset(seed=seed)
        del options
        self._time_s = 0.0
        self._data = _scheduled_initial_pose(
            self.mujoco,
            self.model,
            self.left_target,
            self.right_target,
            self.config.leg_branch,
            self.config.ik_search_radius,
            self.config.ik_search_samples,
        )
        self._last_obs = self._observation_from_data(self._data, None, airborne=False)
        return self._last_obs.copy(), {"task_key": self.task.key}

    def step(self, action):
        if self._data is None:
            self.reset()
        action_array = np.asarray(action, dtype=np.float32).reshape(5)
        result = self._run_rollout(action_array, self.step_mujoco_steps)
        self._data = result.final_data
        self._time_s += result.steps * float(self.model.opt.timestep)
        airborne = result.airborne_steps > 0
        self._last_obs = self._observation_from_data(self._data, result.final_lqr_state, airborne=airborne)
        reward = self._reward(result, action_array)
        terminated = False
        truncated = self._time_s >= self.episode_seconds
        info = {
            "task_key": self.task.key,
            "time_s": self._time_s,
            "airborne_steps": result.airborne_steps,
            "max_base_height": result.max_base_height,
            "max_abs_ctrl": result.max_abs_ctrl,
            "saturated_steps": result.saturated_steps,
        }
        return self._last_obs.copy(), reward, terminated, truncated, info

    def visualize_constant_action(self, action, *, seconds: float, realtime: bool = True, sync_hz: float = 30.0):
        """Open MuJoCo viewer and run one continuous env-configured rollout.

        This is for visual smoke checks.  It uses the same task config and
        residual policy path as ``step()``, but keeps one MuJoCo rollout alive
        so the viewer follows the actual data being stepped.
        """
        if self._data is None:
            self.reset()
        action_array = np.asarray(action, dtype=np.float32).reshape(5)
        steps = max(1, int(round(float(seconds) / float(self.model.opt.timestep))))
        viewer = MujocoViewerObserver(self.mujoco, self.model, realtime, sync_hz=sync_hz)
        try:
            result = self._run_rollout(action_array, steps, on_initialized=viewer.start)
        finally:
            viewer.close()
        self._data = result.final_data
        self._time_s += result.steps * float(self.model.opt.timestep)
        self._last_obs = self._observation_from_data(self._data, result.final_lqr_state, airborne=result.airborne_steps > 0)
        return result

    def _run_rollout(self, action: np.ndarray, steps: int, *, on_initialized=None):
        return _run_virtual_rod_test(
            self.mujoco,
            self.model,
            steps,
            self.config.virtual_rod_lock_base,
            self.left_target,
            self.right_target,
            self.config.virtual_rod_control,
            self.config.leg_branch,
            self.config.ik_search_radius,
            self.config.ik_search_samples,
            self.config.virtual_rod_length_kp,
            self.config.virtual_rod_length_kd,
            self.config.virtual_rod_length_ki,
            self.config.virtual_rod_length_force_ff,
            self.config.virtual_rod_length_integral_limit,
            self.config.virtual_rod_length_force_rate_limit,
            self.config.virtual_rod_theta_kp,
            self.config.virtual_rod_theta_kd,
            self.config.virtual_rod_joint_kd,
            self.config.virtual_rod_theta_pitch_ff,
            self.config.lqr_test,
            self.config.lqr_gain_scale,
            self.config.lqr_k,
            self.config.lqr_x0,
            self.config.lqr_u0,
            self.config.lqr_auto_design,
            self.config.lqr_control_period_steps,
            self.config.lqr_x_reference,
            self.config.lqr_x_source,
            self.config.lqr_x_outer_kp,
            self.config.lqr_x_outer_max_v,
            self.config.lqr_wheel_sign,
            self.config.lqr_pitch_sign,
            self.config.lqr_t_limit,
            self.config.lqr_tp_limit,
            self.config.landing_hold_t_limit,
            self.config.lqr_output_rate_limit,
            self.config.lqr_output_lowpass_hz,
            self.config.wheel_ctrl_deadzone,
            self.config.history_sample_interval,
            initial_data=self._data,
            impact_level=self.config.impact_level,
            speed_profile=self.config.speed_profile,
            turn_direction=self.config.turn_direction,
            turn_speed=self.config.turn_speed,
            turn_test=self.config.turn_test,
            slope_roll_turn_test=bool(self.config.slope_roll_turn_test),
            slope_roll_turn_start_time=float(self.config.slope_roll_turn_start_time),
            leg_sync_kp=self.config.leg_sync_kp,
            leg_sync_kd=self.config.leg_sync_kd,
            yaw_turn_kp=self.config.yaw_turn_kp,
            yaw_turn_kd=self.config.yaw_turn_kd,
            leg_height_test=self.config.leg_height_test,
            leg_height_levels=tuple(float(value) for value in self.config.leg_height_levels),
            leg_length_sine_test=bool(self.config.leg_length_sine_test),
            leg_length_sine_period=float(self.config.leg_length_sine_period),
            jump_test=bool(self.config.jump_test),
            forward_jump_test=self.config.forward_jump_test is not None,
            branch_guard_enabled=bool(self.config.leg_branch_guard_enabled),
            minimum_leg_length=float(self.config.minimum_leg_length),
            maximum_leg_length=float(self.config.maximum_leg_length),
            length_schedule=self.length_schedule,
            startup_ramp_seconds=float(self.config.startup_ramp_seconds),
            flight_detection_enabled=bool(self.config.flight_detection_enabled),
            flight_airborne_force_threshold=float(self.config.flight_airborne_force_threshold),
            flight_airborne_confirm_seconds=float(self.config.flight_airborne_confirm_seconds),
            flight_airborne_rearm_seconds=float(self.config.flight_airborne_rearm_seconds),
            runtime_controls=self.runtime_controls,
            residual_rl_policy=_ConstantResidualPolicy(action, self.action_limits),
            time_offset_s=self._time_s,
            on_initialized=on_initialized,
        )

    def _observation_from_data(self, data, lqr_state, *, airborne: bool) -> np.ndarray:
        if lqr_state is None and data is not None:
            lqr_state = compute_lqr_state(self.mujoco, self.model, data, 0.0, "wheel")
        left = compute_virtual_leg_state(self.mujoco, self.model, data, "left") if data is not None else None
        right = compute_virtual_leg_state(self.mujoco, self.model, data, "right") if data is not None else None
        state_values = (
            (lqr_state.theta, lqr_state.theta_rate, lqr_state.x, lqr_state.x_rate,
             lqr_state.pitch, lqr_state.pitch_rate, lqr_state.length, lqr_state.length_rate)
            if lqr_state is not None
            else (0.0,) * 8
        )
        obs = [
            *state_values,
            float(data.qpos[2]) if data is not None and self.model.nq >= 3 else 0.0,
            float(left.length) if left is not None else 0.0,
            float(right.length) if right is not None else 0.0,
            1.0 if airborne else 0.0,
            float(np.clip(self._time_s / max(self.episode_seconds, 1e-6), 0.0, 1.0)),
            *TASK_ONE_HOT[self.task.task_name],
            *SPEED_ONE_HOT[self.task.commanded_speed],
        ]
        return np.asarray(obs, dtype=np.float32)

    def _reward(self, result, action: np.ndarray) -> float:
        state = result.final_lqr_state
        pitch_cost = abs(state.pitch) if state is not None else 0.0
        theta_cost = abs(state.theta) if state is not None else 0.0
        saturation_cost = 0.05 * result.saturated_steps
        control_cost = 0.002 * float(np.dot(action, action))
        height_bonus = 0.2 * max(0.0, float(result.max_base_height) - float(result.final_base_height))
        return float(height_bonus - pitch_cost - 0.2 * theta_cost - saturation_cost - control_cost)
