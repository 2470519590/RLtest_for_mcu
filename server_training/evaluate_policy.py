"""Evaluate a trained residual PPO policy in the MuJoCo residual Env."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from src.robot_smoke.experiments.viewer import MujocoViewerObserver
from .residual_env import DEFAULT_ACTION_LIMITS, DEFAULT_EPISODE_SECONDS, ResidualCommandEnv, load_residual_tasks


def build_parser() -> argparse.ArgumentParser:
    tasks = load_residual_tasks()
    parser = argparse.ArgumentParser(description="Evaluate a trained residual PPO policy.")
    parser.add_argument("--model", type=Path, required=True, help="Path to Stable-Baselines3 PPO .zip model")
    parser.add_argument("--task-key", choices=sorted(tasks), default="flight_ramp_medium")
    parser.add_argument("--episode-sim-seconds", "--episode-seconds", dest="episode_seconds", type=float, default=DEFAULT_EPISODE_SECONDS)
    parser.add_argument("--step-seconds", type=float, default=0.02)
    parser.add_argument("--control-decimation-steps", type=int, default=None)
    parser.add_argument("--deterministic", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--visualize", action="store_true", help="open MuJoCo viewer")
    parser.add_argument("--no-realtime", action="store_true", help="run viewer as fast as possible")
    parser.add_argument("--viewer-sync-hz", type=float, default=30.0)
    parser.add_argument("--print-every", type=int, default=25, help="print every N policy steps; 0 disables step prints")
    parser.add_argument("--device", default="cpu")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.model.exists():
        parser.error(f"model not found: {args.model}")
    try:
        from stable_baselines3 import PPO
    except ImportError as exc:
        raise SystemExit(
            "Missing evaluation dependency. Install stable-baselines3/torch in this env first.\n"
            f"Original import error: {exc}"
        ) from exc

    model = PPO.load(str(args.model), device=args.device)
    env = ResidualCommandEnv(
        args.task_key,
        episode_seconds=args.episode_seconds,
        step_seconds=args.step_seconds,
        control_decimation_steps=args.control_decimation_steps,
        action_limits=DEFAULT_ACTION_LIMITS,
        controller_mode="lqr_residual",
    )
    obs, info = env.reset()
    print(f"model: {args.model}")
    print(f"task_key: {info['task_key']}")
    print(f"episode_sim_seconds: {args.episode_seconds}, step_seconds: {args.step_seconds}")
    print(f"deterministic: {args.deterministic}")

    viewer = None
    step_observer = None
    if args.visualize:
        viewer = MujocoViewerObserver(env.mujoco, env.model, realtime=not args.no_realtime, sync_hz=args.viewer_sync_hz)
        step_observer = viewer.start(env._data)

    total_reward = 0.0
    step_index = 0
    last_info = {}
    try:
        while True:
            action, _ = model.predict(obs, deterministic=args.deterministic)
            action_array = np.asarray(action, dtype=np.float32).reshape(5)
            if step_observer is None:
                obs, reward, terminated, truncated, last_info = env.step(action_array)
            else:
                obs, reward, terminated, truncated, last_info = _step_env_with_observer(env, action_array, step_observer)
            total_reward += float(reward)
            step_index += 1
            if args.print_every > 0 and (step_index % args.print_every == 0 or terminated or truncated):
                terms = last_info.get("reward_terms", {})
                print(
                    f"step={step_index} time_s={last_info.get('time_s', 0.0):.3f} "
                    f"reward={reward:.5g} total={total_reward:.5g} "
                    f"phase={last_info.get('landing_phase', 'unknown')} "
                    f"airborne_steps={last_info.get('airborne_steps', 0)} "
                    f"posture={terms.get('posture', 0.0):.4g} "
                    f"speed={terms.get('speed', 0.0):.4g} "
                    f"impact={terms.get('impact', 0.0):.4g} "
                    f"recovery={terms.get('recovery', 0.0):.4g} "
                    f"fall={terms.get('fall', 0.0):.4g}"
                )
            if terminated or truncated:
                break
    finally:
        if viewer is not None:
            viewer.close()

    print(f"episode_steps: {step_index}")
    print(f"episode_reward: {total_reward:.6g}")
    print(f"final_time_s: {last_info.get('time_s', 0.0):.6g}")
    print(f"final_phase: {last_info.get('landing_phase', 'unknown')}")
    print(f"result: PASS residual policy eval")
    return 0


def _step_env_with_observer(env: ResidualCommandEnv, action_array: np.ndarray, step_observer):
    result = env._run_rollout(
        action_array,
        env.step_mujoco_steps,
        on_initialized=lambda _data: step_observer,
    )
    env._data = result.final_data
    env._time_s += result.steps * float(env.model.opt.timestep)
    airborne = result.airborne_steps > 0
    env._last_obs = env._observation_from_data(env._data, result.final_lqr_state, airborne=airborne)
    reward, terminated = env._reward(result, action_array, airborne=airborne)
    truncated = env._time_s >= env.episode_seconds
    info = {
        "task_key": env.task.key,
        "time_s": env._time_s,
        "airborne_steps": result.airborne_steps,
        "max_base_height": result.max_base_height,
        "max_abs_ctrl": result.max_abs_ctrl,
        "saturated_steps": result.saturated_steps,
        "reward_terms": env._last_reward_terms,
        "landing_phase": str(env._rollout_context.get("landing_phase", "ground")),
    }
    env._previous_action = action_array.copy()
    env._previous_airborne = airborne
    env._was_airborne_episode = env._was_airborne_episode or airborne or bool(env._rollout_context.get("was_airborne", False))
    return env._last_obs.copy(), reward, terminated, truncated, info
