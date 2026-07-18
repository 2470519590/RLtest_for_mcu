"""Smoke one minimal residual-RL environment task."""

from __future__ import annotations

import argparse

import numpy as np

from src.robot_smoke.control.rl_interface import RL_CONTROLLER_MODES
from server_training.residual_env import DEFAULT_ACTION_LIMITS, ResidualCommandEnv, load_residual_tasks


def main() -> int:
    tasks = load_residual_tasks()
    parser = argparse.ArgumentParser(description="Run a tiny residual-RL env smoke rollout.")
    parser.add_argument("--task-key", choices=sorted(tasks), default="inplace_jump")
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--episode-seconds", type=float, default=0.2)
    parser.add_argument("--step-seconds", type=float, default=0.01)
    parser.add_argument(
        "--control-decimation-steps",
        type=int,
        default=None,
        help="MuJoCo substeps between Python controller updates; default matches --step-seconds",
    )
    parser.add_argument("--controller-mode", choices=RL_CONTROLLER_MODES, default="lqr_residual")
    parser.add_argument(
        "--compare-zero-residual",
        action="store_true",
        help="headless compare pure LQR against LQR+zero residual for the same task",
    )
    parser.add_argument("--visualize", action="store_true", help="open MuJoCo viewer using the same residual-env path")
    parser.add_argument("--visualize-seconds", type=float, help="viewer rollout length; default is max(5 s, steps*step-seconds)")
    parser.add_argument("--no-realtime", action="store_true", help="run viewer rollout as fast as possible")
    parser.add_argument("--viewer-sync-hz", type=float, default=30.0, help="viewer sync rate; lower values reduce render load")
    parser.add_argument(
        "--action",
        nargs=5,
        type=float,
        default=(0.0, 0.0, 0.0, 0.0, 0.0),
        metavar=("dT", "dTp", "dFl", "dLLeft", "dLRight"),
        help="normalized residual action in [-1, 1], held constant for every env step",
    )
    args = parser.parse_args()

    if args.compare_zero_residual:
        return _compare_zero_residual(args)

    env = ResidualCommandEnv(
        args.task_key,
        episode_seconds=args.episode_seconds,
        step_seconds=args.step_seconds,
        control_decimation_steps=args.control_decimation_steps,
        action_limits=DEFAULT_ACTION_LIMITS,
        controller_mode=args.controller_mode,
    )
    obs, info = env.reset()
    action = np.asarray(args.action, dtype=np.float32)
    print(f"task_key: {info['task_key']}")
    print(f"obs_shape: {obs.shape}")
    print(f"residual_mode: {env.config.rl_controller_mode}")
    print(f"normalized_action: {action.tolist()}")
    print(f"action_limits: {DEFAULT_ACTION_LIMITS}")
    if args.visualize:
        seconds = args.visualize_seconds if args.visualize_seconds is not None else max(5.0, args.steps * args.step_seconds)
        result = env.visualize_constant_action(
            action,
            seconds=seconds,
            realtime=not args.no_realtime,
            sync_hz=args.viewer_sync_hz,
        )
        print(
            f"visualized_seconds={result.steps * float(env.model.opt.timestep):.4f} "
            f"airborne_steps={result.airborne_steps} max_base_height={result.max_base_height:.6g} "
            f"max_abs_ctrl={result.max_abs_ctrl:.6g}"
        )
        print("result: PASS residual env visual smoke")
        return 0
    for index in range(args.steps):
        obs, reward, terminated, truncated, info = env.step(action)
        print(
            f"step={index + 1} reward={reward:.6g} "
            f"time_s={info['time_s']:.4f} airborne_steps={info['airborne_steps']} "
            f"max_base_height={info['max_base_height']:.6g} max_abs_ctrl={info['max_abs_ctrl']:.6g}"
        )
        if terminated or truncated:
            break
    print("result: PASS residual env smoke")
    return 0


def _compare_zero_residual(args: argparse.Namespace) -> int:
    seconds = args.visualize_seconds if args.visualize_seconds is not None else args.episode_seconds
    action = np.zeros(5, dtype=np.float32)
    summaries = {}
    for mode in ("lqr", "lqr_residual"):
        env = ResidualCommandEnv(
            args.task_key,
            episode_seconds=args.episode_seconds,
            step_seconds=args.step_seconds,
            control_decimation_steps=args.control_decimation_steps,
            action_limits=DEFAULT_ACTION_LIMITS,
            controller_mode=mode,
        )
        env.reset()
        rollout_steps = max(1, int(round(float(seconds) / float(env.model.opt.timestep))))
        result = env._run_rollout(action, rollout_steps)
        summaries[mode] = {
            "steps": result.steps,
            "airborne_steps": result.airborne_steps,
            "first_airborne_time": result.first_airborne_time,
            "last_airborne_time": result.last_airborne_time,
            "final_base_height": result.final_base_height,
            "max_base_height": result.max_base_height,
            "max_abs_ctrl": result.max_abs_ctrl,
            "saturated_steps": result.saturated_steps,
            "final_theta": result.final_lqr_state.theta if result.final_lqr_state is not None else float("nan"),
            "final_pitch": result.final_lqr_state.pitch if result.final_lqr_state is not None else float("nan"),
        }
    print(f"task_key: {args.task_key}")
    print(f"compare_seconds: {seconds:.6g}")
    print("action: zero residual [0, 0, 0, 0, 0]")
    for mode, summary in summaries.items():
        print(f"{mode}:")
        for key, value in summary.items():
            print(f"  {key}: {value}")
    print("diff_abs(lqr_residual_zero - lqr):")
    for key in ("final_base_height", "max_base_height", "max_abs_ctrl", "saturated_steps", "final_theta", "final_pitch"):
        diff = summaries["lqr_residual"][key] - summaries["lqr"][key]
        print(f"  {key}: {abs(diff):.6g}")
    print("result: PASS zero-residual comparison smoke")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
