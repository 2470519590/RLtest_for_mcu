"""Parallel PPO training entry for the minimal residual-RL Env."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

from .residual_env import DEFAULT_ACTION_LIMITS, DEFAULT_EPISODE_SECONDS, ResidualCommandEnv, load_residual_tasks


def build_parser() -> argparse.ArgumentParser:
    tasks = load_residual_tasks()
    parser = argparse.ArgumentParser(description="Train residual RL with Stable-Baselines3 PPO.")
    parser.add_argument("--tasks", default="all", help="comma-separated task keys, or 'all'")
    parser.add_argument("--total-timesteps", type=int, default=10_000)
    parser.add_argument("--n-envs", type=int, default=5)
    parser.add_argument("--vec-env", choices=("subproc", "dummy"), default="subproc")
    parser.add_argument(
        "--episode-sim-seconds",
        "--episode-seconds",
        dest="episode_seconds",
        type=float,
        default=DEFAULT_EPISODE_SECONDS,
        help="full task horizon in MuJoCo simulation seconds; training runs headless/as-fast-as-possible",
    )
    parser.add_argument("--step-seconds", type=float, default=0.01)
    parser.add_argument(
        "--control-decimation-steps",
        type=int,
        default=None,
        help="MuJoCo substeps between Python controller updates; default matches --step-seconds",
    )
    parser.add_argument("--n-steps", type=int, default=64)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--clip-range", type=float, default=0.2)
    parser.add_argument("--ent-coef", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--output-dir", type=Path, default=Path("runs") / "residual_ppo")
    parser.add_argument("--checkpoint-freq", type=int, default=0, help="save every N env steps; 0 disables checkpoints")
    parser.add_argument("--verbose", type=int, choices=(0, 1, 2), default=1)
    parser.epilog = "Available tasks: " + ", ".join(sorted(tasks))
    return parser


def selected_task_keys(value: str) -> list[str]:
    tasks = load_residual_tasks()
    if value == "all":
        return sorted(tasks)
    keys = [item.strip() for item in value.split(",") if item.strip()]
    unknown = sorted(set(keys) - set(tasks))
    if unknown:
        raise ValueError(f"unknown task key(s): {unknown}; available={sorted(tasks)}")
    if not keys:
        raise ValueError("at least one task key is required")
    return keys


def _make_env(
    task_key: str,
    seed: int,
    episode_seconds: float,
    step_seconds: float,
    control_decimation_steps: int | None,
):
    def factory():
        env = ResidualCommandEnv(
            task_key,
            episode_seconds=episode_seconds,
            step_seconds=step_seconds,
            control_decimation_steps=control_decimation_steps,
            action_limits=DEFAULT_ACTION_LIMITS,
            controller_mode="lqr_residual",
        )
        env.reset(seed=seed)
        return env

    return factory


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    task_keys = selected_task_keys(args.tasks)
    if args.total_timesteps <= 0:
        parser.error("--total-timesteps must be positive")
    if args.n_envs <= 0:
        parser.error("--n-envs must be positive")
    if args.n_steps <= 0 or args.batch_size <= 0:
        parser.error("--n-steps and --batch-size must be positive")

    try:
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import CheckpointCallback
        from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor
    except ImportError as exc:
        raise SystemExit(
            "Missing training dependency. Install on the server py310 env first, for example:\n"
            "  pip install gymnasium stable-baselines3 torch\n"
            f"Original import error: {exc}"
        ) from exc

    run_name = args.run_name or datetime.now().strftime("ppo_%Y%m%d_%H%M%S")
    run_dir = args.output_dir / run_name
    model_dir = run_dir / "models"
    checkpoint_dir = run_dir / "checkpoints"
    tensorboard_dir = run_dir / "tensorboard"
    model_dir.mkdir(parents=True, exist_ok=True)
    if args.checkpoint_freq > 0:
        checkpoint_dir.mkdir(parents=True, exist_ok=True)

    env_fns = []
    for index in range(args.n_envs):
        task_key = task_keys[index % len(task_keys)]
        seed = args.seed + index

        def wrapped_factory(task_key=task_key, seed=seed):
            return _make_env(task_key, seed, args.episode_seconds, args.step_seconds, args.control_decimation_steps)()

        env_fns.append(wrapped_factory)

    if args.vec_env == "subproc" and args.n_envs > 1:
        vec_env = SubprocVecEnv(env_fns, start_method="spawn")
    else:
        vec_env = DummyVecEnv(env_fns)
    vec_env = VecMonitor(vec_env)

    policy_kwargs = {
        "activation_fn": torch.nn.Tanh,
        "net_arch": {"pi": [32, 32], "vf": [32, 32]},
    }
    model = PPO(
        "MlpPolicy",
        vec_env,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_range=args.clip_range,
        ent_coef=args.ent_coef,
        policy_kwargs=policy_kwargs,
        tensorboard_log=str(tensorboard_dir),
        seed=args.seed,
        device=args.device,
        verbose=args.verbose,
    )
    callbacks = []
    if args.checkpoint_freq > 0:
        callbacks.append(
            CheckpointCallback(
                save_freq=max(1, args.checkpoint_freq),
                save_path=str(checkpoint_dir),
                name_prefix="residual_ppo",
            )
        )

    print(f"tasks: {task_keys}")
    print(f"n_envs: {args.n_envs}, vec_env: {args.vec_env}")
    print(f"run_dir: {run_dir}")
    try:
        model.learn(total_timesteps=args.total_timesteps, callback=callbacks or None, tb_log_name=run_name)
        final_path = model_dir / "final_model"
        model.save(str(final_path))
        print(f"saved_model: {final_path}.zip")
    finally:
        vec_env.close()
    print("result: PASS residual PPO training script")
    return 0
