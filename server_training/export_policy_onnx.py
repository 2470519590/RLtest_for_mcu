"""Export a Stable-Baselines3 residual PPO actor to ONNX."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .residual_env import OBSERVATION_SIZE


class DeterministicResidualActor:
    """Torch module wrapper for the SB3 deterministic continuous actor."""

    def __init__(self, policy):
        import torch

        self.torch = torch
        self.policy = policy

    def module(self):
        torch = self.torch
        policy = self.policy

        class _Actor(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.features_extractor = policy.features_extractor
                self.mlp_extractor = policy.mlp_extractor
                self.action_net = policy.action_net

            def forward(self, observation):
                features = self.features_extractor(observation)
                latent_pi = self.mlp_extractor.forward_actor(features)
                action = self.action_net(latent_pi)
                return torch.clamp(action, -1.0, 1.0)

        return _Actor()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export residual PPO deterministic actor to ONNX.")
    parser.add_argument("--model", type=Path, required=True, help="Stable-Baselines3 PPO .zip or extracted model directory")
    parser.add_argument("--output", type=Path, required=True, help="Output .onnx path")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--check", action="store_true", help="Compare ONNXRuntime output against the PyTorch actor")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.model.exists():
        parser.error(f"model not found: {args.model}")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    try:
        import torch
        from stable_baselines3 import PPO
    except ImportError as exc:
        raise SystemExit(
            "Missing export dependency. Install torch and stable-baselines3 first.\n"
            f"Original import error: {exc}"
        ) from exc

    model = PPO.load(str(args.model), device=args.device)
    actor = DeterministicResidualActor(model.policy).module().to(args.device)
    actor.eval()
    dummy_obs = torch.zeros((1, OBSERVATION_SIZE), dtype=torch.float32, device=args.device)
    with torch.no_grad():
        torch_out = actor(dummy_obs).detach().cpu().numpy()

    torch.onnx.export(
        actor,
        dummy_obs,
        str(args.output),
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=["observation"],
        output_names=["normalized_action"],
        dynamic_axes={
            "observation": {0: "batch"},
            "normalized_action": {0: "batch"},
        },
    )

    print(f"model: {args.model}")
    print(f"output: {args.output}")
    print(f"observation_size: {OBSERVATION_SIZE}")
    print("action_size: 5")
    print("output_semantics: normalized residual action clipped to [-1, 1]")
    print(f"zero_obs_action_torch: {np.array2string(torch_out.reshape(-1), precision=7)}")

    if args.check:
        try:
            import onnx
            import onnxruntime as ort
        except ImportError as exc:
            raise SystemExit(
                "Missing ONNX check dependency. Install onnx and onnxruntime first.\n"
                f"Original import error: {exc}"
            ) from exc
        onnx_model = onnx.load(str(args.output))
        onnx.checker.check_model(onnx_model)
        session = ort.InferenceSession(str(args.output), providers=["CPUExecutionProvider"])
        ort_out = session.run(None, {"observation": dummy_obs.detach().cpu().numpy()})[0]
        max_abs_error = float(np.max(np.abs(torch_out - ort_out)))
        print(f"zero_obs_action_onnx: {np.array2string(ort_out.reshape(-1), precision=7)}")
        print(f"onnx_check_max_abs_error: {max_abs_error:.8g}")
        if max_abs_error > 1e-5:
            raise SystemExit(f"ONNX check failed: max_abs_error={max_abs_error}")

    print("result: PASS residual policy ONNX export")
    return 0
