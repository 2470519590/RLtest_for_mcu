"""Root entry for exporting a residual PPO policy to ONNX."""

from __future__ import annotations

from server_training.export_policy_onnx import main


if __name__ == "__main__":
    raise SystemExit(main())
