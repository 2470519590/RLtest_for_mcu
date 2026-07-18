"""MuJoCo viewer observer for an existing smoke rollout."""

from __future__ import annotations

import time


class MujocoViewerObserver:
    """Render the exact MjData stepped by an experiment, without a second loop."""

    def __init__(self, mujoco, model, realtime: bool, sync_hz: float = 60.0) -> None:
        self._mujoco = mujoco
        self._model = model
        self._realtime = realtime
        self._context = None
        self._viewer = None
        self._wall_start = 0.0
        self._sim_start = 0.0
        self._last_sync_sim = 0.0
        self._sync_interval = 1.0 / max(float(sync_hz), 1.0)
        self._phase = "ground"

    def start(self, data):
        import mujoco.viewer  # pylint: disable=import-outside-toplevel

        self._context = mujoco.viewer.launch_passive(self._model, data)
        self._viewer = self._context.__enter__()
        self._sim_start = float(data.time)
        self._last_sync_sim = self._sim_start
        self._wall_start = time.perf_counter()
        self._add_overlay(data)
        self._viewer.sync()
        return self.step

    def _add_overlay(self, data) -> None:
        if self._viewer is None:
            return
        labels = ("time\nphase", f"{float(data.time):.3f} s\n{self._phase}")
        try:
            if hasattr(self._viewer, "set_texts"):
                self._viewer.set_texts((None, self._mujoco.mjtGridPos.mjGRID_TOPLEFT, labels[0], labels[1]))
            elif hasattr(self._viewer, "add_overlay"):
                self._viewer.add_overlay(self._mujoco.mjtGridPos.mjGRID_TOPLEFT, labels[0], labels[1])
        except Exception:  # pragma: no cover - viewer overlay support depends on mujoco build.
            return

    def step(self, data, _step: int, phase: str | None = None) -> bool:
        if self._viewer is None or not self._viewer.is_running():
            return False
        if phase is not None:
            self._phase = phase
        if float(data.time) - self._last_sync_sim >= self._sync_interval:
            self._add_overlay(data)
            self._viewer.sync()
            self._last_sync_sim = float(data.time)
        if self._realtime:
            target_elapsed = float(data.time) - self._sim_start
            delay = target_elapsed - (time.perf_counter() - self._wall_start)
            if delay > 0.0:
                time.sleep(delay)
        return True

    def close(self) -> None:
        if self._context is not None:
            self._context.__exit__(None, None, None)
            self._context = None
            self._viewer = None
