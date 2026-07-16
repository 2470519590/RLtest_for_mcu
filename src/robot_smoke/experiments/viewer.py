"""MuJoCo viewer observer for an existing smoke rollout."""

from __future__ import annotations

import time


class MujocoViewerObserver:
    """Render the exact MjData stepped by an experiment, without a second loop."""

    def __init__(self, mujoco, model, realtime: bool) -> None:
        self._mujoco = mujoco
        self._model = model
        self._realtime = realtime
        self._context = None
        self._viewer = None
        self._wall_start = 0.0
        self._sim_start = 0.0

    def start(self, data):
        import mujoco.viewer  # pylint: disable=import-outside-toplevel

        self._context = mujoco.viewer.launch_passive(self._model, data)
        self._viewer = self._context.__enter__()
        self._sim_start = float(data.time)
        self._wall_start = time.perf_counter()
        self._viewer.sync()
        return self.step

    def step(self, data, _step: int) -> bool:
        if self._viewer is None or not self._viewer.is_running():
            return False
        self._viewer.sync()
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
