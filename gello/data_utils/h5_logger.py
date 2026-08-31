"""Incremental HDF5 logging shared by GELLO recording paths."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


class H5RobotLogger:
    """Append synchronized robot measurements and commands directly to HDF5."""

    def __init__(self, path: str | Path, *, num_dofs: int, flush_every: int = 1):
        try:
            import h5py
        except ImportError as exc:
            raise RuntimeError(
                "H5 logging requires h5py; install it with `python -m pip install h5py`."
            ) from exc

        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.num_dofs = num_dofs
        self.flush_every = max(1, int(flush_every))
        self._h5 = h5py.File(self.path, "w")
        self._samples = 0

        self._h5.attrs["format"] = "gello_lerobot_h5"
        self._h5.attrs["format_version"] = 1
        self._h5.attrs["created_utc"] = datetime.now(timezone.utc).isoformat()
        self._h5.attrs["num_dofs"] = num_dofs
        state = self._h5.create_group("state")
        action = self._h5.create_group("action")
        self._time = self._vector("time", np.float64)
        self._action_time = self._vector("action/time", np.float64)
        self._q = self._matrix(state, "q")
        self._dq = self._matrix(state, "dq")
        self._tau = self._matrix(state, "tau")
        self._raw_action = self._matrix(action, "raw")
        self._sent_action = self._matrix(action, "sent")
        state["q"].attrs["unit"] = "rad (last element: normalized gripper)"
        state["dq"].attrs["unit"] = "rad/s (last element: normalized gripper/s)"
        state["tau"].attrs["unit"] = "Nm (last element unavailable)"
        action["raw"].attrs["description"] = "command received from policy or teleoperator"
        action["sent"].attrs["description"] = "command sent to the robot"
        self._h5.flush()

    def _vector(self, name: str, dtype: Any):
        return self._h5.create_dataset(name, shape=(0,), maxshape=(None,), dtype=dtype)

    def _matrix(self, group: Any, name: str):
        return group.create_dataset(
            name,
            shape=(0, self.num_dofs),
            maxshape=(None, self.num_dofs),
            chunks=(max(1, min(256, self.flush_every)), self.num_dofs),
            dtype=np.float32,
            fillvalue=np.nan,
        )

    @staticmethod
    def _append(dataset: Any, value: Any) -> None:
        dataset.resize(dataset.shape[0] + 1, axis=0)
        dataset[-1] = value

    def log_observation(
        self, timestamp_s: float, q: np.ndarray, dq: np.ndarray, tau: np.ndarray
    ) -> None:
        self._append(self._time, timestamp_s)
        self._append(self._q, q)
        self._append(self._dq, dq)
        self._append(self._tau, tau)
        nan = np.full(self.num_dofs, np.nan, dtype=np.float32)
        self._append(self._action_time, np.nan)
        self._append(self._raw_action, nan)
        self._append(self._sent_action, nan)
        self._samples += 1
        if self._samples % self.flush_every == 0:
            self._h5.flush()

    def log_action(self, timestamp_s: float, raw: np.ndarray, sent: np.ndarray) -> None:
        if not self._samples:
            return
        index = self._samples - 1
        self._action_time[index] = timestamp_s
        self._raw_action[index] = np.asarray(raw, dtype=np.float32).reshape(self.num_dofs)
        self._sent_action[index] = np.asarray(sent, dtype=np.float32).reshape(self.num_dofs)
        if self._samples % self.flush_every == 0:
            self._h5.flush()

    def close(self) -> None:
        if self._h5 is not None:
            self._h5.flush()
            self._h5.close()
            self._h5 = None
