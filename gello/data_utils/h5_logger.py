"""Utilities for streaming robot observations into HDF5 files."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Mapping, Optional

import numpy as np


class H5RobotLogger:
    """Append-only HDF5 logger for robot observations and commands.

    The logger stores one row per control/observation frame. Numeric arrays are
    written as resizable datasets below ``/observations`` and ``/actions``.  Each
    frame also gets ``timestamp_unix`` and ``dt`` entries at the file root.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        metadata: Optional[Mapping[str, Any]] = None,
        compression: Optional[str] = "gzip",
    ) -> None:
        try:
            import h5py
        except ImportError as exc:  # pragma: no cover - exercised without h5py only
            raise ImportError(
                "H5RobotLogger requires h5py. Install it with `pip install h5py`."
            ) from exc

        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._h5py = h5py
        self._file = h5py.File(self.path, "w")
        self._compression = compression
        self._count = 0
        self._last_timestamp: Optional[float] = None
        self._closed = False

        self._file.attrs["format"] = "gello_robot_h5"
        self._file.attrs["format_version"] = "1.0"
        self._file.attrs["created_unix"] = time.time()
        if metadata:
            for key, value in metadata.items():
                self._file.attrs[key] = self._serialize_attr(value)

        self._file.create_group("observations")
        self._file.create_group("actions")
        self._create_dataset("timestamp_unix", np.asarray(0.0, dtype=np.float64))
        self._create_dataset("dt", np.asarray(0.0, dtype=np.float64))

    def __enter__(self) -> "H5RobotLogger":
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()

    @property
    def count(self) -> int:
        return self._count

    def log(
        self,
        observations: Mapping[str, Any],
        *,
        action: Optional[Any] = None,
        timestamp: Optional[float] = None,
        extra: Optional[Mapping[str, Any]] = None,
    ) -> None:
        """Append one frame to the HDF5 file.

        Non-numeric observation entries (for example raw objects) are skipped so
        camera images, joint positions, velocities, torques, forces and other
        numeric arrays can be mixed in the same observation dictionary.
        """

        if self._closed:
            raise RuntimeError("Cannot log to a closed H5RobotLogger")

        now = time.time() if timestamp is None else float(timestamp)
        dt = 0.0 if self._last_timestamp is None else now - self._last_timestamp
        self._last_timestamp = now

        self._append_value("timestamp_unix", now)
        self._append_value("dt", dt)
        self._append_mapping("observations", observations)
        if action is not None:
            self._append_value("actions/joint_command", action)
        if extra:
            self._append_mapping("extra", extra)

        self._count += 1
        self._file.attrs["num_frames"] = self._count
        self._file.flush()

    def close(self) -> None:
        if not self._closed:
            self._file.attrs["num_frames"] = self._count
            self._file.close()
            self._closed = True

    def _append_mapping(self, group_name: str, values: Mapping[str, Any]) -> None:
        if group_name not in self._file:
            self._file.create_group(group_name)
        for key, value in values.items():
            array = self._as_numeric_array(value)
            if array is None:
                continue
            self._append_value(f"{group_name}/{key}", array)

    def _append_value(self, dataset_name: str, value: Any) -> None:
        array = np.asarray(value)
        if dataset_name not in self._file:
            self._create_dataset(dataset_name, array)
        dataset = self._file[dataset_name]
        if dataset.shape[1:] != array.shape:
            raise ValueError(
                f"Shape mismatch for {dataset_name}: expected {dataset.shape[1:]}, "
                f"got {array.shape}"
            )
        dataset.resize((self._count + 1, *dataset.shape[1:]))
        dataset[self._count] = array

    def _create_dataset(self, name: str, sample: np.ndarray) -> None:
        compression = self._compression if sample.size > 1 else None
        self._file.create_dataset(
            name,
            shape=(0, *sample.shape),
            maxshape=(None, *sample.shape),
            chunks=True,
            dtype=sample.dtype,
            compression=compression,
        )

    @staticmethod
    def _as_numeric_array(value: Any) -> Optional[np.ndarray]:
        array = np.asarray(value)
        if array.dtype.kind not in "biufc?":
            return None
        return array

    @staticmethod
    def _serialize_attr(value: Any) -> Any:
        if isinstance(value, (str, bytes, int, float, bool, np.number)):
            return value
        return json.dumps(value)
