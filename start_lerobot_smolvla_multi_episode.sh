#!/usr/bin/env bash
set -euo pipefail

# SmolVLA-Base multi-episode rollout for the GELLO/ZMQ Panda stack.
#
# Features:
# - loads SmolVLA only once for the complete session,
# - executes several fixed-duration episodes,
# - pauses and fully resets RTC/action state between episodes,
# - optionally records every policy pass as a LeRobot episode,
# - writes one H5 file per episode directly from the same episode loop,
# - preserves the original flat H5 schema and stores the episode number only in /labels.

source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "${LEROBOT_ENV:-$HOME/miniconda3/envs/lerobot}"

REPO_DIR="${GELLO_REPO_DIR:-$HOME/gello_software}"
cd "$REPO_DIR"
# HOME/lerobot_outputs/train/bottle_task_smolvlabase/checkpoints/060000/pretrained_model
#Lift the bottle and put it to the right-hand side space
: "${CKPT:=$HOME/lerobot_outputs/train/sorting_algorithm_3_pos_final1_smolvlabase/checkpoints/060000/pretrained_model}"
: "${TASK:= Put all red objects into the red box and all other objects into the white box.}"
: "${DURATION:=35}"
: "${FPS:=10}"
: "${RETURN_TO_INITIAL_POSITION:=false}"
: "${DEVICE:=cuda}"
: "${ROBOT_HOST:=127.0.0.1}"
: "${ROBOT_PORT:=6001}"
: "${CAMERA_HOST:=$ROBOT_HOST}"
: "${WRIST_CAMERA_PORT:=5000}"
: "${BASE_CAMERA_PORT:=5001}"
: "${ZMQ_TIMEOUT_MS:=3000}"
: "${MAX_JOINT_DELTA:=0.2}"
: "${MAX_GRIPPER_DELTA:=1.0}"
: "${ACTION_MODE:=absolute_joint_position}"
: "${INFERENCE_TYPE:=rtc}"
: "${RTC_EXECUTION_HORIZON:=10}"
: "${RTC_MAX_GUIDANCE_WEIGHT:=10.0}"
: "${RTC_PREFIX_ATTENTION_SCHEDULE:=}"
: "${LOG_ACTION_DIAGNOSTICS_EVERY_N:=25}"

: "${RECORD_LEROBOT:=false}"
: "${INFERENCE_BASE_DIR:=$HOME/lerobot_inferences}"
: "${LEROBOT_RECORD_PUSH_TO_HUB:=false}"
: "${LEROBOT_RECORD_STREAMING_ENCODING:=true}"

: "${H5_LOG:=true}"
: "${H5_START_LABEL:=1}"
: "${H5_OVERWRITE:=false}"
: "${H5_LOG_DIR:=}"
: "${H5_LOG_BASENAME:=}"

: "${NUM_EPISODES:=20}"
: "${EPISODE_TIME_S:=35}"
: "${RESET_TIME_S:=15}"
: "${FRESH_OBSERVATION_SETTLE_S:=0.10}"
: "${AUTO_INSTALL_MULTI_EPISODE:=true}"
: "${INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS:=false}"
: "${INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK:=false}"

# The robot observation only contains joint positions. The measured Panda
# torques are therefore read from the same GELLO joint-inference CSV source
# that the working reference logger used. The CSV is patched into /features
# after lerobot-rollout exits and the CSV file has been fully flushed/closed.
: "${JOINT_INFERENCE_LOG_ENABLED:=$H5_LOG}"
: "${JOINT_INFERENCE_LOG_DIR:=}"

export PYTHONUNBUFFERED=1

usage() {
  cat <<EOF_USAGE
Usage:
  $0 [--record-lerobot] [--no-record-lerobot]
     [--h5-log] [--no-h5-log] [--sync] [--rtc] [--help]

The SmolVLA model is loaded once. Between episodes RTC, action queues,
processor state and interpolation are cleared. The manual reset pause is not
stored in LeRobot and is not stored in H5.

Episode settings:
  NUM_EPISODES                  Default: 10
  EPISODE_TIME_S                Default: 17
  RESET_TIME_S                  Default: 15
  FRESH_OBSERVATION_SETTLE_S    Default: 0.10

H5 logging:
  H5_LOG                        Default: true
  H5_LOG_DIR                    Default: ~/lerobot_inferences/h5/MODEL/RUN
  H5_LOG_BASENAME               Default: MODEL_smolvla_TIMESTAMP
  H5_START_LABEL                Default: 1
  H5_OVERWRITE                  Default: false

For episode i (1-based):
  episode_number = i
  episode_label  = H5_START_LABEL + i - 1

Example filenames:
  ..._episode_000001.h5
  ..._episode_000002.h5

The H5 structure stays flat and unchanged:
  /features
  /frame_index
  /labels
  /predicted_labels
  /timestamps

Only /labels changes between episodes. Every frame in episode i receives:
  label = H5_START_LABEL + i - 1

Recording:
  --record-lerobot              Save each policy pass as a LeRobot episode
  --h5-log                      Save one synchronized H5 per episode
  Both options can be used together.
EOF_USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --record-lerobot|--record)
      RECORD_LEROBOT=true
      ;;
    --no-record-lerobot|--no-record)
      RECORD_LEROBOT=false
      ;;
    --h5-log|--hdf5-log)
      H5_LOG=true
      ;;
    --no-h5-log|--no-hdf5-log)
      H5_LOG=false
      ;;
    --sync)
      INFERENCE_TYPE=sync
      ;;
    --rtc)
      INFERENCE_TYPE=rtc
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %q\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

install_multi_episode_strategy() {
  python - <<'PY_MULTI_EPISODE_INSTALL'
from __future__ import annotations

import inspect
import py_compile
import re
import shutil
from pathlib import Path

import lerobot.rollout.configs as rollout_configs

rollout_dir = Path(inspect.getfile(rollout_configs)).resolve().parent
configs_path = rollout_dir / "configs.py"
strategies_dir = rollout_dir / "strategies"
factory_path = strategies_dir / "factory.py"
init_path = strategies_dir / "__init__.py"
strategy_path = strategies_dir / "multi_episode.py"

required = [configs_path, factory_path, init_path, strategies_dir / "core.py"]
missing = [str(p) for p in required if not p.exists()]
if missing:
    raise SystemExit("LeRobot rollout files fehlen: " + ", ".join(missing))

strategy_source = r'''# Copyright 2025 The HuggingFace Inc. team.
"Multi-episode SmolVLA rollout with the original flat inference H5 schema."

from __future__ import annotations

import contextlib
import logging
import re
import time
from pathlib import Path
from typing import Any

from lerobot.datasets import VideoEncodingManager
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame
from lerobot.utils.robot_utils import precise_sleep

from ..context import RolloutContext
from .core import RolloutStrategy, safe_push_to_hub, send_next_action

logger = logging.getLogger(__name__)


class MultiEpisodeStrategy(RolloutStrategy):
    "Keep one SmolVLA policy loaded while executing manually reset episodes."

    def setup(self, ctx: RolloutContext) -> None:
        self._init_engine(ctx)

        # Do not allow a chunk to be generated before episode 1 is primed.
        self._engine.pause()
        self._engine.reset()
        self._interpolator.reset()
        self._cached_obs_processed = None
        logger.info(
            "Manual-reset multi-episode strategy ready; SmolVLA remains loaded"
        )

    def _clear_action_state(self, reason: str) -> None:
        "Pause inference and discard policy, processor, queue and interpolation state."
        self._engine.pause()
        self._engine.reset()
        self._interpolator.reset()
        self._cached_obs_processed = None
        logger.info("%s: RTC/action state cleared", reason)

    def _poll_hardware_during_reset(
        self,
        ctx: RolloutContext,
        duration_s: float,
        control_interval: float,
    ) -> None:
        "Keep robot/camera ZMQ streams alive without notifying inference or H5."
        if duration_s <= 0:
            return

        robot = ctx.hardware.robot_wrapper
        deadline = time.perf_counter() + duration_s

        while (
            time.perf_counter() < deadline
            and not ctx.runtime.shutdown_event.is_set()
        ):
            loop_start = time.perf_counter()
            robot.get_observation()
            dt = time.perf_counter() - loop_start
            precise_sleep(max(control_interval - dt, 0.0))

    def _prime_from_current_pose(
        self,
        ctx: RolloutContext,
        reason: str,
    ) -> None:
        "Clear stale RTC state and provide one fresh post-reset observation."
        robot = ctx.hardware.robot_wrapper
        self._clear_action_state(reason)

        fresh_obs = robot.get_observation()
        self._cached_obs_processed = None
        self._process_observation_and_notify(ctx.processors, fresh_obs)

        settle_s = float(self.config.fresh_observation_settle_s)
        if settle_s > 0:
            precise_sleep(settle_s)

        logger.info(
            "%s: fresh current-pose observation installed; queue is clean",
            reason,
        )

    @staticmethod
    def _safe_h5_name(name: str) -> str:
        cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name)).strip("_")
        return cleaned or "unnamed"

    @staticmethod
    def _to_numpy(value: Any):
        import numpy as np

        if hasattr(value, "detach") and hasattr(value, "cpu"):
            value = value.detach().cpu().numpy()

        try:
            arr = np.asarray(value)
        except Exception:
            return None

        if arr.dtype.kind not in "biufc":
            return None

        while arr.ndim > 1 and arr.shape[0] == 1:
            arr = arr[0]

        # Skip images and other large tensors.
        if arr.size > 128:
            return None

        return np.asarray(arr, dtype=np.float64).reshape(-1)

    @classmethod
    def _flatten_numeric(cls, value: Any) -> dict[str, Any]:
        "Collect small numeric values from an observation while skipping images."
        out: dict[str, Any] = {}

        def walk(obj: Any, path: str) -> None:
            path_lower = path.lower()
            if any(token in path_lower for token in ("image", "pixel", "rgb", "depth")):
                return

            if isinstance(obj, dict):
                for key, item in obj.items():
                    child = f"{path}.{key}" if path else str(key)
                    walk(item, child)
                return

            arr = cls._to_numpy(obj)
            if arr is not None:
                out[path or "value"] = arr

        walk(value, "")
        return out

    @staticmethod
    def _normalise_key(key: str) -> str:
        return re.sub(r"[^a-z0-9]+", "", str(key).lower())

    @classmethod
    def _pick_numeric(
        cls,
        flat: dict[str, Any],
        candidates: tuple[str, ...],
        allowed_sizes: tuple[int, ...],
    ):
        wanted = [cls._normalise_key(candidate) for candidate in candidates]

        # Exact full-path match first.
        for candidate in wanted:
            for key, value in flat.items():
                if cls._normalise_key(key) == candidate:
                    if int(value.size) in allowed_sizes:
                        return value.copy()

        # Then allow nested paths ending in the candidate.
        for candidate in wanted:
            for key, value in flat.items():
                normalised = cls._normalise_key(key)
                if normalised.endswith(candidate) and int(value.size) in allowed_sizes:
                    return value.copy()

        return None

    @classmethod
    def _pick_indexed_joint_scalars(
        cls,
        flat: dict[str, Any],
        suffixes: tuple[str, ...],
        allowed_counts: tuple[int, ...] = (8, 7),
    ):
        """Collect scalar telemetry such as joint_0.pos ... joint_7.pos."""
        import numpy as np

        normalised_suffixes = tuple(cls._normalise_key(suffix) for suffix in suffixes)

        # Prefer all eight values (7 arm joints + gripper), then seven arm joints.
        for count in allowed_counts:
            values: list[float] = []
            complete = True

            for index in range(count):
                value_for_joint = None
                endings = tuple(
                    f"joint{index}{suffix}" for suffix in normalised_suffixes
                )

                for key, value in flat.items():
                    normalised_key = cls._normalise_key(key)
                    if (
                        normalised_key.endswith(endings)
                        and int(value.size) == 1
                    ):
                        value_for_joint = float(value.reshape(-1)[0])
                        break

                if value_for_joint is None:
                    complete = False
                    break

                values.append(value_for_joint)

            if complete:
                return np.asarray(values, dtype=np.float64)

        return None

    @classmethod
    def _extract_h5_frame(
        cls,
        raw_observation: dict[str, Any],
        processed_observation: dict[str, Any],
    ) -> dict[str, Any]:
        "Extract the exact 27-column telemetry schema used by the existing H5 files."
        import numpy as np

        raw = cls._flatten_numeric(raw_observation)
        processed = cls._flatten_numeric(processed_observation)

        # Raw robot telemetry has priority. Processed policy state is only a fallback.
        combined = dict(processed)
        combined.update(raw)

        position = cls._pick_numeric(
            combined,
            (
                "observation.state",
                "robot_state",
                "joint_positions",
                "joint_position",
                "joint_pos",
                "qpos",
                "state",
                "q",
            ),
            (7, 8),
        )

        if position is None:
            position = cls._pick_indexed_joint_scalars(
                combined,
                ("pos", "position", "q"),
            )

        gripper_state = cls._pick_numeric(
            combined,
            (
                "gripper_state",
                "gripper_position",
                "gripper_width",
                "gripper",
            ),
            (1,),
        )

        if position is None:
            available = ", ".join(sorted(combined))
            raise RuntimeError(
                "H5 logging could not find a 7D/8D robot state in the observation. "
                f"Available numeric keys: {available}"
            )

        if position.size == 7:
            gripper_value = (
                float(gripper_state[0])
                if gripper_state is not None
                else float("nan")
            )
            position = np.concatenate(
                [position, np.asarray([gripper_value], dtype=np.float64)]
            )
        else:
            position = position[:8]

        velocity = cls._pick_numeric(
            combined,
            (
                "observation.velocity",
                "robot_velocity",
                "joint_velocities",
                "joint_velocity",
                "joint_vel",
                "qvel",
                "dq",
            ),
            (7, 8),
        )

        if velocity is None:
            velocity = cls._pick_indexed_joint_scalars(
                combined,
                ("vel", "velocity", "dq"),
            )

        gripper_velocity = cls._pick_numeric(
            combined,
            (
                "gripper_velocity",
                "gripper_vel",
            ),
            (1,),
        )

        if velocity is not None:
            if velocity.size == 7:
                gripper_velocity_value = (
                    float(gripper_velocity[0])
                    if gripper_velocity is not None
                    else float("nan")
                )
                velocity = np.concatenate(
                    [
                        velocity,
                        np.asarray([gripper_velocity_value], dtype=np.float64),
                    ]
                )
            else:
                velocity = velocity[:8]

        torque = cls._pick_numeric(
            combined,
            (
                "observation.torque",
                "robot_torque",
                "joint_torques",
                "joint_torque",
                "torques",
                "torque",
                "effort",
                "tau",
            ),
            (7, 8),
        )

        if torque is None:
            torque = cls._pick_indexed_joint_scalars(
                combined,
                ("torque", "effort", "tau"),
            )

        gripper_torque = cls._pick_numeric(
            combined,
            (
                "gripper_torque",
                "gripper_effort",
            ),
            (1,),
        )

        if torque is not None:
            if torque.size == 7:
                gripper_torque_value = (
                    float(gripper_torque[0])
                    if gripper_torque is not None
                    else float("nan")
                )
                torque = np.concatenate(
                    [
                        torque,
                        np.asarray([gripper_torque_value], dtype=np.float64),
                    ]
                )
            else:
                torque = torque[:8]

        def scalar(candidates: tuple[str, ...], default: float) -> float:
            value = cls._pick_numeric(combined, candidates, (1,))
            return float(value[0]) if value is not None else float(default)

        return {
            "position": np.asarray(position, dtype=np.float64),
            "velocity": (
                None if velocity is None else np.asarray(velocity, dtype=np.float64)
            ),
            "torque": (
                None if torque is None else np.asarray(torque, dtype=np.float64)
            ),
            "noise_strength": scalar(("noise_strength",), 0.0),
            "blur_strength": scalar(("blur_strength",), 0.0),
            "brightness_strength": scalar(("brightness_strength",), 0.0),
        }

    @staticmethod
    def _estimated_velocities(
        positions,
        timestamps,
        fps: float,
    ):
        import numpy as np

        frame_count = int(positions.shape[0])
        if frame_count == 0:
            return np.empty((0, 8), dtype=np.float64)
        if frame_count == 1:
            return np.zeros((1, 8), dtype=np.float64)

        try:
            if np.all(np.isfinite(timestamps)) and np.all(np.diff(timestamps) > 0):
                return np.gradient(positions, timestamps, axis=0)
        except Exception:
            pass

        return np.gradient(positions, 1.0 / float(fps), axis=0)

    def _write_h5_episode(
        self,
        *,
        episode_number: int,
        episode_label: int,
        task: str,
        checkpoint: str,
        dataset_repo_id: str,
        fps: float,
        elapsed_times: list[float],
        feature_frames: list[dict[str, Any]],
    ) -> Path | None:
        "Write the original flat H5 schema; only /labels changes per episode."
        if not bool(self.config.h5_log_enabled):
            return None

        import h5py
        import numpy as np

        output_dir = Path(self.config.h5_log_dir).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)

        basename = self._safe_h5_name(self.config.h5_log_basename)
        h5_path = output_dir / f"{basename}_episode_{episode_number:06d}.h5"

        if h5_path.exists() and not bool(self.config.h5_overwrite):
            raise FileExistsError(
                f"H5 already exists and h5_overwrite=false: {h5_path}"
            )

        feature_names = [
            "joint_pos_1",
            "joint_pos_2",
            "joint_pos_3",
            "joint_pos_4",
            "joint_pos_5",
            "joint_pos_6",
            "joint_pos_7",
            "gripper_state",
            "joint_vel_1",
            "joint_vel_2",
            "joint_vel_3",
            "joint_vel_4",
            "joint_vel_5",
            "joint_vel_6",
            "joint_vel_7",
            "gripper_velocity",
            "joint_torque_1",
            "joint_torque_2",
            "joint_torque_3",
            "joint_torque_4",
            "joint_torque_5",
            "joint_torque_6",
            "joint_torque_7",
            "gripper_torque",
            "noise_strength",
            "blur_strength",
            "brightness_strength",
        ]

        timestamps = np.asarray(elapsed_times, dtype=np.float64)
        frame_count = int(len(feature_frames))

        if timestamps.shape != (frame_count,):
            raise RuntimeError(
                "H5 timestamps and feature rows are not aligned: "
                f"{timestamps.shape=} versus {frame_count=}"
            )

        if frame_count:
            positions = np.stack(
                [frame["position"] for frame in feature_frames],
                axis=0,
            ).astype(np.float64, copy=False)

            estimated_velocity = self._estimated_velocities(
                positions,
                timestamps,
                fps,
            )

            velocity_rows = []
            torque_rows = []
            disturbance_rows = []

            for row_index, frame in enumerate(feature_frames):
                velocity = frame["velocity"]
                if velocity is None:
                    velocity = estimated_velocity[row_index]
                else:
                    velocity = np.asarray(velocity, dtype=np.float64)
                    missing = ~np.isfinite(velocity)
                    if np.any(missing):
                        velocity = velocity.copy()
                        velocity[missing] = estimated_velocity[row_index][missing]
                velocity_rows.append(velocity)

                torque = frame["torque"]
                if torque is None:
                    torque = np.full(8, np.nan, dtype=np.float64)
                torque_rows.append(np.asarray(torque, dtype=np.float64))

                disturbance_rows.append(
                    [
                        frame["noise_strength"],
                        frame["blur_strength"],
                        frame["brightness_strength"],
                    ]
                )

            velocities = np.stack(velocity_rows, axis=0)
            torques = np.stack(torque_rows, axis=0)
            disturbances = np.asarray(disturbance_rows, dtype=np.float64)
            feature_matrix = np.concatenate(
                [positions, velocities, torques, disturbances],
                axis=1,
            )
        else:
            feature_matrix = np.empty((0, 27), dtype=np.float64)

        if feature_matrix.shape != (frame_count, 27):
            raise RuntimeError(
                f"Unexpected H5 feature shape: {feature_matrix.shape}; "
                f"expected ({frame_count}, 27)"
            )

        with h5py.File(h5_path, "w") as h5:
            # Keep the root attributes and flat dataset layout of the existing
            # ACT/SmolVLA inference H5 files.
            h5.attrs["checkpoint"] = str(checkpoint)
            h5.attrs["dataset_repo_id"] = str(dataset_repo_id)
            h5.attrs["fault_mode"] = "none"
            h5.attrs["fault_strength_steps"] = np.int64(0)
            h5.attrs["fault_strength_strategy"] = "none"
            h5.attrs["fps"] = np.float64(fps)
            h5.attrs["frame_alignment"] = (
                "frame_index resets to zero and aligns with each episode"
            )
            h5.attrs["gripper_source_joint_index"] = np.int64(7)
            h5.attrs["gripper_state_definition"] = (
                "normalized gripper width: 0=closed, 1=fully open"
            )
            h5.attrs["joint_position_unit"] = "radians"
            h5.attrs["joint_torque_unit"] = "newton_meter"
            h5.attrs["joint_velocity_unit"] = "radians_per_second"
            h5.attrs["labels_semantics"] = "continuous_episode_number"
            h5.attrs["num_arm_joints"] = np.int64(7)
            h5.attrs["num_dofs"] = np.int64(8)
            h5.attrs["predicted_labels_semantics"] = "-1 means unavailable"
            h5.attrs["task"] = str(task)
            h5.attrs["timestamp_origin"] = "episode_start"
            h5.attrs["timestamp_unit"] = "seconds"

            feature_dataset = h5.create_dataset(
                "features",
                data=feature_matrix,
                dtype=np.float64,
            )
            feature_dataset.attrs["feature_names"] = np.asarray(
                feature_names,
                dtype=h5py.string_dtype(encoding="utf-8"),
            )

            h5.create_dataset(
                "frame_index",
                data=np.arange(frame_count, dtype=np.int64),
            )
            h5.create_dataset(
                "labels",
                data=np.full(frame_count, episode_label, dtype=np.int64),
            )
            h5.create_dataset(
                "predicted_labels",
                data=np.full(frame_count, -1, dtype=np.int64),
            )
            h5.create_dataset(
                "timestamps",
                data=timestamps,
                dtype=np.float64,
            )

        logger.info(
            "H5 episode saved in original flat schema: %s "
            "(episode=%d, labels=%d, frames=%d)",
            h5_path,
            episode_number,
            episode_label,
            frame_count,
        )
        return h5_path
    def run(self, ctx: RolloutContext) -> None:
        cfg = ctx.runtime.cfg
        strategy_cfg = self.config
        robot = ctx.hardware.robot_wrapper
        engine = self._engine
        interpolator = self._interpolator
        dataset = ctx.data.dataset
        features = ctx.data.dataset_features

        control_interval = interpolator.get_control_interval(cfg.fps)
        num_episodes = int(strategy_cfg.num_episodes)
        episode_time_s = float(strategy_cfg.episode_time_s)
        reset_time_s = float(strategy_cfg.reset_time_s)
        recording = dataset is not None
        task_str = cfg.dataset.single_task if cfg.dataset else cfg.task
        checkpoint = str(getattr(getattr(cfg, "policy", None), "path", "unknown"))
        dataset_repo_id = (
            str(dataset.repo_id)
            if recording
            else f"local/rollout_{self._safe_h5_name(strategy_cfg.h5_log_basename)}"
        )

        logger.info(
            "LeRobot recording=%s, H5 logging=%s, first H5 label=%d",
            recording,
            bool(strategy_cfg.h5_log_enabled),
            int(strategy_cfg.h5_start_label),
        )

        encoding_context = (
            VideoEncodingManager(dataset)
            if recording
            else contextlib.nullcontext()
        )

        with encoding_context:
            self._prime_from_current_pose(ctx, "Before episode 1")

            for episode_index in range(num_episodes):
                if ctx.runtime.shutdown_event.is_set():
                    break

                episode_number = episode_index + 1
                episode_label = int(strategy_cfg.h5_start_label) + episode_index

                logger.info(
                    "Starting episode %d/%d (H5 label=%d, %.1f s)%s",
                    episode_number,
                    num_episodes,
                    episode_label,
                    episode_time_s,
                    " [LeRobot recording]" if recording else "",
                )

                elapsed_times: list[float] = []
                h5_feature_frames: list[dict[str, Any]] = []
                action_frames = 0

                engine.resume()
                episode_start = time.perf_counter()

                while (
                    time.perf_counter() - episode_start < episode_time_s
                    and not ctx.runtime.shutdown_event.is_set()
                ):
                    loop_start = time.perf_counter()
                    obs = robot.get_observation()
                    obs_processed = self._process_observation_and_notify(
                        ctx.processors,
                        obs,
                    )

                    if self._handle_warmup(
                        cfg.use_torch_compile,
                        loop_start,
                        control_interval,
                    ):
                        continue

                    action_dict = send_next_action(
                        obs_processed,
                        obs,
                        ctx,
                        interpolator,
                    )

                    if action_dict is not None:
                        self._log_telemetry(
                            obs_processed,
                            action_dict,
                            ctx.runtime,
                        )

                        if bool(strategy_cfg.h5_log_enabled):
                            elapsed_times.append(time.perf_counter() - episode_start)
                            h5_feature_frames.append(
                                self._extract_h5_frame(obs, obs_processed)
                            )

                        if recording:
                            obs_frame = build_dataset_frame(
                                features,
                                obs_processed,
                                prefix=OBS_STR,
                            )
                            action_frame = build_dataset_frame(
                                features,
                                action_dict,
                                prefix=ACTION,
                            )
                            dataset.add_frame(
                                {
                                    **obs_frame,
                                    **action_frame,
                                    "task": task_str,
                                }
                            )

                        action_frames += 1

                    dt = time.perf_counter() - loop_start
                    sleep_t = control_interval - dt
                    if sleep_t > 0:
                        precise_sleep(sleep_t)
                    else:
                        logger.warning(
                            "Control loop slower than target: %.1f Hz instead of %.1f Hz",
                            1.0 / max(dt, 1e-9),
                            cfg.fps,
                        )

                self._clear_action_state(
                    f"Episode {episode_number}/{num_episodes} finished"
                )

                lerobot_episode_index = -1
                if recording:
                    if action_frames > 0:
                        lerobot_episode_index = int(dataset.num_episodes)
                        dataset.save_episode()
                        logger.info(
                            "LeRobot episode %d saved with %d frames "
                            "(dataset index=%d, dataset total=%d)",
                            episode_number,
                            action_frames,
                            lerobot_episode_index,
                            dataset.num_episodes,
                        )
                    else:
                        logger.warning(
                            "Episode %d produced no ready actions; no empty LeRobot episode saved",
                            episode_number,
                        )

                # Only the flat /labels dataset changes between episodes.
                # Every frame of one H5 receives the same continuous episode label.
                self._write_h5_episode(
                    episode_number=episode_number,
                    episode_label=episode_label,
                    task=task_str,
                    checkpoint=checkpoint,
                    dataset_repo_id=dataset_repo_id,
                    fps=float(cfg.fps),
                    elapsed_times=elapsed_times,
                    feature_frames=h5_feature_frames,
                )

                if (
                    episode_index >= num_episodes - 1
                    or ctx.runtime.shutdown_event.is_set()
                ):
                    break

                logger.info(
                    "MANUAL RESET %.1f s: no policy action, no LeRobot frame and no H5 frame is recorded.",
                    reset_time_s,
                )
                self._poll_hardware_during_reset(
                    ctx,
                    duration_s=reset_time_s,
                    control_interval=control_interval,
                )

                if ctx.runtime.shutdown_event.is_set():
                    break

                self._prime_from_current_pose(
                    ctx,
                    f"Before episode {episode_number + 1}",
                )

        logger.info("All requested episodes finished")

    def teardown(self, ctx: RolloutContext) -> None:
        self._engine.pause()
        self._engine.reset()
        self._interpolator.reset()
        self._cached_obs_processed = None

        dataset = ctx.data.dataset
        cfg = ctx.runtime.cfg

        if dataset is not None:
            logger.info("Finalizing LeRobot dataset...")
            dataset.finalize()
            logger.info(
                "Dataset finalized: %s (%d episodes)",
                dataset.repo_id,
                dataset.num_episodes,
            )

            if cfg.dataset and cfg.dataset.push_to_hub:
                logger.info("Pushing finalized dataset to the Hugging Face Hub...")
                if safe_push_to_hub(
                    dataset,
                    tags=getattr(cfg.dataset, "tags", None),
                    private=getattr(cfg.dataset, "private", False),
                ):
                    logger.info("Dataset uploaded to the Hub")

        self._teardown_hardware(
            ctx.hardware,
            return_to_initial_position=cfg.return_to_initial_position,
        )
        logger.info("Manual-reset multi-episode strategy teardown complete")
'''


config_block = r'''
@RolloutStrategyConfig.register_subclass("multi_episode")
@dataclass
class MultiEpisodeStrategyConfig(RolloutStrategyConfig):
    "Run multiple episodes with manual reset and episode-aligned H5 logging."

    num_episodes: int = 10
    episode_time_s: float = 20.0
    reset_time_s: float = 15.0
    fresh_observation_settle_s: float = 0.10
    h5_log_enabled: bool = True
    h5_log_dir: str = "logs/smolvla_h5"
    h5_log_basename: str = "smolvla_rollout"
    h5_start_label: int = 1
    h5_overwrite: bool = False


'''


def backup_once(path: Path) -> None:
    backup = path.with_name(path.name + ".pre_multi_episode")
    if not backup.exists():
        shutil.copy2(path, backup)

def replace_file(path: Path, content: str) -> None:
    backup_once(path)
    path.write_text(content)

strategies_dir.mkdir(parents=True, exist_ok=True)
strategy_path.write_text(strategy_source)

configs_text = configs_path.read_text()
config_pattern = re.compile(
    r'@RolloutStrategyConfig\.register_subclass\("multi_episode"\)'
    r'.*?'
    r'(?=@RolloutStrategyConfig\.register_subclass\()|\Z',
    flags=re.DOTALL,
)

if config_pattern.search(configs_text):
    configs_text = config_pattern.sub(config_block.lstrip("\n"), configs_text, count=1)
else:
    marker = '@RolloutStrategyConfig.register_subclass("dagger")'
    if marker not in configs_text:
        raise SystemExit(f"Konnte Einfuegepunkt in {configs_path} nicht finden.")
    configs_text = configs_text.replace(marker, config_block + marker, 1)

replace_file(configs_path, configs_text)

factory_text = factory_path.read_text()
if "from .multi_episode import MultiEpisodeStrategy" not in factory_text:
    import_marker = "from .highlight import HighlightStrategy"
    if import_marker not in factory_text:
        import_marker = "from .sentry import SentryStrategy"
    if import_marker not in factory_text:
        raise SystemExit(f"Konnte Import-Einfuegepunkt in {factory_path} nicht finden.")
    factory_text = factory_text.replace(
        import_marker,
        "from .multi_episode import MultiEpisodeStrategy\n" + import_marker,
        1,
    )

if 'config.type == "multi_episode"' not in factory_text:
    dispatch_marker = "    raise ValueError("
    if dispatch_marker not in factory_text:
        raise SystemExit(f"Konnte Factory-Einfuegepunkt in {factory_path} nicht finden.")
    factory_text = factory_text.replace(
        dispatch_marker,
        '    if config.type == "multi_episode":\n'
        '        return MultiEpisodeStrategy(config)\n'
        + dispatch_marker,
        1,
    )
replace_file(factory_path, factory_text)

init_text = init_path.read_text()
if "from .multi_episode import MultiEpisodeStrategy" not in init_text:
    import_marker = "from .highlight import HighlightStrategy"
    if import_marker not in init_text:
        import_marker = "from .sentry import SentryStrategy"
    if import_marker in init_text:
        init_text = init_text.replace(
            import_marker,
            "from .multi_episode import MultiEpisodeStrategy\n" + import_marker,
            1,
        )

if '"MultiEpisodeStrategy",' not in init_text and "__all__" in init_text:
    list_marker = '    "HighlightStrategy",'
    if list_marker in init_text:
        init_text = init_text.replace(
            list_marker,
            list_marker + '\n    "MultiEpisodeStrategy",',
            1,
        )
replace_file(init_path, init_text)

for path in (configs_path, factory_path, init_path, strategy_path):
    py_compile.compile(str(path), doraise=True)

print(f"Multi-episode strategy installed in: {rollout_dir}")
PY_MULTI_EPISODE_INSTALL
}


if [[ "$AUTO_INSTALL_MULTI_EPISODE" == "true" ]]; then
  install_multi_episode_strategy
fi


resolve_local_policy_path() {
  local candidate="$1"

  if [[ -f "$candidate/config.json" ]]; then
    printf '%s\n' "$candidate"
    return 0
  fi

  if [[ -f "$candidate/pretrained_model/config.json" ]]; then
    printf '%s\n' "$candidate/pretrained_model"
    return 0
  fi

  return 1
}

ORIGINAL_CKPT="$CKPT"
if ! CKPT="$(resolve_local_policy_path "$CKPT")"; then
  echo "FEHLT: CKPT muss auf ein pretrained_model-Verzeichnis mit config.json zeigen: $ORIGINAL_CKPT" >&2
  exit 1
fi
export CKPT

POLICY_CONFIG_PATH="$CKPT/config.json"

readarray -t POLICY_META < <(python - "$POLICY_CONFIG_PATH" <<'PY_POLICY_META'
import json
import sys
from pathlib import Path
cfg = json.loads(Path(sys.argv[1]).read_text())
print(cfg.get("type", ""))
PY_POLICY_META
)
POLICY_TYPE="${POLICY_META[0]:-}"

if [[ "$POLICY_TYPE" != "smolvla" ]]; then
  echo "FEHLT: Der Checkpoint ist type=$POLICY_TYPE, erwartet wird type=smolvla: $CKPT" >&2
  exit 2
fi

infer_model_name_from_ckpt() {
  python - "$CKPT" <<'PY_MODEL_NAME'
import sys
from pathlib import Path
p = Path(sys.argv[1]).resolve()
parts = p.parts
if "train" in parts:
    i = parts.index("train")
    if i + 1 < len(parts):
        print(parts[i + 1])
        raise SystemExit
if "checkpoints" in parts:
    i = parts.index("checkpoints")
    if i > 0:
        print(parts[i - 1])
        raise SystemExit
print(p.name)
PY_MODEL_NAME
}

MODEL_NAME_RESOLVED="${MODEL_NAME:-$(infer_model_name_from_ckpt)}"
MODEL_NAME_RESOLVED="${MODEL_NAME_RESOLVED//[^A-Za-z0-9._-]/_}"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"

H5_LOG_DIR_RESOLVED="${H5_LOG_DIR:-$INFERENCE_BASE_DIR/h5/$MODEL_NAME_RESOLVED/$RUN_STAMP}"
H5_LOG_BASENAME_RESOLVED="${H5_LOG_BASENAME:-${MODEL_NAME_RESOLVED}_smolvla_${RUN_STAMP}}"
JOINT_INFERENCE_LOG_DIR_RESOLVED="${JOINT_INFERENCE_LOG_DIR:-$INFERENCE_BASE_DIR/joint_csv/$MODEL_NAME_RESOLVED/$RUN_STAMP}"

if [[ "$H5_LOG" == "true" ]]; then
  mkdir -p "$H5_LOG_DIR_RESOLVED"
  mkdir -p "$JOINT_INFERENCE_LOG_DIR_RESOLVED"
  python - <<'PY_H5_CHECK'
import importlib.util
missing = [name for name in ("h5py", "numpy") if importlib.util.find_spec(name) is None]
if missing:
    raise SystemExit(
        "FEHLT: Pakete fuer H5-Logging fehlen: " + ", ".join(missing)
        + "\nInstalliere: python -m pip install " + " ".join(missing)
    )
PY_H5_CHECK
fi


infer_policy_image_names() {
  python - "$POLICY_CONFIG_PATH" <<'PY'
import json
import sys
from pathlib import Path

cfg = json.loads(Path(sys.argv[1]).read_text())
input_features = cfg.get("input_features", {})
names = []

def add_from_key(key):
    if isinstance(key, str) and key.startswith("observation.images."):
        name = key.removeprefix("observation.images.")
        if name not in names:
            names.append(name)

for key, feat in input_features.items():
    if key.startswith("observation.images."):
        if isinstance(feat, dict):
            typ = str(feat.get("type", ""))
            if feat.get("type") in ("VISUAL", "FeatureType.VISUAL") or typ.endswith("VISUAL"):
                add_from_key(key)
        else:
            add_from_key(key)

if not names:
    def walk(value):
        if isinstance(value, dict):
            for key, item in value.items():
                add_from_key(key)
                walk(item)
        elif isinstance(value, list):
            for item in value:
                walk(item)
    walk(cfg)

print(",".join(names))
PY
}

filter_policy_camera_names_for_robot() {
  python - "$1" "$INCLUDE_EMPTY_CAMERAS_IN_ROBOT_OBS" "$INCLUDE_UNMAPPED_POLICY_CAMERAS_AS_BLACK" <<'PY'
import sys
names = [name.strip() for name in sys.argv[1].split(",") if name.strip()]
include_empty = sys.argv[2].lower() == "true"
include_unmapped = sys.argv[3].lower() == "true"
live_mappable = {"camera1", "camera2", "wrist", "base"}
selected, omitted_empty, omitted_unmapped = [], [], []
for name in names:
    if name.startswith("empty_camera"):
        (selected if include_empty else omitted_empty).append(name)
    elif name in live_mappable:
        selected.append(name)
    else:
        (selected if include_unmapped else omitted_unmapped).append(name)
print(",".join(selected))
if omitted_empty:
    print("OMITTED_EMPTY=" + ",".join(omitted_empty), file=sys.stderr)
if omitted_unmapped:
    print("OMITTED_UNMAPPED=" + ",".join(omitted_unmapped), file=sys.stderr)
PY
}

infer_live_camera_names_from_policy_names() {
  python - "$1" <<'PY'
import sys
names = [name.strip() for name in sys.argv[1].split(",") if name.strip()]
live = []
for name in names:
    if name in {"camera1", "wrist"} and "wrist" not in live:
        live.append("wrist")
    elif name in {"camera2", "base"} and "base" not in live:
        live.append("base")
print(",".join(live) if live else "wrist")
PY
}

INFERRED_POLICY_IMAGE_NAMES="$(infer_policy_image_names)"
ROBOT_POLICY_CAMERA_NAMES_DEFAULT="$(filter_policy_camera_names_for_robot "$INFERRED_POLICY_IMAGE_NAMES")"
POLICY_CAMERA_NAMES_RESOLVED="${POLICY_CAMERA_NAMES:-$ROBOT_POLICY_CAMERA_NAMES_DEFAULT}"
LIVE_CAMERA_NAMES_RESOLVED="${LIVE_CAMERA_NAMES:-${CAMERA_NAMES:-$(infer_live_camera_names_from_policy_names "$POLICY_CAMERA_NAMES_RESOLVED")}}"

if [[ -z "$POLICY_CAMERA_NAMES_RESOLVED" ]]; then
  echo "FEHLT: Konnte keine nutzbaren Policy-Kameras aus $POLICY_CONFIG_PATH ableiten." >&2
  exit 1
fi

if (( ZMQ_TIMEOUT_MS < 3000 )); then
  echo "WARNUNG: ZMQ_TIMEOUT_MS=$ZMQ_TIMEOUT_MS ist fuer Remote-Kameras knapp; empfohlen sind mindestens 3000 ms." >&2
fi

python - "$FPS" "$RTC_EXECUTION_HORIZON" "$NUM_EPISODES" "$H5_START_LABEL" <<'PY_VALIDATE'
import sys
fps = float(sys.argv[1])
horizon = int(sys.argv[2])
num_episodes = int(sys.argv[3])
start_label = int(sys.argv[4])
if fps <= 0:
    raise SystemExit("FEHLT: FPS muss groesser als 0 sein.")
if horizon <= 0:
    raise SystemExit("FEHLT: RTC_EXECUTION_HORIZON muss groesser als 0 sein.")
if num_episodes <= 0:
    raise SystemExit("FEHLT: NUM_EPISODES muss groesser als 0 sein.")
if start_label < 0:
    raise SystemExit("FEHLT: H5_START_LABEL darf nicht negativ sein.")
PY_VALIDATE

if [[ "$DEVICE" == cuda* ]]; then
  python - <<'PY_CUDA_CHECK'
import torch
if not torch.cuda.is_available():
    raise SystemExit("FEHLT: DEVICE=cuda, aber torch.cuda.is_available() ist false.")
PY_CUDA_CHECK
fi

ROLLOUT_STRATEGY="${STRATEGY_TYPE:-multi_episode}"
if [[ "$ROLLOUT_STRATEGY" != "multi_episode" ]]; then
  echo "FEHLT: Dieses Skript erwartet STRATEGY_TYPE=multi_episode." >&2
  exit 2
fi

cat <<EOF_CONFIG
Using SmolVLA-Base multi-episode rollout with robot.type=gello_zmq
CKPT=$CKPT
POLICY_TYPE=$POLICY_TYPE
MODEL_NAME=$MODEL_NAME_RESOLVED
TASK=$TASK
FPS=$FPS
DEVICE=$DEVICE
NUM_EPISODES=$NUM_EPISODES
EPISODE_TIME_S=$EPISODE_TIME_S
RESET_TIME_S=$RESET_TIME_S
FRESH_OBSERVATION_SETTLE_S=$FRESH_OBSERVATION_SETTLE_S
RECORD_LEROBOT=$RECORD_LEROBOT
H5_LOG=$H5_LOG
H5_LOG_DIR=$H5_LOG_DIR_RESOLVED
H5_LOG_BASENAME=$H5_LOG_BASENAME_RESOLVED
H5_START_LABEL=$H5_START_LABEL
H5_LAST_LABEL=$((H5_START_LABEL + NUM_EPISODES - 1))
H5_OVERWRITE=$H5_OVERWRITE
INFERENCE_TYPE=$INFERENCE_TYPE
RTC_EXECUTION_HORIZON=$RTC_EXECUTION_HORIZON
RTC_MAX_GUIDANCE_WEIGHT=$RTC_MAX_GUIDANCE_WEIGHT
INFERRED_POLICY_IMAGE_NAMES=${INFERRED_POLICY_IMAGE_NAMES:-<none>}
ROBOT_POLICY_CAMERA_NAMES=$POLICY_CAMERA_NAMES_RESOLVED
LIVE_CAMERA_NAMES=$LIVE_CAMERA_NAMES_RESOLVED
MAX_JOINT_DELTA=$MAX_JOINT_DELTA
MAX_GRIPPER_DELTA=$MAX_GRIPPER_DELTA
ACTION_MODE=$ACTION_MODE
JOINT_INFERENCE_LOG_ENABLED=$JOINT_INFERENCE_LOG_ENABLED
JOINT_INFERENCE_LOG_DIR=$JOINT_INFERENCE_LOG_DIR_RESOLVED
EOF_CONFIG

cmd=(
  lerobot-rollout
  --strategy.type="$ROLLOUT_STRATEGY"
  --policy.path="$CKPT"
  --fps="$FPS"
  --duration="$DURATION"
  --device="$DEVICE"
  --return_to_initial_position="$RETURN_TO_INITIAL_POSITION"
  --robot.type=gello_zmq
  --robot.robot_host="$ROBOT_HOST"
  --robot.robot_port="$ROBOT_PORT"
  --robot.camera_host="$CAMERA_HOST"
  --robot.wrist_camera_port="$WRIST_CAMERA_PORT"
  --robot.base_camera_port="$BASE_CAMERA_PORT"
  --robot.zmq_timeout_ms="$ZMQ_TIMEOUT_MS"
  --robot.camera_names="$LIVE_CAMERA_NAMES_RESOLVED"
  --robot.policy_camera_names="$POLICY_CAMERA_NAMES_RESOLVED"
  --robot.max_joint_delta="$MAX_JOINT_DELTA"
  --robot.max_gripper_delta="$MAX_GRIPPER_DELTA"
  --robot.action_mode="$ACTION_MODE"
  --robot.log_action_diagnostics_every_n="$LOG_ACTION_DIAGNOSTICS_EVERY_N"
  --robot.joint_inference_log_enabled="$JOINT_INFERENCE_LOG_ENABLED"
  --robot.joint_inference_log_dir="$JOINT_INFERENCE_LOG_DIR_RESOLVED"
  --task="$TASK"
  --strategy.num_episodes="$NUM_EPISODES"
  --strategy.episode_time_s="$EPISODE_TIME_S"
  --strategy.reset_time_s="$RESET_TIME_S"
  --strategy.fresh_observation_settle_s="$FRESH_OBSERVATION_SETTLE_S"
  --strategy.h5_log_enabled="$H5_LOG"
  --strategy.h5_log_dir="$H5_LOG_DIR_RESOLVED"
  --strategy.h5_log_basename="$H5_LOG_BASENAME_RESOLVED"
  --strategy.h5_start_label="$H5_START_LABEL"
  --strategy.h5_overwrite="$H5_OVERWRITE"
)

if [[ "$INFERENCE_TYPE" == "rtc" ]]; then
  cmd+=(
    --inference.type=rtc
    --inference.rtc.execution_horizon="$RTC_EXECUTION_HORIZON"
    --inference.rtc.max_guidance_weight="$RTC_MAX_GUIDANCE_WEIGHT"
  )
  if [[ -n "$RTC_PREFIX_ATTENTION_SCHEDULE" ]]; then
    cmd+=(--inference.rtc.prefix_attention_schedule="$RTC_PREFIX_ATTENTION_SCHEDULE")
  fi
elif [[ "$INFERENCE_TYPE" == "sync" ]]; then
  cmd+=(--inference.type=sync)
else
  echo "Unsupported INFERENCE_TYPE=$INFERENCE_TYPE; expected rtc or sync" >&2
  exit 2
fi

if [[ "$RECORD_LEROBOT" == "true" ]]; then
  LEROBOT_RECORD_ROOT_RESOLVED="${LEROBOT_RECORD_ROOT:-$INFERENCE_BASE_DIR/${MODEL_NAME_RESOLVED}_smolvla_inference_${RUN_STAMP}}"
  LEROBOT_RECORD_REPO_ID_RESOLVED="${LEROBOT_RECORD_REPO_ID:-local/rollout_${MODEL_NAME_RESOLVED}_smolvla_inference_${RUN_STAMP}}"
  LEROBOT_RECORD_FPS_RESOLVED="${LEROBOT_RECORD_FPS:-$FPS}"
  LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED="${LEROBOT_RECORD_PUSH_TO_HUB:-false}"
  LEROBOT_RECORD_STREAMING_ENCODING_RESOLVED="${LEROBOT_RECORD_STREAMING_ENCODING:-true}"

  if [[ "$LEROBOT_RECORD_REPO_ID_RESOLVED" != */rollout_* ]]; then
    echo "FEHLT: LEROBOT_RECORD_REPO_ID muss mit local/rollout_ bzw. USER/rollout_ beginnen: $LEROBOT_RECORD_REPO_ID_RESOLVED" >&2
    exit 2
  fi

  mkdir -p "$(dirname "$LEROBOT_RECORD_ROOT_RESOLVED")"

  cmd+=(
    --dataset.root="$LEROBOT_RECORD_ROOT_RESOLVED"
    --dataset.repo_id="$LEROBOT_RECORD_REPO_ID_RESOLVED"
    --dataset.single_task="$TASK"
    --dataset.fps="$LEROBOT_RECORD_FPS_RESOLVED"
    --dataset.push_to_hub="$LEROBOT_RECORD_PUSH_TO_HUB_RESOLVED"
    --dataset.streaming_encoding="$LEROBOT_RECORD_STREAMING_ENCODING_RESOLVED"
    --dataset.num_episodes="$NUM_EPISODES"
    --dataset.episode_time_s="$EPISODE_TIME_S"
    --dataset.reset_time_s="$RESET_TIME_S"
  )
fi

cat <<EOF_MULTI

Multi-episode SmolVLA session:
  Episodes:          $NUM_EPISODES
  Episode duration:  $EPISODE_TIME_S s
  Manual reset:      $RESET_TIME_S s
  Record LeRobot:    $RECORD_LEROBOT
  H5 logging:        $H5_LOG
  H5 labels:         $H5_START_LABEL .. $((H5_START_LABEL + NUM_EPISODES - 1))
${H5_LOG_DIR_RESOLVED:+  H5 directory:      $H5_LOG_DIR_RESOLVED}
${LEROBOT_RECORD_ROOT_RESOLVED:+  Dataset root:      $LEROBOT_RECORD_ROOT_RESOLVED}
${LEROBOT_RECORD_REPO_ID_RESOLVED:+  Dataset repo ID:   $LEROBOT_RECORD_REPO_ID_RESOLVED}

During every reset pause:
  - RTC remains paused.
  - Policy/action/interpolation state is empty.
  - No policy action is sent.
  - No LeRobot frame is recorded.
  - No H5 frame is recorded.

For each episode, the H5 layout stays identical to the existing flat
inference files. Only every value in /labels is set from the loop index.
The SmolVLA model remains loaded for the complete session.
Press Ctrl+C to stop safely.

EOF_MULTI

patch_h5_torques_from_joint_csv() {
  if [[ "$H5_LOG" != "true" ]]; then
    return 0
  fi

  if [[ "$JOINT_INFERENCE_LOG_ENABLED" != "true" ]]; then
    echo "FEHLT: H5 torque logging requires JOINT_INFERENCE_LOG_ENABLED=true." >&2
    return 1
  fi

  python - \
    "$JOINT_INFERENCE_LOG_DIR_RESOLVED" \
    "$H5_LOG_DIR_RESOLVED" \
    "$H5_LOG_BASENAME_RESOLVED" <<'PY_PATCH_TORQUES'
from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

import h5py
import numpy as np

csv_dir = Path(sys.argv[1]).expanduser()
h5_dir = Path(sys.argv[2]).expanduser()
basename = sys.argv[3]

h5_paths = sorted(h5_dir.glob(f"{basename}_episode_*.h5"))
if not h5_paths:
    raise SystemExit(f"FEHLT: Keine H5-Episoden zum Torque-Patchen gefunden in {h5_dir}")

csv_paths = sorted(
    csv_dir.glob("*.csv"),
    key=lambda path: (path.stat().st_mtime_ns, path.name),
)
if not csv_paths:
    raise SystemExit(
        "FEHLT: Kein GELLO joint-inference CSV gefunden. "
        f"Erwartetes Verzeichnis: {csv_dir}"
    )


def lower(name: str) -> str:
    return str(name).strip().lower()


def is_action_like(name: str) -> bool:
    value = lower(name)
    return any(token in value for token in (
        "action", "target", "command", "desired", "sent", "policy",
    ))


def is_torque_column(name: str) -> bool:
    value = lower(name)
    if is_action_like(value):
        return False
    return any(token in value for token in (
        "joint_torque", "torque", "effort", "tau_", "tau.", "tau[",
    ))


def joint_index(name: str) -> int | None:
    value = lower(name)
    patterns = (
        r"joint[^0-9]*([0-9]+)",
        r"(?:^|[^a-z])j[^0-9]*([0-9]+)",
        r"tau[^0-9]*([0-9]+)",
        r"torque[^0-9]*([0-9]+)",
        r"effort[^0-9]*([0-9]+)",
    )
    for pattern in patterns:
        match = re.search(pattern, value)
        if match:
            return int(match.group(1))
    return None


def column_score(name: str) -> int:
    value = lower(name)
    score = 0
    if "joint_torque" in value:
        score += 40
    if "measured" in value or "actual" in value or "current" in value:
        score += 20
    if "robot" in value or "state" in value:
        score += 5
    if "external" in value:
        score -= 5
    return score


all_rows: list[dict[str, str]] = []
field_order: list[str] = []
for path in csv_paths:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            continue
        for name in reader.fieldnames:
            if name not in field_order:
                field_order.append(name)
        all_rows.extend(dict(row) for row in reader)

if not all_rows:
    raise SystemExit(f"FEHLT: Joint-CSV-Dateien enthalten keine Datenzeilen: {csv_dir}")

candidates = [name for name in field_order if is_torque_column(name)]
if not candidates:
    raise SystemExit(
        "FEHLT: Keine Torque-Spalten im GELLO CSV erkannt. Verfügbare Spalten: "
        + ", ".join(field_order)
    )

# Prefer one best measured column per explicit joint index. This avoids
# accidentally selecting both measured and auxiliary/external torque columns.
indexed: dict[int, str] = {}
unindexed: list[str] = []
for name in candidates:
    index = joint_index(name)
    if index is None:
        unindexed.append(name)
        continue
    previous = indexed.get(index)
    if previous is None or column_score(name) > column_score(previous):
        indexed[index] = name

if len(indexed) >= 7:
    indices = sorted(indexed)
    # Accept zero-based joint_0..joint_7 and one-based joint_1..joint_8.
    if all(index in indexed for index in range(0, 7)):
        ordered_columns = [indexed[index] for index in range(0, 8) if index in indexed]
    elif all(index in indexed for index in range(1, 8)):
        ordered_columns = [indexed[index] for index in range(1, 9) if index in indexed]
    else:
        ordered_columns = [indexed[index] for index in indices[:8]]
else:
    ordered_columns = candidates[:8]

if len(ordered_columns) < 7:
    raise SystemExit(
        "FEHLT: Weniger als sieben gemessene Torque-Spalten erkannt: "
        + ", ".join(ordered_columns)
    )

numeric_rows: list[list[float]] = []
for row in all_rows:
    values: list[float] = []
    for column in ordered_columns:
        raw = row.get(column, "")
        try:
            values.append(float(raw))
        except (TypeError, ValueError):
            values.append(float("nan"))
    # Ignore incomplete/metadata lines, but keep genuine rows with at least
    # the seven Panda arm torque values.
    if np.count_nonzero(np.isfinite(values[:7])) == 7:
        numeric_rows.append(values)

if not numeric_rows:
    raise SystemExit(
        "FEHLT: Torque-Spalten wurden gefunden, enthalten aber keine vollständigen "
        "numerischen Panda-Torque-Zeilen."
    )

csv_torques = np.asarray(numeric_rows, dtype=np.float64)
if csv_torques.ndim != 2 or csv_torques.shape[1] < 7:
    raise SystemExit(f"FEHLT: Unerwartete Torque-Matrix aus CSV: {csv_torques.shape}")

frame_counts: list[int] = []
for path in h5_paths:
    with h5py.File(path, "r") as h5:
        if "features" not in h5:
            raise SystemExit(f"FEHLT: /features fehlt in {path}")
        shape = tuple(h5["features"].shape)
        if len(shape) != 2 or shape[1] != 27:
            raise SystemExit(f"FEHLT: Unerwartete /features-Shape in {path}: {shape}")
        frame_counts.append(int(shape[0]))

total_frames = sum(frame_counts)
if csv_torques.shape[0] < total_frames:
    raise SystemExit(
        "FEHLT: Zu wenige GELLO-Torque-Zeilen für die H5-Frames: "
        f"CSV={csv_torques.shape[0]}, H5={total_frames}. "
        "Prüfe JOINT_INFERENCE_LOG_ENABLED=true und den GELLO CSV-Logger."
    )

# A custom reused CSV directory can contain startup/older rows. The H5 files
# belong to the current rollout and are aligned with the most recent actions.
if csv_torques.shape[0] > total_frames:
    extra = csv_torques.shape[0] - total_frames
    print(f"Hinweis: {extra} ältere/zusätzliche CSV-Zeilen werden übersprungen.")
    csv_torques = csv_torques[-total_frames:]

cursor = 0
for path, frame_count in zip(h5_paths, frame_counts, strict=True):
    segment = csv_torques[cursor:cursor + frame_count]
    cursor += frame_count

    with h5py.File(path, "r+") as h5:
        features = h5["features"][:]
        # Exact flat schema: torque columns are 16..22; column 23 is the
        # gripper torque. The reference H5 also leaves gripper_torque as NaN
        # when GELLO only provides the seven Panda arm torques.
        features[:, 16:23] = segment[:, :7]
        if segment.shape[1] >= 8 and np.any(np.isfinite(segment[:, 7])):
            features[:, 23] = segment[:, 7]
        h5["features"][:] = features

        arm_torques = h5["features"][:, 16:23]
        if np.isnan(arm_torques).all():
            raise SystemExit(f"FEHLT: Arm-Torques sind nach dem Patch weiterhin vollständig NaN: {path}")

    print(
        f"Torques eingetragen: {path.name} "
        f"({frame_count} Frames, CSV-Zeilen {cursor-frame_count}:{cursor})"
    )

print("Verwendete Torque-Spalten:")
for column in ordered_columns[:8]:
    print(f"  - {column}")
print(f"Torque-Patch abgeschlossen: {len(h5_paths)} H5-Episoden, {total_frames} Frames")
PY_PATCH_TORQUES
}

printf 'Executing:'
printf ' %q' "${cmd[@]}"
printf '\n'

rollout_exit=0
"${cmd[@]}" || rollout_exit=$?

# The CSV writer is closed when lerobot-rollout exits, so all measured torque
# rows are now available. Patch only the torque columns; layout and labels stay unchanged.
if [[ "$H5_LOG" == "true" ]]; then
  patch_h5_torques_from_joint_csv
fi

exit "$rollout_exit"
