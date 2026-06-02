"""Utilities for safely running LeRobot policies on the real GELLO robot stack.

The goal of this module is to keep deployment close to LeRobot's inference
contract while still using this repository's existing ZMQ ``RobotEnv`` bridge.
It centralizes the conversion between live GELLO observations and LeRobot policy
batches, plus the safety checks that happen before an action reaches hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np


PANDA_LOWER = np.array(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, 0.0],
    dtype=np.float32,
)
PANDA_UPPER = np.array(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 2.2],
    dtype=np.float32,
)


@dataclass(frozen=True)
class PolicyBundle:
    """Loaded LeRobot policy and the dataset metadata used for construction."""

    policy: Any
    dataset: Any
    device: str


@dataclass(frozen=True)
class SafetyConfig:
    """Limits applied after policy inference and before commanding hardware."""

    max_joint_delta: float = 0.015
    max_gripper_delta: float = 0.03
    joint_lower: Tuple[float, ...] = tuple(PANDA_LOWER.tolist())
    joint_upper: Tuple[float, ...] = tuple(PANDA_UPPER.tolist())
    action_mode: str = "absolute_joint_position"

    def __post_init__(self) -> None:
        if self.action_mode not in {"absolute_joint_position", "delta_joint_position"}:
            raise ValueError(
                "action_mode must be 'absolute_joint_position' or "
                f"'delta_joint_position', got {self.action_mode!r}"
            )
        if self.max_joint_delta <= 0:
            raise ValueError("max_joint_delta must be positive")
        if self.max_gripper_delta <= 0:
            raise ValueError("max_gripper_delta must be positive")
        if len(self.joint_lower) != len(self.joint_upper):
            raise ValueError("joint_lower and joint_upper must have the same length")


@dataclass(frozen=True)
class SafeActionResult:
    """Policy action plus the clipped hardware command derived from it."""

    policy_action: np.ndarray
    raw_delta: np.ndarray
    clipped_delta: np.ndarray
    target: np.ndarray


class LeRobotObservationAdapter:
    """Convert live ``RobotEnv`` observations into a LeRobot policy batch.

    This intentionally keeps the same keys used during dataset creation:
    ``observation.state`` and ``observation.images.<camera>``.  It fails fast on
    missing cameras, wrong image layouts, or wrong state dimensions so a rollout
    does not silently run with a train/deploy schema mismatch.
    """

    def __init__(
        self,
        *,
        device: str,
        camera_keys: Sequence[str] = ("wrist",),
        state_key: str = "joint_positions",
        expected_state_dim: int = 8,
        expected_image_shape: Optional[Tuple[int, int, int]] = (480, 640, 3),
    ) -> None:
        self.device = device
        self.camera_keys = tuple(camera_keys)
        self.state_key = state_key
        self.expected_state_dim = expected_state_dim
        self.expected_image_shape = expected_image_shape
        if not self.camera_keys:
            raise ValueError("At least one camera key is required")

    @property
    def policy_keys(self) -> Tuple[str, ...]:
        return ("observation.state",) + tuple(
            f"observation.images.{camera}" for camera in self.camera_keys
        )

    def make_batch(self, obs: Mapping[str, Any]) -> Dict[str, Any]:
        if self.state_key not in obs:
            raise KeyError(
                f"Live observation is missing {self.state_key!r}. "
                f"Available keys: {list(obs.keys())}"
            )

        state = np.asarray(obs[self.state_key], dtype=np.float32)
        if state.shape != (self.expected_state_dim,):
            raise ValueError(
                f"Expected state shape {(self.expected_state_dim,)}, got {state.shape}"
            )
        if not np.all(np.isfinite(state)):
            raise ValueError(f"Live state contains NaN/Inf: {state}")

        images = {
            camera: self._get_camera_image(obs, camera)
            for camera in self.camera_keys
        }

        import torch

        batch: Dict[str, Any] = {
            "observation.state": torch.from_numpy(state).unsqueeze(0).to(self.device)
        }

        for camera, img in images.items():
            batch[f"observation.images.{camera}"] = self._image_to_tensor(img)

        return batch

    def state_from_obs(self, obs: Mapping[str, Any]) -> np.ndarray:
        state = np.asarray(obs[self.state_key], dtype=np.float32)
        if state.shape != (self.expected_state_dim,):
            raise ValueError(
                f"Expected state shape {(self.expected_state_dim,)}, got {state.shape}"
            )
        return state

    def _get_camera_image(self, obs: Mapping[str, Any], camera: str) -> np.ndarray:
        candidates = (f"{camera}_rgb", camera)
        for key in candidates:
            if key in obs:
                img = np.asarray(obs[key])
                break
        else:
            raise KeyError(
                f"Live observation is missing camera {camera!r}. Tried keys "
                f"{candidates}. Available keys: {list(obs.keys())}"
            )

        if img.ndim != 3 or img.shape[2] != 3:
            raise ValueError(
                f"Camera {camera!r} must be HxWx3 RGB, got shape {img.shape}"
            )
        if (
            self.expected_image_shape is not None
            and img.shape != self.expected_image_shape
        ):
            raise ValueError(
                f"Camera {camera!r} shape mismatch: expected "
                f"{self.expected_image_shape}, got {img.shape}. Fix the camera "
                "pipeline instead of resizing silently during deployment."
            )
        if img.dtype != np.uint8:
            img = img.astype(np.uint8)
        return img

    def _image_to_tensor(self, img: np.ndarray) -> Any:
        import torch

        x = torch.from_numpy(np.ascontiguousarray(img)).float() / 255.0
        x = x.permute(2, 0, 1)  # H,W,C -> C,H,W
        return x.unsqueeze(0).to(self.device)


class SafeJointActionExecutor:
    """Turn a policy output into a bounded joint target for ``RobotEnv.step``."""

    def __init__(self, config: SafetyConfig) -> None:
        self.config = config
        self.lower = np.asarray(config.joint_lower, dtype=np.float32)
        self.upper = np.asarray(config.joint_upper, dtype=np.float32)
        if self.lower.shape != self.upper.shape:
            raise ValueError("joint limits must have the same shape")
        if np.any(self.lower >= self.upper):
            raise ValueError("Every lower joint limit must be smaller than upper limit")

    @property
    def action_dim(self) -> int:
        return int(self.lower.shape[0])

    def make_safe_target(
        self, policy_action: Any, current_state: np.ndarray
    ) -> SafeActionResult:
        action = self._to_numpy_action(policy_action)
        current = np.asarray(current_state, dtype=np.float32)

        if current.shape != (self.action_dim,):
            raise ValueError(
                "Expected current state shape "
                f"{(self.action_dim,)}, got {current.shape}"
            )
        if action.shape != (self.action_dim,):
            raise ValueError(
                f"Expected action shape {(self.action_dim,)}, got {action.shape}"
            )
        if not np.all(np.isfinite(action)):
            raise ValueError(f"Policy produced NaN/Inf action: {action}")

        if self.config.action_mode == "absolute_joint_position":
            raw_delta = action - current
        else:
            raw_delta = action

        clipped_delta = raw_delta.copy()
        clipped_delta[:-1] = np.clip(
            clipped_delta[:-1],
            -self.config.max_joint_delta,
            self.config.max_joint_delta,
        )
        clipped_delta[-1] = np.clip(
            clipped_delta[-1],
            -self.config.max_gripper_delta,
            self.config.max_gripper_delta,
        )

        target = np.clip(current + clipped_delta, self.lower, self.upper).astype(
            np.float32
        )
        return SafeActionResult(
            policy_action=action.astype(np.float32),
            raw_delta=raw_delta.astype(np.float32),
            clipped_delta=clipped_delta.astype(np.float32),
            target=target,
        )

    @staticmethod
    def _to_numpy_action(policy_action: Any) -> np.ndarray:
        if hasattr(policy_action, "detach"):
            policy_action = policy_action.squeeze(0).detach().cpu().numpy()
        return np.asarray(policy_action, dtype=np.float32).reshape(-1)


def load_lerobot_policy(
    *,
    checkpoint: str,
    dataset_root: str,
    repo_id: str,
    device: Optional[str] = None,
) -> PolicyBundle:
    """Load a trained LeRobot policy using the dataset metadata it was trained on."""

    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets import LeRobotDataset
    from lerobot.policies.factory import make_policy

    resolved_device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    dataset = LeRobotDataset(repo_id=repo_id, root=dataset_root)
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.device = resolved_device
    policy = make_policy(cfg=cfg, ds_meta=dataset.meta)
    policy.to(resolved_device)
    policy.eval()
    return PolicyBundle(policy=policy, dataset=dataset, device=resolved_device)


def validate_policy_batch_keys(
    adapter: LeRobotObservationAdapter, dataset_meta: Any
) -> None:
    """Best-effort validation that live batch keys exist in dataset metadata."""

    features = getattr(dataset_meta, "features", None)
    if features is None and isinstance(dataset_meta, Mapping):
        features = dataset_meta.get("features")
    if features is None:
        return

    missing = [key for key in adapter.policy_keys if key not in features]
    if missing:
        raise KeyError(
            "Live policy batch keys are missing from the LeRobot dataset metadata: "
            f"{missing}. Available feature keys: {list(features.keys())}"
        )
