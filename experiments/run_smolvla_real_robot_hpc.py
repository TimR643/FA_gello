"""Run a SmolVLA LeRobot policy on the real GELLO/ZMQ robot stack via SSH tunnel.

This script does not modify existing gello/lerobot/real_robot.py.

It is made for a SmolVLA policy trained with:
    --rename_map='{"observation.images.wrist": "observation.images.camera1"}'
    --policy.empty_cameras=2
"""

from __future__ import annotations

import copy
import time
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import torch
import tyro
from lerobot.policies import make_pre_post_processors

from gello.env import RobotEnv
import gello.lerobot.real_robot as real_robot
from gello.zmq_core.camera_node import ZMQClientCamera
from gello.zmq_core.robot_node import ZMQClientRobot


@dataclass
class Args:
    checkpoint: str
    dataset_root: str
    repo_id: str

    robot_host: str = "127.0.0.1"
    camera_host: Optional[str] = None
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    cameras: Tuple[str, ...] = ("wrist",)

    duration: float = 10.0
    hz: float = 2.0
    execute: bool = False
    require_enter: bool = True
    device: Optional[str] = None

    expected_state_dim: int = 8
    expected_image_height: int = 480
    expected_image_width: int = 640

    max_joint_delta: float = 0.5
    max_gripper_delta: float = 1.0
    action_mode: str = "absolute_joint_position"

    task: str = "Move right when the red block is visible, otherwise move left."


def _make_camera_clients(args: Args) -> dict[str, ZMQClientCamera]:
    host = args.camera_host or args.robot_host
    clients: dict[str, ZMQClientCamera] = {}

    for camera in args.cameras:
        if camera == "wrist":
            clients[camera] = ZMQClientCamera(port=args.wrist_camera_port, host=host)
        elif camera == "base":
            clients[camera] = ZMQClientCamera(port=args.base_camera_port, host=host)
        else:
            raise ValueError(
                f"Unsupported camera {camera!r}; expected 'wrist' or 'base'."
            )

    return clients


SMOLVLA_IMAGE_FEATURES = (
    "observation.images.camera1",
    "observation.images.camera2",
    "observation.images.camera3",
    "observation.images.empty_camera_0",
    "observation.images.empty_camera_1",
)


def _copy_mapping_with_updates(
    original: Any, updates: Dict[str, Any], remove: Sequence[str] = ()
) -> Dict[str, Any]:
    copied = copy.deepcopy(dict(original))
    for key in remove:
        copied.pop(key, None)
    copied.update(updates)
    return copied


def _copy_meta_with_features_and_stats(
    dataset_meta: Any, features: Any, stats: Optional[Any]
) -> Any:
    try:
        patched = copy.copy(dataset_meta)
        setattr(patched, "features", features)
        if stats is not None:
            setattr(patched, "stats", stats)
        return patched
    except Exception:
        attrs = dict(getattr(dataset_meta, "__dict__", {}))
        attrs["features"] = features
        if stats is not None:
            attrs["stats"] = stats
        return SimpleNamespace(**attrs)


def _make_smolvla_dataset_meta(dataset_meta: Any) -> Any:
    """Patch dataset metadata so LeRobot make_policy accepts SmolVLA cameras."""

    features = getattr(dataset_meta, "features", None)
    if features is None and isinstance(dataset_meta, Mapping):
        features = dataset_meta.get("features")
    if features is None or "observation.images.wrist" not in features:
        return dataset_meta

    wrist_feature = copy.deepcopy(features["observation.images.wrist"])
    feature_updates = {
        key: copy.deepcopy(wrist_feature) for key in SMOLVLA_IMAGE_FEATURES
    }
    patched_features = _copy_mapping_with_updates(
        features, feature_updates, remove=("observation.images.wrist",)
    )

    stats = getattr(dataset_meta, "stats", None)
    if stats is None and isinstance(dataset_meta, Mapping):
        stats = dataset_meta.get("stats")

    patched_stats = None
    if stats is not None:
        stat_updates = {}
        if "observation.images.wrist" in stats:
            wrist_stats = copy.deepcopy(stats["observation.images.wrist"])
            stat_updates = {
                key: copy.deepcopy(wrist_stats) for key in SMOLVLA_IMAGE_FEATURES
            }
        patched_stats = _copy_mapping_with_updates(
            stats, stat_updates, remove=("observation.images.wrist",)
        )

    print(
        "Patched SmolVLA metadata: observation.images.wrist -> "
        "camera1/camera2/camera3/empty_camera_0/empty_camera_1"
    )
    return _copy_meta_with_features_and_stats(
        dataset_meta, patched_features, patched_stats
    )


def _load_lerobot_policy_with_patched_meta(
    checkpoint: str,
    dataset_root: str,
    repo_id: str,
    device: Optional[str],
):
    """Load policy like real_robot.load_lerobot_policy, with SmolVLA camera metadata patched."""

    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets import LeRobotDataset
    from lerobot.policies.factory import make_policy

    resolved_device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    dataset = LeRobotDataset(repo_id=repo_id, root=dataset_root)
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.device = resolved_device
    policy_meta = _make_smolvla_dataset_meta(dataset.meta)
    policy = make_policy(cfg=cfg, ds_meta=policy_meta)
    policy.to(resolved_device)
    policy.eval()
    return real_robot.PolicyBundle(
        policy=policy, dataset=dataset, device=resolved_device
    )


def _prepare_smolvla_batch(batch: dict, task: str) -> dict:
    batch["task"] = [task]
    batch["robot_type"] = [""]

    # Runtime mapping: live wrist camera -> policy camera1
    if (
        "observation.images.wrist" in batch
        and "observation.images.camera1" not in batch
    ):
        batch["observation.images.camera1"] = batch.pop("observation.images.wrist")

    if "observation.images.camera1" not in batch:
        raise KeyError(
            "Missing observation.images.camera1 after wrist->camera1 mapping. "
            f"Available keys: {sorted(batch.keys())}"
        )

    cam1 = batch["observation.images.camera1"]

    # Dummy empty cameras because policy was trained with empty_cameras=2
    if "observation.images.camera2" not in batch:
        batch["observation.images.camera2"] = torch.zeros_like(cam1)

    if "observation.images.camera3" not in batch:
        batch["observation.images.camera3"] = torch.zeros_like(cam1)

    if "observation.images.empty_camera_0" not in batch:
        batch["observation.images.empty_camera_0"] = torch.zeros_like(cam1)

    if "observation.images.empty_camera_1" not in batch:
        batch["observation.images.empty_camera_1"] = torch.zeros_like(cam1)

    return batch


def main(args: Args) -> None:
    bundle = _load_lerobot_policy_with_patched_meta(
        checkpoint=args.checkpoint,
        dataset_root=args.dataset_root,
        repo_id=args.repo_id,
        device=args.device,
    )

    preprocess, postprocess = make_pre_post_processors(
        bundle.policy.config,
        args.checkpoint,
        preprocessor_overrides={"device_processor": {"device": str(bundle.device)}},
    )

    adapter = real_robot.LeRobotObservationAdapter(
        device=bundle.device,
        camera_keys=args.cameras,
        expected_state_dim=args.expected_state_dim,
        expected_image_shape=(
            args.expected_image_height,
            args.expected_image_width,
            3,
        ),
    )

    safety = real_robot.SafetyConfig(
        max_joint_delta=args.max_joint_delta,
        max_gripper_delta=args.max_gripper_delta,
        action_mode=args.action_mode,
    )
    executor = real_robot.SafeJointActionExecutor(safety)

    robot = ZMQClientRobot(port=args.robot_port, host=args.robot_host)
    env = RobotEnv(
        robot,
        control_rate_hz=args.hz,
        camera_dict=_make_camera_clients(args),
    )

    print("\nSMOLVLA REAL-ROBOT ROLLOUT VIA HPC")
    print("checkpoint:", args.checkpoint)
    print("dataset_root:", args.dataset_root)
    print("repo_id:", args.repo_id)
    print("device:", bundle.device)
    print("live cameras:", args.cameras)
    print("execute:", args.execute)
    print("duration:", args.duration)
    print("hz:", args.hz)
    print("action_mode:", args.action_mode)
    print("max_joint_delta:", args.max_joint_delta)
    print("max_gripper_delta:", args.max_gripper_delta)
    print("task:", args.task)

    print("\nRuntime mapping:")
    print("  observation.images.wrist   -> observation.images.camera1")
    print("  observation.images.camera2 -> zeros_like(camera1)")
    print("  observation.images.camera3 -> zeros_like(camera1)")

    if not args.execute:
        print("\nDRY RUN: policy will be evaluated, but robot will NOT move.")
    else:
        print("\nEXECUTE MODE: robot CAN move.")
        print("Keep your hand on the enabling switch / emergency stop.")

    if args.require_enter:
        input("\nPress ENTER to start...")

    steps = int(args.duration * args.hz)
    if steps <= 0:
        raise ValueError("duration * hz must produce at least one step.")

    dt = 1.0 / args.hz

    for step in range(steps):
        started = time.time()

        obs = env.get_obs()
        batch = adapter.make_batch(obs)
        state = adapter.state_from_obs(obs)

        with torch.no_grad():
            batch = _prepare_smolvla_batch(batch, args.task)
            batch = preprocess(batch)

            policy_action = bundle.policy.select_action(batch)
            policy_action = postprocess(policy_action)

        safe = executor.make_safe_target(policy_action, state)

        print(f"\nStep {step + 1}/{steps}")
        print("state         :", np.round(state, 3))
        print("policy_action :", np.round(safe.policy_action, 3))
        print("raw_delta     :", np.round(safe.raw_delta, 3))
        print("clipped_delta :", np.round(safe.clipped_delta, 3))
        print("target        :", np.round(safe.target, 3))

        if args.execute:
            env.step(safe.target)

        remaining = dt - (time.time() - started)
        if remaining > 0:
            time.sleep(remaining)

    print("\nFinished SmolVLA real-robot rollout.")


if __name__ == "__main__":
    main(tyro.cli(Args))
