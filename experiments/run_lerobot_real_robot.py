"""Safely run a trained LeRobot SmolVLA policy on the real GELLO/ZMQ robot stack.

Default mode is a dry run: live observations are read and the policy is queried,
but no command is sent to the robot unless ``--execute`` is passed.

This version is adapted for a SmolVLA policy trained with:
    observation.images.wrist -> observation.images.camera1
and:
    policy.empty_cameras=2

So at runtime by default:
    live wrist camera  -> observation.images.camera1
    live base camera   -> observation.images.camera2 (when --cameras wrist base)
    camera3            -> zero dummy image
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import torch
import tyro
from lerobot.policies import make_pre_post_processors

from gello.env import RobotEnv
from gello.lerobot.real_robot import (
    LeRobotObservationAdapter,
    SafeJointActionExecutor,
    SafetyConfig,
    load_lerobot_policy,
)
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
    smolvla_image_keys: Tuple[str, ...] = ("camera1", "camera2", "camera3")

    duration: float = 10.0
    hz: float = 2.0
    execute: bool = False
    require_enter: bool = True
    device: Optional[str] = None

    expected_state_dim: int = 8
    expected_image_height: int = 480
    expected_image_width: int = 640

    max_joint_delta: float = 0.005
    max_gripper_delta: float = 0.01
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


def _prepare_smolvla_batch(
    batch: dict, task: str, smolvla_image_keys: Tuple[str, ...]
) -> dict:
    """Prepare live GELLO/LeRobot batch for SmolVLA inference.

    The live robot adapter creates keys such as:
        observation.images.wrist
        observation.images.base

    SmolVLA trainings in this repository commonly use generic camera keys such
    as camera1/camera2/camera3. Runtime mapping is positional: the cameras
    passed via ``--cameras`` are mapped, in insertion order, to
    ``--smolvla-image-keys``. Missing SmolVLA image keys are filled with black
    dummy images so checkpoints trained with empty cameras can still run.
    """

    batch["task"] = [task]
    batch["robot_type"] = [""]

    live_image_keys = [
        key for key in list(batch.keys()) if key.startswith("observation.images.")
    ]
    if not live_image_keys:
        raise KeyError("Missing live image observations for SmolVLA inference.")

    first_image = batch[live_image_keys[0]]
    for live_key, smolvla_key in zip(live_image_keys, smolvla_image_keys):
        target_key = f"observation.images.{smolvla_key}"
        if target_key not in batch:
            batch[target_key] = batch[live_key]
        if live_key != target_key:
            batch.pop(live_key, None)

    for smolvla_key in smolvla_image_keys:
        target_key = f"observation.images.{smolvla_key}"
        if target_key not in batch:
            batch[target_key] = torch.zeros_like(first_image)

    return batch


def main(args: Args) -> None:
    bundle = load_lerobot_policy(
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

    adapter = LeRobotObservationAdapter(
        device=bundle.device,
        camera_keys=args.cameras,
        expected_state_dim=args.expected_state_dim,
        expected_image_shape=(
            args.expected_image_height,
            args.expected_image_width,
            3,
        ),
    )

    safety = SafetyConfig(
        max_joint_delta=args.max_joint_delta,
        max_gripper_delta=args.max_gripper_delta,
        action_mode=args.action_mode,
    )
    executor = SafeJointActionExecutor(safety)

    robot = ZMQClientRobot(port=args.robot_port, host=args.robot_host)

    env = RobotEnv(
        robot,
        control_rate_hz=args.hz,
        camera_dict=_make_camera_clients(args),
    )

    print("\nLEROBOT SMOLVLA REAL-ROBOT ROLLOUT")
    print("checkpoint:", args.checkpoint)
    print("dataset_root:", args.dataset_root)
    print("repo_id:", args.repo_id)
    print("device:", bundle.device)
    print("live cameras:", args.cameras)
    print("SmolVLA image keys:", args.smolvla_image_keys)
    print("execute:", args.execute)
    print("duration:", args.duration)
    print("hz:", args.hz)
    print("action_mode:", args.action_mode)
    print("max_joint_delta:", args.max_joint_delta)
    print("max_gripper_delta:", args.max_gripper_delta)
    print("task:", args.task)

    print("\nRuntime image mapping:")
    for live_camera, smolvla_key in zip(args.cameras, args.smolvla_image_keys):
        print(f"  observation.images.{live_camera} -> observation.images.{smolvla_key}")
    if len(args.smolvla_image_keys) > len(args.cameras):
        for smolvla_key in args.smolvla_image_keys[len(args.cameras):]:
            print(f"  observation.images.{smolvla_key} -> zeros_like(first live camera)")

    if not args.execute:
        print("\nDRY RUN: policy will be evaluated, but the robot will not move.")
    else:
        print("\nEXECUTE MODE: the real robot can move.")
        print("Keep your hand on the enabling switch / emergency stop.")
        print("Start with an empty workspace and conservative deltas.")

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
            batch = _prepare_smolvla_batch(batch, args.task, args.smolvla_image_keys)
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

    print("\nFinished LeRobot SmolVLA real-robot rollout.")


if __name__ == "__main__":
    main(tyro.cli(Args))
