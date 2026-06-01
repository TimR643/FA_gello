"""Safely run a trained LeRobot policy on the real GELLO/ZMQ robot stack.

Default mode is a dry run: live observations are read and the policy is queried,
but no command is sent to the robot unless ``--execute`` is passed.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import torch
import tyro

from gello.env import RobotEnv
from gello.lerobot.real_robot import (
    LeRobotObservationAdapter,
    SafeJointActionExecutor,
    SafetyConfig,
    load_lerobot_policy,
    validate_policy_batch_keys,
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

    duration: float = 2.0
    hz: float = 5.0
    execute: bool = False
    require_enter: bool = True
    device: Optional[str] = None

    expected_state_dim: int = 8
    expected_image_height: int = 480
    expected_image_width: int = 640
    max_joint_delta: float = 0.015
    max_gripper_delta: float = 0.03
    action_mode: str = "absolute_joint_position"


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
                f"Unsupported camera {camera!r}; expected 'wrist' or 'base'"
            )
    return clients


def main(args: Args) -> None:
    bundle = load_lerobot_policy(
        checkpoint=args.checkpoint,
        dataset_root=args.dataset_root,
        repo_id=args.repo_id,
        device=args.device,
    )

    adapter = LeRobotObservationAdapter(
        device=bundle.device,
        camera_keys=args.cameras,
        expected_state_dim=args.expected_state_dim,
        expected_image_shape=(args.expected_image_height, args.expected_image_width, 3),
    )
    validate_policy_batch_keys(adapter, bundle.dataset.meta)

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

    print("\nLEROBOT REAL-ROBOT ROLLOUT")
    print("checkpoint:", args.checkpoint)
    print("dataset_root:", args.dataset_root)
    print("repo_id:", args.repo_id)
    print("device:", bundle.device)
    print("policy keys:", adapter.policy_keys)
    print("execute:", args.execute)
    print("duration:", args.duration)
    print("hz:", args.hz)
    print("action_mode:", args.action_mode)
    print("max_joint_delta:", args.max_joint_delta)
    print("max_gripper_delta:", args.max_gripper_delta)

    if not args.execute:
        print("\nDRY RUN: policy will be evaluated, but the robot will not move.")
    else:
        print("\nEXECUTE MODE: keep your hand on the enabling switch / emergency stop.")
        print("Start with an empty workspace and conservative deltas.")

    if args.require_enter:
        input("\nPress ENTER to start...")

    steps = int(args.duration * args.hz)
    if steps <= 0:
        raise ValueError("duration * hz must produce at least one step")

    dt = 1.0 / args.hz
    for step in range(steps):
        started = time.time()
        obs = env.get_obs()
        batch = adapter.make_batch(obs)
        state = adapter.state_from_obs(obs)

        with torch.no_grad():
            policy_action = bundle.policy.select_action(batch)
        safe = executor.make_safe_target(policy_action, state)

        print(f"\nStep {step + 1}/{steps}")
        print("state        :", np.round(state, 3))
        print("policy_action:", np.round(safe.policy_action, 3))
        print("raw_delta    :", np.round(safe.raw_delta, 3))
        print("clipped_delta:", np.round(safe.clipped_delta, 3))
        print("target       :", np.round(safe.target, 3))

        if args.execute:
            env.step(safe.target)

        remaining = dt - (time.time() - started)
        if remaining > 0:
            time.sleep(remaining)

    print("\nFinished LeRobot real-robot rollout.")


if __name__ == "__main__":
    main(tyro.cli(Args))
