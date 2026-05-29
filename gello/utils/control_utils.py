"""Shared utilities for robot control loops."""

import datetime
import inspect
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.env import RobotEnv

DEFAULT_MAX_JOINT_DELTA = 1.0


def move_to_start_position(
    env: RobotEnv, agent: Agent, max_delta: float = 1.0, steps: int = 25
) -> bool:
    """Move robot to start position gradually.

    Args:
        env: Robot environment
        agent: Agent that provides target position
        max_delta: Maximum joint delta per step
        steps: Number of steps for gradual movement

    Returns:
        bool: True if successful, False if position too far
    """
    print("Going to start position")
    start_pos = agent.act(env.get_obs())
    obs = env.get_obs()
    joints = obs["joint_positions"]

    abs_deltas = np.abs(start_pos - joints)
    id_max_joint_delta = np.argmax(abs_deltas)

    max_joint_delta = DEFAULT_MAX_JOINT_DELTA
    if abs_deltas[id_max_joint_delta] > max_joint_delta:
        id_mask = abs_deltas > max_joint_delta
        print()
        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, joint, current_j in zip(
            ids,
            abs_deltas[id_mask],
            start_pos[id_mask],
            joints[id_mask],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
        return False

    print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
    assert len(start_pos) == len(
        joints
    ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"

    for _ in range(steps):
        obs = env.get_obs()
        command_joints = agent.act(obs)
        current_joints = obs["joint_positions"]
        delta = command_joints - current_joints
        max_joint_delta = np.abs(delta).max()
        if max_joint_delta > max_delta:
            delta = delta / max_joint_delta * max_delta
        env.step(current_joints + delta)

    return True


class SaveInterface:
    """Handles keyboard-based data saving interface."""

    def __init__(
        self,
        data_dir: str = "data",
        agent_name: str = "Agent",
        expand_user: bool = False,
    ):
        """Initialize save interface.

        Args:
            data_dir: Base directory for saving data
            agent_name: Name of agent (used for subdirectory)
            expand_user: Whether to expand ~ in data_dir path
        """
        from gello.data_utils.keyboard_interface import KBReset

        self.kb_interface = KBReset()
        self.data_dir = Path(data_dir).expanduser() if expand_user else Path(data_dir)
        self.agent_name = agent_name
        self.save_path: Optional[Path] = None

        print("Save interface enabled. Use keyboard controls:")
        print("  S: Start recording")
        print("  Q: Stop recording")

    def update(self, obs: Dict[str, Any], action: np.ndarray) -> Optional[str]:
        """Update save interface and handle saving.

        Args:
            obs: Current observations
            action: Current action

        Returns:
            Optional[str]: "quit" if user wants to exit, None otherwise
        """
        from gello.data_utils.format_obs import save_frame

        dt = datetime.datetime.now()
        state = self.kb_interface.update()

        if state == "start":
            dt_time = datetime.datetime.now()
            self.save_path = (
                self.data_dir / self.agent_name / dt_time.strftime("%m%d_%H%M%S")
            )
            self.save_path.mkdir(parents=True, exist_ok=True)
            print(f"Saving to {self.save_path}")
        elif state == "save":
            if self.save_path is not None:
                save_frame(self.save_path, dt, obs, action)
        elif state == "normal":
            self.save_path = None
        elif state == "quit":
            print("\nExiting.")
            return "quit"
        else:
            raise ValueError(f"Invalid state {state}")

        return None



JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
    "gripper",
]


class LeRobotSaveInterface:
    """Keyboard-based direct recording into LeRobot dataset format."""

    def __init__(
        self,
        root: str,
        repo_id: str,
        fps: int,
        task: str,
        robot_type: str = "panda_gello",
        camera_keys: Tuple[str, ...] = ("wrist", "base"),
        streaming_encoding: bool = True,
        batch_encoding_size: int = 1,
    ):
        from gello.data_utils.keyboard_interface import KBReset
        from lerobot.datasets import LeRobotDataset

        self.kb_interface = KBReset()
        self.task = task
        self.camera_keys = camera_keys
        create_kwargs = {
            "repo_id": repo_id,
            "fps": fps,
            "features": self._build_features(camera_keys),
            "robot_type": robot_type,
            "root": Path(root).expanduser(),
            "use_videos": True,
        }
        create_params = inspect.signature(LeRobotDataset.create).parameters
        if "streaming_encoding" in create_params:
            create_kwargs["streaming_encoding"] = streaming_encoding
        elif streaming_encoding:
            print(
                "Warning: installed LeRobot does not support streaming_encoding; "
                "videos will be encoded when episodes are saved."
            )

        if "batch_encoding_size" in create_params:
            create_kwargs["batch_encoding_size"] = batch_encoding_size
        elif "video_encoding_batch_size" in create_params:
            create_kwargs["video_encoding_batch_size"] = batch_encoding_size

        dataset_root = create_kwargs["root"]
        if dataset_root.exists():
            print(f"Existing LeRobot dataset found, resuming: {dataset_root}")
            self.dataset = LeRobotDataset.resume(repo_id=repo_id, root=dataset_root)
        else:
            self.dataset = LeRobotDataset.create(**create_kwargs)
        self._recording = False

        print("LeRobot save interface enabled. Video storage is enabled.")
        if streaming_encoding:
            print("Streaming video encoding requested: frames are encoded during capture when supported by your LeRobot version.")
        print("Use keyboard controls:")
        print("  S: Start recording")
        print("  Q: Stop recording")

    def _build_features(self, camera_keys: Tuple[str, ...]) -> Dict[str, Any]:
        features: Dict[str, Any] = {
            "observation.state": {"dtype": "float32", "shape": (8,), "names": JOINT_NAMES},
            "action": {"dtype": "float32", "shape": (8,), "names": JOINT_NAMES},
        }
        for cam in camera_keys:
            features[f"observation.images.{cam}"] = {
                "dtype": "video",
                "shape": (480, 640, 3),
                "names": ["height", "width", "channel"],
            }
        return features

    def update(self, obs: Dict[str, Any], action: np.ndarray) -> Optional[str]:
        state = self.kb_interface.update()
        if state == "start":
            self._recording = True
            print("Started recording episode into LeRobot dataset")
        elif state == "save" and self._recording:
            frame: Dict[str, Any] = {
                "observation.state": np.asarray(obs["joint_positions"], dtype=np.float32),
                "action": np.asarray(action, dtype=np.float32),
                "task": self.task,
            }
            for cam in self.camera_keys:
                key = f"{cam}_rgb"
                if key in obs:
                    frame[f"observation.images.{cam}"] = np.asarray(obs[key], dtype=np.uint8)
            self.dataset.add_frame(frame)
        elif state == "normal" and self._recording:
            self.dataset.save_episode()
            self._recording = False
            print("Episode saved")
        elif state == "quit":
            if self._recording:
                self.dataset.save_episode()
                self._recording = False
            self.dataset.finalize()
            print("\nExiting.")
            return "quit"
        return None

def run_control_loop(
    env: RobotEnv,
    agent: Agent,
    save_interface: Optional[SaveInterface] = None,
    print_timing: bool = True,
    use_colors: bool = False,
) -> None:
    """Run the main control loop.

    Args:
        env: Robot environment
        agent: Agent for control
        save_interface: Optional save interface for data collection
        print_timing: Whether to print timing information
        use_colors: Whether to use colored terminal output
    """
    # Check if we can use colors
    colors_available = False
    if use_colors:
        try:
            from termcolor import colored

            colors_available = True
            start_msg = colored("\nStart 🚀🚀🚀", color="green", attrs=["bold"])
        except ImportError:
            start_msg = "\nStart 🚀🚀🚀"
    else:
        start_msg = "\nStart 🚀🚀🚀"

    print(start_msg)

    start_time = time.time()
    obs = env.get_obs()

    while True:
        if print_timing:
            num = time.time() - start_time
            message = f"\rTime passed: {round(num, 2)}          "

            if colors_available:
                print(
                    colored(message, color="white", attrs=["bold"]), end="", flush=True
                )
            else:
                print(message, end="", flush=True)

        action = agent.act(obs)

        # Handle save interface
        if save_interface is not None:
            result = save_interface.update(obs, action)
            if result == "quit":
                break

        obs = env.step(action)
