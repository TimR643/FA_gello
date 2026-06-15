"""LeRobot ``Robot`` implementation backed by GELLO's ZMQ robot server."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np
from lerobot.robots.robot import Robot

from gello.lerobot.real_robot import SafeJointActionExecutor, SafetyConfig
from gello.zmq_core.camera_node import ZMQClientCamera
from gello.zmq_core.robot_node import ZMQClientRobot

from .config_gello_zmq import GelloZMQConfig


class GelloZMQ(Robot):
    """Expose the existing Panda ZMQ bridge to LeRobot's native CLI tools."""

    config_class = GelloZMQConfig
    name = "gello_zmq"

    def __init__(self, config: GelloZMQConfig):
        super().__init__(config)
        self.config = config
        self.robot: ZMQClientRobot | None = None
        self.cameras: dict[str, ZMQClientCamera] = {}
        self._is_connected = False
        self._last_state: np.ndarray | None = None
        self.executor = SafeJointActionExecutor(
            SafetyConfig(
                max_joint_delta=config.max_joint_delta,
                max_gripper_delta=config.max_gripper_delta,
                joint_lower=config.joint_lower,
                joint_upper=config.joint_upper,
                action_mode=config.action_mode,
            )
        )

    @property
    def observation_features(self) -> dict[str, Any]:
        features: dict[str, Any] = {self.config.state_key: (self.config.num_dofs,)}
        for camera in self.config.cameras:
            features[f"observation.images.{camera}"] = (
                self.config.image_height,
                self.config.image_width,
                3,
            )
        return features

    @property
    def action_features(self) -> dict[str, Any]:
        return {self.config.action_key: (self.config.num_dofs,)}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        self.robot = ZMQClientRobot(
            port=self.config.robot_port, host=self.config.robot_host
        )
        camera_host = self.config.camera_host or self.config.robot_host
        for camera in self.config.cameras:
            if camera == "wrist":
                self.cameras[camera] = ZMQClientCamera(
                    port=self.config.wrist_camera_port, host=camera_host
                )
            elif camera == "base":
                self.cameras[camera] = ZMQClientCamera(
                    port=self.config.base_camera_port, host=camera_host
                )
            else:
                raise ValueError(f"Unsupported GELLO camera {camera!r}; use wrist/base")
        self._is_connected = True

    def disconnect(self) -> None:
        if self.robot is not None:
            self.robot.close()
        self.robot = None
        self.cameras = {}
        self._is_connected = False

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_observation(self) -> dict[str, Any]:
        if self.robot is None or not self.is_connected:
            raise ConnectionError(f"{self} is not connected")
        raw = self.robot.get_observations()
        state = np.asarray(
            raw.get("joint_positions", self.robot.get_joint_state()), dtype=np.float32
        )
        if state.shape != (self.config.num_dofs,):
            raise ValueError(
                f"Expected state shape {(self.config.num_dofs,)}, got {state.shape}"
            )
        self._last_state = state
        obs: dict[str, Any] = {self.config.state_key: state}
        for camera, client in self.cameras.items():
            rgb, _depth = client.read(
                (self.config.image_height, self.config.image_width)
            )
            image = np.asarray(rgb)
            if image.shape != (self.config.image_height, self.config.image_width, 3):
                raise ValueError(
                    f"Camera {camera!r} returned {image.shape}; expected "
                    f"{(self.config.image_height, self.config.image_width, 3)}"
                )
            obs[f"observation.images.{camera}"] = image
        return obs

    def send_action(self, action: dict[str, Any] | Any) -> dict[str, Any]:
        if self.robot is None or not self.is_connected:
            raise ConnectionError(f"{self} is not connected")
        current = self._last_state
        if current is None:
            current = np.asarray(self.robot.get_joint_state(), dtype=np.float32)
        raw_action = self._extract_action_array(action)
        safe = self.executor.make_safe_target(raw_action, current)
        self.robot.command_joint_state(safe.target)
        return {self.config.action_key: safe.target}

    def _extract_action_array(self, action: dict[str, Any] | Any) -> Any:
        if not isinstance(action, Mapping):
            return action
        if self.config.action_key in action:
            return action[self.config.action_key]
        if len(action) == 1:
            return next(iter(action.values()))
        available = ", ".join(str(key) for key in action)
        raise KeyError(
            f"Action is missing {self.config.action_key!r}; available keys: {available}"
        )
