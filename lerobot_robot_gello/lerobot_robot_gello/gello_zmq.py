"""LeRobot ``Robot`` implementation backed by GELLO's ZMQ robot server."""

from __future__ import annotations

import ast
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np
import zmq
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
        for camera in self._camera_names():
            features[f"observation.images.{camera}"] = (
                self.config.image_height,
                self.config.image_width,
                3,
            )
        return features

    @property
    def action_features(self) -> dict[str, Any]:
        return {self.config.action_key: (self.config.num_dofs,)}

    def _camera_names(self) -> tuple[str, ...]:
        camera_names = self.config.camera_names
        if isinstance(camera_names, str):
            text = camera_names.strip()
            if not text:
                return ()
            if text.startswith(("(", "[")):
                parsed = ast.literal_eval(text)
                if isinstance(parsed, str):
                    return (parsed,)
                if isinstance(parsed, Sequence):
                    return tuple(str(camera).strip() for camera in parsed)
            return tuple(camera.strip() for camera in text.split(",") if camera.strip())
        if isinstance(camera_names, Sequence):
            return tuple(str(camera).strip() for camera in camera_names)
        raise TypeError(f"Unsupported camera_names value: {camera_names!r}")

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
        self._configure_zmq_timeout(self.robot, name="robot")
        camera_host = self.config.camera_host or self.config.robot_host
        for camera in self._camera_names():
            if camera == "wrist":
                self.cameras[camera] = ZMQClientCamera(
                    port=self.config.wrist_camera_port, host=camera_host
                )
                self._configure_zmq_timeout(self.cameras[camera], name="wrist camera")
            elif camera == "base":
                self.cameras[camera] = ZMQClientCamera(
                    port=self.config.base_camera_port, host=camera_host
                )
                self._configure_zmq_timeout(self.cameras[camera], name="base camera")
            else:
                raise ValueError(f"Unsupported GELLO camera {camera!r}; use wrist/base")
        self._preflight_robot_connection()
        self._is_connected = True

    def _configure_zmq_timeout(self, client: Any, *, name: str) -> None:
        socket = getattr(client, "_socket", None)
        if socket is None:
            raise AttributeError(f"{name} client has no _socket attribute")
        socket.setsockopt(zmq.RCVTIMEO, self.config.zmq_timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, self.config.zmq_timeout_ms)
        socket.setsockopt(zmq.LINGER, 0)

    def _preflight_robot_connection(self) -> None:
        if self.robot is None:
            raise ConnectionError("Robot client was not created")
        try:
            self.robot.num_dofs()
        except Exception as exc:
            raise ConnectionError(
                "Could not reach GELLO robot ZMQ server at "
                f"{self.config.robot_host}:{self.config.robot_port} within "
                f"{self.config.zmq_timeout_ms} ms. Check that the robot ZMQ node "
                "is running and that the SSH tunnel forwards this port."
            ) from exc

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
                (self.config.image_width, self.config.image_height)
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
