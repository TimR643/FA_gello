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
        self._last_record_obs: dict[str, Any] | None = None
        self._lerobot_writer: Any | None = None
        self._recorded_frames = 0
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
        features: dict[str, Any] = self._joint_features()
        for camera in self._policy_camera_names():
            features[self._policy_camera_key(camera)] = (
                self.config.image_height,
                self.config.image_width,
                3,
            )
        return features

    @property
    def action_features(self) -> dict[str, Any]:
        return self._joint_features()

    def _joint_features(self) -> dict[str, type]:
        return {f"joint_{index}.pos": float for index in range(self.config.num_dofs)}

    def _joint_feature_names(self) -> tuple[str, ...]:
        return tuple(f"joint_{index}.pos" for index in range(self.config.num_dofs))

    def _policy_camera_names(self) -> tuple[str, ...]:
        return self._parse_names(self.config.policy_camera_names)

    def _camera_names(self) -> tuple[str, ...]:
        return self._parse_names(self.config.camera_names)

    @staticmethod
    def _policy_camera_key(camera: str) -> str:
        return camera.removeprefix("observation.images.")

    @staticmethod
    def _parse_names(camera_names: Any) -> tuple[str, ...]:
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
        self._start_lerobot_recording_if_enabled()

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
        self._finish_lerobot_recording()
        if self.robot is not None:
            self.robot.close()
        self.robot = None
        self.cameras = {}
        self._is_connected = False

    def _start_lerobot_recording_if_enabled(self) -> None:
        if not self.config.record_lerobot:
            return
        from gello.utils.control_utils import LeRobotDatasetWriter

        self._lerobot_writer = LeRobotDatasetWriter(
            root=self.config.lerobot_root,
            repo_id=self.config.lerobot_repo_id,
            fps=self.config.lerobot_fps,
            task=self.config.lerobot_task,
            robot_type=self.config.lerobot_robot_type,
            camera_keys=self._parse_names(self.config.lerobot_camera_names),
            streaming_encoding=self.config.lerobot_streaming_encoding,
            batch_encoding_size=self.config.lerobot_batch_encoding_size,
        )
        self._recorded_frames = 0
        print(
            "Native LeRobot recording enabled: "
            f"root={self.config.lerobot_root}, repo_id={self.config.lerobot_repo_id}"
        )

    def _finish_lerobot_recording(self) -> None:
        if self._lerobot_writer is None:
            return
        try:
            if self._recorded_frames > 0:
                self._lerobot_writer.save_episode()
                print(f"Saved native LeRobot rollout with {self._recorded_frames} frames")
            else:
                print("Native LeRobot recording had no frames; nothing to save")
        finally:
            self._lerobot_writer.finalize()
            self._lerobot_writer = None
            self._recorded_frames = 0

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
        obs: dict[str, Any] = {
            key: float(value) for key, value in zip(self._joint_feature_names(), state)
        }
        record_obs: dict[str, Any] = {"joint_positions": state.copy()}
        live_camera_names = self._camera_names()
        policy_camera_names = self._policy_camera_names()
        for policy_camera_index, policy_camera in enumerate(policy_camera_names):
            if policy_camera_index >= len(live_camera_names):
                obs[self._policy_camera_key(policy_camera)] = np.zeros(
                    (self.config.image_height, self.config.image_width, 3),
                    dtype=np.uint8,
                )
                continue
            live_camera = live_camera_names[policy_camera_index]
            client = self.cameras[live_camera]
            rgb, _depth = client.read(
                (self.config.image_width, self.config.image_height)
            )
            image = np.asarray(rgb)
            if image.shape != (self.config.image_height, self.config.image_width, 3):
                raise ValueError(
                    f"Camera {live_camera!r} returned {image.shape}; expected "
                    f"{(self.config.image_height, self.config.image_width, 3)}"
                )
            obs[self._policy_camera_key(policy_camera)] = image
            record_obs[f"{live_camera}_rgb"] = image
        self._last_record_obs = record_obs
        return obs

    def send_action(self, action: dict[str, Any] | Any) -> dict[str, Any]:
        if self.robot is None or not self.is_connected:
            raise ConnectionError(f"{self} is not connected")
        current = self._last_state
        if current is None:
            current = np.asarray(self.robot.get_joint_state(), dtype=np.float32)
        raw_action = self._extract_action_array(action)
        safe = self.executor.make_safe_target(raw_action, current)
        target = safe.target.astype(np.float32)
        self.robot.command_joint_state(target)
        self._record_lerobot_frame(target)
        return {
            key: float(value) for key, value in zip(self._joint_feature_names(), target)
        }

    def _record_lerobot_frame(self, action: np.ndarray) -> None:
        if self._lerobot_writer is None:
            return
        if self._last_record_obs is None:
            raise RuntimeError(
                "Cannot record LeRobot frame before an observation was captured"
            )
        self._lerobot_writer.add_frame(self._last_record_obs, action)
        self._recorded_frames += 1

    def _extract_action_array(self, action: dict[str, Any] | Any) -> Any:
        if not isinstance(action, Mapping):
            return action
        if self.config.action_key in action:
            return action[self.config.action_key]
        joint_keys = self._joint_feature_names()
        if all(key in action for key in joint_keys):
            return np.asarray([action[key] for key in joint_keys], dtype=np.float32)
        if len(action) == 1:
            return next(iter(action.values()))
        available = ", ".join(str(key) for key in action)
        raise KeyError(
            f"Action is missing {self.config.action_key!r} or joint_*.pos keys; "
            f"available keys: {available}"
        )
