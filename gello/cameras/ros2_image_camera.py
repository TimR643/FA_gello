"""ROS 2 image-topic camera adapter for simulated wrist cameras."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from gello.cameras.camera import CameraDriver


@dataclass(frozen=True)
class Ros2ImageCameraConfig:
    node_name: str = "gello_ros2_wrist_camera"
    rgb_topic: str = "/wrist_camera/image_raw"
    depth_topic: Optional[str] = "/wrist_camera/depth/image_raw"
    state_timeout: float = 5.0
    spin_period: float = 0.001


class Ros2ImageCamera(CameraDriver):
    """Expose a ROS 2 RGB/depth image pair through the GELLO camera protocol."""

    def __init__(self, config: Optional[Ros2ImageCameraConfig] = None):
        self.config = config or Ros2ImageCameraConfig()

        import rclpy
        from sensor_msgs.msg import Image

        if not rclpy.ok():
            rclpy.init(args=None)

        self._rclpy = rclpy
        self._lock = threading.Lock()
        self._rgb: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._last_rgb_time: Optional[float] = None
        self._node = rclpy.create_node(self.config.node_name)
        self._rgb_sub = self._node.create_subscription(
            Image, self.config.rgb_topic, self._rgb_callback, 10
        )
        self._depth_sub = None
        if self.config.depth_topic:
            self._depth_sub = self._node.create_subscription(
                Image, self.config.depth_topic, self._depth_callback, 10
            )
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self._wait_for_rgb()

    def read(
        self, img_size: Optional[Tuple[int, int]] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        self._ensure_rgb_fresh()
        with self._lock:
            rgb = np.asarray(self._rgb, dtype=np.uint8).copy()
            if self._depth is None:
                depth = np.zeros((*rgb.shape[:2], 1), dtype=np.uint16)
            else:
                depth = np.asarray(self._depth).copy()
        if img_size is not None and rgb.shape[:2] != img_size:
            import cv2

            rgb = cv2.resize(rgb, (img_size[1], img_size[0]))
            depth = cv2.resize(depth[:, :, 0], (img_size[1], img_size[0]))[:, :, None]
        return rgb, depth

    def _spin(self) -> None:
        while self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=self.config.spin_period)

    def _rgb_callback(self, msg: object) -> None:
        rgb = self._image_to_numpy(msg)
        if rgb.ndim == 2:
            rgb = np.repeat(rgb[:, :, None], 3, axis=2)
        if rgb.shape[2] == 4:
            rgb = rgb[:, :, :3]
        with self._lock:
            self._rgb = rgb.astype(np.uint8)
            self._last_rgb_time = time.monotonic()

    def _depth_callback(self, msg: object) -> None:
        depth = self._image_to_numpy(msg)
        if depth.ndim == 2:
            depth = depth[:, :, None]
        with self._lock:
            self._depth = depth

    def _image_to_numpy(self, msg: object) -> np.ndarray:
        dtype = self._encoding_to_dtype(getattr(msg, "encoding", "rgb8"))
        array = np.frombuffer(msg.data, dtype=dtype)
        channels = max(
            1,
            int(getattr(msg, "step"))
            // int(getattr(msg, "width"))
            // np.dtype(dtype).itemsize,
        )
        array = array.reshape(
            (int(getattr(msg, "height")), int(getattr(msg, "width")), channels)
        )
        encoding = getattr(msg, "encoding", "rgb8").lower()
        if encoding in {"bgr8", "bgra8"}:
            array = array[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
        if channels == 1:
            array = array[:, :, 0]
        return array

    def _encoding_to_dtype(self, encoding: str) -> np.dtype:
        encoding = encoding.lower()
        if encoding in {"16uc1", "mono16"}:
            return np.dtype(np.uint16)
        if encoding == "32fc1":
            return np.dtype(np.float32)
        return np.dtype(np.uint8)

    def _wait_for_rgb(self) -> None:
        deadline = time.monotonic() + self.config.state_timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._last_rgb_time is not None:
                    return
            time.sleep(0.02)
        raise TimeoutError(
            f"No ROS 2 wrist RGB image received on {self.config.rgb_topic!r}."
        )

    def _ensure_rgb_fresh(self) -> None:
        with self._lock:
            last_rgb_time = self._last_rgb_time
        if (
            last_rgb_time is None
            or time.monotonic() - last_rgb_time > self.config.state_timeout
        ):
            raise TimeoutError(
                f"ROS 2 wrist camera is stale on {self.config.rgb_topic!r}."
            )
