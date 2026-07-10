import os
import threading
import time
from typing import List, Optional, Tuple

import numpy as np

from gello.cameras.camera import CameraDriver


def get_device_ids() -> List[str]:
    import pyrealsense2 as rs

    ctx = rs.context()
    devices = ctx.query_devices()
    device_ids = []
    for dev in devices:
        dev.hardware_reset()
        device_ids.append(dev.get_info(rs.camera_info.serial_number))
    time.sleep(2)
    return device_ids


class RealSenseCamera(CameraDriver):
    def __repr__(self) -> str:
        return f"RealSenseCamera(device_id={self._device_id})"

    def __init__(
        self,
        device_id: Optional[str] = None,
        flip: bool = False,
        wait_timeout_ms: int = 10000,
        max_read_retries: int = 2,
        reset_on_timeout: bool = True,
        keep_stream_alive: bool = True,
        first_frame_timeout_ms: int = 15000,
    ):
        import pyrealsense2 as rs

        self._device_id = device_id
        self._flip = flip
        self._wait_timeout_ms = wait_timeout_ms
        self._max_read_retries = max_read_retries
        self._reset_on_timeout = reset_on_timeout
        self._keep_stream_alive = keep_stream_alive
        self._first_frame_timeout_ms = first_frame_timeout_ms
        self._rs = rs
        self._config = self._make_config()
        self._pipeline = rs.pipeline()
        self._frame_condition = threading.Condition()
        self._latest_color_image: np.ndarray | None = None
        self._latest_depth_image: np.ndarray | None = None
        self._latest_frame_time_s = 0.0
        self._last_capture_error: Exception | None = None
        self._stop_event = threading.Event()
        self._capture_thread: threading.Thread | None = None
        self._start_pipeline()
        if self._keep_stream_alive:
            self._capture_thread = threading.Thread(
                target=self._capture_loop,
                name=f"RealSenseCamera-{self._device_id or 'default'}",
                daemon=True,
            )
            self._capture_thread.start()

    def _make_config(self):
        config = self._rs.config()
        if self._device_id is not None:
            config.enable_device(self._device_id)
        config.enable_stream(self._rs.stream.depth, 640, 480, self._rs.format.z16, 30)
        config.enable_stream(self._rs.stream.color, 640, 480, self._rs.format.bgr8, 30)
        return config

    def _start_pipeline(self) -> None:
        if self._device_id is None:
            ctx = self._rs.context()
            devices = ctx.query_devices()
            for dev in devices:
                dev.hardware_reset()
            time.sleep(2)
        self._pipeline.start(self._config)

    def _restart_pipeline(self) -> None:
        try:
            self._pipeline.stop()
        except RuntimeError:
            pass
        time.sleep(0.5)
        self._pipeline = self._rs.pipeline()
        self._start_pipeline()

    def _wait_for_frames_with_recovery(self):
        last_error: Exception | None = None
        for attempt in range(self._max_read_retries + 1):
            try:
                return self._pipeline.wait_for_frames(self._wait_timeout_ms)
            except RuntimeError as exc:
                last_error = exc
                if not self._reset_on_timeout or attempt >= self._max_read_retries:
                    raise
                print(
                    f"RealSense read timed out after {self._wait_timeout_ms} ms; "
                    f"restarting pipeline (attempt {attempt + 1}/"
                    f"{self._max_read_retries})",
                    flush=True,
                )
                self._restart_pipeline()
        raise RuntimeError("RealSense read failed") from last_error

    def _read_raw_frames(self) -> Tuple[np.ndarray, np.ndarray]:
        frames = self._wait_for_frames_with_recovery()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("RealSense returned an incomplete frame set")
        return (
            np.asanyarray(color_frame.get_data()).copy(),
            np.asanyarray(depth_frame.get_data()).copy(),
        )

    def _capture_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                color_image, depth_image = self._read_raw_frames()
            except Exception as exc:
                self._last_capture_error = exc
                print(f"RealSense background capture failed: {exc}", flush=True)
                time.sleep(0.1)
                continue
            with self._frame_condition:
                self._latest_color_image = color_image
                self._latest_depth_image = depth_image
                self._latest_frame_time_s = time.monotonic()
                self._last_capture_error = None
                self._frame_condition.notify_all()

    def _latest_raw_frames(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self._keep_stream_alive:
            return self._read_raw_frames()

        deadline_s = time.monotonic() + self._first_frame_timeout_ms / 1000.0
        with self._frame_condition:
            while self._latest_color_image is None or self._latest_depth_image is None:
                remaining_s = deadline_s - time.monotonic()
                if remaining_s <= 0.0:
                    raise RuntimeError(
                        "Timed out waiting for first RealSense background frame"
                    ) from self._last_capture_error
                self._frame_condition.wait(remaining_s)
            return self._latest_color_image.copy(), self._latest_depth_image.copy()

    def read(
        self,
        img_size: Optional[Tuple[int, int]] = None,  # farthest: float = 0.12
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Read the latest frame from the camera.

        Args:
            img_size: The size of the image to return. If None, the original size is returned.
            farthest: The farthest distance to map to 255.

        Returns:
            np.ndarray: The color image, shape=(H, W, 3)
            np.ndarray: The depth image, shape=(H, W, 1)
        """
        import cv2

        color_image, depth_image = self._latest_raw_frames()
        # depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
        if img_size is None:
            image = color_image[:, :, ::-1]
            depth = depth_image
        else:
            image = cv2.resize(color_image, img_size)[:, :, ::-1]
            depth = cv2.resize(depth_image, img_size)

        # rotate 180 degree's because everything is upside down in order to center the camera
        if self._flip:
            image = cv2.rotate(image, cv2.ROTATE_180)
            depth = cv2.rotate(depth, cv2.ROTATE_180)[:, :, None]
        else:
            depth = depth[:, :, None]

        return image, depth

    def close(self) -> None:
        self._stop_event.set()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=2.0)
        try:
            self._pipeline.stop()
        except RuntimeError:
            pass


def _debug_read(camera, save_datastream=False):
    import cv2

    cv2.namedWindow("image")
    cv2.namedWindow("depth")
    counter = 0
    if not os.path.exists("images"):
        os.makedirs("images")
    if save_datastream and not os.path.exists("stream"):
        os.makedirs("stream")
    while True:
        time.sleep(0.1)
        image, depth = camera.read()
        depth = np.concatenate([depth, depth, depth], axis=-1)
        key = cv2.waitKey(1)
        cv2.imshow("image", image[:, :, ::-1])
        cv2.imshow("depth", depth)
        if key == ord("s"):
            cv2.imwrite(f"images/image_{counter}.png", image[:, :, ::-1])
            cv2.imwrite(f"images/depth_{counter}.png", depth)
        if save_datastream:
            cv2.imwrite(f"stream/image_{counter}.png", image[:, :, ::-1])
            cv2.imwrite(f"stream/depth_{counter}.png", depth)
        counter += 1
        if key == 27:
            break


if __name__ == "__main__":
    device_ids = get_device_ids()
    print(f"Found {len(device_ids)} devices")
    print(device_ids)
    rs = RealSenseCamera(flip=True, device_id=device_ids[0])
    im, depth = rs.read()
    _debug_read(rs, save_datastream=True)
