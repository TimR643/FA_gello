import time
from typing import Optional

import numpy as np
import pyrealsense2 as rs
import pyrealsense2_net as rsnet


class RealSenseCamera:
    def __init__(
        self,
        device_id: Optional[str] = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self._net_device = None

        self._pipeline = rs.pipeline()
        self._config = rs.config()

        if device_id is not None and self._is_ip_address(device_id):
            print(f"Connecting to Ethernet RealSense/FRAMOS camera at IP: {device_id}")

            try:
                import pyrealsense2_net as rsnet
            except ImportError as exc:
                raise ImportError(
                    "pyrealsense2_net is not available in this Python environment. "
                    "Your rsviewer can see the Ethernet camera, but Python also needs "
                    "the RealSense network bindings."
                ) from exc

            self._net_device = rsnet.net_device(device_id)

            serial = self._net_device.get_info(rs.camera_info.serial_number)
            name = self._net_device.get_info(rs.camera_info.name)

            print(f"Connected network camera: {name}, serial: {serial}")

            self._config.enable_device(serial)

        elif device_id is not None:
            print(f"Using USB RealSense camera serial: {device_id}")
            self._config.enable_device(device_id)

        else:
            print("Using first available RealSense camera.")

        self._config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps,
        )

        self._config.enable_stream(
            rs.stream.depth,
            self.width,
            self.height,
            rs.format.z16,
            self.fps,
        )

        print("Starting RealSense pipeline...")
        self._profile = self._pipeline.start(self._config)
        print("RealSense pipeline started successfully.")

    @staticmethod
    def _is_ip_address(value: str) -> bool:
        parts = value.split(".")
        if len(parts) != 4:
            return False

        try:
            return all(0 <= int(part) <= 255 for part in parts)
        except ValueError:
            return False

    def read(self):
        frames = self._pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame:
            raise RuntimeError("No color frame received from RealSense camera.")

        color_image = np.asanyarray(color_frame.get_data())

        if depth_frame:
            depth_image = np.asanyarray(depth_frame.get_data())
        else:
            depth_image = None

        return {
            "color": color_image,
            "depth": depth_image,
        }

    def get_intrinsics(self):
        color_stream = self._profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()

        return {
            "width": intr.width,
            "height": intr.height,
            "fx": intr.fx,
            "fy": intr.fy,
            "ppx": intr.ppx,
            "ppy": intr.ppy,
            "coeffs": intr.coeffs,
        }

    def close(self):
        try:
            self._pipeline.stop()
        except Exception:
            pass

    def __del__(self):
        self.close()


def get_device_ids(reset_devices: bool = False):
    ctx = rs.context()
    devices = ctx.query_devices()

    ids = []

    for dev in devices:
        if reset_devices:
            dev.hardware_reset()
            time.sleep(2.0)

        try:
            serial = dev.get_info(rs.camera_info.serial_number)
            ids.append(serial)
        except Exception:
            pass

    return ids