from typing import Optional, Tuple

import numpy as np

from gello.cameras.camera import CameraDriver


class OpenCVCamera(CameraDriver):
    def __init__(self, source: str):
        import cv2

        self._source = source
        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera stream: {source}")

    def __repr__(self) -> str:
        return f"OpenCVCamera(source={self._source})"

    def read(self, img_size: Optional[Tuple[int, int]] = None):
        import cv2

        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"Failed to read frame from {self._source}")

        if img_size is not None:
            frame = cv2.resize(frame, img_size)

        rgb = frame[:, :, ::-1]
        depth = np.zeros((rgb.shape[0], rgb.shape[1], 1), dtype=np.uint16)
        return rgb, depth
