#!/usr/bin/env python3
"""Low-rate OpenCV preview for a GELLO ZMQ camera server."""

from __future__ import annotations

import argparse
import time
from collections.abc import Sequence

import cv2
import numpy as np
import zmq

from gello.zmq_core.camera_node import ZMQClientCamera


def _configure_timeout(camera: ZMQClientCamera, timeout_ms: int) -> None:
    socket = getattr(camera, "_socket", None)
    if socket is None:
        raise AttributeError("ZMQClientCamera has no _socket attribute")
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    socket.setsockopt(zmq.LINGER, 0)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    parser.add_argument("--window-name", default="GELLO ZMQ camera")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    period_s = 1.0 / args.fps if args.fps > 0 else 0.0
    camera = ZMQClientCamera(port=args.port, host=args.host)
    _configure_timeout(camera, args.timeout_ms)
    print(
        "Previewing ZMQ camera "
        f"{args.host}:{args.port} at <= {args.fps:.2f} FPS. "
        "Press 'q' or Esc to quit."
    )
    print(
        "Warning: this is another ZMQ camera client. Do not leave it running "
        "during recording/rollout if camera FPS matters."
    )
    try:
        while True:
            start = time.monotonic()
            rgb, _depth = camera.read((args.width, args.height))
            image = np.asarray(rgb, dtype=np.uint8)
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"Expected RGB image with shape HxWx3, got {image.shape}")
            bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow(args.window_name, bgr)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elapsed = time.monotonic() - start
            if period_s > elapsed:
                time.sleep(period_s - elapsed)
    finally:
        camera._socket.close()
        camera._context.term()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
