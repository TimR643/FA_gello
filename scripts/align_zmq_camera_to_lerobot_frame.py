#!/usr/bin/env python3
"""Align a live ZMQ camera against a reference frame from a LeRobot dataset.

Use this after a physical camera was bumped: load a frame from a known-good
recording and show it next to the live camera feed, with an overlay/difference
view, so the camera can be moved back to the recorded viewpoint.
"""

from __future__ import annotations

import argparse
import re
import time
from collections.abc import Sequence
from pathlib import Path

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


def _episode_pattern(episode_index: int) -> re.Pattern[str]:
    return re.compile(rf"episode_0*{episode_index}(?:\D|$)")


def _find_video(dataset_root: Path, camera: str, episode_index: int) -> Path:
    camera_key = f"observation.images.{camera}"
    candidates = []
    for path in dataset_root.rglob("*.mp4"):
        text = path.as_posix()
        if camera_key not in text and camera not in path.name:
            continue
        if not _episode_pattern(episode_index).search(path.stem):
            continue
        candidates.append(path)
    if not candidates:
        raise FileNotFoundError(
            "Could not find a LeRobot video for "
            f"camera={camera!r}, episode={episode_index} under {dataset_root}. "
            "Expected paths containing observation.images.<camera> and "
            "episode_<index>.mp4."
        )
    return sorted(candidates, key=lambda path: len(path.as_posix()))[0]


def _load_reference_rgb(video_path: Path, frame_index: int) -> np.ndarray:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise OSError(f"Could not open video: {video_path}")
    try:
        capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, bgr = capture.read()
        if not ok or bgr is None:
            raise ValueError(f"Could not read frame {frame_index} from {video_path}")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    finally:
        capture.release()


def _resize_like(image: np.ndarray, reference: np.ndarray) -> np.ndarray:
    if image.shape[:2] == reference.shape[:2]:
        return image
    height, width = reference.shape[:2]
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def _build_view(reference_rgb: np.ndarray, live_rgb: np.ndarray, alpha: float) -> np.ndarray:
    live_rgb = _resize_like(live_rgb, reference_rgb)
    overlay = cv2.addWeighted(reference_rgb, alpha, live_rgb, 1.0 - alpha, 0)
    diff = cv2.absdiff(reference_rgb, live_rgb)
    top = np.concatenate([reference_rgb, live_rgb], axis=1)
    bottom = np.concatenate([overlay, diff], axis=1)
    canvas_rgb = np.concatenate([top, bottom], axis=0)
    return cv2.cvtColor(canvas_rgb, cv2.COLOR_RGB2BGR)


def _draw_labels(canvas_bgr: np.ndarray, tile_width: int, tile_height: int) -> None:
    labels = (
        ("reference recording", 10, 28),
        ("live camera", tile_width + 10, 28),
        ("overlay", 10, tile_height + 28),
        ("absolute difference", tile_width + 10, tile_height + 28),
    )
    for text, x, y in labels:
        cv2.putText(
            canvas_bgr,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--camera", default="wrist", choices=("wrist", "base"))
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--frame-index", type=int, default=0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    parser.add_argument("--alpha", type=float, default=0.5)
    parser.add_argument("--save-reference", type=Path, default=None)
    parser.add_argument("--window-name", default="Align live ZMQ camera to LeRobot frame")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    dataset_root = args.dataset_root.expanduser()
    video_path = _find_video(dataset_root, args.camera, args.episode_index)
    reference_rgb = _load_reference_rgb(video_path, args.frame_index)
    if args.save_reference is not None:
        output_path = args.save_reference.expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), cv2.cvtColor(reference_rgb, cv2.COLOR_RGB2BGR))
        print(f"Saved reference frame to {output_path}")

    camera = ZMQClientCamera(port=args.port, host=args.host)
    _configure_timeout(camera, args.timeout_ms)
    period_s = 1.0 / args.fps if args.fps > 0 else 0.0
    print(f"Reference video: {video_path}")
    print(
        "Move the physical camera until live camera, overlay, and difference match. "
        "Press 'q' or Esc to quit."
    )
    try:
        while True:
            start = time.monotonic()
            live_rgb, _depth = camera.read((args.width, args.height))
            live_rgb = np.asarray(live_rgb, dtype=np.uint8)
            canvas = _build_view(reference_rgb, live_rgb, args.alpha)
            _draw_labels(canvas, reference_rgb.shape[1], reference_rgb.shape[0])
            cv2.imshow(args.window_name, canvas)
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
