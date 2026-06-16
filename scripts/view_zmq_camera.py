#!/usr/bin/env python3
"""Low-rate preview for a GELLO ZMQ camera server."""

from __future__ import annotations

import argparse
import threading
import time
from collections.abc import Sequence
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import zmq

from gello.zmq_core.camera_node import ZMQClientCamera


class LatestJPEG:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.frame: bytes | None = None

    def update(self, frame: bytes) -> None:
        with self.condition:
            self.frame = frame
            self.condition.notify_all()


def _configure_timeout(camera: ZMQClientCamera, timeout_ms: int) -> None:
    socket = getattr(camera, "_socket", None)
    if socket is None:
        raise AttributeError("ZMQClientCamera has no _socket attribute")
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    socket.setsockopt(zmq.LINGER, 0)


def _has_opencv_gui() -> bool:
    try:
        cv2.namedWindow("__gello_zmq_preview_probe__", cv2.WINDOW_NORMAL)
        cv2.destroyWindow("__gello_zmq_preview_probe__")
        return True
    except cv2.error:
        return False


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    parser.add_argument("--window-name", default="GELLO ZMQ camera")
    parser.add_argument(
        "--backend",
        choices=("auto", "opencv", "http"),
        default="auto",
        help="Preview backend. auto uses OpenCV GUI when available, otherwise HTTP MJPEG.",
    )
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    return parser


def _make_http_handler(latest: LatestJPEG) -> type[BaseHTTPRequestHandler]:
    class MJPEGHandler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: object) -> None:
            return None

        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(
                    b"<html><body><h1>GELLO ZMQ camera</h1>"
                    b"<img src='/stream.mjpg' /></body></html>"
                )
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with latest.condition:
                    latest.condition.wait(timeout=5.0)
                    frame = latest.frame
                if frame is None:
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break

    return MJPEGHandler


def _start_http_server(args: argparse.Namespace, latest: LatestJPEG) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((args.http_host, args.http_port), _make_http_handler(latest))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"HTTP preview: http://{args.http_host}:{args.http_port}/")
    return server


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    period_s = 1.0 / args.fps if args.fps > 0 else 0.0
    backend = args.backend
    if backend == "auto":
        backend = "opencv" if _has_opencv_gui() else "http"
    latest = LatestJPEG()
    server: ThreadingHTTPServer | None = None
    if backend == "http":
        server = _start_http_server(args, latest)

    camera = ZMQClientCamera(port=args.port, host=args.host)
    _configure_timeout(camera, args.timeout_ms)
    print(
        "Previewing ZMQ camera "
        f"{args.host}:{args.port} at <= {args.fps:.2f} FPS via {backend}. "
        "Press Ctrl-C to quit."
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
            if backend == "opencv":
                cv2.imshow(args.window_name, bgr)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            else:
                ok, encoded = cv2.imencode(
                    ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]
                )
                if not ok:
                    raise RuntimeError("Could not encode preview frame as JPEG")
                latest.update(encoded.tobytes())
            elapsed = time.monotonic() - start
            if period_s > elapsed:
                time.sleep(period_s - elapsed)
    finally:
        camera._socket.close()
        camera._context.term()
        if backend == "opencv":
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        if server is not None:
            server.shutdown()
            server.server_close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
