"""Record a LeRobot dataset on this machine while pulling cameras remotely.

This recorder is intended for setups where the robot-control laptop must not read
or serialize camera frames in its real-time control loop. The laptop streams only
small robot observations/actions via ``RecordingStreamInterface``. This process
runs on the recording machine, receives those small messages, pulls RGB frames
from camera ZMQ servers (typically running on the laptop), and writes a LeRobot
video dataset.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Any, Deque, Dict, Tuple

import numpy as np
import tyro
import zmq

from gello.utils.control_utils import LeRobotDatasetWriter, confirm_episode_keep
from gello.zmq_core.camera_node import ZMQClientCamera
from gello.zmq_core.recording_node import ZMQRecordingReceiver

IMAGE_SIZE = (640, 480)
IMAGE_SHAPE = (480, 640, 3)


@dataclass
class Args:
    bind_hostname: str = "0.0.0.0"
    port: int = 7000

    camera_hostname: str = "127.0.0.1"
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    cameras: Tuple[str, ...] = ("wrist", "base")

    lerobot_root: str = "~/lerobot_data"
    lerobot_repo_id: str = "local/panda_gello_remote_cameras"
    lerobot_fps: int = 20
    lerobot_task: str = "Teleoperate Panda with GELLO."
    lerobot_robot_type: str = "panda_gello"
    lerobot_streaming_encoding: bool = True
    lerobot_batch_encoding_size: int = 1
    camera_timeout_ms: int = 3000
    camera_sync_mode: str = "on_frame"
    camera_poll_fps: float = 10.0
    camera_max_age_ms: float = 250.0
    camera_frame_delay_ms: float = 100.0
    camera_frame_delay_frames: int = 1
    camera_buffer_seconds: float = 2.0
    log_sync_every: int = 50


class RemoteCameraPoller:
    """Continuously pull one remote camera so frames are ready when robot state arrives."""

    def __init__(self, args: Args, camera: str):
        self.args = args
        self.camera = camera
        self.period_s = 1.0 / args.camera_poll_fps if args.camera_poll_fps > 0 else 0.0
        self._client = _make_camera_client(args, camera)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name=f"remote-camera-poller-{camera}",
            daemon=True,
        )
        self._frames: Deque[tuple[float, np.ndarray]] = deque()
        self._failures = 0

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        _close_camera_client(self._client)

    def frame_at_or_before(
        self, target_monotonic_s: float
    ) -> tuple[np.ndarray | None, float | None, int, int]:
        with self._lock:
            if not self._frames:
                return None, None, self._failures, 0

            selected_s, selected_image = self._frames[0]
            for captured_s, image in self._frames:
                if captured_s > target_monotonic_s:
                    break
                selected_s = captured_s
                selected_image = image
            return (
                selected_image.copy(),
                selected_s,
                self._failures,
                len(self._frames),
            )

    def _replace_client(self) -> None:
        try:
            _close_camera_client(self._client)
        except zmq.ZMQError:
            pass
        self._client = _make_camera_client(self.args, self.camera)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            start_s = time.monotonic()
            try:
                image, _depth = self._client.read(IMAGE_SIZE)
                image = np.asarray(image, dtype=np.uint8)
                if image.shape != IMAGE_SHAPE:
                    raise ValueError(
                        f"Camera {self.camera!r} returned {image.shape}; "
                        f"expected {IMAGE_SHAPE}"
                    )
                captured_s = time.monotonic()
                with self._lock:
                    self._frames.append((captured_s, image))
                    oldest_allowed_s = captured_s - self.args.camera_buffer_seconds
                    while (
                        len(self._frames) > 1
                        and self._frames[0][0] < oldest_allowed_s
                    ):
                        self._frames.popleft()
            except (zmq.ZMQError, ValueError) as exc:
                self._failures += 1
                if self._failures % 25 == 1:
                    print(
                        f"WARNING: remote camera {self.camera!r} poll failed; "
                        f"reconnecting camera: {exc}"
                    )
                self._replace_client()

            elapsed_s = time.monotonic() - start_s
            if self.period_s > elapsed_s:
                self._stop_event.wait(self.period_s - elapsed_s)


def _configure_camera_timeout(camera: ZMQClientCamera, timeout_ms: int) -> None:
    socket = getattr(camera, "_socket", None)
    if socket is None:
        raise AttributeError("ZMQClientCamera has no _socket attribute")
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    socket.setsockopt(zmq.LINGER, 0)


def _close_camera_client(client: ZMQClientCamera) -> None:
    client._socket.close()
    client._context.term()


def _close_camera_pollers(camera_pollers: Dict[str, RemoteCameraPoller]) -> None:
    for poller in camera_pollers.values():
        poller.close()


def _make_camera_client(args: Args, camera: str) -> ZMQClientCamera:
    if camera == "wrist":
        client = ZMQClientCamera(
            port=args.wrist_camera_port, host=args.camera_hostname
        )
    elif camera == "base":
        client = ZMQClientCamera(
            port=args.base_camera_port, host=args.camera_hostname
        )
    else:
        raise ValueError(
            f"Unsupported camera {camera!r}; expected 'wrist' or 'base'."
        )
    _configure_camera_timeout(client, args.camera_timeout_ms)
    return client


def _make_camera_pollers(args: Args) -> Dict[str, RemoteCameraPoller]:
    return {camera: RemoteCameraPoller(args, camera) for camera in args.cameras}


def _make_camera_clients(args: Args) -> Dict[str, ZMQClientCamera]:
    return {camera: _make_camera_client(args, camera) for camera in args.cameras}


def _close_camera_clients(camera_clients: Dict[str, ZMQClientCamera]) -> None:
    for client in camera_clients.values():
        _close_camera_client(client)


def _copy_robot_obs(obs: Dict[str, Any]) -> Dict[str, Any]:
    """Keep the small robot-state fields from a streamed observation."""

    kept: Dict[str, Any] = {}
    for key in (
        "joint_positions",
        "joint_velocities",
        "ee_pos_quat",
        "gripper_position",
    ):
        if key in obs:
            kept[key] = np.asarray(obs[key])
    if "joint_positions" not in kept:
        raise KeyError(
            "Streamed observation is missing 'joint_positions'. Start the laptop "
            "control loop with save-mode=recording_stream so it sends robot state."
        )
    return kept


def _fallback_camera_frame(
    camera: str, last_images: Dict[str, np.ndarray]
) -> np.ndarray:
    if camera in last_images:
        return last_images[camera].copy()
    return np.zeros(IMAGE_SHAPE, dtype=np.uint8)


def _read_remote_camera_frame(
    camera: str,
    client: ZMQClientCamera,
) -> np.ndarray:
    image, _depth = client.read(IMAGE_SIZE)
    image = np.asarray(image, dtype=np.uint8)
    if image.shape != IMAGE_SHAPE:
        raise ValueError(
            f"Camera {camera!r} returned {image.shape}; expected {IMAGE_SHAPE}"
        )
    return image


def _attach_on_frame_remote_camera_frames(
    obs: Dict[str, Any],
    camera_clients: Dict[str, ZMQClientCamera],
    camera_frame_buffers: Dict[str, Deque[np.ndarray]],
    last_images: Dict[str, np.ndarray],
    args: Args,
    frame_count: int,
) -> Dict[str, Any]:
    """Read cameras once per robot frame and attach a configurable older frame.

    This is intentionally lower impact than background polling: no camera ZMQ
    traffic happens while idle, while waiting for the first episode, or between
    streamed robot frames.  Use ``camera_frame_delay_frames`` to delay images
    when the camera appears ahead of the joints.
    """

    delay_frames = max(0, args.camera_frame_delay_frames)
    buffer_len = delay_frames + 1
    for camera, client in list(camera_clients.items()):
        buffer = camera_frame_buffers.setdefault(camera, deque(maxlen=buffer_len))
        try:
            image = _read_remote_camera_frame(camera, client)
            buffer.append(image)
            if len(buffer) > delay_frames:
                selected = buffer[0].copy()
            else:
                selected = _fallback_camera_frame(camera, last_images)
            last_images[camera] = selected
        except (zmq.ZMQError, ValueError) as exc:
            print(
                f"WARNING: remote camera {camera!r} read failed; using "
                f"{'last frame' if camera in last_images else 'black placeholder'} "
                f"and reconnecting camera: {exc}"
            )
            _close_camera_client(client)
            camera_clients[camera] = _make_camera_client(args, camera)
            selected = _fallback_camera_frame(camera, last_images)

        if args.log_sync_every > 0 and frame_count % args.log_sync_every == 0:
            print(
                f"sync: frame={frame_count} camera={camera} mode=on_frame "
                f"delay_frames={delay_frames} buffered_frames={len(buffer)}"
            )
        obs[f"{camera}_rgb"] = selected
    return obs


def _attach_buffered_remote_camera_frames(
    obs: Dict[str, Any],
    camera_pollers: Dict[str, RemoteCameraPoller],
    last_images: Dict[str, np.ndarray],
    args: Args,
    frame_count: int,
    message_received_s: float,
) -> Dict[str, Any]:
    target_camera_s = message_received_s - (args.camera_frame_delay_ms / 1000.0)
    for camera, poller in camera_pollers.items():
        image, captured_s, failures, buffered_frames = poller.frame_at_or_before(
            target_camera_s
        )
        age_ms = (
            None
            if captured_s is None
            else (message_received_s - captured_s) * 1000.0
        )
        target_error_ms = (
            None
            if captured_s is None
            else (captured_s - target_camera_s) * 1000.0
        )
        if image is None:
            print(
                f"WARNING: no polled frame from remote camera {camera!r} yet; "
                f"using {'last frame' if camera in last_images else 'black placeholder'}"
            )
            image = _fallback_camera_frame(camera, last_images)
        else:
            last_images[camera] = image
            if age_ms is not None and age_ms > args.camera_max_age_ms:
                print(
                    f"WARNING: latest remote camera {camera!r} frame is "
                    f"{age_ms:.1f} ms old; check camera poll FPS/network load."
                )
        if args.log_sync_every > 0 and frame_count % args.log_sync_every == 0:
            age_text = "n/a" if age_ms is None else f"{age_ms:.1f} ms"
            target_error_text = (
                "n/a" if target_error_ms is None else f"{target_error_ms:.1f} ms"
            )
            print(
                f"sync: frame={frame_count} camera={camera} "
                f"selected_age={age_text} target_error={target_error_text} "
                f"configured_camera_delay={args.camera_frame_delay_ms:.1f} ms "
                f"buffered_frames={buffered_frames} poll_failures={failures}"
            )
        obs[f"{camera}_rgb"] = image
    return obs


def main(args: Args) -> None:
    if args.camera_sync_mode not in {"on_frame", "background"}:
        raise ValueError(
            "camera_sync_mode must be 'on_frame' or 'background', got "
            f"{args.camera_sync_mode!r}"
        )

    receiver = ZMQRecordingReceiver(host=args.bind_hostname, port=args.port)
    camera_pollers: Dict[str, RemoteCameraPoller] = {}
    camera_clients: Dict[str, ZMQClientCamera] = {}
    if args.camera_sync_mode == "background":
        camera_pollers = _make_camera_pollers(args)
        for poller in camera_pollers.values():
            poller.start()
    else:
        camera_clients = _make_camera_clients(args)
    writer = LeRobotDatasetWriter(
        root=args.lerobot_root,
        repo_id=args.lerobot_repo_id,
        fps=args.lerobot_fps,
        task=args.lerobot_task,
        robot_type=args.lerobot_robot_type,
        camera_keys=args.cameras,
        streaming_encoding=args.lerobot_streaming_encoding,
        batch_encoding_size=args.lerobot_batch_encoding_size,
    )
    recording = False
    frame_count = 0
    last_images: Dict[str, np.ndarray] = {}
    camera_frame_buffers: Dict[str, Deque[np.ndarray]] = {}

    print("Waiting for state/action stream messages...")
    print("Remote camera host:", args.camera_hostname)
    print("Remote cameras:", args.cameras)
    print("Remote camera sync mode:", args.camera_sync_mode)
    if args.camera_sync_mode == "background":
        print(
            "Remote camera polling:",
            f"{args.camera_poll_fps} FPS, camera delay {args.camera_frame_delay_ms} ms, "
            f"stale warning after {args.camera_max_age_ms} ms",
        )
    else:
        print(
            "Remote camera on-frame reads:",
            f"delay {args.camera_frame_delay_frames} frame(s); "
            "no background camera polling",
        )

    try:
        while True:
            message = receiver.recv()
            if message is None:
                continue
            message_received_s = time.monotonic()

            message_type = message.get("type")
            if message_type == "start":
                recording = True
                frame_count = 0
                print(f"Started streamed episode at {message.get('timestamp')}")
            elif message_type == "frame":
                if not recording:
                    recording = True
                    frame_count = 0
                    print(
                        "Received frame before start marker; "
                        "starting streamed episode implicitly."
                    )

                obs = _copy_robot_obs(message["obs"])
                if args.camera_sync_mode == "background":
                    obs = _attach_buffered_remote_camera_frames(
                        obs,
                        camera_pollers,
                        last_images,
                        args,
                        frame_count,
                        message_received_s,
                    )
                else:
                    obs = _attach_on_frame_remote_camera_frames(
                        obs,
                        camera_clients,
                        camera_frame_buffers,
                        last_images,
                        args,
                        frame_count,
                    )
                writer.add_frame(obs, message["action"])
                frame_count += 1
                if frame_count % 100 == 0:
                    print(f"Recorded {frame_count} streamed frames")
            elif message_type == "stop" and recording:
                if confirm_episode_keep(frame_count):
                    writer.save_episode()
                    print(f"Saved streamed episode with {frame_count} frames")
                else:
                    writer.discard_episode()
                    print(f"Discarded streamed episode with {frame_count} frames")
                recording = False
                frame_count = 0
            elif message_type == "quit":
                if recording:
                    if confirm_episode_keep(frame_count):
                        writer.save_episode()
                        print(f"Saved streamed episode with {frame_count} frames")
                    else:
                        writer.discard_episode()
                        print(f"Discarded streamed episode with {frame_count} frames")
                break
            else:
                print(f"Ignoring recording stream message: {message_type}")
    finally:
        writer.finalize()
        _close_camera_pollers(camera_pollers)
        _close_camera_clients(camera_clients)
        receiver.close()
        print("LeRobot remote-camera stream recorder finalized")


if __name__ == "__main__":
    main(tyro.cli(Args))
