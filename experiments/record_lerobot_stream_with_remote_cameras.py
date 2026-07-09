"""Record a LeRobot dataset on this machine while pulling cameras remotely.

This recorder is intended for setups where the robot-control laptop must not read
or serialize camera frames in its real-time control loop. The laptop streams only
small robot observations/actions via ``RecordingStreamInterface``. This process
runs on the recording machine, receives those small messages, pulls RGB frames
from camera ZMQ servers (typically running on the laptop), and writes a LeRobot
video dataset.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

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
    camera_poll_period_s: float = 0.05
    camera_startup_timeout_s: float = 5.0


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


class RemoteCameraPoller:
    """Continuously keep the newest frame from one remote ZMQ camera.

    The recorder must not block the state/action receive loop on synchronous
    camera reads. If it does, two cameras plus network/encoding jitter can make
    the PULL socket accumulate old robot samples, so the video appears seconds
    behind the joint/action plots. This poller absorbs camera latency in a
    background thread and lets the recorder attach the latest available image in
    constant time. The polling rate is deliberately capped so the camera servers
    on the Franka/Polymetis laptop are not hammered while it is controlling the
    robot.
    """

    def __init__(self, camera: str, args: Args):
        self.camera = camera
        self.args = args
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._first_frame_event = threading.Event()
        self._client: Optional[ZMQClientCamera] = None
        self._frame: Optional[np.ndarray] = None
        self._frame_time_monotonic: Optional[float] = None
        self._failures = 0
        self._thread = threading.Thread(
            target=self._run,
            name=f"remote-camera-poller-{camera}",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def wait_for_first_frame(self, timeout_s: float) -> bool:
        return self._first_frame_event.wait(timeout=max(0.0, timeout_s))

    def latest_frame(self) -> np.ndarray:
        with self._lock:
            if self._frame is None:
                return np.zeros(IMAGE_SHAPE, dtype=np.uint8)
            return self._frame.copy()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._close_client()

    def _make_client(self) -> ZMQClientCamera:
        if self.camera == "wrist":
            client = ZMQClientCamera(
                port=self.args.wrist_camera_port, host=self.args.camera_hostname
            )
        elif self.camera == "base":
            client = ZMQClientCamera(
                port=self.args.base_camera_port, host=self.args.camera_hostname
            )
        else:
            raise ValueError(
                f"Unsupported camera {self.camera!r}; expected 'wrist' or 'base'."
            )
        _configure_camera_timeout(client, self.args.camera_timeout_ms)
        return client

    def _close_client(self) -> None:
        if self._client is not None:
            _close_camera_client(self._client)
            self._client = None

    def _reconnect_after_failure(self, exc: Exception) -> None:
        self._failures += 1
        if self._failures % 50 == 1:
            print(
                f"WARNING: remote camera {self.camera!r} polling failed; "
                f"reconnecting and keeping last frame: {exc}"
            )
        self._close_client()
        time.sleep(0.1)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                if self._client is None:
                    self._client = self._make_client()
                image, _depth = self._client.read(IMAGE_SIZE)
                image = np.asarray(image, dtype=np.uint8)
                if image.shape != IMAGE_SHAPE:
                    raise ValueError(
                        f"Camera {self.camera!r} returned {image.shape}; "
                        f"expected {IMAGE_SHAPE}"
                    )
                with self._lock:
                    self._frame = image.copy()
                    self._frame_time_monotonic = time.monotonic()
                self._first_frame_event.set()
                self._failures = 0
                if self.args.camera_poll_period_s > 0:
                    self._stop_event.wait(self.args.camera_poll_period_s)
            except (zmq.ZMQError, ValueError) as exc:
                self._reconnect_after_failure(exc)


def _make_camera_pollers(args: Args) -> Dict[str, RemoteCameraPoller]:
    pollers = {camera: RemoteCameraPoller(camera, args) for camera in args.cameras}
    for poller in pollers.values():
        poller.start()
    return pollers


def _close_camera_pollers(camera_pollers: Dict[str, RemoteCameraPoller]) -> None:
    for poller in camera_pollers.values():
        poller.stop()


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


def _attach_remote_camera_frames(
    obs: Dict[str, Any],
    camera_pollers: Dict[str, RemoteCameraPoller],
) -> Dict[str, Any]:
    for camera, poller in camera_pollers.items():
        obs[f"{camera}_rgb"] = poller.latest_frame()
    return obs


def main(args: Args) -> None:
    receiver = ZMQRecordingReceiver(host=args.bind_hostname, port=args.port)
    camera_pollers: Optional[Dict[str, RemoteCameraPoller]] = None
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

    print("Waiting for state/action stream messages...")
    print("Remote camera host:", args.camera_hostname)
    print("Remote cameras:", args.cameras)

    def start_camera_pollers() -> Dict[str, RemoteCameraPoller]:
        pollers = _make_camera_pollers(args)
        for camera, poller in pollers.items():
            if not poller.wait_for_first_frame(args.camera_startup_timeout_s):
                print(
                    f"WARNING: no initial frame from {camera!r} within "
                    f"{args.camera_startup_timeout_s}s; black frames will be used until "
                    "the camera responds."
                )
        return pollers

    try:
        while True:
            message = receiver.recv()
            if message is None:
                continue

            message_type = message.get("type")
            if message_type == "start":
                recording = True
                frame_count = 0
                if camera_pollers is None:
                    camera_pollers = start_camera_pollers()
                print(f"Started streamed episode at {message.get('timestamp')}")
            elif message_type == "frame":
                if not recording:
                    recording = True
                    frame_count = 0
                    print(
                        "Received frame before start marker; "
                        "starting streamed episode implicitly."
                    )
                if camera_pollers is None:
                    camera_pollers = start_camera_pollers()

                obs = _copy_robot_obs(message["obs"])
                obs = _attach_remote_camera_frames(obs, camera_pollers)
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
                if camera_pollers is not None:
                    _close_camera_pollers(camera_pollers)
                    camera_pollers = None
            elif message_type == "quit":
                if recording:
                    if confirm_episode_keep(frame_count):
                        writer.save_episode()
                        print(f"Saved streamed episode with {frame_count} frames")
                    else:
                        writer.discard_episode()
                        print(f"Discarded streamed episode with {frame_count} frames")
                if camera_pollers is not None:
                    _close_camera_pollers(camera_pollers)
                    camera_pollers = None
                break
            else:
                print(f"Ignoring recording stream message: {message_type}")
    finally:
        writer.finalize()
        if camera_pollers is not None:
            _close_camera_pollers(camera_pollers)
        receiver.close()
        print("LeRobot remote-camera stream recorder finalized")


if __name__ == "__main__":
    main(tyro.cli(Args))
