"""Record synchronized LeRobot episodes without loading the Franka laptop.

The Franka/Polymetis laptop streams only small robot observations/actions via
``RecordingStreamInterface``. Camera capture should run on this recording machine
(or another non-real-time machine), not on the robot-control laptop. The recorder
keeps the newest frame from both cameras in background workers and attaches those
frames to each received robot sample so video and joints share the same LeRobot
frame index without blocking the Franka real-time loop.
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

    camera_source: str = "mixed"
    wrist_camera_source: str = "remote_zmq"
    base_camera_source: str = "local_realsense"
    camera_hostname: str = "127.0.0.1"
    wrist_camera_hostname: str = ""
    base_camera_hostname: str = ""
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    wrist_camera_id: str = "6CD1460304A5"
    base_camera_id: str = "318122303303"
    cameras: Tuple[str, ...] = ("wrist", "base")

    lerobot_root: str = "~/lerobot_data"
    lerobot_repo_id: str = "local/panda_gello_remote_cameras"
    lerobot_fps: int = 20
    lerobot_task: str = "Teleoperate Panda with GELLO."
    lerobot_robot_type: str = "panda_gello"
    lerobot_streaming_encoding: bool = True
    lerobot_batch_encoding_size: int = 1
    enable_remote_camera_polling: bool = False
    camera_timeout_ms: int = 3000
    camera_poll_period_s: float = 0.05
    camera_startup_timeout_s: float = 5.0



def _camera_source_for(camera: str, args: Args) -> str:
    if args.enable_remote_camera_polling:
        return "remote_zmq"
    if args.camera_source != "mixed":
        return args.camera_source
    if camera == "wrist":
        return args.wrist_camera_source
    if camera == "base":
        return args.base_camera_source
    raise ValueError(f"Unsupported camera {camera!r}; expected 'wrist' or 'base'.")


def _camera_host_for(camera: str, args: Args) -> str:
    if camera == "wrist":
        return args.wrist_camera_hostname or args.camera_hostname
    if camera == "base":
        return args.base_camera_hostname or args.camera_hostname
    raise ValueError(f"Unsupported camera {camera!r}; expected 'wrist' or 'base'.")

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
    """Continuously keep the newest frame from one camera source.

    The recorder must not block the state/action receive loop on camera reads.
    Camera workers run on the recording machine by default via local RealSense
    devices, so the Franka/Polymetis laptop only sends small robot messages. A
    remote ZMQ source remains available for non-real-time camera hosts, but it
    should not point at the robot-control laptop.
    """

    def __init__(self, camera: str, args: Args):
        self.camera = camera
        self.args = args
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._first_frame_event = threading.Event()
        self._client: Optional[Any] = None
        self._frame: Optional[np.ndarray] = None
        self._frame_time_monotonic: Optional[float] = None
        self._last_error: Optional[str] = None
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

    def last_error(self) -> Optional[str]:
        with self._lock:
            return self._last_error

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._close_client()

    def _make_client(self) -> Any:
        if self.camera not in {"wrist", "base"}:
            raise ValueError(
                f"Unsupported camera {self.camera!r}; expected 'wrist' or 'base'."
            )

        camera_source = _camera_source_for(self.camera, self.args)

        if camera_source == "local_realsense":
            from gello.cameras.realsense_camera import RealSenseCamera

            camera_id = (
                self.args.wrist_camera_id
                if self.camera == "wrist"
                else self.args.base_camera_id
            )
            if not camera_id:
                raise ValueError(
                    f"Missing {self.camera}_camera_id for local RealSense capture. "
                    "Pass --wrist-camera-id/--base-camera-id or use "
                    "--camera-source remote_zmq with cameras hosted away from the "
                    "Franka realtime laptop."
                )
            return RealSenseCamera(camera_id)

        if camera_source == "remote_zmq":
            if self.camera == "wrist":
                client = ZMQClientCamera(
                    port=self.args.wrist_camera_port,
                    host=_camera_host_for(self.camera, self.args),
                )
            else:
                client = ZMQClientCamera(
                    port=self.args.base_camera_port,
                    host=_camera_host_for(self.camera, self.args),
                )
            _configure_camera_timeout(client, self.args.camera_timeout_ms)
            return client

        raise ValueError(
            f"Unsupported camera_source {camera_source!r}; expected "
            "'local_realsense', 'remote_zmq', or 'disabled'."
        )

    def _close_client(self) -> None:
        if self._client is None:
            return
        if isinstance(self._client, ZMQClientCamera):
            _close_camera_client(self._client)
        else:
            pipeline = getattr(self._client, "_pipeline", None)
            if pipeline is not None:
                pipeline.stop()
        self._client = None

    def _reconnect_after_failure(self, exc: Exception) -> None:
        self._failures += 1
        if self._failures % 50 == 1:
            print(
                f"WARNING: camera {self.camera!r} polling failed; "
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
                    self._last_error = None
                self._first_frame_event.set()
                self._failures = 0
                if self.args.camera_poll_period_s > 0:
                    self._stop_event.wait(self.args.camera_poll_period_s)
            except Exception as exc:
                with self._lock:
                    self._last_error = repr(exc)
                self._reconnect_after_failure(exc)


def _make_camera_pollers(args: Args) -> Dict[str, RemoteCameraPoller]:
    pollers = {
        camera: RemoteCameraPoller(camera, args)
        for camera in args.cameras
        if _camera_source_for(camera, args) != "disabled"
    }
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
    cameras: Tuple[str, ...],
    camera_pollers: Optional[Dict[str, RemoteCameraPoller]],
) -> Dict[str, Any]:
    for camera in cameras:
        if camera_pollers is None or camera not in camera_pollers:
            obs[f"{camera}_rgb"] = np.zeros(IMAGE_SHAPE, dtype=np.uint8)
        else:
            obs[f"{camera}_rgb"] = camera_pollers[camera].latest_frame()
    return obs


def _camera_failure_hint(camera: str, source: str, detail: str) -> str:
    if source == "remote_zmq":
        return (
            f"{camera}: {detail} (remote_zmq: check "
            "WRIST_CAMERA_HOST/HPC_CAMERA_HOST, port, tunnel, and that the "
            "Ethernet/ZMQ camera server is running)"
        )
    if source == "local_realsense" and (
        "busy" in detail.lower() or "errno=16" in detail
    ):
        return (
            f"{camera}: {detail} (local_realsense: the device is busy; close "
            "realsense-viewer/old recorder/camera server or find the holder with "
            "fuser -v /dev/video*)"
        )
    if source == "local_realsense":
        return (
            f"{camera}: {detail} (local_realsense: check USB connection, serial "
            "ID, permissions, and that no other process owns the camera)"
        )
    return f"{camera}: {detail}"


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

    camera_sources = {camera: _camera_source_for(camera, args) for camera in args.cameras}
    print("Waiting for state/action stream messages...")
    print("Camera sources:", camera_sources)
    print(
        "Camera hosts for remote_zmq:",
        {camera: _camera_host_for(camera, args) for camera in args.cameras},
    )
    print("Cameras:", args.cameras)
    if any(source == "remote_zmq" for source in camera_sources.values()):
        print(
            "WARNING: remote_zmq camera capture must not point at the "
            "Franka/Polymetis realtime laptop. Use it only for Ethernet cameras "
            "or non-realtime camera hosts."
        )
    if all(source == "disabled" for source in camera_sources.values()):
        print("Camera capture disabled; recording black video placeholders.")

    def start_camera_pollers() -> Optional[Dict[str, RemoteCameraPoller]]:
        if all(source == "disabled" for source in camera_sources.values()):
            return None
        pollers = _make_camera_pollers(args)
        missing_frames = []
        for camera, poller in pollers.items():
            if not poller.wait_for_first_frame(args.camera_startup_timeout_s):
                detail = poller.last_error() or "no error reported by camera worker"
                missing_frames.append(
                    _camera_failure_hint(camera, camera_sources[camera], detail)
                )
        if missing_frames:
            _close_camera_pollers(pollers)
            raise RuntimeError(
                "Camera capture did not produce an initial frame within "
                f"{args.camera_startup_timeout_s}s. Refusing to record black "
                "videos. Check camera_source, USB connection, camera serial IDs, "
                "and RealSense permissions. Details: " + "; ".join(missing_frames)
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
                obs = _attach_remote_camera_frames(obs, args.cameras, camera_pollers)
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
