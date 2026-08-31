"""Record a LeRobot dataset on this machine while pulling cameras remotely.

This recorder is intended for setups where the robot-control laptop must not read
or serialize camera frames in its real-time control loop. The laptop streams only
small robot observations/actions via ``RecordingStreamInterface``. This process
runs on the recording machine, receives those small messages, pulls RGB frames
from camera ZMQ servers (typically running on the laptop), and writes a LeRobot
video dataset.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import tyro
import zmq

from gello.data_utils.h5_logger import H5RobotLogger
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
    h5_log_enabled: bool = True
    h5_log_dir: str = "~/lerobot_data/h5"
    h5_log_basename: str = "teleoperation"
    h5_flush_every: int = 1


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


def _close_camera_clients(camera_clients: Dict[str, ZMQClientCamera]) -> None:
    for client in camera_clients.values():
        _close_camera_client(client)


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


def _make_camera_clients(args: Args) -> Dict[str, ZMQClientCamera]:
    return {camera: _make_camera_client(args, camera) for camera in args.cameras}


def _copy_robot_obs(obs: Dict[str, Any]) -> Dict[str, Any]:
    """Keep the small robot-state fields from a streamed observation."""

    kept: Dict[str, Any] = {}
    for key in (
        "joint_positions",
        "joint_velocities",
        "joint_torques",
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


def _attach_remote_camera_frames(
    obs: Dict[str, Any],
    camera_clients: Dict[str, ZMQClientCamera],
    last_images: Dict[str, np.ndarray],
    args: Args,
) -> Dict[str, Any]:
    for camera, client in list(camera_clients.items()):
        try:
            image, _depth = client.read(IMAGE_SIZE)
            image = np.asarray(image, dtype=np.uint8)
            if image.shape != IMAGE_SHAPE:
                raise ValueError(
                    f"Camera {camera!r} returned {image.shape}; expected {IMAGE_SHAPE}"
                )
            last_images[camera] = image
        except (zmq.ZMQError, ValueError) as exc:
            print(
                f"WARNING: remote camera {camera!r} read failed; using "
                f"{'last frame' if camera in last_images else 'black placeholder'} "
                f"and reconnecting camera: {exc}"
            )
            _close_camera_client(client)
            camera_clients[camera] = _make_camera_client(args, camera)
            image = _fallback_camera_frame(camera, last_images)
        obs[f"{camera}_rgb"] = image
    return obs


def _message_time_s(message: Dict[str, Any]) -> float:
    value = message.get("timestamp")
    if isinstance(value, str):
        try:
            return datetime.fromisoformat(value).timestamp()
        except ValueError:
            pass
    return datetime.now(timezone.utc).timestamp()


def _open_episode_h5(args: Args) -> H5RobotLogger | None:
    if not args.h5_log_enabled:
        return None
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    path = Path(args.h5_log_dir).expanduser() / f"{args.h5_log_basename}_{stamp}.h5"
    logger = H5RobotLogger(path, num_dofs=8, flush_every=args.h5_flush_every)
    print(f"Started H5 episode log: {path}")
    return logger


def _log_h5_frame(
    logger: H5RobotLogger | None, message: Dict[str, Any], obs: Dict[str, Any]
) -> None:
    if logger is None:
        return
    q = np.asarray(obs["joint_positions"], dtype=np.float32)
    nan = np.full(q.shape, np.nan, dtype=np.float32)
    dq = np.asarray(obs.get("joint_velocities", nan), dtype=np.float32)
    tau = np.asarray(obs.get("joint_torques", nan), dtype=np.float32)
    action = np.asarray(message["action"], dtype=np.float32)
    timestamp_s = _message_time_s(message)
    logger.log_observation(timestamp_s, q, dq, tau)
    # The streamed action is already the command used by the GELLO control loop.
    logger.log_action(timestamp_s, action, action)


def main(args: Args) -> None:
    receiver = ZMQRecordingReceiver(host=args.bind_hostname, port=args.port)
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
    h5_logger: H5RobotLogger | None = None

    print("Waiting for state/action stream messages...")
    print("Remote camera host:", args.camera_hostname)
    print("Remote cameras:", args.cameras)

    try:
        while True:
            message = receiver.recv()
            if message is None:
                continue

            message_type = message.get("type")
            if message_type == "start":
                if h5_logger is not None:
                    h5_logger.close()
                h5_logger = _open_episode_h5(args)
                recording = True
                frame_count = 0
                print(f"Started streamed episode at {message.get('timestamp')}")
            elif message_type == "frame":
                if not recording:
                    h5_logger = _open_episode_h5(args)
                    recording = True
                    frame_count = 0
                    print(
                        "Received frame before start marker; "
                        "starting streamed episode implicitly."
                    )

                obs = _copy_robot_obs(message["obs"])
                obs = _attach_remote_camera_frames(
                    obs, camera_clients, last_images, args
                )
                writer.add_frame(obs, message["action"])
                _log_h5_frame(h5_logger, message, obs)
                frame_count += 1
                if frame_count % 100 == 0:
                    print(f"Recorded {frame_count} streamed frames")
            elif message_type == "stop" and recording:
                if confirm_episode_keep(frame_count):
                    writer.save_episode()
                    if h5_logger is not None:
                        h5_logger.close()
                    print(f"Saved streamed episode with {frame_count} frames")
                else:
                    writer.discard_episode()
                    if h5_logger is not None:
                        h5_path = h5_logger.path
                        h5_logger.close()
                        h5_path.unlink(missing_ok=True)
                    print(f"Discarded streamed episode with {frame_count} frames")
                h5_logger = None
                recording = False
                frame_count = 0
            elif message_type == "quit":
                if recording:
                    if confirm_episode_keep(frame_count):
                        writer.save_episode()
                        if h5_logger is not None:
                            h5_logger.close()
                        print(f"Saved streamed episode with {frame_count} frames")
                    else:
                        writer.discard_episode()
                        if h5_logger is not None:
                            h5_path = h5_logger.path
                            h5_logger.close()
                            h5_path.unlink(missing_ok=True)
                        print(f"Discarded streamed episode with {frame_count} frames")
                    h5_logger = None
                break
            else:
                print(f"Ignoring recording stream message: {message_type}")
    finally:
        if h5_logger is not None:
            h5_logger.close()
        writer.finalize()
        _close_camera_clients(camera_clients)
        receiver.close()
        print("LeRobot remote-camera stream recorder finalized")


if __name__ == "__main__":
    main(tyro.cli(Args))
