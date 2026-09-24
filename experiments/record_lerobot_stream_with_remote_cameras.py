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
from typing import Any, Dict, Tuple

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
                obs = _attach_remote_camera_frames(
                    obs, camera_clients, last_images, args
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
        _close_camera_clients(camera_clients)
        receiver.close()
        print("LeRobot remote-camera stream recorder finalized")


if __name__ == "__main__":
    main(tyro.cli(Args))
