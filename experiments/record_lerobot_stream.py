from dataclasses import dataclass
from typing import Any, Dict, Tuple

import tyro

from gello.utils.control_utils import LeRobotDatasetWriter, confirm_episode_keep
from gello.zmq_core.recording_node import ZMQRecordingReceiver


@dataclass
class Args:
    bind_hostname: str = "0.0.0.0"
    port: int = 7000
    lerobot_root: str = "~/lerobot_data"
    lerobot_repo_id: str = "local/panda_gello_wrist"
    lerobot_fps: int = 10
    lerobot_task: str = "Teleoperate Panda with GELLO (wrist only)."
    lerobot_robot_type: str = "panda_gello"
    cameras: Tuple[str, ...] = ("wrist",)
    lerobot_streaming_encoding: bool = True
    lerobot_batch_encoding_size: int = 1


def _message_keep_decision(message: Dict[str, Any], frame_count: int) -> bool:
    if "keep" in message:
        keep = bool(message["keep"])
        print(
            "Using keep/discard decision sent by the control laptop: "
            f"{'keep' if keep else 'discard'}"
        )
        return keep
    return confirm_episode_keep(frame_count)


def main(args: Args) -> None:
    receiver = ZMQRecordingReceiver(host=args.bind_hostname, port=args.port)
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

    print("Waiting for recording stream messages...")
    print(
        "Start/stop episodes in the control terminal. "
        "The sender can pass the RIGHT/LEFT keep/discard decision to this recorder."
    )
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
                writer.add_frame(message["obs"], message["action"])
                frame_count += 1
                if frame_count % 100 == 0:
                    print(f"Recorded {frame_count} streamed frames")
            elif message_type == "stop" and recording:
                if _message_keep_decision(message, frame_count):
                    writer.save_episode()
                    print(f"Saved streamed episode with {frame_count} frames")
                else:
                    writer.discard_episode()
                    print(f"Discarded streamed episode with {frame_count} frames")
                recording = False
                frame_count = 0
            elif message_type == "quit":
                if recording:
                    if _message_keep_decision(message, frame_count):
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
        receiver.close()
        print("LeRobot stream recorder finalized")


if __name__ == "__main__":
    main(tyro.cli(Args))
