"""Run a fixed pick task through the same Polymetis path as the real robot."""

from __future__ import annotations

import datetime
import time
from dataclasses import dataclass
from typing import Literal

import tyro

from gello.agents.hardcoded_panda_pick_agent import HardcodedPandaPickAgent
from gello.env import RobotEnv
from gello.utils.control_utils import LeRobotDatasetWriter
from gello.zmq_core.camera_node import ZMQClientCamera
from gello.zmq_core.recording_node import ZMQRecordingPublisher
from gello.zmq_core.robot_node import ZMQClientRobot


@dataclass
class Args:
    robot_host: str = "127.0.0.1"
    robot_port: int = 6001
    hz: int = 30
    steps_per_waypoint: int = 80
    save_mode: Literal["none", "lerobot", "recording_stream"] = "lerobot"
    use_wrist_camera: bool = False
    wrist_camera_host: str = "127.0.0.1"
    wrist_camera_port: int = 5000
    lerobot_root: str = "~/lerobot_data/polymetis_mujoco_pick"
    lerobot_repo_id: str = "local/polymetis_mujoco_pick_wrist"
    lerobot_fps: int = 30
    lerobot_task: str = "Pick the cube in MuJoCo through the Polymetis Panda interface."
    lerobot_robot_type: str = "panda_polymetis_mujoco"
    lerobot_streaming_encoding: bool = True
    lerobot_batch_encoding_size: int = 1
    record_stream_host: str = "127.0.0.1"
    record_stream_port: int = 7000
    record_stream_hwm: int = 2
    robot_zmq_timeout_ms: int = 5000


def main(args: Args) -> None:
    robot = ZMQClientRobot(
        port=args.robot_port, host=args.robot_host, timeout_ms=args.robot_zmq_timeout_ms
    )
    camera_dict = {}
    if args.use_wrist_camera:
        camera_dict["wrist"] = ZMQClientCamera(
            port=args.wrist_camera_port, host=args.wrist_camera_host
        )
    env = RobotEnv(robot, control_rate_hz=args.hz, camera_dict=camera_dict)
    agent = HardcodedPandaPickAgent(steps_per_waypoint=args.steps_per_waypoint)

    writer = None
    publisher = None
    if args.save_mode == "lerobot":
        try:
            writer = LeRobotDatasetWriter(
                root=args.lerobot_root,
                repo_id=args.lerobot_repo_id,
                fps=args.lerobot_fps,
                task=args.lerobot_task,
                robot_type=args.lerobot_robot_type,
                camera_keys=("wrist",) if args.use_wrist_camera else tuple(),
                streaming_encoding=args.lerobot_streaming_encoding,
                batch_encoding_size=args.lerobot_batch_encoding_size,
            )
        except ModuleNotFoundError as exc:
            if exc.name != "lerobot":
                raise
            raise SystemExit(
                "LeRobot is not installed in the active Python environment. "
                "Install LeRobot into the polymetis environment before recording, "
                "or run a movement-only MuJoCo smoke test with "
                "--save-mode none. Important: this runner does not start MuJoCo; "
                "start the full stack with ./start_polymetis_mujoco_pick_pipeline.sh."
            ) from exc
    elif args.save_mode == "recording_stream":
        publisher = ZMQRecordingPublisher(
            host=args.record_stream_host,
            port=args.record_stream_port,
            send_hwm=args.record_stream_hwm,
        )
        publisher.send(
            {"type": "start", "timestamp": datetime.datetime.now().isoformat()}
        )

    print("Polymetis MuJoCo pick pipeline client started")
    print(
        "This script only runs the fixed pick policy against an already running "
        "ZMQ robot node. To start MuJoCo + Polymetis + ZMQ + this client, use "
        "./start_polymetis_mujoco_pick_pipeline.sh."
    )
    print(
        f"save_mode={args.save_mode}, hz={args.hz}, "
        f"use_wrist_camera={args.use_wrist_camera}"
    )

    frame_count = 0
    try:
        obs = env.get_obs()
    except Exception as exc:
        raise SystemExit(
            f"Could not read the first robot observation from "
            f"{args.robot_host}:{args.robot_port}: {exc}\n"
            "Make sure the full stack is running. Recommended command:\n"
            "  ./start_polymetis_mujoco_pick_pipeline.sh\n"
            "For a no-recording movement test when LeRobot is not installed:\n"
            "  SAVE_MODE=none ./start_polymetis_mujoco_pick_pipeline.sh"
        ) from exc
    try:
        while not agent.done:
            started = time.time()
            action = agent.act(obs)
            if writer is not None:
                writer.add_frame(obs, action)
            if publisher is not None:
                publisher.send(
                    {
                        "type": "frame",
                        "timestamp": datetime.datetime.now().isoformat(),
                        "obs": obs,
                        "action": action,
                    }
                )
            try:
                obs = env.step(action)
            except Exception as exc:
                raise SystemExit(
                    f"Robot/ZMQ stack failed while executing frame {frame_count}: {exc}\n"
                    "The simulator or robot ZMQ node likely crashed. Inspect the "
                    "polymetis_sim and robot_zmq tmux panes/logs from "
                    "./start_polymetis_mujoco_pick_pipeline.sh. If you are using "
                    "the built-in Polymetis smoke simulator, remember that it is not "
                    "the FER MuJoCo scene and may reset/crash under commands it cannot track."
                ) from exc
            frame_count += 1
            if frame_count % args.hz == 0:
                print(f"t={frame_count / args.hz:.1f}s joints={obs['joint_positions']}")
            time.sleep(max(0.0, (1.0 / args.hz) - (time.time() - started)))
    finally:
        if writer is not None:
            writer.save_episode()
            writer.finalize()
            print(f"Saved one LeRobot episode with {frame_count} frames")
        if publisher is not None:
            publisher.send(
                {"type": "stop", "timestamp": datetime.datetime.now().isoformat()}
            )
            publisher.close()
            print(f"Streamed one recording episode with {frame_count} frames")


if __name__ == "__main__":
    main(tyro.cli(Args))
