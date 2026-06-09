"""Run the real recording pipeline against a local MuJoCo pick task."""

from __future__ import annotations

import datetime
import time
from dataclasses import dataclass
from typing import Literal, Optional

import tyro

from gello.env import RobotEnv
from gello.robots.mujoco_panda_pick import (
    HardcodedPandaPickAgent,
    MujocoPandaPickRobot,
    MujocoPickSceneConfig,
    MujocoWristCamera,
)
from gello.utils.control_utils import LeRobotDatasetWriter
from gello.zmq_core.recording_node import ZMQRecordingPublisher


@dataclass
class Args:
    hz: int = 30
    steps_per_waypoint: int = 80
    save_mode: Literal["none", "lerobot", "recording_stream"] = "lerobot"
    lerobot_root: str = "~/lerobot_data/mujoco_panda_pick"
    lerobot_repo_id: str = "local/mujoco_panda_pick_wrist"
    lerobot_fps: int = 30
    lerobot_task: str = "Pick the red cube from the table in MuJoCo."
    lerobot_robot_type: str = "panda_gello"
    lerobot_streaming_encoding: bool = True
    lerobot_batch_encoding_size: int = 1
    record_stream_host: str = "127.0.0.1"
    record_stream_port: int = 7000
    record_stream_hwm: int = 2
    image_height: int = 480
    image_width: int = 640
    xml_dump_path: Optional[str] = "mujoco_panda_pick_scene.xml"


def main(args: Args) -> None:
    robot = MujocoPandaPickRobot(
        MujocoPickSceneConfig(
            image_height=args.image_height,
            image_width=args.image_width,
            xml_dump_path=args.xml_dump_path,
        )
    )
    env = RobotEnv(
        robot,
        control_rate_hz=args.hz,
        camera_dict={"wrist": MujocoWristCamera(robot)},
    )
    agent = HardcodedPandaPickAgent(steps_per_waypoint=args.steps_per_waypoint)

    writer = None
    publisher = None
    if args.save_mode == "lerobot":
        writer = LeRobotDatasetWriter(
            root=args.lerobot_root,
            repo_id=args.lerobot_repo_id,
            fps=args.lerobot_fps,
            task=args.lerobot_task,
            robot_type=args.lerobot_robot_type,
            camera_keys=("wrist",),
            streaming_encoding=args.lerobot_streaming_encoding,
            batch_encoding_size=args.lerobot_batch_encoding_size,
        )
    elif args.save_mode == "recording_stream":
        publisher = ZMQRecordingPublisher(
            host=args.record_stream_host,
            port=args.record_stream_port,
            send_hwm=args.record_stream_hwm,
        )
        publisher.send(
            {"type": "start", "timestamp": datetime.datetime.now().isoformat()}
        )

    print("MuJoCo pick pipeline simulation started")
    print(
        f"save_mode={args.save_mode}, hz={args.hz}, camera=wrist {args.image_width}x{args.image_height}"
    )

    frame_count = 0
    obs = env.get_obs()
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
            obs = env.step(action)
            frame_count += 1
            if frame_count % args.hz == 0:
                cube = obs.get("cube_position")
                grasped = bool(obs.get("cube_grasped", [False])[0])
                print(f"t={frame_count / args.hz:.1f}s cube={cube} grasped={grasped}")
            sleep_time = max(0.0, (1.0 / args.hz) - (time.time() - started))
            time.sleep(sleep_time)
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
