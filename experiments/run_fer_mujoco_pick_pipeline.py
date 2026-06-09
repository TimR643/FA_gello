"""Run the real recording pipeline against the FER ROS 2 MuJoCo simulator."""

from __future__ import annotations

import datetime
import time
from dataclasses import dataclass
from typing import Literal, Optional

import tyro

from gello.cameras.ros2_image_camera import Ros2ImageCamera, Ros2ImageCameraConfig
from gello.env import RobotEnv
from gello.robots.fer_mujoco_ros2 import (
    FerMujocoRos2Config,
    FerMujocoRos2Robot,
    HardcodedFerPickAgent,
)
from gello.utils.control_utils import LeRobotDatasetWriter
from gello.zmq_core.recording_node import ZMQRecordingPublisher


@dataclass
class Args:
    hz: int = 30
    steps_per_waypoint: int = 80
    save_mode: Literal["none", "lerobot", "recording_stream"] = "lerobot"
    use_wrist_camera: bool = True
    wrist_rgb_topic: str = "/wrist_camera/image_raw"
    wrist_depth_topic: Optional[str] = "/wrist_camera/depth/image_raw"
    joint_state_topic: str = "/joint_states"
    arm_command_topic: str = "/joint_effort_traj_controller/joint_trajectory"
    gripper_action_name: str = "/gripper_effort_controller/gripper_cmd"
    lerobot_root: str = "~/lerobot_data/fer_mujoco_pick"
    lerobot_repo_id: str = "local/fer_mujoco_pick_wrist"
    lerobot_fps: int = 30
    lerobot_task: str = "Pick the cube from the table in the FER MuJoCo simulator."
    lerobot_robot_type: str = "fer_mujoco"
    lerobot_streaming_encoding: bool = True
    lerobot_batch_encoding_size: int = 1
    record_stream_host: str = "127.0.0.1"
    record_stream_port: int = 7000
    record_stream_hwm: int = 2


def main(args: Args) -> None:
    robot = FerMujocoRos2Robot(
        FerMujocoRos2Config(
            joint_state_topic=args.joint_state_topic,
            arm_command_topic=args.arm_command_topic,
            gripper_action_name=args.gripper_action_name,
        )
    )
    camera_dict = {}
    if args.use_wrist_camera:
        camera_dict["wrist"] = Ros2ImageCamera(
            Ros2ImageCameraConfig(
                rgb_topic=args.wrist_rgb_topic,
                depth_topic=args.wrist_depth_topic,
            )
        )
    env = RobotEnv(robot, control_rate_hz=args.hz, camera_dict=camera_dict)
    agent = HardcodedFerPickAgent(steps_per_waypoint=args.steps_per_waypoint)

    writer = None
    publisher = None
    camera_keys = ("wrist",) if args.use_wrist_camera else tuple()
    if args.save_mode == "lerobot":
        writer = LeRobotDatasetWriter(
            root=args.lerobot_root,
            repo_id=args.lerobot_repo_id,
            fps=args.lerobot_fps,
            task=args.lerobot_task,
            robot_type=args.lerobot_robot_type,
            camera_keys=camera_keys,
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

    print("FER MuJoCo pick pipeline started")
    print(
        f"save_mode={args.save_mode}, hz={args.hz}, wrist_camera={args.use_wrist_camera}"
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
        robot.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
