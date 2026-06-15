"""Configuration for the LeRobot GELLO/ZMQ robot plugin."""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("gello_zmq")
@dataclass
class GelloZMQConfig(RobotConfig):
    """Connect LeRobot's native rollout scripts to GELLO's ZMQ robot bridge.

    The defaults match this repository's Panda launch scripts: the robot server
    is on port 6001 and the wrist/base camera servers are on ports 5000/5001.
    """

    robot_host: str = "127.0.0.1"
    robot_port: int = 6001
    camera_host: str | None = None
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    zmq_timeout_ms: int = 3000
    camera_names: str = "wrist,base"
    state_key: str = "observation.state"
    action_key: str = "action"
    num_dofs: int = 8
    image_height: int = 480
    image_width: int = 640
    max_joint_delta: float = 0.015
    max_gripper_delta: float = 0.03
    action_mode: str = "absolute_joint_position"
    joint_lower: tuple[float, ...] = field(
        default_factory=lambda: (
            -2.8973,
            -1.7628,
            -2.8973,
            -3.0718,
            -2.8973,
            -0.0175,
            -2.8973,
            0.0,
        )
    )
    joint_upper: tuple[float, ...] = field(
        default_factory=lambda: (
            2.8973,
            1.7628,
            2.8973,
            -0.0698,
            2.8973,
            3.7525,
            2.8973,
            2.2,
        )
    )
