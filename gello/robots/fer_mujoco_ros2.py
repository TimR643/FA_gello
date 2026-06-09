"""ROS 2 bridge for the FER MuJoCo simulator used by Tim's pipeline.

The simulator itself is provided by
https://github.com/GKnerd/fer_ros2_simulation and its
``franka_mujoco_sim_bringup`` workspace.  This module intentionally does not
build a second, approximate MuJoCo model.  Instead it adapts the correct FER
robot exposed by ``ros2_control`` to this repository's existing ``RobotEnv`` /
ZMQ / LeRobot recording pipeline.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

from gello.robots.robot import Robot

FER_ARM_JOINTS: Tuple[str, ...] = tuple(f"fer_joint{i}" for i in range(1, 8))
FER_GRIPPER_JOINT = "fer_finger_joint1"
FER_JOINTS: Tuple[str, ...] = FER_ARM_JOINTS + (FER_GRIPPER_JOINT,)
FER_HOME = np.array(
    [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.08], dtype=np.float32
)
FER_PREGRASP = np.array(
    [0.0, -0.60, 0.0, -2.20, 0.0, 1.65, 0.78, 0.08], dtype=np.float32
)
FER_GRASP = np.array([0.0, -0.78, 0.0, -2.38, 0.0, 1.60, 0.78, 0.08], dtype=np.float32)
FER_LIFT = np.array([0.0, -0.40, 0.0, -1.95, 0.0, 1.45, 0.78, 0.0], dtype=np.float32)
FER_PLACE = np.array(
    [0.35, -0.48, -0.20, -1.95, 0.10, 1.50, 0.55, 0.0], dtype=np.float32
)


@dataclass(frozen=True)
class FerMujocoRos2Config:
    """ROS interface details for ``franka_mujoco_sim_bringup``."""

    node_name: str = "gello_fer_mujoco_robot"
    joint_state_topic: str = "/joint_states"
    arm_command_topic: str = "/joint_effort_traj_controller/joint_trajectory"
    gripper_action_name: str = "/gripper_effort_controller/gripper_cmd"
    base_frame: str = "base"
    ee_frame: str = "fer_hand_tcp"
    command_time_from_start: float = 0.08
    state_timeout: float = 5.0
    max_gripper_width: float = 0.08
    spin_period: float = 0.001


class FerMujocoRos2Robot(Robot):
    """Expose the FER MuJoCo ROS 2 simulation through the GELLO ``Robot`` API."""

    def __init__(self, config: Optional[FerMujocoRos2Config] = None):
        self.config = config or FerMujocoRos2Config()

        import rclpy
        from builtin_interfaces.msg import Duration
        from control_msgs.action import GripperCommand
        from rclpy.action import ActionClient
        from rclpy.time import Time
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        if not rclpy.ok():
            rclpy.init(args=None)

        self._rclpy = rclpy
        self._duration_type = Duration
        self._time_type = Time
        self._trajectory_type = JointTrajectory
        self._trajectory_point_type = JointTrajectoryPoint
        self._gripper_goal_type = GripperCommand.Goal
        self._lock = threading.Lock()
        self._joint_positions = np.zeros(8, dtype=np.float32)
        self._joint_velocities = np.zeros(8, dtype=np.float32)
        self._last_state_time: Optional[float] = None
        self._last_gripper_goal: Optional[float] = None

        self._node = rclpy.create_node(self.config.node_name)
        self._arm_pub = self._node.create_publisher(
            JointTrajectory, self.config.arm_command_topic, 10
        )
        self._gripper_client = ActionClient(
            self._node, GripperCommand, self.config.gripper_action_name
        )
        self._joint_sub = self._node.create_subscription(
            JointState, self.config.joint_state_topic, self._joint_state_callback, 10
        )

        import tf2_ros

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self._wait_for_state()

    def num_dofs(self) -> int:
        return 8

    def get_joint_state(self) -> np.ndarray:
        self._ensure_state_fresh()
        with self._lock:
            return self._joint_positions.copy()

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        joint_state = np.asarray(joint_state, dtype=np.float32)
        if joint_state.shape != (8,):
            raise ValueError(f"Expected 8-DoF FER command, got {joint_state.shape}")
        self._publish_arm_command(joint_state[:7])
        self._send_gripper_goal(float(joint_state[7]))

    def get_observations(self) -> Dict[str, np.ndarray]:
        self._ensure_state_fresh()
        with self._lock:
            joints = self._joint_positions.copy()
            velocities = self._joint_velocities.copy()
        return {
            "joint_positions": joints,
            "joint_velocities": velocities,
            "ee_pos_quat": self._lookup_ee_pose(),
            "gripper_position": np.asarray([joints[-1]], dtype=np.float32),
        }

    def close(self) -> None:
        self._rclpy.shutdown()

    def _spin(self) -> None:
        while self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=self.config.spin_period)

    def _joint_state_callback(self, msg: Any) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        positions = np.zeros(8, dtype=np.float32)
        velocities = np.zeros(8, dtype=np.float32)
        for out_idx, joint_name in enumerate(FER_ARM_JOINTS):
            msg_idx = name_to_index.get(joint_name)
            if msg_idx is not None:
                positions[out_idx] = msg.position[msg_idx]
                if msg_idx < len(msg.velocity):
                    velocities[out_idx] = msg.velocity[msg_idx]
        finger_idx = name_to_index.get(FER_GRIPPER_JOINT)
        if finger_idx is not None:
            width = 2.0 * float(msg.position[finger_idx])
            positions[7] = np.clip(width / self.config.max_gripper_width, 0.0, 1.0)
            if finger_idx < len(msg.velocity):
                velocities[7] = msg.velocity[finger_idx]
        with self._lock:
            self._joint_positions = positions
            self._joint_velocities = velocities
            self._last_state_time = time.monotonic()

    def _publish_arm_command(self, arm_joints: Sequence[float]) -> None:
        msg = self._trajectory_type()
        msg.joint_names = list(FER_ARM_JOINTS)
        point = self._trajectory_point_type()
        point.positions = [float(value) for value in arm_joints]
        point.time_from_start = self._duration(self.config.command_time_from_start)
        msg.points.append(point)
        self._arm_pub.publish(msg)

    def _send_gripper_goal(self, normalized_width: float) -> None:
        normalized_width = float(np.clip(normalized_width, 0.0, 1.0))
        if (
            self._last_gripper_goal is not None
            and abs(normalized_width - self._last_gripper_goal) < 0.01
        ):
            return
        goal = self._gripper_goal_type()
        goal.command.position = normalized_width * self.config.max_gripper_width / 2.0
        goal.command.max_effort = 20.0
        self._gripper_client.wait_for_server(timeout_sec=0.01)
        self._gripper_client.send_goal_async(goal)
        self._last_gripper_goal = normalized_width

    def _duration(self, seconds: float) -> Any:
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1_000_000_000)
        return self._duration_type(sec=sec, nanosec=nanosec)

    def _lookup_ee_pose(self) -> np.ndarray:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.config.base_frame,
                self.config.ee_frame,
                self._time_type(),
            )
        except Exception:
            pose = np.zeros(7, dtype=np.float32)
            pose[3] = 1.0
            return pose
        trans = transform.transform.translation
        rot = transform.transform.rotation
        return np.asarray(
            [trans.x, trans.y, trans.z, rot.w, rot.x, rot.y, rot.z], dtype=np.float32
        )

    def _wait_for_state(self) -> None:
        deadline = time.monotonic() + self.config.state_timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._last_state_time is not None:
                    return
            time.sleep(0.02)
        raise TimeoutError(
            f"No FER joint state received on {self.config.joint_state_topic!r}. "
            "Start the franka_mujoco_sim_bringup launch file first."
        )

    def _ensure_state_fresh(self) -> None:
        with self._lock:
            last_state_time = self._last_state_time
        if (
            last_state_time is None
            or time.monotonic() - last_state_time > self.config.state_timeout
        ):
            raise TimeoutError(
                f"FER joint states are stale on {self.config.joint_state_topic!r}."
            )


class HardcodedFerPickAgent:
    """Deterministic pick trajectory for the correct FER simulation robot."""

    def __init__(self, steps_per_waypoint: int = 80):
        if steps_per_waypoint <= 0:
            raise ValueError("steps_per_waypoint must be positive")
        self._trajectory = self._make_trajectory(steps_per_waypoint)
        self._idx = 0

    @property
    def done(self) -> bool:
        return self._idx >= len(self._trajectory) - 1

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        del obs
        action = self._trajectory[min(self._idx, len(self._trajectory) - 1)]
        self._idx += 1
        return action.copy()

    def _make_trajectory(self, steps_per_waypoint: int) -> np.ndarray:
        waypoints = [
            FER_HOME,
            FER_PREGRASP,
            FER_GRASP,
            FER_GRASP.copy(),
            FER_LIFT,
            FER_PLACE,
        ]
        waypoints[3][-1] = 0.0
        pieces = []
        for start, stop in zip(waypoints[:-1], waypoints[1:]):
            for alpha in np.linspace(0.0, 1.0, steps_per_waypoint, endpoint=False):
                smooth = 0.5 - 0.5 * math.cos(math.pi * alpha)
                pieces.append((1.0 - smooth) * start + smooth * stop)
        pieces.append(waypoints[-1])
        return np.asarray(pieces, dtype=np.float32)


def validate_joint_names(names: Iterable[str]) -> None:
    """Fail early if a FER simulator exposes an unexpected joint schema."""

    missing = sorted(set(FER_JOINTS) - set(names))
    if missing:
        raise ValueError(f"FER joint state is missing expected joints: {missing}")
