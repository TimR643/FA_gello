import time
from typing import Dict

import numpy as np

from gello.robots.robot import Robot

MAX_OPEN = 0.09
GRIPPER_CMD_EPS = 0.01
GRIPPER_SPEED = 255
GRIPPER_FORCE = 255
GRIPPER_MAX_STEP = 0.015
GRIPPER_INIT_SPEED = 180
GRIPPER_INIT_FORCE = 120
GRIPPER_INIT_SETTLE_S = 0.15
GRIPPER_INIT_CLOSE_RATIO = 0.35
GRIPPER_FULL_CLOSE_THRESHOLD = 0.95
GRIPPER_FULL_CLOSE_RELEASE = 0.85
GRIPPER_FORCE_OPEN_THRESHOLD = 0.10


class PandaRobot(Robot):
    """A class representing a UR robot."""

    def __init__(
        self,
        robot_ip: str = "100.97.47.74",
        move_home: bool = True,
        run_gripper_startup_cycle: bool = False,
    ):
        from polymetis import GripperInterface, RobotInterface

        self.robot = RobotInterface(
            ip_address=robot_ip,
        )
        self.gripper = GripperInterface(
            ip_address="localhost",
        )

        if move_home:
            self.robot.go_home()
        self.robot.start_joint_impedance()
        if run_gripper_startup_cycle:
            # Keep the previous behavior where the gripper visibly opens/closes
            # once during startup, which also helps wake up the gripper state.
            self.gripper.goto(
                width=MAX_OPEN, speed=GRIPPER_INIT_SPEED, force=GRIPPER_INIT_FORCE
            )
            time.sleep(GRIPPER_INIT_SETTLE_S)
            self.gripper.goto(
                width=MAX_OPEN * GRIPPER_INIT_CLOSE_RATIO,
                speed=GRIPPER_INIT_SPEED,
                force=GRIPPER_INIT_FORCE,
            )
            time.sleep(GRIPPER_INIT_SETTLE_S)
            self.gripper.goto(
                width=MAX_OPEN, speed=GRIPPER_INIT_SPEED, force=GRIPPER_INIT_FORCE
            )
        else:
            self.gripper.goto(width=MAX_OPEN, speed=GRIPPER_SPEED, force=GRIPPER_FORCE)

        self._last_gripper_width = float(np.clip(self.gripper.get_state().width, 0.0, MAX_OPEN))
        self._gripper_synced = False
        self._leader_gripper_ref = 0.0
        self._follower_gripper_ref = self._last_gripper_width / MAX_OPEN
        self._full_close_latched = False
        self._gripper_toggle_closed = False
        time.sleep(1)

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        return 8

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.robot.get_joint_positions()
        gripper_pos = self.gripper.get_state()
        pos = np.append(robot_joints, gripper_pos.width / MAX_OPEN)
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        import torch

        self.robot.update_desired_joint_positions(torch.tensor(joint_state[:-1]))

        gripper_cmd = float(np.clip(joint_state[-1], 0.0, 1.0))

        # Sync first command to current follower gripper state to avoid
        # startup transients that can cause a sudden close jerk.
        if not self._gripper_synced:
            self._last_gripper_width = float(np.clip(self.gripper.get_state().width, 0.0, MAX_OPEN))
            self._leader_gripper_ref = gripper_cmd
            self._follower_gripper_ref = self._last_gripper_width / MAX_OPEN
            self._full_close_latched = gripper_cmd >= GRIPPER_FULL_CLOSE_THRESHOLD
            self._gripper_synced = True
            return

        # Gripper toggle gesture:
        # fully close once -> close follower gripper
        # fully close again (after releasing) -> open follower gripper
        if gripper_cmd <= GRIPPER_FORCE_OPEN_THRESHOLD:
            self._gripper_toggle_closed = False
            self._full_close_latched = False

        if (
            gripper_cmd >= GRIPPER_FULL_CLOSE_THRESHOLD
            and not self._full_close_latched
        ):
            self._full_close_latched = True
            self._gripper_toggle_closed = not self._gripper_toggle_closed
        elif gripper_cmd <= GRIPPER_FULL_CLOSE_RELEASE:
            self._full_close_latched = False

        mapped_gripper_cmd = 1.0 if self._gripper_toggle_closed else 0.0
        target_width = MAX_OPEN * (1 - mapped_gripper_cmd)

        # Rate limit gripper motions to avoid sudden rucks on startup.
        width_delta = target_width - self._last_gripper_width
        width_delta = float(np.clip(width_delta, -GRIPPER_MAX_STEP, GRIPPER_MAX_STEP))
        limited_target_width = float(
            np.clip(self._last_gripper_width + width_delta, 0.0, MAX_OPEN)
        )

        if abs(limited_target_width - self._last_gripper_width) > GRIPPER_CMD_EPS:
            self.gripper.goto(
                width=limited_target_width, speed=GRIPPER_SPEED, force=GRIPPER_FORCE
            )
            self._last_gripper_width = limited_target_width

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    robot = PandaRobot()
    current_joints = robot.get_joint_state()
    # move a small delta 0.1 rad
    move_joints = current_joints + 0.05
    # make last joint (gripper) closed
    move_joints[-1] = 0.5
    time.sleep(1)
    m = 0.09
    robot.gripper.goto(1 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.05 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.1 * m, speed=255, force=255)
    time.sleep(1)


if __name__ == "__main__":
    main()
