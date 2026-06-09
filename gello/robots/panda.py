import threading
import time
from typing import Dict, Optional

import numpy as np
import polymetis
import torch

from gello.robots.robot import Robot

MAX_OPEN = 0.09


class PandaRobot(Robot):
    """Polymetis-backed Franka Panda robot wrapper."""

    def __init__(
        self,
        robot_ip: str = "100.97.47.74",
        polymetis_port: int = 50051,
        gripper_ip: Optional[str] = None,
        use_gripper: bool = True,
        initialize_robot: bool = True,
        manual_gripper_override: bool = True,
    ):
        self.robot = polymetis.RobotInterface(
            ip_address=robot_ip,
            port=polymetis_port,
        )
        self.gripper = None
        self.gripper_closed = False
        self.last_target_width = MAX_OPEN
        self.manual_release = False

        if initialize_robot:
            self.robot.go_home()
            self.robot.start_joint_impedance()
            time.sleep(1)

        if use_gripper:
            target_gripper_ip = robot_ip if gripper_ip is None else gripper_ip
            try:
                self.gripper = polymetis.GripperInterface(ip_address=target_gripper_ip)
                self.gripper.goto(width=MAX_OPEN, speed=1.0, force=1.0)
            except Exception as exc:
                print(
                    "Warning: could not connect to Polymetis gripper server; "
                    f"continuing with simulated scalar gripper state only: {exc}"
                )
                self.gripper = None

        if manual_gripper_override and self.gripper is not None:
            threading.Thread(target=self._listen_for_manual_open, daemon=True).start()
            print(">>> MANUELLER OVERRIDE AKTIV: Drücke ENTER im Terminal zum Öffnen! <<<")

    def _listen_for_manual_open(self):
        while True:
            input()
            self.manual_release = True
            print("!!! MANUELLER BEFEHL: GREIFER ÖFFNEN !!!")

    def num_dofs(self) -> int:
        """Get the number of joints of the robot."""
        return 8

    def get_joint_state(self) -> np.ndarray:
        """Get current Panda joints plus normalized gripper command state."""
        robot_joints = self.robot.get_joint_positions()
        if self.gripper is None:
            gripper_width = self.last_target_width
        else:
            try:
                gripper_width = self.gripper.get_state().width
            except Exception as exc:
                print(f"Warning: could not read gripper state, reusing last target: {exc}")
                gripper_width = self.last_target_width
        return np.append(robot_joints, gripper_width / MAX_OPEN)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the Panda arm and normalized gripper state."""
        try:
            self.robot.update_desired_joint_positions(
                torch.tensor(joint_state[:-1], dtype=torch.float32)
            )
        except Exception as exc:
            print("Could not update joint positions:", exc)

        close_threshold = 0.25
        open_threshold = 0.15

        gripper_closed = joint_state[-1] > close_threshold
        gripper_open = joint_state[-1] < open_threshold

        if gripper_closed:
            self.last_target_width = 0.0
        elif gripper_open:
            self.last_target_width = MAX_OPEN

        if self.gripper is None:
            self.gripper_closed = gripper_closed
            return

        if gripper_closed and not self.gripper_closed:
            self.gripper_closed = True
            try:
                self.gripper.grasp(speed=0.1, force=1.0)
            except Exception as exc:
                print("Could not close gripper:", exc)

        elif gripper_open and self.gripper_closed:
            self.gripper_closed = False
            try:
                self.gripper.goto(width=MAX_OPEN, speed=1.0, force=1.0)
            except Exception as exc:
                print("Could not open gripper:", exc)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": np.zeros_like(joints),
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    robot = PandaRobot()
    current_joints = robot.get_joint_state()
    move_joints = current_joints + 0.05
    move_joints[-1] = 0.5
    robot.command_joint_state(move_joints)
    time.sleep(1)


if __name__ == "__main__":
    main()
