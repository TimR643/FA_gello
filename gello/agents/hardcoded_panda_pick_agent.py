"""Deterministic Panda pick trajectory for simulation smoke tests."""

from __future__ import annotations

import math
from typing import Any, Dict

import numpy as np

from gello.agents.agent import Agent

PANDA_PICK_HOME = np.array(
    [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0], dtype=np.float32
)
PANDA_PICK_PREGRASP = np.array(
    [0.0, -0.60, 0.0, -2.20, 0.0, 1.65, 0.78, 0.0], dtype=np.float32
)
PANDA_PICK_GRASP = np.array(
    [0.0, -0.78, 0.0, -2.38, 0.0, 1.60, 0.78, 0.0], dtype=np.float32
)
PANDA_PICK_LIFT = np.array(
    [0.0, -0.40, 0.0, -1.95, 0.0, 1.45, 0.78, 1.0], dtype=np.float32
)
PANDA_PICK_PLACE = np.array(
    [0.35, -0.48, -0.20, -1.95, 0.10, 1.50, 0.55, 1.0], dtype=np.float32
)


class HardcodedPandaPickAgent(Agent):
    """Replace teleoperation with a fixed Panda joint-space pick task.

    The output is the same 8-value command schema used by the real Polymetis
    Panda path in this repository: 7 arm joints plus one normalized gripper
    value.  This follows ``PandaRobot.command_joint_state`` where high
    gripper commands close and low commands open the hand.  This keeps the movement interface identical between hardware and
    simulation; only the Polymetis robot server behind the same API changes.
    """

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
        close_gripper = PANDA_PICK_GRASP.copy()
        close_gripper[-1] = 1.0
        waypoints = [
            PANDA_PICK_HOME,
            PANDA_PICK_PREGRASP,
            PANDA_PICK_GRASP,
            close_gripper,
            PANDA_PICK_LIFT,
            PANDA_PICK_PLACE,
        ]
        pieces = []
        for start, stop in zip(waypoints[:-1], waypoints[1:]):
            for alpha in np.linspace(0.0, 1.0, steps_per_waypoint, endpoint=False):
                smooth = 0.5 - 0.5 * math.cos(math.pi * alpha)
                pieces.append((1.0 - smooth) * start + smooth * stop)
        pieces.append(waypoints[-1])
        return np.asarray(pieces, dtype=np.float32)
