from types import SimpleNamespace

import numpy as np

from gello.robots.panda import PandaRobot


class FakeGripper:
    def get_state(self):
        return SimpleNamespace(width=0.045)


def make_robot(state, **getters):
    panda = PandaRobot.__new__(PandaRobot)
    panda.robot = SimpleNamespace(get_robot_state=lambda: state, **getters)
    panda.gripper = FakeGripper()
    return panda


def test_observations_support_alternate_polymetis_torque_field():
    state = SimpleNamespace(
        joint_positions=np.arange(7),
        joint_velocities=np.arange(7) + 10,
        joint_torques=np.arange(7) + 20,
    )

    observations = make_robot(state).get_observations()

    np.testing.assert_array_equal(observations["joint_positions"][:7], np.arange(7))
    np.testing.assert_array_equal(
        observations["joint_velocities"][:7], np.arange(7) + 10
    )
    np.testing.assert_array_equal(
        observations["joint_torques"][:7], np.arange(7) + 20
    )
    assert np.isnan(observations["joint_torques"][-1])


def test_observations_fall_back_to_public_polymetis_getters():
    observations = make_robot(
        SimpleNamespace(),
        get_joint_positions=lambda: np.arange(7),
        get_joint_velocities=lambda: np.arange(7) + 10,
        get_joint_torques=lambda: np.arange(7) + 20,
    ).get_observations()

    np.testing.assert_array_equal(observations["joint_velocities"][:7], np.arange(7) + 10)
    np.testing.assert_array_equal(observations["joint_torques"][:7], np.arange(7) + 20)
