import sys
import types

import pytest

np = pytest.importorskip("numpy")


def _install_lerobot_stubs():
    lerobot = types.ModuleType("lerobot")
    robots = types.ModuleType("lerobot.robots")
    robot_mod = types.ModuleType("lerobot.robots.robot")

    class RobotConfig:
        @classmethod
        def register_subclass(cls, _name):
            def decorator(subclass):
                return subclass

            return decorator

    class Robot:
        def __init__(self, config):
            self.config = config

    robots.RobotConfig = RobotConfig
    robot_mod.Robot = Robot
    sys.modules.setdefault("lerobot", lerobot)
    sys.modules.setdefault("lerobot.robots", robots)
    sys.modules.setdefault("lerobot.robots.robot", robot_mod)


_install_lerobot_stubs()

from lerobot_robot_gello.config_gello_zmq import GelloZMQConfig
from lerobot_robot_gello.gello_zmq import GelloZMQ


def test_observation_features_use_bare_camera_keys_for_lerobot_prefixing():
    robot = GelloZMQ(
        GelloZMQConfig(
            camera_names="wrist",
            policy_camera_names="wrist,base",
        )
    )

    features = robot.observation_features

    assert "wrist" in features
    assert "base" in features
    assert "observation.images.wrist" not in features
    assert "observation.images.base" not in features


def test_get_observation_emits_bare_camera_keys_and_dummy_missing_camera():
    robot = GelloZMQ(
        GelloZMQConfig(
            camera_names="wrist",
            policy_camera_names="wrist,base",
            image_height=2,
            image_width=3,
        )
    )
    robot._is_connected = True
    robot.robot = _FakeRobot()
    wrist_image = np.full((2, 3, 3), 7, dtype=np.uint8)
    robot.cameras = {"wrist": _FakeCamera(wrist_image)}

    obs = robot.get_observation()

    np.testing.assert_array_equal(obs["wrist"], wrist_image)
    np.testing.assert_array_equal(obs["base"], np.zeros((2, 3, 3), dtype=np.uint8))
    assert "observation.images.wrist" not in obs
    assert "observation.images.base" not in obs


def test_full_policy_image_keys_are_stripped_to_robot_camera_keys():
    robot = GelloZMQ(
        GelloZMQConfig(policy_camera_names="observation.images.wrist")
    )

    features = robot.observation_features

    assert "wrist" in features
    assert "observation.images.wrist" not in features
    assert "observation.images.observation.images.wrist" not in features


def test_send_action_forwards_each_policy_action_without_alpha_smoothing():
    robot = GelloZMQ(
        GelloZMQConfig(
            max_joint_delta=0.5,
            max_gripper_delta=0.5,
        )
    )
    fake_robot = _FakeRobot()
    robot._is_connected = True
    robot.robot = fake_robot
    robot._last_state = np.zeros(8, dtype=np.float32)

    first_action = np.full(8, 0.4, dtype=np.float32)
    second_action = np.full(8, -0.4, dtype=np.float32)

    first_returned = robot.send_action(first_action)
    np.testing.assert_array_equal(fake_robot.commanded, first_action)
    np.testing.assert_array_equal(
        np.array(list(first_returned.values()), dtype=np.float32), first_action
    )

    second_returned = robot.send_action(second_action)

    np.testing.assert_array_equal(fake_robot.commanded, second_action)
    np.testing.assert_array_equal(
        np.array(list(second_returned.values()), dtype=np.float32), second_action
    )


class _FakeRobot:
    def __init__(self):
        self.commanded = None

    def get_observations(self):
        return {"joint_positions": np.zeros(8, dtype=np.float32)}

    def get_joint_state(self):
        return np.zeros(8, dtype=np.float32)

    def command_joint_state(self, target):
        self.commanded = target


class _FakeCamera:
    def __init__(self, image):
        self.image = image

    def read(self, shape):
        return self.image, None
