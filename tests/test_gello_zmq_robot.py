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


def test_observation_features_use_lerobot_image_keys_for_bare_camera_names():
    robot = GelloZMQ(
        GelloZMQConfig(
            camera_names="wrist",
            policy_camera_names="wrist,base",
        )
    )

    features = robot.observation_features

    assert "observation.images.wrist" in features
    assert "observation.images.base" in features
    assert "wrist" not in features
    assert "base" not in features


def test_get_observation_emits_prefixed_policy_keys_and_dummy_missing_camera():
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

    np.testing.assert_array_equal(obs["observation.images.wrist"], wrist_image)
    np.testing.assert_array_equal(
        obs["observation.images.base"], np.zeros((2, 3, 3), dtype=np.uint8)
    )
    assert "wrist" not in obs
    assert "base" not in obs


def test_full_policy_image_keys_are_not_double_prefixed():
    robot = GelloZMQ(
        GelloZMQConfig(policy_camera_names="observation.images.wrist")
    )

    features = robot.observation_features

    assert "observation.images.wrist" in features
    assert "observation.images.observation.images.wrist" not in features


class _FakeRobot:
    def get_observations(self):
        return {"joint_positions": np.zeros(8, dtype=np.float32)}

    def get_joint_state(self):
        return np.zeros(8, dtype=np.float32)


class _FakeCamera:
    def __init__(self, image):
        self.image = image

    def read(self, shape):
        return self.image, None
