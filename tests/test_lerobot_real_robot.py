import pytest

np = pytest.importorskip("numpy")

from gello.lerobot.real_robot import (
    LeRobotObservationAdapter,
    SafeJointActionExecutor,
    SafetyConfig,
    diagnose_action_state_interpretation,
    validate_policy_batch_keys,
)


def test_safe_executor_clips_absolute_joint_targets():
    executor = SafeJointActionExecutor(
        SafetyConfig(max_joint_delta=0.1, max_gripper_delta=0.2)
    )
    state = np.zeros(8, dtype=np.float32)
    policy_action = np.array([1.0, -1.0, 0.05, 0.5, -0.5, 0.0, 0.2, 1.0])

    result = executor.make_safe_target(policy_action, state)

    np.testing.assert_allclose(
        result.clipped_delta,
        np.array([0.1, -0.1, 0.05, 0.1, -0.1, 0.0, 0.1, 0.2], dtype=np.float32),
    )
    np.testing.assert_allclose(result.target, result.clipped_delta)


def test_safe_executor_supports_delta_mode():
    executor = SafeJointActionExecutor(
        SafetyConfig(
            max_joint_delta=0.1,
            max_gripper_delta=0.2,
            action_mode="delta_joint_position",
        )
    )
    state = np.ones(8, dtype=np.float32)
    delta = np.full(8, 0.5, dtype=np.float32)

    result = executor.make_safe_target(delta, state)

    np.testing.assert_allclose(
        result.target,
        np.array([1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.2], dtype=np.float32),
    )


def test_observation_adapter_reports_missing_camera_key():
    adapter = LeRobotObservationAdapter(device="cpu", camera_keys=("wrist",))
    obs = {"joint_positions": np.zeros(8, dtype=np.float32)}

    with pytest.raises(KeyError, match="missing camera"):
        adapter.make_batch(obs)


def test_validate_policy_batch_keys_detects_schema_mismatch():
    adapter = LeRobotObservationAdapter(device="cpu", camera_keys=("wrist", "base"))

    class Meta:
        features = {
            "observation.state": {},
            "observation.images.wrist": {},
            "action": {},
        }

    with pytest.raises(KeyError, match="observation.images.base"):
        validate_policy_batch_keys(adapter, Meta())


def test_diagnostics_flag_likely_delta_action_when_absolute_delta_is_large():
    current = np.array([0.0, 0.0, 1.0, -2.3, 0.0, 2.3, -0.8, 0.1], dtype=np.float32)
    action = np.array(
        [0.01, 0.01, 0.02, -0.01, 0.0, 0.01, 0.0, 0.0], dtype=np.float32
    )

    diagnostics = diagnose_action_state_interpretation(action, current)

    assert diagnostics.likely_delta_action
    np.testing.assert_allclose(diagnostics.absolute_delta, action - current)
    np.testing.assert_allclose(diagnostics.delta_target, action + current)


def test_diagnostics_do_not_flag_absolute_targets_close_to_state():
    current = np.array([0.0, 0.0, 1.0, -2.3, 0.0, 2.3, -0.8, 0.1], dtype=np.float32)
    action = current + np.array(
        [0.01, 0.01, 0.02, -0.01, 0.0, 0.01, 0.0, 0.0], dtype=np.float32
    )

    diagnostics = diagnose_action_state_interpretation(action, current)

    assert not diagnostics.likely_delta_action


def test_gello_zmq_action_diagnostics_reports_delta_clipping(capsys):
    from lerobot_robot_gello.config_gello_zmq import GelloZMQConfig
    from lerobot_robot_gello.gello_zmq import GelloZMQ

    robot = GelloZMQ(GelloZMQConfig(log_action_diagnostics_every_n=1))
    current = np.zeros(8, dtype=np.float32)
    safe = robot.executor.make_safe_target(np.ones(8, dtype=np.float32), current)

    robot._log_action_diagnostics(current, safe)

    output = capsys.readouterr().out
    assert "delta-clipped" in output
    assert "joint_0.pos" in output


def test_gello_zmq_send_action_uses_live_state_after_external_reset():
    from lerobot_robot_gello.config_gello_zmq import GelloZMQConfig
    from lerobot_robot_gello.gello_zmq import GelloZMQ

    class FakeRobotClient:
        def __init__(self, live_state):
            self.live_state = live_state
            self.commands = []

        def get_joint_state(self):
            return self.live_state.copy()

        def command_joint_state(self, target):
            self.commands.append(target.copy())

    config = GelloZMQConfig(
        max_joint_delta=0.1,
        max_gripper_delta=0.2,
        log_action_diagnostics_every_n=0,
    )
    robot = GelloZMQ(config)
    reset_pose = np.array(
        [0.08, -0.13, -0.15, -2.42, -0.06, 2.24, -0.79, 1.0],
        dtype=np.float32,
    )
    client = FakeRobotClient(reset_pose)
    robot.robot = client
    robot._is_connected = True
    robot._last_state = np.array(
        [0.7, 0.6, 0.5, -1.5, 0.4, 1.2, 0.3, 0.0], dtype=np.float32
    )

    policy_target = reset_pose + 0.05
    robot.send_action(policy_target)

    np.testing.assert_allclose(client.commands[-1], policy_target)
    np.testing.assert_allclose(robot._last_state, reset_pose)


def test_start_position_target_can_keep_arm_and_open_gripper():
    from scripts.move_gello_start_position import _build_parser, _target_from_args

    current_arm = np.array(
        [0.4, -0.3, 0.2, -2.1, 0.1, 1.8, -0.5], dtype=np.float32
    )
    args = _build_parser().parse_args(
        ["--keep-arm", "--target-gripper", "1.0"]
    )

    target = _target_from_args(args, current_arm)

    np.testing.assert_allclose(target[:7], current_arm)
    assert target[7] == pytest.approx(1.0)
