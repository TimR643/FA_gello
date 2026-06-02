import pytest

np = pytest.importorskip("numpy")

from gello.lerobot.real_robot import (
    LeRobotObservationAdapter,
    SafeJointActionExecutor,
    SafetyConfig,
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


def test_safe_executor_can_hold_gripper_position():
    executor = SafeJointActionExecutor(
        SafetyConfig(
            max_joint_delta=0.1,
            max_gripper_delta=0.2,
            gripper_action_mode="hold",
        )
    )
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8], dtype=np.float32)
    policy_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1])

    result = executor.make_safe_target(policy_action, state)

    assert result.raw_delta[-1] == 0.0
    assert result.clipped_delta[-1] == 0.0
    assert result.target[-1] == state[-1]


def test_safety_config_rejects_unknown_gripper_action_mode():
    with pytest.raises(ValueError, match="gripper_action_mode"):
        SafetyConfig(gripper_action_mode="close")


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
