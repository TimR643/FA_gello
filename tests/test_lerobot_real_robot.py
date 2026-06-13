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
