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
    action = np.array([0.01, 0.01, 0.02, -0.01, 0.0, 0.01, 0.0, 0.0], dtype=np.float32)

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


def test_gello_zmq_inference_log_writes_positions_velocities_and_torques(tmp_path):
    import csv

    from lerobot_robot_gello.config_gello_zmq import GelloZMQConfig
    from lerobot_robot_gello.gello_zmq import GelloZMQ

    positions = np.arange(8, dtype=np.float32)
    velocities = np.arange(8, dtype=np.float32) / 10
    torques = np.arange(8, dtype=np.float32) / 100

    class RobotStub:
        def get_observations(self):
            return {
                "joint_positions": positions,
                "joint_velocities": velocities,
                "joint_torques": torques,
            }

    robot = GelloZMQ(
        GelloZMQConfig(
            camera_names="",
            policy_camera_names="",
            joint_inference_log_dir=str(tmp_path),
        )
    )
    robot.robot = RobotStub()
    robot._is_connected = True
    robot._open_joint_inference_log()

    robot.get_observation()
    log_path = robot._joint_log_path
    robot._close_joint_inference_log()

    assert log_path is not None
    with log_path.open(newline="", encoding="utf-8") as log_file:
        rows = list(csv.DictReader(log_file))
    assert len(rows) == 1
    assert float(rows[0]["joint_3_position_rad"]) == pytest.approx(3.0)
    assert float(rows[0]["joint_3_velocity_rad_s"]) == pytest.approx(0.3)
    assert float(rows[0]["joint_3_torque_nm"]) == pytest.approx(0.03)
