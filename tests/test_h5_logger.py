import h5py
import numpy as np

from lerobot_robot_gello.h5_logger import H5RobotLogger


def test_h5_logger_writes_observation_and_teleoperation_action(tmp_path):
    path = tmp_path / "episode.h5"
    logger = H5RobotLogger(path, num_dofs=3)
    logger.log_observation(
        10.0,
        np.array([1, 2, 3]),
        np.array([0.1, 0.2, 0.3]),
        np.array([4, 5, 6]),
    )
    logger.log_action(10.1, np.array([7, 8, 9]), np.array([7, 8, 8.5]))
    logger.close()

    with h5py.File(path) as log:
        assert log.attrs["format"] == "gello_lerobot_h5"
        np.testing.assert_array_equal(log["state/q"][:], [[1, 2, 3]])
        np.testing.assert_array_equal(log["state/dq"][:], [[0.1, 0.2, 0.3]])
        np.testing.assert_array_equal(log["state/tau"][:], [[4, 5, 6]])
        np.testing.assert_array_equal(log["action/raw"][:], [[7, 8, 9]])
        np.testing.assert_array_equal(log["action/sent"][:], [[7, 8, 8.5]])
        np.testing.assert_array_equal(log["action/time"][:], [10.1])


def test_h5_logger_keeps_missing_action_as_nan(tmp_path):
    path = tmp_path / "interrupted.h5"
    logger = H5RobotLogger(path, num_dofs=2)
    logger.log_observation(1.0, np.ones(2), np.ones(2), np.ones(2))
    logger.close()

    with h5py.File(path) as log:
        assert np.isnan(log["action/raw"][:]).all()
        assert np.isnan(log["action/time"][:]).all()
