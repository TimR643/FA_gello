import pytest

h5py = pytest.importorskip("h5py")
import numpy as np

from gello.data_utils.h5_logger import H5RobotLogger


def test_h5_robot_logger_records_numeric_observations_and_actions(tmp_path):
    path = tmp_path / "recording.h5"

    with H5RobotLogger(path, metadata={"robot": "test"}) as logger:
        logger.log(
            {
                "joint_positions": np.array([1.0, 2.0]),
                "joint_velocities": np.array([0.1, 0.2]),
                "joint_torques": np.array([3.0, 4.0]),
                "status": "ignored",
            },
            action=np.array([1.5, 2.5]),
            timestamp=10.0,
        )
        logger.log(
            {
                "joint_positions": np.array([2.0, 3.0]),
                "joint_velocities": np.array([0.2, 0.3]),
                "joint_torques": np.array([4.0, 5.0]),
            },
            action=np.array([2.5, 3.5]),
            timestamp=10.1,
        )

    with h5py.File(path, "r") as handle:
        assert handle.attrs["format"] == "gello_robot_h5"
        assert handle.attrs["robot"] == "test"
        assert handle.attrs["num_frames"] == 2
        np.testing.assert_allclose(handle["timestamp_unix"][:], [10.0, 10.1])
        np.testing.assert_allclose(handle["dt"][:], [0.0, 0.1])
        np.testing.assert_allclose(
            handle["observations/joint_positions"][:], [[1.0, 2.0], [2.0, 3.0]]
        )
        np.testing.assert_allclose(
            handle["observations/joint_torques"][:], [[3.0, 4.0], [4.0, 5.0]]
        )
        np.testing.assert_allclose(
            handle["actions/joint_command"][:], [[1.5, 2.5], [2.5, 3.5]]
        )
        assert "observations/status" not in handle
