import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.validate_lerobot_crisp_ready import (
    CheckReport,
    flatten_numeric,
    load_info,
    pearson,
    validate_metadata,
)


def write_info(root: Path, info: dict) -> None:
    meta = root / "meta"
    meta.mkdir(parents=True)
    (meta / "info.json").write_text(json.dumps(info))


def test_metadata_validation_accepts_expected_panda_schema(tmp_path):
    write_info(
        tmp_path,
        {
            "fps": 10,
            "features": {
                "observation.state": {"shape": [8], "dtype": "float32"},
                "action": {"shape": [8], "dtype": "float32"},
                "observation.images.wrist": {"shape": [480, 640, 3], "dtype": "video"},
            },
        },
    )
    report = CheckReport()

    validate_metadata(
        load_info(tmp_path),
        expected_cameras=("wrist",),
        expected_state_dim=8,
        expected_action_dim=8,
        expected_fps=10,
        fps_tolerance=1,
        report=report,
    )

    assert not report.failures
    assert any("observation.state" in item for item in report.passes)
    assert any("observation.images.wrist" in item for item in report.passes)


def test_metadata_validation_detects_missing_camera(tmp_path):
    write_info(
        tmp_path,
        {
            "fps": 10,
            "features": {
                "observation.state": {"shape": [8]},
                "action": {"shape": [8]},
            },
        },
    )
    report = CheckReport()

    validate_metadata(
        load_info(tmp_path),
        expected_cameras=("wrist",),
        expected_state_dim=8,
        expected_action_dim=8,
        expected_fps=10,
        fps_tolerance=1,
        report=report,
    )

    assert any("observation.images.wrist" in item for item in report.failures)


def test_flatten_numeric_handles_nested_sequences():
    assert flatten_numeric([[1, 2], [3.5]]) == [1.0, 2.0, 3.5]


def test_pearson_detects_inverted_gripper_signal():
    corr = pearson([0, 0, 1, 1], [1, 1, 0, 0])

    assert corr is not None
    assert corr < -0.9
