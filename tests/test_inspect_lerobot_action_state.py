import importlib.util
import sys
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "inspect_lerobot_action_state.py"
SPEC = importlib.util.spec_from_file_location("inspect_lerobot_action_state", MODULE_PATH)
inspect = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = inspect
SPEC.loader.exec_module(inspect)


def test_summarize_records_detects_absolute_actions_and_direction():
    records = inspect._records_from_rows(
        [
            {
                "episode_index": 0,
                "frame_index": 0,
                "task": "green left",
                "observation.state": [0.0, 0.0, 0.4],
                "action": [0.0, 0.0, 0.35],
            },
            {
                "episode_index": 0,
                "frame_index": 1,
                "task": "green left",
                "observation.state": [0.0, 0.0, 0.35],
                "action": [0.0, 0.0, 0.30],
            },
        ]
    )

    [summary] = inspect.summarize_records(records, direction_joint_index=2)

    assert summary.likely_absolute_action
    assert summary.direction_label == "negative"
    assert summary.mean_direction_command < 0


def test_summarize_records_flags_delta_like_actions():
    records = inspect._records_from_rows(
        [
            {
                "episode_index": 0,
                "frame_index": 0,
                "observation.state": [1.0, -2.0, 0.4],
                "action": [0.0, 0.0, -0.05],
            },
            {
                "episode_index": 0,
                "frame_index": 1,
                "observation.state": [1.0, -2.0, 0.35],
                "action": [0.0, 0.0, -0.05],
            },
        ]
    )

    [summary] = inspect.summarize_records(records, direction_joint_index=2)

    assert not summary.likely_absolute_action
    assert summary.mean_direction_command < 0
