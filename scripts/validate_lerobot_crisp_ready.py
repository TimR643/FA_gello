#!/usr/bin/env python3
"""Validate a local LeRobot dataset before trying a CRISP deployment.

The script intentionally checks the parts that matter when using existing GELLO
recordings to decide whether a failure is caused by the data or by the local
rollout/controller bridge:

* LeRobot metadata exists and contains the expected state/action/image keys.
* State and action dimensions match the Panda/GELLO 8-DoF convention.
* Dataset fps matches the expected policy/control fps.
* Optional parquet inspection estimates real timestamp fps and gripper
  state/action consistency.

It does not require CRISP to be installed. If ``pyarrow`` is available it will
also inspect frame parquet files; otherwise it still performs metadata checks.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Any, Iterable, Mapping, Sequence

DEFAULT_EXPECTED_CAMERAS = ("wrist",)
DEFAULT_EXPECTED_STATE_DIM = 8
DEFAULT_EXPECTED_ACTION_DIM = 8
DEFAULT_EXPECTED_FPS = 10.0


@dataclass
class CheckReport:
    """Collect validation results for terminal and JSON output."""

    passes: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)
    details: dict[str, Any] = field(default_factory=dict)

    def pass_(self, message: str) -> None:
        self.passes.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    def fail(self, message: str) -> None:
        self.failures.append(message)

    @property
    def exit_code(self) -> int:
        return 2 if self.failures else 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "passes": self.passes,
            "warnings": self.warnings,
            "failures": self.failures,
            "details": self.details,
        }


def load_info(dataset_root: Path) -> Mapping[str, Any]:
    info_path = dataset_root / "meta" / "info.json"
    if not info_path.exists():
        raise FileNotFoundError(f"Missing LeRobot metadata: {info_path}")
    return json.loads(info_path.read_text())


def normalize_shape(raw_shape: Any) -> tuple[int, ...] | None:
    if raw_shape is None:
        return None
    if isinstance(raw_shape, int):
        return (raw_shape,)
    if isinstance(raw_shape, Sequence) and not isinstance(raw_shape, str):
        return tuple(int(x) for x in raw_shape)
    return None


def get_features(info: Mapping[str, Any]) -> Mapping[str, Any]:
    features = info.get("features")
    if isinstance(features, Mapping):
        return features
    return {}


def get_feature_shape(features: Mapping[str, Any], key: str) -> tuple[int, ...] | None:
    feature = features.get(key)
    if isinstance(feature, Mapping):
        return normalize_shape(feature.get("shape"))
    return None


def list_episode_parquets(dataset_root: Path, max_files: int) -> list[Path]:
    candidates = sorted((dataset_root / "data").glob("**/*.parquet"))
    return candidates[:max_files]


def flatten_numeric(value: Any) -> list[float]:
    if value is None:
        return []
    if isinstance(value, (int, float)):
        return [float(value)]
    if hasattr(value, "tolist"):
        return flatten_numeric(value.tolist())
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        out: list[float] = []
        for item in value:
            out.extend(flatten_numeric(item))
        return out
    return []


def pearson(xs: Sequence[float], ys: Sequence[float]) -> float | None:
    if len(xs) != len(ys) or len(xs) < 3:
        return None
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    dx = [x - mean_x for x in xs]
    dy = [y - mean_y for y in ys]
    var_x = sum(x * x for x in dx)
    var_y = sum(y * y for y in dy)
    if var_x <= 0 or var_y <= 0:
        return None
    return sum(x * y for x, y in zip(dx, dy)) / math.sqrt(var_x * var_y)


def inspect_parquets(
    dataset_root: Path,
    *,
    max_files: int,
    expected_fps: float,
    fps_tolerance: float,
    report: CheckReport,
) -> None:
    if importlib.util.find_spec("pyarrow") is None:
        report.warn("pyarrow is not installed; skipped frame parquet inspection")
        return
    if importlib.util.find_spec("pyarrow.parquet") is None:
        report.warn("pyarrow.parquet is not available; skipped frame parquet inspection")
        return

    pq = importlib.import_module("pyarrow.parquet")
    parquet_files = list_episode_parquets(dataset_root, max_files=max_files)
    report.details["inspected_parquet_files"] = [str(path) for path in parquet_files]
    if not parquet_files:
        report.warn("No parquet files found below data/; only metadata was checked")
        return

    state_gripper: list[float] = []
    action_gripper: list[float] = []
    all_timestamps: list[float] = []
    sampled_rows = 0

    for parquet_path in parquet_files:
        table = pq.read_table(parquet_path)
        columns = set(table.column_names)
        if "observation.state" not in columns:
            report.fail(f"{parquet_path} is missing column observation.state")
            continue
        if "action" not in columns:
            report.fail(f"{parquet_path} is missing column action")
            continue

        states = table["observation.state"].to_pylist()
        actions = table["action"].to_pylist()
        timestamps = table["timestamp"].to_pylist() if "timestamp" in columns else []

        for state, action in zip(states, actions):
            state_values = flatten_numeric(state)
            action_values = flatten_numeric(action)
            if len(state_values) >= DEFAULT_EXPECTED_STATE_DIM:
                state_gripper.append(state_values[DEFAULT_EXPECTED_STATE_DIM - 1])
            if len(action_values) >= DEFAULT_EXPECTED_ACTION_DIM:
                action_gripper.append(action_values[DEFAULT_EXPECTED_ACTION_DIM - 1])
            sampled_rows += 1

        for timestamp in timestamps:
            values = flatten_numeric(timestamp)
            if values:
                all_timestamps.append(values[0])

    report.details["sampled_rows"] = sampled_rows
    if state_gripper:
        report.details["state_gripper_min"] = min(state_gripper)
        report.details["state_gripper_max"] = max(state_gripper)
    if action_gripper:
        report.details["action_gripper_min"] = min(action_gripper)
        report.details["action_gripper_max"] = max(action_gripper)

    if len(state_gripper) == len(action_gripper) and state_gripper:
        corr = pearson(state_gripper, action_gripper)
        report.details["state_action_gripper_correlation"] = corr
        if corr is not None and corr < -0.5:
            report.fail(
                "Gripper state/action correlation is strongly negative. This is a "
                "typical sign that one side uses open=1 while the other uses open=0."
            )
        elif corr is not None:
            report.pass_(f"Gripper state/action correlation looks plausible: {corr:.3f}")
        else:
            report.warn("Could not infer gripper correlation, likely too little variation")

    if len(all_timestamps) >= 3:
        all_timestamps = sorted(all_timestamps)
        deltas = [b - a for a, b in zip(all_timestamps, all_timestamps[1:]) if b > a]
        if deltas:
            inferred_fps = 1.0 / median(deltas)
            report.details["inferred_timestamp_fps"] = inferred_fps
            if abs(inferred_fps - expected_fps) > fps_tolerance:
                report.warn(
                    f"Timestamp-derived fps is {inferred_fps:.2f}, but expected "
                    f"{expected_fps:.2f}. Check recording/control-rate mismatch."
                )
            else:
                report.pass_(f"Timestamp-derived fps matches expectation: {inferred_fps:.2f}")
    else:
        report.warn("No timestamp column found in sampled parquet files; real fps not inferred")


def validate_metadata(
    info: Mapping[str, Any],
    *,
    expected_cameras: Iterable[str],
    expected_state_dim: int,
    expected_action_dim: int,
    expected_fps: float,
    fps_tolerance: float,
    report: CheckReport,
) -> None:
    features = get_features(info)
    report.details["metadata_fps"] = info.get("fps")
    report.details["feature_keys"] = sorted(features.keys())

    if not features:
        report.fail("meta/info.json does not contain a features mapping")
        return

    state_shape = get_feature_shape(features, "observation.state")
    action_shape = get_feature_shape(features, "action")
    report.details["state_shape"] = state_shape
    report.details["action_shape"] = action_shape

    if state_shape != (expected_state_dim,):
        report.fail(
            f"observation.state shape is {state_shape}, expected {(expected_state_dim,)}"
        )
    else:
        report.pass_("observation.state has expected shape")

    if action_shape != (expected_action_dim,):
        report.fail(f"action shape is {action_shape}, expected {(expected_action_dim,)}")
    else:
        report.pass_("action has expected shape")

    for camera in expected_cameras:
        key = f"observation.images.{camera}"
        image_shape = get_feature_shape(features, key)
        if image_shape is None:
            report.fail(f"Missing expected image feature: {key}")
        elif len(image_shape) != 3 or image_shape[-1] != 3:
            report.fail(f"{key} shape is {image_shape}, expected HxWx3 RGB")
        else:
            report.pass_(f"{key} exists with RGB image shape {image_shape}")

    metadata_fps = info.get("fps")
    if metadata_fps is None:
        report.warn("meta/info.json has no fps field")
    else:
        fps = float(metadata_fps)
        if abs(fps - expected_fps) > fps_tolerance:
            report.warn(
                f"metadata fps is {fps:.2f}, but expected {expected_fps:.2f}. "
                "CRISP deployment should use the same effective policy rate."
            )
        else:
            report.pass_(f"metadata fps matches expectation: {fps:.2f}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate a LeRobot dataset before using it for CRISP deployment."
    )
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument(
        "--expected-camera",
        action="append",
        default=None,
        help="Expected camera name without observation.images prefix. Can be passed multiple times.",
    )
    parser.add_argument("--expected-state-dim", type=int, default=DEFAULT_EXPECTED_STATE_DIM)
    parser.add_argument("--expected-action-dim", type=int, default=DEFAULT_EXPECTED_ACTION_DIM)
    parser.add_argument("--expected-fps", type=float, default=DEFAULT_EXPECTED_FPS)
    parser.add_argument("--fps-tolerance", type=float, default=1.0)
    parser.add_argument("--max-parquet-files", type=int, default=3)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON report")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    dataset_root = args.dataset_root.expanduser().resolve()
    expected_cameras = tuple(args.expected_camera or DEFAULT_EXPECTED_CAMERAS)

    report = CheckReport()
    report.details["dataset_root"] = str(dataset_root)
    report.details["expected_cameras"] = expected_cameras

    info = load_info(dataset_root)
    validate_metadata(
        info,
        expected_cameras=expected_cameras,
        expected_state_dim=args.expected_state_dim,
        expected_action_dim=args.expected_action_dim,
        expected_fps=args.expected_fps,
        fps_tolerance=args.fps_tolerance,
        report=report,
    )
    inspect_parquets(
        dataset_root,
        max_files=args.max_parquet_files,
        expected_fps=args.expected_fps,
        fps_tolerance=args.fps_tolerance,
        report=report,
    )

    if args.json:
        print(json.dumps(report.to_dict(), indent=2, sort_keys=True))
    else:
        print("\nCRISP readiness report")
        print("=" * 24)
        for message in report.passes:
            print(f"PASS: {message}")
        for message in report.warnings:
            print(f"WARN: {message}")
        for message in report.failures:
            print(f"FAIL: {message}")
        print("\nDetails:")
        print(json.dumps(report.details, indent=2, sort_keys=True, default=str))

    return report.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
