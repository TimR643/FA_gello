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

It does not require CRISP to be installed. Frame-level inspection uses
``pyarrow`` when available, or ``pandas.read_parquet`` as a fallback. If neither
can read parquet files, the metadata checks still run and the report explains how
to enable the deeper checks.
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
from typing import Any, Callable, Iterable, Mapping, Sequence

DEFAULT_EXPECTED_CAMERAS = ("wrist",)
DEFAULT_EXPECTED_STATE_DIM = 8
DEFAULT_EXPECTED_ACTION_DIM = 8
DEFAULT_EXPECTED_FPS = 10.0
GRIPPER_LOW_VARIATION_EPS = 0.02
GRIPPER_RANGE_GAP_FAIL_THRESHOLD = 0.25
GRIPPER_MEAN_GAP_FAIL_THRESHOLD = 0.25


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


@dataclass(frozen=True)
class ParquetRows:
    """Small reader-independent view of a sampled parquet file."""

    columns: set[str]
    states: list[Any]
    actions: list[Any]
    timestamps: list[Any]


ParquetReader = Callable[[Path], ParquetRows]


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


def has_module(module_name: str) -> bool:
    parent, _, _ = module_name.partition(".")
    if importlib.util.find_spec(parent) is None:
        return False
    return importlib.util.find_spec(module_name) is not None


def read_parquet_with_pyarrow(parquet_path: Path) -> ParquetRows:
    pq = importlib.import_module("pyarrow.parquet")
    table = pq.read_table(parquet_path)
    columns = set(table.column_names)
    return ParquetRows(
        columns=columns,
        states=table["observation.state"].to_pylist()
        if "observation.state" in columns
        else [],
        actions=table["action"].to_pylist() if "action" in columns else [],
        timestamps=table["timestamp"].to_pylist() if "timestamp" in columns else [],
    )


def read_parquet_with_pandas(parquet_path: Path) -> ParquetRows:
    pandas = importlib.import_module("pandas")
    dataframe = pandas.read_parquet(parquet_path)
    columns = set(dataframe.columns)
    return ParquetRows(
        columns=columns,
        states=dataframe["observation.state"].tolist()
        if "observation.state" in columns
        else [],
        actions=dataframe["action"].tolist() if "action" in columns else [],
        timestamps=dataframe["timestamp"].tolist() if "timestamp" in columns else [],
    )


def select_parquet_reader(report: CheckReport) -> tuple[str | None, ParquetReader | None]:
    if has_module("pyarrow.parquet"):
        return "pyarrow", read_parquet_with_pyarrow
    if importlib.util.find_spec("pandas") is not None:
        report.warn(
            "pyarrow is not installed; trying pandas.read_parquet fallback. If this "
            "fails, install pyarrow in the same environment."
        )
        return "pandas", read_parquet_with_pandas
    report.warn(
        "No parquet reader is installed; frame-level checks were skipped. Install "
        "pyarrow in the same environment, for example: python3 -m pip install pyarrow"
    )
    return None, None


def update_dimension_ranges(ranges: list[list[float]], values: Sequence[float]) -> None:
    for idx, value in enumerate(values):
        while len(ranges) <= idx:
            ranges.append([math.inf, -math.inf])
        ranges[idx][0] = min(ranges[idx][0], value)
        ranges[idx][1] = max(ranges[idx][1], value)


def summarize_ranges(ranges: list[list[float]]) -> list[list[float]]:
    return [[round(low, 6), round(high, 6)] for low, high in ranges]


def range_span(values: Sequence[float]) -> float:
    return max(values) - min(values) if values else 0.0


def range_gap(xs: Sequence[float], ys: Sequence[float]) -> float:
    if not xs or not ys:
        return 0.0
    low_x, high_x = min(xs), max(xs)
    low_y, high_y = min(ys), max(ys)
    return max(0.0, max(low_x, low_y) - min(high_x, high_y))


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values)


def evaluate_gripper_consistency(
    state_gripper: Sequence[float],
    action_gripper: Sequence[float],
    report: CheckReport,
) -> None:
    if len(state_gripper) != len(action_gripper) or not state_gripper:
        return

    state_span = range_span(state_gripper)
    action_span = range_span(action_gripper)
    state_mean = mean(state_gripper)
    action_mean = mean(action_gripper)
    gap = range_gap(state_gripper, action_gripper)
    mean_gap = abs(state_mean - action_mean)

    report.details["state_gripper_span"] = state_span
    report.details["action_gripper_span"] = action_span
    report.details["state_gripper_mean"] = state_mean
    report.details["action_gripper_mean"] = action_mean
    report.details["state_action_gripper_range_gap"] = gap
    report.details["state_action_gripper_mean_gap"] = mean_gap

    if gap > GRIPPER_RANGE_GAP_FAIL_THRESHOLD:
        report.fail(
            "Gripper state/action numeric ranges do not overlap. This suggests "
            "the observation gripper and action gripper use different scaling, "
            "offsets, or open/close conventions."
        )
    elif mean_gap > GRIPPER_MEAN_GAP_FAIL_THRESHOLD:
        report.fail(
            "Gripper state/action means are far apart despite overlapping ranges. "
            "Check whether state and action gripper values are in the same units "
            "and convention."
        )

    if (
        state_span < GRIPPER_LOW_VARIATION_EPS
        or action_span < GRIPPER_LOW_VARIATION_EPS
    ):
        report.warn(
            "Gripper variation is too small for correlation to be meaningful. "
            "Use the reported gripper ranges/gaps to judge whether state and "
            "action are on the same scale."
        )
        return

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


def inspect_parquet_rows(
    rows: ParquetRows,
    parquet_path: Path,
    *,
    expected_state_dim: int,
    expected_action_dim: int,
    state_gripper: list[float],
    action_gripper: list[float],
    all_timestamps: list[float],
    state_ranges: list[list[float]],
    action_ranges: list[list[float]],
    report: CheckReport,
) -> int:
    if "observation.state" not in rows.columns:
        report.fail(f"{parquet_path} is missing column observation.state")
        return 0
    if "action" not in rows.columns:
        report.fail(f"{parquet_path} is missing column action")
        return 0

    sampled_rows = 0
    for row_index, (state, action) in enumerate(zip(rows.states, rows.actions)):
        state_values = flatten_numeric(state)
        action_values = flatten_numeric(action)
        if len(state_values) != expected_state_dim:
            report.fail(
                f"{parquet_path} row {row_index}: observation.state has length "
                f"{len(state_values)}, expected {expected_state_dim}"
            )
        if len(action_values) != expected_action_dim:
            report.fail(
                f"{parquet_path} row {row_index}: action has length "
                f"{len(action_values)}, expected {expected_action_dim}"
            )
        if not all(math.isfinite(value) for value in state_values):
            report.fail(f"{parquet_path} row {row_index}: observation.state has NaN/Inf")
        if not all(math.isfinite(value) for value in action_values):
            report.fail(f"{parquet_path} row {row_index}: action has NaN/Inf")

        update_dimension_ranges(state_ranges, state_values)
        update_dimension_ranges(action_ranges, action_values)
        if len(state_values) >= expected_state_dim:
            state_gripper.append(state_values[expected_state_dim - 1])
        if len(action_values) >= expected_action_dim:
            action_gripper.append(action_values[expected_action_dim - 1])
        sampled_rows += 1

    for timestamp in rows.timestamps:
        values = flatten_numeric(timestamp)
        if values:
            all_timestamps.append(values[0])

    return sampled_rows


def inspect_parquets(
    dataset_root: Path,
    *,
    max_files: int,
    expected_state_dim: int,
    expected_action_dim: int,
    expected_fps: float,
    fps_tolerance: float,
    require_frame_inspection: bool,
    report: CheckReport,
) -> None:
    parquet_files = list_episode_parquets(dataset_root, max_files=max_files)
    report.details["inspected_parquet_files"] = [str(path) for path in parquet_files]
    if not parquet_files:
        message = "No parquet files found below data/; only metadata was checked"
        if require_frame_inspection:
            report.fail(message)
        else:
            report.warn(message)
        return

    reader_name, reader = select_parquet_reader(report)
    report.details["parquet_reader"] = reader_name
    if reader is None:
        if require_frame_inspection:
            report.fail("Frame-level parquet inspection is required but no reader is available")
        return

    state_gripper: list[float] = []
    action_gripper: list[float] = []
    all_timestamps: list[float] = []
    state_ranges: list[list[float]] = []
    action_ranges: list[list[float]] = []
    sampled_rows = 0
    read_failures = 0

    for parquet_path in parquet_files:
        try:
            rows = reader(parquet_path)
        except Exception as exc:
            read_failures += 1
            report.warn(f"Could not read {parquet_path} with {reader_name}: {exc}")
            continue
        sampled_rows += inspect_parquet_rows(
            rows,
            parquet_path,
            expected_state_dim=expected_state_dim,
            expected_action_dim=expected_action_dim,
            state_gripper=state_gripper,
            action_gripper=action_gripper,
            all_timestamps=all_timestamps,
            state_ranges=state_ranges,
            action_ranges=action_ranges,
            report=report,
        )

    report.details["sampled_rows"] = sampled_rows
    report.details["parquet_read_failures"] = read_failures
    if read_failures and require_frame_inspection:
        report.fail("Frame-level parquet inspection is required but at least one file could not be read")
    if sampled_rows == 0:
        message = "No frame rows were inspected; data-level problems cannot be ruled out"
        if require_frame_inspection:
            report.fail(message)
        else:
            report.warn(message)
        return

    report.pass_(f"Inspected {sampled_rows} frame rows with {reader_name}")
    if state_ranges:
        report.details["state_dim_ranges"] = summarize_ranges(state_ranges)
    if action_ranges:
        report.details["action_dim_ranges"] = summarize_ranges(action_ranges)
    if state_gripper:
        report.details["state_gripper_min"] = min(state_gripper)
        report.details["state_gripper_max"] = max(state_gripper)
    if action_gripper:
        report.details["action_gripper_min"] = min(action_gripper)
        report.details["action_gripper_max"] = max(action_gripper)

    evaluate_gripper_consistency(state_gripper, action_gripper, report)

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
    parser.add_argument(
        "--require-frame-inspection",
        action="store_true",
        help="Fail if parquet frame rows cannot be inspected. Use this before hardware tests.",
    )
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
        expected_state_dim=args.expected_state_dim,
        expected_action_dim=args.expected_action_dim,
        expected_fps=args.expected_fps,
        fps_tolerance=args.fps_tolerance,
        require_frame_inspection=args.require_frame_inspection,
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
