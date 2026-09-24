#!/usr/bin/env python3
"""Inspect LeRobot state/action conventions and directional bias.

This script is intentionally offline: point it at a LeRobot dataset root and it
summarizes whether ``action`` looks like an absolute joint target or a delta, and
whether episodes move positive or negative on a chosen direction joint.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


DEFAULT_PARQUET_GLOB = "data/**/*.parquet"


@dataclass(frozen=True)
class FrameRecord:
    episode_index: int
    frame_index: int
    task: str
    state: tuple[float, ...]
    action: tuple[float, ...]


@dataclass(frozen=True)
class EpisodeSummary:
    episode_index: int
    task: str
    frames: int
    first_state: tuple[float, ...]
    last_state: tuple[float, ...]
    mean_abs_action_minus_state: float
    mean_abs_action: float
    direction_joint_index: int
    state_direction_delta: float
    action_direction_delta: float
    mean_direction_command: float

    @property
    def likely_absolute_action(self) -> bool:
        return self.mean_abs_action_minus_state < self.mean_abs_action

    @property
    def direction_label(self) -> str:
        if self.action_direction_delta > 0:
            return "positive"
        if self.action_direction_delta < 0:
            return "negative"
        return "flat"


def _flatten_numeric(value: Any) -> tuple[float, ...]:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (int, float)):
        return (float(value),)
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        out: list[float] = []
        for item in value:
            out.extend(_flatten_numeric(item))
        return tuple(out)
    raise TypeError(f"Cannot flatten numeric value of type {type(value).__name__}")


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _row_value(row: Mapping[str, Any], name: str, default: Any = None) -> Any:
    return row[name] if name in row else default


def _records_from_rows(rows: Iterable[Mapping[str, Any]]) -> list[FrameRecord]:
    records: list[FrameRecord] = []
    for fallback_index, row in enumerate(rows):
        state = _flatten_numeric(_row_value(row, "observation.state"))
        action = _flatten_numeric(_row_value(row, "action"))
        if len(state) != len(action):
            raise ValueError(
                f"state/action length mismatch at row {fallback_index}: "
                f"{len(state)} != {len(action)}"
            )
        if not _finite(state) or not _finite(action):
            raise ValueError(f"state/action contains NaN/Inf at row {fallback_index}")
        records.append(
            FrameRecord(
                episode_index=int(_row_value(row, "episode_index", 0)),
                frame_index=int(_row_value(row, "frame_index", fallback_index)),
                task=str(_row_value(row, "task", "")),
                state=state,
                action=action,
            )
        )
    return records


def _load_parquet_records(dataset_root: Path, parquet_glob: str) -> list[FrameRecord]:
    paths = sorted(dataset_root.glob(parquet_glob))
    if not paths:
        raise FileNotFoundError(f"No parquet files matched {dataset_root / parquet_glob}")

    try:
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise RuntimeError(
            "pyarrow is required to inspect LeRobot parquet files. Install pyarrow "
            "in the lerobot environment and rerun this script."
        ) from exc

    records: list[FrameRecord] = []
    for path in paths:
        table = pq.read_table(path)
        records.extend(_records_from_rows(table.to_pylist()))
    return records


def summarize_records(
    records: Sequence[FrameRecord], *, direction_joint_index: int
) -> list[EpisodeSummary]:
    grouped: dict[int, list[FrameRecord]] = {}
    for record in records:
        grouped.setdefault(record.episode_index, []).append(record)

    summaries: list[EpisodeSummary] = []
    for episode_index, episode_records in sorted(grouped.items()):
        ordered = sorted(episode_records, key=lambda record: record.frame_index)
        if not ordered:
            continue
        width = len(ordered[0].state)
        if not 0 <= direction_joint_index < width:
            raise ValueError(
                f"direction_joint_index {direction_joint_index} is outside state/action width {width}"
            )

        abs_action_minus_state: list[float] = []
        abs_action: list[float] = []
        direction_commands: list[float] = []
        for record in ordered:
            abs_action_minus_state.extend(
                abs(action_value - state_value)
                for state_value, action_value in zip(record.state, record.action)
            )
            abs_action.extend(abs(value) for value in record.action)
            direction_commands.append(
                record.action[direction_joint_index] - record.state[direction_joint_index]
            )

        first = ordered[0]
        last = ordered[-1]
        summaries.append(
            EpisodeSummary(
                episode_index=episode_index,
                task=first.task,
                frames=len(ordered),
                first_state=first.state,
                last_state=last.state,
                mean_abs_action_minus_state=sum(abs_action_minus_state)
                / len(abs_action_minus_state),
                mean_abs_action=sum(abs_action) / len(abs_action),
                direction_joint_index=direction_joint_index,
                state_direction_delta=last.state[direction_joint_index]
                - first.state[direction_joint_index],
                action_direction_delta=last.action[direction_joint_index]
                - first.action[direction_joint_index],
                mean_direction_command=sum(direction_commands) / len(direction_commands),
            )
        )
    return summaries


def _print_summary(summaries: Sequence[EpisodeSummary]) -> None:
    if not summaries:
        print("No episodes found.")
        return

    absolute_votes = sum(summary.likely_absolute_action for summary in summaries)
    print(f"episodes: {len(summaries)}")
    print(
        "action convention vote: "
        f"{absolute_votes}/{len(summaries)} episodes look more absolute than delta"
    )
    print()
    print(
        "episode  frames  convention  dir_joint  state_delta  action_delta  "
        "mean_cmd  direction  task"
    )
    for summary in summaries:
        convention = "absolute" if summary.likely_absolute_action else "delta?"
        print(
            f"{summary.episode_index:7d}  {summary.frames:6d}  {convention:10s}  "
            f"{summary.direction_joint_index:9d}  "
            f"{summary.state_direction_delta:11.4f}  "
            f"{summary.action_direction_delta:12.4f}  "
            f"{summary.mean_direction_command:8.4f}  "
            f"{summary.direction_label:9s}  {summary.task}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_root", type=Path)
    parser.add_argument("--parquet-glob", default=DEFAULT_PARQUET_GLOB)
    parser.add_argument("--direction-joint-index", type=int, default=2)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    records = _load_parquet_records(args.dataset_root, args.parquet_glob)
    summaries = summarize_records(records, direction_joint_index=args.direction_joint_index)
    if args.json:
        print(json.dumps([summary.__dict__ for summary in summaries], indent=2))
    else:
        _print_summary(summaries)


if __name__ == "__main__":
    main()
