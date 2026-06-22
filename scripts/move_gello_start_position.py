#!/usr/bin/env python3
"""Move the live GELLO/Panda ZMQ robot to a configured start pose.

The script talks directly to an already running ZMQ robot server. It does not
start GELLO, cameras, Polymetis, or any tmux session.
"""

from __future__ import annotations

import argparse
import time
from collections.abc import Sequence

import numpy as np
import zmq

from gello.zmq_core.robot_node import ZMQClientRobot

# Start pose requested for the three-Lego-block setup.
DEFAULT_START_RAD = (0.0831, -0.1316, -0.1535, -2.4256, -0.0616, 2.2407, -0.7906)
DEFAULT_GRIPPER = 1.0


def _parse_floats(text: str) -> tuple[float, ...]:
    return tuple(float(part.strip()) for part in text.split(",") if part.strip())


def _target_from_args(args: argparse.Namespace) -> np.ndarray:
    if args.target_rad and args.target_deg:
        raise ValueError("Use only one of --target-rad or --target-deg")
    if args.target_rad:
        arm = np.asarray(_parse_floats(args.target_rad), dtype=np.float32)
    elif args.target_deg:
        arm = np.deg2rad(np.asarray(_parse_floats(args.target_deg), dtype=np.float32))
    else:
        arm = np.asarray(DEFAULT_START_RAD, dtype=np.float32)
    if arm.shape != (7,):
        raise ValueError(f"Expected 7 arm joints, got {arm.shape[0]} values")
    if args.ignore_gripper:
        return arm
    return np.concatenate([arm, np.asarray([args.target_gripper], dtype=np.float32)])


def _configure_timeout(robot: ZMQClientRobot, timeout_ms: int) -> None:
    socket = getattr(robot, "_socket", None)
    if socket is None:
        raise AttributeError("ZMQClientRobot has no _socket attribute")
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    socket.setsockopt(zmq.LINGER, 0)


def _format_joint_line(name: str, current: float, target: float, error: float, tolerance: float) -> str:
    status = "OK " if error <= tolerance else "BAD"
    return f"{status} {name:>8s}: current={current:+.4f} target={target:+.4f} error={error:.4f} tol={tolerance:.4f}"


def _print_pose_check(current: np.ndarray, target: np.ndarray, arm_tolerance: float, gripper_tolerance: float) -> bool:
    errors = np.abs(current - target)
    tolerances = np.full(target.shape, arm_tolerance, dtype=np.float32)
    if target.shape[0] == 8:
        tolerances[-1] = gripper_tolerance

    for index, (current_value, target_value, error, tolerance) in enumerate(
        zip(current, target, errors, tolerances)
    ):
        name = f"joint_{index}" if index < 7 else "gripper"
        print(_format_joint_line(name, float(current_value), float(target_value), float(error), float(tolerance)))
    return bool(np.all(errors <= tolerances))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=6001)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    parser.add_argument("--target-rad", default=None, help="Comma-separated 7-DoF arm target in radians")
    parser.add_argument("--target-deg", default=None, help="Comma-separated 7-DoF arm target in degrees")
    parser.add_argument("--target-gripper", type=float, default=DEFAULT_GRIPPER)
    parser.add_argument("--ignore-gripper", action="store_true")
    parser.add_argument("--steps", type=int, default=80, help="Number of interpolation commands")
    parser.add_argument("--period-s", type=float, default=0.04, help="Sleep between interpolation commands")
    parser.add_argument("--hold-s", type=float, default=0.5, help="Time to keep commanding the final target")
    parser.add_argument("--arm-tolerance-rad", type=float, default=0.03)
    parser.add_argument("--gripper-tolerance", type=float, default=0.08)
    parser.add_argument("--dry-run", action="store_true", help="Only print current and target pose; do not move")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    target = _target_from_args(args)
    robot = ZMQClientRobot(port=args.robot_port, host=args.robot_host)
    _configure_timeout(robot, args.timeout_ms)
    try:
        current = np.asarray(robot.get_joint_state(), dtype=np.float32)
        if current.shape[0] < target.shape[0]:
            raise ValueError(f"Robot returned {current.shape[0]} joints, target needs {target.shape[0]}")
        current = current[: target.shape[0]]

        print("Current pose before movement:")
        already_ok = _print_pose_check(current, target, args.arm_tolerance_rad, args.gripper_tolerance)
        if already_ok:
            print("Already at target pose within tolerance.")
            return 0
        if args.dry_run:
            print("Dry run only; no command was sent.")
            return 1

        if args.steps < 1:
            raise ValueError("--steps must be at least 1")
        print(f"\nMoving to target over {args.steps} steps ({args.steps * args.period_s:.2f} s) ...")
        for command in np.linspace(current, target, args.steps, dtype=np.float32):
            robot.command_joint_state(command)
            time.sleep(args.period_s)

        end_time = time.monotonic() + max(0.0, args.hold_s)
        while time.monotonic() < end_time:
            robot.command_joint_state(target)
            time.sleep(args.period_s)

        final = np.asarray(robot.get_joint_state(), dtype=np.float32)[: target.shape[0]]
        print("\nCurrent pose after movement:")
        ok = _print_pose_check(final, target, args.arm_tolerance_rad, args.gripper_tolerance)
        print("PASS: target pose reached" if ok else "FAIL: target pose is still outside tolerance")
        return 0 if ok else 1
    finally:
        robot.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
