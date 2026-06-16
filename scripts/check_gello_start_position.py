#!/usr/bin/env python3
"""Check whether the GELLO/Panda ZMQ robot is in the recording start pose."""

from __future__ import annotations

import argparse
import time
from collections.abc import Sequence

import numpy as np
import zmq

from gello.zmq_core.robot_node import ZMQClientRobot

DEFAULT_START_RAD = (0.0905, 0.0, 0.0, -2.2131, 2.1220, -0.9493, 0.8868)
DEFAULT_GRIPPER_OPEN = 1.0


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


def _format_row(name: str, current: float, target: float, error: float, tolerance: float) -> str:
    status = "OK " if error <= tolerance else "BAD"
    return f"{status} {name:>8s}: current={current:+.4f} target={target:+.4f} error={error:.4f} tol={tolerance:.4f}"


def _check_once(
    args: argparse.Namespace, target: np.ndarray, robot: ZMQClientRobot | None = None
) -> bool:
    owns_robot = robot is None
    if robot is None:
        robot = ZMQClientRobot(port=args.robot_port, host=args.robot_host)
        _configure_timeout(robot, args.timeout_ms)
    try:
        current = np.asarray(robot.get_joint_state(), dtype=np.float32)
    except Exception as exc:
        print(
            "ERROR: Could not read robot joint state from "
            f"{args.robot_host}:{args.robot_port} within {args.timeout_ms} ms. "
            "The ZMQ robot server may be stopped, the SSH tunnel may be down, "
            "or the server may be busy with recording/rollout. "
            "Do not run this checker during timing-sensitive control; run it before recording. "
            "If the server is alive but slow, retry with ZMQ_TIMEOUT_MS=10000.",
            flush=True,
        )
        print(f"Underlying error: {exc}", flush=True)
        return False
    finally:
        if owns_robot:
            robot.close()

    if current.shape[0] < target.shape[0]:
        raise ValueError(f"Robot returned {current.shape[0]} joints, target needs {target.shape[0]}")
    current = current[: target.shape[0]]
    errors = np.abs(current - target)
    tolerances = np.full(target.shape, args.arm_tolerance_rad, dtype=np.float32)
    if target.shape[0] == 8:
        tolerances[-1] = args.gripper_tolerance

    print("Current GELLO/Panda start-pose check:")
    for index, (current_value, target_value, error, tolerance) in enumerate(
        zip(current, target, errors, tolerances)
    ):
        name = f"joint_{index}" if index < 7 else "gripper"
        print(_format_row(name, float(current_value), float(target_value), float(error), float(tolerance)))

    max_arm_error = float(errors[:7].max())
    gripper_ok = True if target.shape[0] == 7 else bool(errors[-1] <= tolerances[-1])
    arm_ok = bool(np.all(errors[:7] <= tolerances[:7]))
    ok = arm_ok and gripper_ok
    print()
    print(f"Max arm error: {max_arm_error:.4f} rad ({np.rad2deg(max_arm_error):.2f} deg)")
    if target.shape[0] == 8:
        print(f"Gripper error: {float(errors[-1]):.4f}")
    print("PASS: start pose is within tolerance" if ok else "FAIL: move robot/GELLO to the start pose")
    return ok


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=6001)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    parser.add_argument(
        "--target-deg",
        default=None,
        help="Comma-separated 7-DoF arm target in degrees",
    )
    parser.add_argument(
        "--target-rad",
        default=None,
        help="Comma-separated 7-DoF arm target in radians. Default: 0.0905,0,0,-2.2131,2.1220,-0.9493,0.8868",
    )
    parser.add_argument("--target-gripper", type=float, default=DEFAULT_GRIPPER_OPEN)
    parser.add_argument("--arm-tolerance-rad", type=float, default=0.035)
    parser.add_argument("--gripper-tolerance", type=float, default=0.08)
    parser.add_argument("--ignore-gripper", action="store_true")
    parser.add_argument("--watch", action="store_true", help="Repeat until interrupted")
    parser.add_argument("--period-s", type=float, default=5.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    target = _target_from_args(args)
    if not args.watch:
        return 0 if _check_once(args, target) else 1

    while True:
        _check_once(args, target)
        print(
            "Press Ctrl-C to stop. Do not leave --watch running during "
            "recording/rollout; it shares the robot ZMQ server.\n"
        )
        time.sleep(args.period_s)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
