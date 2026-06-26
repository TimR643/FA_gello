"""Record robot observations from a GELLO ZMQ robot server to HDF5."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import tyro

from gello.data_utils.h5_logger import H5RobotLogger
from gello.zmq_core.robot_node import ZMQClientRobot


@dataclass
class Args:
    output: Path = Path("logs/robot_recording.h5")
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    rate_hz: float = 100.0
    duration_s: Optional[float] = None
    include_joint_command: bool = False


def main(args: Args) -> None:
    robot = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    period_s = 1.0 / args.rate_hz
    start = time.time()
    next_tick = start

    metadata = {
        "source": "scripts/record_robot_h5.py",
        "hostname": args.hostname,
        "robot_port": args.robot_port,
        "rate_hz": args.rate_hz,
    }
    with H5RobotLogger(args.output, metadata=metadata) as logger:
        print(f"Recording robot observations to {args.output}")
        try:
            while args.duration_s is None or time.time() - start < args.duration_s:
                now = time.time()
                if now < next_tick:
                    time.sleep(next_tick - now)
                timestamp = time.time()
                obs = robot.get_observations()
                action = robot.get_joint_state() if args.include_joint_command else None
                logger.log(obs, action=action, timestamp=timestamp)
                next_tick += period_s
        except KeyboardInterrupt:
            print("Stopping recording after keyboard interrupt.")
        finally:
            robot.close()
            print(f"Wrote {logger.count} frames to {args.output}")


if __name__ == "__main__":
    main(tyro.cli(Args))
