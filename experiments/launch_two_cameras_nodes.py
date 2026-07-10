from dataclasses import dataclass
from multiprocessing import Process

import tyro

from gello.cameras.realsense_camera import RealSenseCamera
from gello.zmq_core.camera_node import ZMQServerCamera


@dataclass
class Args:
    hostname: str = "127.0.0.1"

    wrist_camera_id: str = "6CD1460304A5"
    wrist_port: int = 5000

    base_camera_id: str = ""
    base_port: int = 5001

    wait_timeout_ms: int = 10000
    max_read_retries: int = 2
    reset_on_timeout: bool = True
    keep_stream_alive: bool = False
    first_frame_timeout_ms: int = 15000


def launch_server(
    name: str,
    port: int,
    camera_id: str,
    hostname: str,
    wait_timeout_ms: int,
    max_read_retries: int,
    reset_on_timeout: bool,
    keep_stream_alive: bool,
    first_frame_timeout_ms: int,
):
    print(f"Opening {name} camera {camera_id}", flush=True)
    camera = RealSenseCamera(
        camera_id,
        wait_timeout_ms=wait_timeout_ms,
        max_read_retries=max_read_retries,
        reset_on_timeout=reset_on_timeout,
        keep_stream_alive=keep_stream_alive,
        first_frame_timeout_ms=first_frame_timeout_ms,
    )

    server = ZMQServerCamera(camera, port=port, host=hostname)
    print(f"Starting {name} camera server on {hostname}:{port}", flush=True)

    server.serve()


def main(args: Args):
    if not args.base_camera_id:
        raise ValueError("Please provide --base-camera-id for the D455.")

    servers = [
        Process(
            target=launch_server,
            args=(
                "wrist",
                args.wrist_port,
                args.wrist_camera_id,
                args.hostname,
                args.wait_timeout_ms,
                args.max_read_retries,
                args.reset_on_timeout,
                args.keep_stream_alive,
                args.first_frame_timeout_ms,
            ),
        ),
        Process(
            target=launch_server,
            args=(
                "base",
                args.base_port,
                args.base_camera_id,
                args.hostname,
                args.wait_timeout_ms,
                args.max_read_retries,
                args.reset_on_timeout,
                args.keep_stream_alive,
                args.first_frame_timeout_ms,
            ),
        ),
    ]

    for server in servers:
        server.start()

    for server in servers:
        server.join()


if __name__ == "__main__":
    main(tyro.cli(Args))
