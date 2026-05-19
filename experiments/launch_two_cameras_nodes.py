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


def launch_server(name: str, port: int, camera_id: str, hostname: str):
    print(f"Opening {name} camera {camera_id}", flush=True)
    camera = RealSenseCamera(camera_id)

    server = ZMQServerCamera(camera, port=port, host=hostname)
    print(f"Starting {name} camera server on {hostname}:{port}", flush=True)

    server.serve()


def main(args: Args):
    if not args.base_camera_id:
        raise ValueError("Please provide --base-camera-id for the D455.")

    servers = [
        Process(
            target=launch_server,
            args=("wrist", args.wrist_port, args.wrist_camera_id, args.hostname),
        ),
        Process(
            target=launch_server,
            args=("base", args.base_port, args.base_camera_id, args.hostname),
        ),
    ]

    for server in servers:
        server.start()

    for server in servers:
        server.join()


if __name__ == "__main__":
    main(tyro.cli(Args))