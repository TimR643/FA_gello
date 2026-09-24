from dataclasses import dataclass
import tyro

from gello.cameras.realsense_camera import RealSenseCamera
from gello.zmq_core.camera_node import ZMQServerCamera


@dataclass
class Args:
    hostname: str = "127.0.0.1"
    port: int = 5000
    camera_id: str = "6CD1460304A5"


def main(args: Args):
    print(f"Opening RealSense camera {args.camera_id}", flush=True)
    camera = RealSenseCamera(args.camera_id)
    print("Camera opened successfully", flush=True)

    server = ZMQServerCamera(camera, port=args.port, host=args.hostname)
    print(f"Starting camera server on {args.hostname}:{args.port}", flush=True)
    server.serve()


if __name__ == "__main__":
    main(tyro.cli(Args))
