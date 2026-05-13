from dataclasses import dataclass
from multiprocessing import Process
import time

import tyro

from gello.cameras.realsense_camera import RealSenseCamera
from gello.zmq_core.camera_node import ZMQServerCamera


@dataclass
class Args:
    hostname: str = "127.0.0.1"


HARDCODED_CAMERA_ID = "172.16.0.50"


def launch_server(port: int, camera_id: str, args: Args):
    try:
        camera = RealSenseCamera(device_id=camera_id)
        server = ZMQServerCamera(camera, port=port, host=args.hostname)

        print(f"Starting camera server for {camera_id} on port {port}")
        server.serve()

    except Exception as exc:
        print(f"Camera server for {camera_id} on port {port} crashed: {exc}")
        raise


def main(args: Args):
    camera_id = HARDCODED_CAMERA_ID
    camera_port = 5000

    print(f"Using hardcoded Ethernet RealSense/FRAMOS camera IP: {camera_id}")
    print(f"Launching camera {camera_id} on port {camera_port}")

    proc = Process(target=launch_server, args=(camera_port, camera_id, args))
    proc.start()

    print("Camera server started. Press Ctrl+C to stop.")

    try:
        while True:
            if not proc.is_alive():
                raise RuntimeError(
                    f"Camera server process {proc.pid} exited unexpectedly."
                )
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("Stopping camera server...")

    finally:
        if proc.is_alive():
            proc.terminate()
        proc.join(timeout=2)


if __name__ == "__main__":
    main(tyro.cli(Args))