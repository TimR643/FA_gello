from dataclasses import dataclass
from multiprocessing import Process
import time

import tyro

from gello.cameras.realsense_camera import RealSenseCamera, get_device_ids
from gello.zmq_core.camera_node import ZMQServerCamera


@dataclass
class Args:
    hostname: str = "127.0.0.1"


def launch_server(port: int, camera_id: str, args: Args):
    try:
        camera = RealSenseCamera(camera_id)
        server = ZMQServerCamera(camera, port=port, host=args.hostname)
        print(f"Starting camera server for {camera_id} on port {port}")
        server.serve()
    except Exception as exc:
        print(f"Camera server for {camera_id} on port {port} crashed: {exc}")
        raise


def main(args):
    ids = get_device_ids(reset_devices=False)
    if len(ids) == 0:
        raise RuntimeError("No RealSense/FRAMOS cameras detected. Check USB/power and rsviewer setup.")

    print(f"Detected camera serials: {ids}")
    camera_port = 5000
    camera_servers = []
    for camera_id in ids:
        print(f"Launching camera {camera_id} on port {camera_port}")
        proc = Process(target=launch_server, args=(camera_port, camera_id, args))
        proc.start()
        camera_servers.append(proc)
        camera_port += 1

    print("Camera servers started. Press Ctrl+C to stop.")
    try:
        while True:
            for proc in camera_servers:
                if not proc.is_alive():
                    raise RuntimeError(f"Camera server process {proc.pid} exited unexpectedly.")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopping camera servers...")
    finally:
        for proc in camera_servers:
            if proc.is_alive():
                proc.terminate()
        for proc in camera_servers:
            proc.join(timeout=2)


if __name__ == "__main__":
    main(tyro.cli(Args))
