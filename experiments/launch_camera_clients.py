from dataclasses import dataclass
from typing import Tuple
import time

import numpy as np
import tyro

from gello.zmq_core.camera_node import ZMQClientCamera


@dataclass
class Args:
    ports: Tuple[int, ...] = (5000, 5001)
    hostname: str = "127.0.0.1"
    # hostname: str = "128.32.175.167"
    visualize: bool = False
    hz: float = 5.0


def main(args):
    cameras = []

    for port in args.ports:
        cameras.append(ZMQClientCamera(port=port, host=args.hostname))

    if args.visualize:
        import cv2

        images_display_names = []
        for port in args.ports:
            images_display_names.append(f"image_{port}")
            cv2.namedWindow(images_display_names[-1], cv2.WINDOW_NORMAL)

        while True:
            for display_name, camera in zip(images_display_names, cameras):
                image, depth = camera.read()
                stacked_depth = np.dstack([depth, depth, depth]).astype(np.uint8)
                image_depth = cv2.hconcat([image[:, :, ::-1], stacked_depth])
                cv2.imshow(display_name, image_depth)
                cv2.waitKey(1)
    else:
        period = 1.0 / max(args.hz, 0.1)
        while True:
            for port, camera in zip(args.ports, cameras):
                image, depth = camera.read()
                print(f"camera {port}: rgb={image.shape}, depth={depth.shape}")
            time.sleep(period)



if __name__ == "__main__":
    main(tyro.cli(Args))
