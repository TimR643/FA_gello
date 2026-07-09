"""Validate that expected RealSense camera serial numbers are visible."""

from __future__ import annotations

import argparse
import sys


def _list_realsense_serials() -> list[str]:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError(
            "pyrealsense2 is not installed in this environment. Install the "
            "RealSense Python bindings in the recorder environment; "
            "realsense-viewer is useful for debugging but is not required."
        ) from exc

    ctx = rs.context()
    return [
        dev.get_info(rs.camera_info.serial_number)
        for dev in ctx.query_devices()
    ]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check that required RealSense serials are visible."
    )
    parser.add_argument("--wrist-camera-id", required=True)
    parser.add_argument("--base-camera-id", required=True)
    args = parser.parse_args()

    try:
        serials = _list_realsense_serials()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print("Visible RealSense serials:", ", ".join(serials) if serials else "<none>")
    expected = {
        "wrist": args.wrist_camera_id,
        "base": args.base_camera_id,
    }
    missing = [
        f"{name}={serial}"
        for name, serial in expected.items()
        if serial not in serials
    ]
    if missing:
        print(
            "ERROR: missing expected RealSense camera(s): " + ", ".join(missing),
            file=sys.stderr,
        )
        print(
            "Connect both cameras to this recorder machine, check USB/power, "
            "or correct WRIST_CAMERA_ID/BASE_CAMERA_ID.",
            file=sys.stderr,
        )
        return 1

    print("All expected RealSense cameras are visible.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
