"""Validate that expected RealSense camera serial numbers are visible."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys


def _serials_from_pyrealsense() -> list[str]:
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


def _serials_from_rs_enumerate() -> list[str]:
    try:
        result = subprocess.run(
            ["rs-enumerate-devices", "-s"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError:
        return []
    if result.returncode != 0:
        return []

    serials: list[str] = []
    for line in result.stdout.splitlines():
        # Example row: ``Intel RealSense D455          318122303303        5.17.0.10``
        match = re.search(r"\b([A-Z0-9]{10,})\b", line)
        if match:
            serials.append(match.group(1))
    return serials


def _list_realsense_serials() -> list[str]:
    serials = _serials_from_pyrealsense()
    # Network/FRAMOS devices can show up in rs-enumerate-devices even when the
    # pyrealsense context query only reports USB devices. Include both views for
    # startup validation; the actual recorder still opens by serial.
    for serial in _serials_from_rs_enumerate():
        if serial not in serials:
            serials.append(serial)
    return serials


def _open_streams(serials: list[str]) -> None:
    import pyrealsense2 as rs

    pipelines = []
    try:
        for serial in serials:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            pipelines.append(pipeline)
        for pipeline in pipelines:
            pipeline.wait_for_frames()
    finally:
        for pipeline in reversed(pipelines):
            pipeline.stop()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check that required RealSense serials are visible."
    )
    parser.add_argument("--wrist-camera-id")
    parser.add_argument("--base-camera-id")
    parser.add_argument(
        "--open-streams",
        action="store_true",
        help="Also open all expected cameras as 640x480@30 RealSense streams.",
    )
    args = parser.parse_args()

    try:
        serials = _list_realsense_serials()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print("Visible RealSense serials:", ", ".join(serials) if serials else "<none>")
    expected = {
        name: serial
        for name, serial in (
            ("wrist", args.wrist_camera_id),
            ("base", args.base_camera_id),
        )
        if serial
    }
    if not expected:
        print("ERROR: pass at least one expected camera ID.", file=sys.stderr)
        return 1
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
            "Close realsense-viewer and any old recorder/camera process, then "
            "check USB/power or correct WRIST_CAMERA_ID/BASE_CAMERA_ID.",
            file=sys.stderr,
        )
        return 1

    print("All expected RealSense cameras are visible.")
    if args.open_streams:
        try:
            _open_streams(list(expected.values()))
        except Exception as exc:
            print(
                "ERROR: visible RealSense camera(s) could not be opened as "
                f"streams by pyrealsense2: {exc!r}",
                file=sys.stderr,
            )
            print(
                "If rs-enumerate-devices sees a FRAMOS/network camera but "
                "pyrealsense2 cannot open it, use a pyrealsense2/librealsense "
                "build that supports that camera or run that camera through a "
                "separate bridge/source.",
                file=sys.stderr,
            )
            return 1
        print("All expected RealSense cameras opened successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
