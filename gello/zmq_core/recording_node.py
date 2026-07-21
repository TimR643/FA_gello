import time
from typing import Any, Dict, Optional

import zmq

from gello.zmq_core.serialization import dumps_message, loads_message

DEFAULT_RECORDING_PORT = 7000


class ZMQRecordingPublisher:
    """Non-blocking PUSH publisher for recording frames."""

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_RECORDING_PORT,
        send_hwm: int = 2,
    ):
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUSH)
        self._socket.setsockopt(zmq.SNDHWM, send_hwm)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._addr = f"tcp://{host}:{port}"
        self._socket.connect(self._addr)
        print(f"Recording stream publisher connecting to {self._addr}")

    def send(
        self,
        message: Dict[str, Any],
        *,
        block: bool = False,
        timeout_s: float = 2.0,
    ) -> bool:
        payload = dumps_message(message)
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                self._socket.send(payload, flags=zmq.NOBLOCK)
                return True
            except zmq.Again:
                if not block or time.monotonic() >= deadline:
                    return False
                time.sleep(0.01)

    def close(self) -> None:
        self._socket.close()
        self._context.term()


class ZMQRecordingReceiver:
    """PULL receiver for recording frames."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = DEFAULT_RECORDING_PORT,
        recv_timeout_ms: Optional[int] = 1000,
    ):
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PULL)
        if recv_timeout_ms is not None:
            self._socket.setsockopt(zmq.RCVTIMEO, recv_timeout_ms)
        self._addr = f"tcp://{host}:{port}"
        try:
            self._socket.bind(self._addr)
        except zmq.ZMQError as exc:
            if exc.errno == zmq.EADDRINUSE:
                raise RuntimeError(
                    f"Recording stream port is already in use: {self._addr}. "
                    "Another recorder is probably still running. Stop the old "
                    "process or choose a different RECORD_STREAM_PORT/--port on "
                    "both the recorder and Franka laptop."
                ) from exc
            raise
        print(f"Recording stream receiver binding to {self._addr}")

    def recv(self) -> Optional[Dict[str, Any]]:
        try:
            return loads_message(self._socket.recv())
        except zmq.Again:
            return None

    def close(self) -> None:
        self._socket.close()
        self._context.term()
