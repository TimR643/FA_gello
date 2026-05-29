import pickle
import threading
from typing import Any, Dict

import numpy as np
import zmq

from gello.robots.robot import Robot

DEFAULT_ROBOT_PORT = 6000


def _pack_numpy(value: Any) -> Any:
    """Convert NumPy values to pickle-stable Python containers."""
    if isinstance(value, np.ndarray):
        contiguous = np.ascontiguousarray(value)
        return {
            "__ndarray__": True,
            "dtype": str(contiguous.dtype),
            "shape": contiguous.shape,
            "data": contiguous.tobytes(),
        }
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _pack_numpy(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(_pack_numpy(item) for item in value)
    return value


def _unpack_numpy(value: Any) -> Any:
    """Reconstruct NumPy values from pickle-stable Python containers."""
    if isinstance(value, dict):
        if value.get("__ndarray__") is True:
            array = np.frombuffer(value["data"], dtype=np.dtype(value["dtype"]))
            return array.reshape(value["shape"]).copy()
        return {key: _unpack_numpy(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(_unpack_numpy(item) for item in value)
    return value


def _dumps_message(value: Any) -> bytes:
    return pickle.dumps(_pack_numpy(value), protocol=pickle.HIGHEST_PROTOCOL)


def _loads_message(message: bytes) -> Any:
    return _unpack_numpy(pickle.loads(message))


class ZMQServerRobot:
    def __init__(
        self,
        robot: Robot,
        port: int = DEFAULT_ROBOT_PORT,
        host: str = "127.0.0.1",
    ):
        self._robot = robot
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        addr = f"tcp://{host}:{port}"
        debug_message = f"Robot Sever Binding to {addr}, Robot: {robot}"
        print(debug_message)
        self._timout_message = f"Timeout in Robot Server, Robot: {robot}"
        self._socket.bind(addr)
        self._stop_event = threading.Event()

    def serve(self) -> None:
        """Serve the leader robot state over ZMQ."""
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # Set timeout to 1000 ms
        while not self._stop_event.is_set():
            try:
                # Wait for next request from client
                message = self._socket.recv()
                request = _loads_message(message)

                # Call the appropriate method based on the request
                method = request.get("method")
                args = request.get("args", {})
                result: Any
                if method == "num_dofs":
                    result = self._robot.num_dofs()
                elif method == "get_joint_state":
                    result = self._robot.get_joint_state()
                elif method == "command_joint_state":
                    result = self._robot.command_joint_state(**args)
                elif method == "get_observations":
                    result = self._robot.get_observations()
                else:
                    result = {"error": "Invalid method"}
                    print(result)
                    raise NotImplementedError(
                        f"Invalid method: {method}, {args, result}"
                    )

                self._socket.send(_dumps_message(result))
            except zmq.Again:
                # Timeout occurred - don't spam the console
                pass

    def stop(self) -> None:
        """Signal the server to stop serving."""
        self._stop_event.set()


class ZMQClientRobot(Robot):
    """A class representing a ZMQ client for a leader robot."""

    def __init__(self, port: int = DEFAULT_ROBOT_PORT, host: str = "127.0.0.1"):
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.connect(f"tcp://{host}:{port}")

    def num_dofs(self) -> int:
        """Get the number of joints in the robot.

        Returns:
            int: The number of joints in the robot.
        """
        request = {"method": "num_dofs"}
        send_message = _dumps_message(request)
        self._socket.send(send_message)
        result = _loads_message(self._socket.recv())
        return result

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        request = {"method": "get_joint_state"}
        send_message = _dumps_message(request)
        try:
            self._socket.send(send_message)
            result = _loads_message(self._socket.recv())
            if isinstance(result, dict) and "error" in result:
                raise RuntimeError(result["error"])
            return result
        except zmq.Again:
            raise RuntimeError("ZMQ timeout - robot may be disconnected")

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to the given state.

        Args:
            joint_state (T): The state to command the leader robot to.
        """
        request = {
            "method": "command_joint_state",
            "args": {"joint_state": joint_state},
        }
        send_message = _dumps_message(request)
        self._socket.send(send_message)
        result = _loads_message(self._socket.recv())
        return result

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get the current observations of the leader robot.

        Returns:
            Dict[str, np.ndarray]: The current observations of the leader robot.
        """
        request = {"method": "get_observations"}
        send_message = _dumps_message(request)
        try:
            self._socket.send(send_message)
            result = _loads_message(self._socket.recv())
            if isinstance(result, dict) and "error" in result:
                raise RuntimeError(result["error"])
            return result
        except zmq.Again:
            raise RuntimeError("ZMQ timeout - robot may be disconnected")

    def close(self) -> None:
        """Close the ZMQ socket and context."""
        self._socket.close()
        self._context.term()
