import select
import sys
import termios
import tty

import rclpy
from franka_msgs.action import Move
from rclpy.action import ActionClient
from rclpy.node import Node


DEFAULT_MOVE_ACTION_TOPIC = "franka_gripper/move"
DEFAULT_OPEN_WIDTH = 0.08
DEFAULT_SPEED = 1.0


class KeyboardOpenGripper(Node):
    def __init__(self):
        super().__init__("keyboard_open_gripper")
        self.declare_parameter("move_action_topic", DEFAULT_MOVE_ACTION_TOPIC)
        self.declare_parameter("open_width", DEFAULT_OPEN_WIDTH)
        self.declare_parameter("speed", DEFAULT_SPEED)

        self._move_action_topic = (
            self.get_parameter("move_action_topic").get_parameter_value().string_value
        )
        self._open_width = self.get_parameter("open_width").get_parameter_value().double_value
        self._speed = self.get_parameter("speed").get_parameter_value().double_value

        self._action_client = ActionClient(self, Move, self._move_action_topic)
        self.get_logger().info(f"Waiting for action server: {self._move_action_topic}")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Move action server not available on {self._move_action_topic}")

        self.get_logger().info("Press SPACE to open gripper, 'q' to quit.")
        self._old_term_settings = termios.tcgetattr(sys.stdin)

    def run(self):
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                if self._key_pressed(timeout=0.1):
                    key = sys.stdin.read(1)
                    if key == " ":
                        self._open_gripper()
                    elif key.lower() == "q":
                        self.get_logger().info("Quit requested by user.")
                        break
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term_settings)

    def _key_pressed(self, timeout: float) -> bool:
        read_ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return bool(read_ready)

    def _open_gripper(self):
        goal_msg = Move.Goal()
        goal_msg.width = self._open_width
        goal_msg.speed = self._speed
        self.get_logger().info(
            f"Sending open command: width={goal_msg.width:.3f} speed={goal_msg.speed:.3f}"
        )
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Open gripper goal was rejected.")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Open result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardOpenGripper()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
