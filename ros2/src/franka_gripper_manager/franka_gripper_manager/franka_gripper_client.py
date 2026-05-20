import rclpy
import time
from rclpy.node import Node
from franka_msgs.action import Move
from franka_msgs.action import Homing
from franka_msgs.action import Grasp
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from std_msgs.msg import Float32

DEFAULT_MOVE_ACTION_TOPIC = "franka_gripper/move"
DEFAULT_GRASP_ACTION_TOPIC = "franka_gripper/grasp"
DEFAULT_HOMING_ACTION_TOPIC = "franka_gripper/homing"
DEFAULT_JOINT_STATES_TOPIC = "franka_gripper/joint_states"
DEFAULT_GRIPPER_COMMAND_TOPIC = "gripper/gripper_client/target_gripper_width_percent"


class GripperClient(Node):
    def __init__(self):
        super().__init__("gripper_client")

        self.declare_parameter("move_action_topic", DEFAULT_MOVE_ACTION_TOPIC)
        self.declare_parameter("grasp_action_topic", DEFAULT_GRASP_ACTION_TOPIC)
        self.declare_parameter("homing_action_topic", DEFAULT_HOMING_ACTION_TOPIC)
        self.declare_parameter("gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC)
        self.declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC)

        move_action_topic = (
            self.get_parameter("move_action_topic").get_parameter_value().string_value
        )
        grasp_action_topic = (
            self.get_parameter("grasp_action_topic").get_parameter_value().string_value
        )
        homing_action_topic = (
            self.get_parameter("homing_action_topic").get_parameter_value().string_value
        )
        gripper_command_topic = (
            self.get_parameter("gripper_command_topic").get_parameter_value().string_value
        )
        joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )

        self._ACTION_SERVER_TIMEOUT = 10.0
        self._MIN_GRIPPER_WIDTH_PERCENT = 0.0
        self._MAX_GRIPPER_WIDTH_PERCENT = 1.0
        self._PRESS_THRESHOLD = 0.2
        self._TOGGLE_DEBOUNCE_SEC = 0.35
        self._DEFAULT_GRASP_FORCE = 20.0
        self._DEFAULT_GRASP_SPEED = 1.0
        self._DEFAULT_GRASP_EPSILON = 0.005
        self._gripper_command_transmitted = True
        self._gripper_closed = False
        self._last_toggle_time = 0.0
        self._max_width = 0.0

        self.get_logger().info("Initializing gripper client...")
        self._home_gripper(homing_action_topic)
        self._get_max_gripper_width(joint_states_topic)

        self.get_logger().info("Subscribing to gripper commands...")
        self._gripper_command_subscription = self.create_subscription(
            Float32, gripper_command_topic, self._gripper_command_callback, 10
        )
        self._move_action_client = ActionClient(self, Move, move_action_topic)
        self._grasp_action_client = ActionClient(self, Grasp, grasp_action_topic)

        self.get_logger().info("Waiting for gripper move action server...")
        if not self._move_action_client.wait_for_server(timeout_sec=self._ACTION_SERVER_TIMEOUT):
            raise RuntimeError(
                f"Move action server not available after {self._ACTION_SERVER_TIMEOUT} seconds!"
            )
        self.get_logger().info("Waiting for gripper grasp action server...")
        if not self._grasp_action_client.wait_for_server(timeout_sec=self._ACTION_SERVER_TIMEOUT):
            raise RuntimeError(
                f"Grasp action server not available after {self._ACTION_SERVER_TIMEOUT} seconds!"
            )

        self.get_logger().info("Gripper client initialized!")

    def _home_gripper(self, homing_action_topic: str) -> None:
        self.get_logger().info("Starting gripper homing...")
        homing_client = ActionClient(self, Homing, homing_action_topic)

        self.get_logger().info(f"Waiting for homing action server {homing_action_topic}...")
        if not homing_client.wait_for_server(timeout_sec=self._ACTION_SERVER_TIMEOUT):
            raise RuntimeError(
                f"Homing action server not available after {self._ACTION_SERVER_TIMEOUT} seconds!"
            )

        self.get_logger().info("Homing action server found!")
        goal_msg = Homing.Goal()
        future = homing_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Homing action rejected!")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        time.sleep(2)

        if result.result.success:
            self.get_logger().info("Gripper homing successful!")
        else:
            raise RuntimeError("Gripper homing failed!")

    def _get_max_gripper_width(self, joint_states_topic: str) -> None:
        self.get_logger().info("Readout maximum gripper width...")
        future = rclpy.task.Future()

        def joint_state_callback(msg):
            _INDEX_FINGER_LEFT = 0
            self._max_width = 2 * msg.position[_INDEX_FINGER_LEFT]
            self.get_logger().info(f"Maximum gripper width determined: {self._max_width}")
            future.set_result(True)

        self.get_logger().info(f"Subscribing to {joint_states_topic}...")
        gripper_subscription = self.create_subscription(
            JointState, joint_states_topic, joint_state_callback, 10
        )

        self.get_logger().info(f"Waiting for {joint_states_topic}...")
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(f"Unsubscribing from {joint_states_topic}")
        self.destroy_subscription(gripper_subscription)

    def _gripper_command_callback(self, msg: Float32) -> None:
        open_width_percent = max(
            self._MIN_GRIPPER_WIDTH_PERCENT, min(self._MAX_GRIPPER_WIDTH_PERCENT, msg.data)
        )

        # Toggle behavior:
        # - First press closes with Grasp
        # - Second press opens with Move
        # A press is registered if the trigger is below threshold and debounce time has elapsed.
        now = time.monotonic()
        if (
            open_width_percent <= self._PRESS_THRESHOLD
            and (now - self._last_toggle_time) >= self._TOGGLE_DEBOUNCE_SEC
        ):
            self._last_toggle_time = now
            self._toggle_gripper()

    def _toggle_gripper(self) -> None:
        if not self._gripper_command_transmitted:
            self.get_logger().warning("Ignoring toggle while previous gripper command is active.")
            return

        self._gripper_command_transmitted = False
        if self._gripper_closed:
            self._send_move_command(self._max_width)
            self._gripper_closed = False
        else:
            self._send_grasp_command(0.0)
            self._gripper_closed = True

    def _send_move_command(self, gripper_position: float) -> None:
        goal_msg = Move.Goal()
        goal_msg.width = gripper_position
        goal_msg.speed = self._DEFAULT_GRASP_SPEED
        self._future = self._move_action_client.send_goal_async(goal_msg)
        self._future.add_done_callback(self._gripper_response_callback)

    def _send_grasp_command(self, gripper_position: float) -> None:
        goal_msg = Grasp.Goal()
        goal_msg.width = gripper_position
        goal_msg.speed = self._DEFAULT_GRASP_SPEED
        goal_msg.force = self._DEFAULT_GRASP_FORCE
        goal_msg.epsilon.inner = self._DEFAULT_GRASP_EPSILON
        goal_msg.epsilon.outer = self._DEFAULT_GRASP_EPSILON
        self._future = self._grasp_action_client.send_goal_async(goal_msg)
        self._future.add_done_callback(self._gripper_response_callback)

    def _gripper_response_callback(self, future: rclpy.task.Future) -> None:
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._gripper_command_transmitted = True
            raise RuntimeError(f"Goal rejected with status: {goal_handle.status}")

        # Allow the next toggle as soon as the goal is accepted.
        # Waiting for the final result can block opening after a successful grasp.
        self._gripper_command_transmitted = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: rclpy.task.Future) -> None:
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result))


def main(args=None):
    rclpy.init(args=args)
    gripper_client = GripperClient()
    rclpy.spin(gripper_client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
