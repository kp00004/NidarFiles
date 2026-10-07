"""Subscribes to /gcs/command (the operator's ONLY two possible actions)
and republishes a sanitized copy internally. This node's entire job is to
be the one place in the system that decides whether an incoming message
counts as a real command -- see Hard Safety Rule 5 in this repo's
CLAUDE.md. It does not itself decide what "start"/"abort" *do*; that's
mission_state_node's job, and it only trusts this node's output, never
the raw /gcs/command topic.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .topics import COMMAND_TOPIC, VALID_COMMANDS, VALIDATED_COMMAND_TOPIC


class CommandNode(Node):
    def __init__(self) -> None:
        super().__init__("command_node")
        self._publisher = self.create_publisher(String, VALIDATED_COMMAND_TOPIC, 10)
        self._subscription = self.create_subscription(
            String, COMMAND_TOPIC, self._on_command, 10
        )
        self.get_logger().info(f"Listening for commands on {COMMAND_TOPIC}")

    def _on_command(self, msg: String) -> None:
        value = msg.data
        if value not in VALID_COMMANDS:
            self.get_logger().warning(
                f"Rejected message on {COMMAND_TOPIC}: {value!r} is not a valid "
                f"command (valid: {VALID_COMMANDS}). Ignoring."
            )
            return

        self.get_logger().info(f"Received valid command: {value!r}")
        self._publisher.publish(String(data=value))


def main() -> None:
    rclpy.init()
    node = CommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
