"""Publishes /gcs/heartbeat at 1 Hz. Deliberately the simplest node in
this repo -- per custom-gcs/docs/DATA_MODELS.md section 6, this should be
the first thing proven working end-to-end against the real GCS, before
anything else, since it only tells the operator "the Jetson + link is
alive" with no other logic involved.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from .topics import HEARTBEAT_TOPIC

_RATE_HZ = 1.0


class HeartbeatNode(Node):
    def __init__(self) -> None:
        super().__init__("heartbeat_node")
        self._publisher = self.create_publisher(Header, HEARTBEAT_TOPIC, 10)
        self._timer = self.create_timer(1.0 / _RATE_HZ, self._publish_heartbeat)

    def _publish_heartbeat(self) -> None:
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = "nidar_autonomy"
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
