"""Local-frame geofence breach detector -- rclpy node.

Migrated from gps_denied/raj-dev's geofence_monitor.py (NIDAR Autonomy
Migration, see CHECKPOINT/CURRENT_STATE.md). ArduCopter's own native
geofence is GPS lat/lon based, which is useless indoors (no GPS at the
venue, forbidden by the competition rules -- see
custom-gcs/docs/REQUIREMENTS.md). This node is a local-coordinate substitute:
it watches the vehicle's map-frame position (fed by a future SLAM->EKF
localization pipeline, AUTONOMY_ROADMAP.md Phase 4) against a bounding box
sized to the arena, and publishes a breach flag -- the "geofence breach"
failsafe the mission rules require (Rulebook Section 10), implemented in
local coordinates since GPS doesn't exist here.

READ-ONLY OBSERVER: this node only watches telemetry and publishes a
boolean. It never arms, disarms, changes mode, or issues a flight command --
see this repo's CLAUDE.md Hard Safety Rule 1. Consuming this signal to
actually do something (e.g. forcing a ground abort) is a `mission_state_node`
concern, deliberately not this node's -- see the migration report's "one
mission owner" section.

Not yet wired into `mission_state_node.py`'s abort path -- this is
implemented and tested, but deploying it as a real safety trigger is a
separate step pending safety_reviewer sign-off (see the migration report).
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool

from .topics import GEOFENCE_BREACH_TOPIC, VISION_POSE_TOPIC


class GeofenceMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("geofence_monitor")
        # Arena rectangle in the map frame. Defaults are a placeholder --
        # must be set to the real arena bounds (per-venue) via ROS params
        # before this node's output is trusted; see docs/gcs_telemetry_contract.md
        # and AUTONOMY_ROADMAP.md Phase 4/5 for how the real arena bounds
        # get established once real SLAM/mapping exists.
        self.declare_parameter("arena_min_x", -7.5)
        self.declare_parameter("arena_max_x", 7.5)
        self.declare_parameter("arena_min_y", -7.5)
        self.declare_parameter("arena_max_y", 7.5)
        # Margin added outside the arena rectangle before a breach is
        # declared, so localization jitter near a boundary wall doesn't
        # false-trip.
        self.declare_parameter("margin_m", 1.5)
        # Ceiling well below any real arena ceiling, so a climb-out is
        # caught before the vehicle could leave the mapped volume.
        self.declare_parameter("max_height_m", 2.0)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        margin = get("margin_m")
        self.min_x = get("arena_min_x") - margin
        self.max_x = get("arena_max_x") + margin
        self.min_y = get("arena_min_y") - margin
        self.max_y = get("arena_max_y") + margin
        self.max_height = get("max_height_m")

        self.breached = False

        self.create_subscription(PoseStamped, VISION_POSE_TOPIC, self._on_pose, 10)
        self.pub = self.create_publisher(Bool, GEOFENCE_BREACH_TOPIC, 10)

        self.get_logger().info(
            f"geofence_monitor: arena rect x[{self.min_x:.1f},{self.max_x:.1f}] "
            f"y[{self.min_y:.1f},{self.max_y:.1f}] (incl. margin), "
            f"0-{self.max_height}m height"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

        inside = (
            self.min_x <= x <= self.max_x
            and self.min_y <= y <= self.max_y
            and 0.0 <= z <= self.max_height
        )

        if not inside and not self.breached:
            self.breached = True
            self.get_logger().error(
                f"GEOFENCE BREACH: x={x:.2f} y={y:.2f} z={z:.2f} "
                f"(arena x[{self.min_x:.1f},{self.max_x:.1f}] "
                f"y[{self.min_y:.1f},{self.max_y:.1f}], 0-{self.max_height}m)"
            )
        elif inside and self.breached:
            self.breached = False
            self.get_logger().warning("Back inside geofence")

        self.pub.publish(Bool(data=self.breached))


def main() -> None:
    rclpy.init()
    node = GeofenceMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
