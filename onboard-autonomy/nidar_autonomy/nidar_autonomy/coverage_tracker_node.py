"""Coverage grid -- rclpy node wrapper around coverage_grid.CoverageGrid.

Migrated/adapted from gps_denied/raj-dev's coverage_tracker.py (NIDAR
Autonomy Migration, see CHECKPOINT/CURRENT_STATE.md). The actual
classification/visibility algorithm lives in coverage_grid.py (pure, unit
tested); this node is only the ROS plumbing: TF lookup for the vehicle's
pose, subscribing MAP_TOPIC, and publishing COVERAGE_GRID_TOPIC /
COVERAGE_PERCENT_TOPIC.

HARDWARE/SLAM STATUS: this node requires a real `map -> base_link` TF and a
real MAP_TOPIC publisher, neither of which exists on this Jetson yet (no
LiDAR mounted, no Cartographer/SLAM installed -- AUTONOMY_ROADMAP.md Phase
4/5 are NOT STARTED). This node builds and imports cleanly and is ready to
run once that upstream exists; until then it simply sits idle (no TF, no
map -> no update() calls, same pattern coverage_grid.py's own tests exercise
directly). See the migration report for the full hardware-blocked list --
do not read "this file exists" as "coverage tracking is working."

READ-ONLY as far as flight goes: this node never touches mavros or issues
any flight command.
"""
from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from .coverage_grid import CoverageGrid
from .geometry import yaw_from_quaternion
from .ros_conversions import dict_to_occupancy_grid_msg, occupancy_grid_msg_to_dict
from .topics import COVERAGE_GRID_TOPIC, COVERAGE_PERCENT_TOPIC, MAP_TOPIC


class CoverageTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("coverage_tracker")
        # Arena bounds + cell size define the mission grid. Cell size is a
        # parameter on purpose (the Rulebook doesn't fix it) -- see
        # coverage_grid.py's module docstring. Defaults are a placeholder;
        # set to the real arena bounds + confirmed cell size once known.
        self.declare_parameter("arena_min_x", -7.5)
        self.declare_parameter("arena_max_x", 7.5)
        self.declare_parameter("arena_min_y", -7.5)
        self.declare_parameter("arena_max_y", 7.5)
        self.declare_parameter("cell_size", 1.0)
        # Camera model used to decide when a cell counts as SEARCHED. MUST
        # be set to the real camera's usable human-detection range/FOV once
        # chosen (AUTONOMY_ROADMAP.md Phase 10) -- not the lens's raw spec.
        self.declare_parameter("camera_range", 2.0)
        self.declare_parameter("camera_fov_deg", 100.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("update_rate_hz", 4.0)
        # Occupancy thresholds -- a real SLAM stack (e.g. Cartographer)
        # reports free space as a probability range, not always exactly 0.
        self.declare_parameter("free_below", 50)
        self.declare_parameter("occupied_at", 65)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.map_frame = get("map_frame")
        self.body_frame = get("body_frame")

        self.grid = CoverageGrid(
            min_x=get("arena_min_x"),
            max_x=get("arena_max_x"),
            min_y=get("arena_min_y"),
            max_y=get("arena_max_y"),
            cell_size=get("cell_size"),
            camera_range_m=get("camera_range"),
            camera_fov_deg=get("camera_fov_deg"),
            free_below=get("free_below"),
            occupied_at=get("occupied_at"),
        )

        self.map_msg: OccupancyGrid | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, map_qos)
        self.grid_pub = self.create_publisher(OccupancyGrid, COVERAGE_GRID_TOPIC, latched)
        self.pct_pub = self.create_publisher(Float32, COVERAGE_PERCENT_TOPIC, latched)

        self._tick_count = 0
        self.create_timer(1.0 / get("update_rate_hz"), self._tick)
        self.get_logger().info(
            f"coverage_tracker: {self.grid.ncols}x{self.grid.nrows} cells @ "
            f"{self.grid.cell_size}m over x[{self.grid.min_x},{self.grid.max_x}] "
            f"y[{self.grid.min_y},{self.grid.max_y}], "
            f"camera {get('camera_fov_deg'):.0f}deg / {get('camera_range')}m"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _tick(self) -> None:
        if self.map_msg is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.body_frame, Time())
        except TransformException:
            return
        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        dyaw = yaw_from_quaternion(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )
        grid_dict = occupancy_grid_msg_to_dict(self.map_msg)
        free_total, searched_total = self.grid.update(grid_dict, dx, dy, dyaw)
        self._publish(free_total, searched_total)

    def _publish(self, free_total: int, searched_total: int) -> None:
        out = self.grid.to_occupancy_grid_data()
        out["header"]["frame_id"] = self.map_frame
        msg = dict_to_occupancy_grid_msg(out, stamp=self.get_clock().now().to_msg())
        self.grid_pub.publish(msg)

        pct = (100.0 * searched_total / free_total) if free_total else 0.0
        self.pct_pub.publish(Float32(data=pct))
        self._tick_count += 1
        if self._tick_count % 8 == 0:  # ~ every 2s at 4Hz
            self.get_logger().info(
                f"coverage: {searched_total}/{free_total} free cells searched ({pct:.0f}%)"
            )


def main() -> None:
    rclpy.init()
    node = CoverageTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
