"""Frontier exploration -- rclpy node wrapper around frontier_detector.py +
exploration_policy.py + grid_astar_planner.py.

Migrated/adapted from gps_denied/raj-dev's frontier_explorer.py (NIDAR
Autonomy Migration, see CHECKPOINT/CURRENT_STATE.md). The source file did
frontier DETECTION, target SELECTION, and PATH REQUESTING all inline,
coupled to a Nav2 `ComputePathToPose` action client (Nav2 is not installed
on this Jetson -- confirmed absent, see the migration report). This node
instead composes three already-canonical, independently-tested onboard-
autonomy modules:

    /map (OccupancyGrid)
        |
        v
    frontier_detector.detect_frontiers()   <- AUTONOMY_ROADMAP.md Phase 6,
        |                                      already canonical, NOT
        |                                      replaced by this migration
        v
    exploration_policy.select_target()     <- migrated selection policy
        |                                      (hysteresis/visited/blacklist)
        v
    occupancy_grid_environment.OccupancyGridEnvironment
        + grid_astar_planner.GridAStarPlanner.plan()   <- AUTONOMY_ROADMAP.md
        |                                                  Phase 7, already
        |                                                  canonical
        v
    /planned_path (nav_msgs/Path)

This node NEVER touches mavros, never issues a flight command, and never
publishes `/mission/state` or `/mission/complete` -- see the migration
report's "one mission owner" section: `mission_state_node.py` remains the
sole mission authority. This node answers "what should we explore next" and
publishes that as a *suggestion* (`/planned_path`, `/explorer/status`) for a
future mission manager (AUTONOMY_ROADMAP.md Phase 9) to act on -- it does not
decide whether the vehicle actually flies anywhere.

NOT PORTED FROM THE SOURCE (explicitly, not an oversight): gps_denied's
`_tick()` also had goal-timeout/stuck-detection abandonment, a coverage-
percentage return-to-entry commit latch, and boundary-phantom-frontier
filtering -- all flight-tuned against that repo's specific simulated maze
runs. Porting untunable, unvalidated thresholds here and presenting them as
working would be worse than not having them; they are not included. This
node's "exploration done" signal is deliberately just "no reachable frontier
right now" -- observational telemetry, not a mission-completion decision.

HARDWARE/SLAM STATUS: like coverage_tracker_node.py, this node requires a
real `map -> base_link` TF and a real `/map` publisher, neither of which
exists on this Jetson yet (AUTONOMY_ROADMAP.md Phase 4/5 NOT STARTED). It
builds and imports cleanly and is ready to run once that upstream exists.
"""
from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .exploration_policy import apply_sweep_radius, record_failure, record_visit, score_candidates
from .frontier_detector import detect_frontiers
from .grid_astar_planner import GridAStarPlanner
from .occupancy_grid_environment import OccupancyGridEnvironment
from .path_planner_interface import PathPlanningError
from .ros_conversions import occupancy_grid_msg_to_dict
from .topics import EXPLORER_STATUS_TOPIC, FRONTIERS_TOPIC, MAP_TOPIC, PLANNED_PATH_TOPIC


class FrontierExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        self.declare_parameter("min_frontier_cells", 8)
        self.declare_parameter("min_goal_distance", 0.7)
        self.declare_parameter("info_weight", 0.4)
        self.declare_parameter("sweep_radius", 5.0)
        self.declare_parameter("hysteresis_bonus", 2.0)
        self.declare_parameter("blacklist_after_failures", 3)
        self.declare_parameter("visit_cell_size", 0.75)
        self.declare_parameter("visit_weight", 0.6)
        self.declare_parameter("visit_radius", 1.5)
        self.declare_parameter("replan_period_sec", 2.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("body_frame", "base_link")

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.min_frontier_cells = get("min_frontier_cells")
        self.min_goal_distance = get("min_goal_distance")
        self.info_weight = get("info_weight")
        self.sweep_radius = get("sweep_radius")
        self.hysteresis_bonus = get("hysteresis_bonus")
        self.max_failures = get("blacklist_after_failures")
        self.visit_cell_size = get("visit_cell_size")
        self.visit_weight = get("visit_weight")
        self.visit_radius = get("visit_radius")
        self.map_frame = get("map_frame")
        self.body_frame = get("body_frame")

        self.map_msg: OccupancyGrid | None = None
        self.pose_xy: tuple[float, float] | None = None
        self.goal: tuple[float, float] | None = None
        self.visited: dict[tuple[int, int], int] = {}
        self.blacklist: dict[tuple[float, float], int] = {}
        self.planner = GridAStarPlanner()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, FRONTIERS_TOPIC, 5)
        self.path_pub = self.create_publisher(Path, PLANNED_PATH_TOPIC, 10)
        self.status_pub = self.create_publisher(String, EXPLORER_STATUS_TOPIC, 5)

        self._last_frontier_count = 0
        self._last_candidate_count = 0

        self.create_timer(get("replan_period_sec"), self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "frontier_explorer: detection=frontier_detector.py, "
            "selection=exploration_policy.py, planning=grid_astar_planner.py "
            "(no Nav2 -- not installed on this Jetson), waiting for /map and TF..."
        )

    # ---------------------------------------------------------------- inputs
    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _read_pose(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.body_frame, Time())
        except TransformException:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        self.pose_xy = (x, y)
        record_visit(self.visited, x, y, self.visit_cell_size)

    # ------------------------------------------------------------- main loop
    def _tick(self) -> None:
        self._read_pose()
        if self.map_msg is None or self.pose_xy is None:
            return

        grid_dict = occupancy_grid_msg_to_dict(self.map_msg)
        try:
            candidates = detect_frontiers(grid_dict)
        except Exception as exc:  # malformed grid -- log, don't crash the node
            self.get_logger().warning(f"detect_frontiers failed: {exc}")
            return
        candidates = [c for c in candidates if c.cell_count >= self.min_frontier_cells]
        self._last_frontier_count = len(candidates)
        self._publish_markers(candidates)

        scored = score_candidates(
            candidates,
            pose=self.pose_xy,
            current_goal=self.goal,
            visited=self.visited,
            blacklist=self.blacklist,
            min_goal_distance=self.min_goal_distance,
            info_weight=self.info_weight,
            hysteresis_bonus=self.hysteresis_bonus,
            visit_cell_size=self.visit_cell_size,
            visit_weight=self.visit_weight,
            visit_radius=self.visit_radius,
            max_failures=self.max_failures,
        )
        self._last_candidate_count = len(scored)
        if not scored:
            self.goal = None
            return
        target = apply_sweep_radius(scored, self.sweep_radius)[0]

        self.goal = (target.x, target.y)
        try:
            env = OccupancyGridEnvironment(grid_dict)
            waypoints = self.planner.plan(self.pose_xy, self.goal, env)
        except PathPlanningError as exc:
            self.get_logger().warning(
                f"no path to candidate goal ({self.goal[0]:.2f}, {self.goal[1]:.2f}): {exc}"
            )
            record_failure(self.blacklist, self.goal[0], self.goal[1])
            self.goal = None
            return

        self._publish_path(waypoints)

    # ------------------------------------------------------------ publishing
    def _publish_markers(self, candidates) -> None:
        arr = MarkerArray()
        for i, c in enumerate(candidates):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = "frontiers", i, Marker.SPHERE, Marker.ADD
            m.pose = Pose()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = c.x, c.y, 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.9, 0.8
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _publish_path(self, waypoints: list[tuple[float, float]]) -> None:
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _publish_status(self) -> None:
        """Read-only observer timer -- reports current state for the GCS via
        telemetry_bridge_node.py. Deliberately reads only already-computed
        instance attributes and makes no decisions, so it cannot affect
        exploration behaviour, same pattern as the migration source."""
        if self.map_msg is None or self.pose_xy is None:
            state = "waiting"
        elif self._last_frontier_count == 0:
            state = "done"
        else:
            state = "exploring"
        msg = String()
        msg.data = json.dumps(
            {
                "state": state,
                "target": list(self.goal) if self.goal is not None else None,
                "frontier_count": self._last_frontier_count,
                "candidate_count": self._last_candidate_count,
                "blacklisted_count": len(self.blacklist),
            }
        )
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
