"""GCS telemetry bridge -- rclpy node.

Migrated/adapted from gps_denied/raj-dev's telemetry_bridge.py (NIDAR
Autonomy Migration, see CHECKPOINT/CURRENT_STATE.md). The normalized-contract
building logic lives in telemetry_contract.py (pure, unit tested); this node
is only the ROS plumbing around it, same "pure logic + thin node" split as
state_machine.py/mission_state_node.py.

WHY THIS EXISTS
================
custom-gcs already visualizes vehicle/mission state via `/api/telemetry`.
The next step is letting it visualize the mapping/exploration/planning
stack -- map, drone pose, path, target, coverage, sensor health -- without
custom-gcs's backend needing to know this repo's topic names or internal
state-machine details. This node collects raw ROS state from whichever
mapping/exploration nodes are running and republishes it as one versioned,
JSON-encoded std_msgs/String on TELEMETRY_STATE_TOPIC
("/telemetry/state") -- rosbridge_suite forwards std_msgs/String to the
browser with zero extra plumbing.

WHAT THIS NODE DELIBERATELY DOES NOT DO
=========================================
It does NOT republish the occupancy grid's cell data, the full planned
path, or raw /scan ranges into this JSON blob -- those already have
well-formed ROS message types (nav_msgs/OccupancyGrid, nav_msgs/Path,
sensor_msgs/LaserScan) that rosbridge forwards natively and efficiently.
This node only extracts lightweight SUMMARIES from those messages.

It also makes NO decisions and sends NO commands: this is a read-only
observer, matching "the GCS is not the autonomy brain" (see
CHECKPOINT/INTEGRATION_CHECKPOINTS.md's architectural principle). It cannot
affect flight behaviour even if it crashes or lags. It never touches
mavros's arming service -- see this repo's CLAUDE.md Hard Safety Rule 1;
only flight_command.py is permitted to do that.

ADAPTATION NOTES FROM THE SOURCE
==================================
* `/mission/state` here is `onboard-autonomy`'s own canonical publisher
  (`mission_state_node.py`/`state_machine.py`) -- gps_denied's competing
  `mission_fsm.py` publisher of the same topic name was explicitly NOT
  migrated (see the migration report's "one mission owner" section).
* `/follower/status`-derived navigation fields (path_progress, brake_active,
  escaping_local_minimum, map_local_offset_m) are always None here --
  gps_denied's `path_follower_position.py` (the source of those fields) is
  flight-control-issuing code and was explicitly NOT migrated (see the
  migration report). These fields are extension points for whenever a
  canonical, sign-off-reviewed path follower exists.
* `/gcs/heartbeat` connection-liveness monitoring is NOT wired here.
  gps_denied's version assumed a GCS-side publisher on this topic name
  (C2-link-alive signal, `gcs_heartbeat.py`, not migrated -- see the
  migration report); this repo's own `heartbeat_node.py` already publishes
  the SAME topic name in the opposite direction (drone -> GCS). Subscribing
  here would just read back this node's own drone-side heartbeat, which is
  not a meaningful "is the operator link alive" signal -- so
  `connection.heartbeat_age_sec` is always None until that conflict is
  resolved with a distinctly-named topic (flagged, not silently faked).
"""
from __future__ import annotations

import json
import math
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float32, String

from .geometry import yaw_from_quaternion
from .telemetry_contract import autonomy_state, build_contract, stale
from .topics import (
    BATTERY_TOPIC,
    COVERAGE_GRID_TOPIC,
    COVERAGE_PERCENT_TOPIC,
    EXPLORER_STATUS_TOPIC,
    FCU_STATE_TOPIC,
    GEOFENCE_BREACH_TOPIC,
    MAP_TOPIC,
    MISSION_STATE_TOPIC,
    SCAN_TOPIC,
    SLAM_OK_TOPIC,
    TELEMETRY_STATE_TOPIC,
    VISION_POSE_TOPIC,
)


class TelemetryBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_bridge")
        self.declare_parameter("publish_rate_hz", 2.0)
        # How old a topic's last message may be before this bridge reports
        # it as disconnected/stale to the GCS. Independent of
        # mission_state_node's own (safety-critical) staleness handling --
        # this is purely for display.
        self.declare_parameter("topic_timeout_sec", 3.0)
        rate_hz = self.get_parameter("publish_rate_hz").value
        self.timeout = self.get_parameter("topic_timeout_sec").value

        # --- raw state, updated by subscriptions, read by _tick ---
        self.mavros_state: Optional[State] = None
        self.battery: Optional[BatteryState] = None
        self.pose: Optional[PoseStamped] = None
        self._prev_pose: Optional[PoseStamped] = None
        self.slam_ok: Optional[bool] = None
        self.map_info = None
        self.coverage_info = None
        self.coverage_pct: Optional[float] = None
        self.mission_state: Optional[str] = None
        self.explorer_status: dict[str, Any] = {}
        self.geofence_breached = False
        self.mission_complete = False
        self._scan_last = None  # rclpy.time.Time
        self._mission_start = None  # rclpy.time.Time, set on first pose

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5
        )
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(State, FCU_STATE_TOPIC, self._on_state, sensor_qos)
        self.create_subscription(BatteryState, BATTERY_TOPIC, self._on_battery, sensor_qos)
        self.create_subscription(PoseStamped, VISION_POSE_TOPIC, self._on_pose, 10)
        self.create_subscription(Bool, SLAM_OK_TOPIC, self._on_slam_ok, latched)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, latched)
        self.create_subscription(OccupancyGrid, COVERAGE_GRID_TOPIC, self._on_coverage_grid, latched)
        self.create_subscription(Float32, COVERAGE_PERCENT_TOPIC, self._on_coverage_pct, latched)
        self.create_subscription(String, MISSION_STATE_TOPIC, self._on_mission_state, 10)
        self.create_subscription(String, EXPLORER_STATUS_TOPIC, self._on_explorer_status, 10)
        self.create_subscription(Bool, GEOFENCE_BREACH_TOPIC, self._on_geofence, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, sensor_qos)

        self.pub = self.create_publisher(String, TELEMETRY_STATE_TOPIC, 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"telemetry_bridge: normalizing autonomy state -> {TELEMETRY_STATE_TOPIC} "
            f"@ {rate_hz} Hz (see CHECKPOINT/docs/gcs_telemetry_contract.md)"
        )

    # ---------------------------------------------------------------- inputs
    def _on_state(self, msg: State) -> None:
        self.mavros_state = msg

    def _on_battery(self, msg: BatteryState) -> None:
        self.battery = msg

    def _on_slam_ok(self, msg: Bool) -> None:
        self.slam_ok = msg.data

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_info = msg.info

    def _on_coverage_grid(self, msg: OccupancyGrid) -> None:
        self.coverage_info = msg.info

    def _on_coverage_pct(self, msg: Float32) -> None:
        self.coverage_pct = msg.data

    def _on_mission_state(self, msg: String) -> None:
        self.mission_state = msg.data

    def _on_geofence(self, msg: Bool) -> None:
        self.geofence_breached = msg.data

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan_last = self.get_clock().now()

    def _on_pose(self, msg: PoseStamped) -> None:
        self._prev_pose = self.pose
        self.pose = msg
        if self._mission_start is None:
            self._mission_start = self.get_clock().now()

    def _on_explorer_status(self, msg: String) -> None:
        try:
            self.explorer_status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("malformed explorer status payload, ignoring")

    # ------------------------------------------------------------- building
    def _position_block(self) -> dict[str, Any]:
        if self.pose is None:
            return {"x": None, "y": None, "z": None, "yaw_deg": None, "velocity_mps": None}
        p = self.pose.pose.position
        yaw = yaw_from_quaternion(
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w,
        )
        vel = None
        if self._prev_pose is not None:
            dt = (
                (self.pose.header.stamp.sec - self._prev_pose.header.stamp.sec)
                + (self.pose.header.stamp.nanosec - self._prev_pose.header.stamp.nanosec) / 1e9
            )
            if dt > 1e-3:
                pp = self._prev_pose.pose.position
                vel = math.dist((p.x, p.y, p.z), (pp.x, pp.y, pp.z)) / dt
        return {"x": p.x, "y": p.y, "z": p.z, "yaw_deg": math.degrees(yaw), "velocity_mps": vel}

    def _mapping_block(self) -> dict[str, Any]:
        info = self.map_info
        if info is None:
            return {"available": False}
        return {
            "available": True,
            "resolution_m": info.resolution,
            "width_cells": info.width,
            "height_cells": info.height,
            "origin_x": info.origin.position.x,
            "origin_y": info.origin.position.y,
            "coverage_cell_size_m": self.coverage_info.resolution if self.coverage_info else None,
            "explored_pct": self.coverage_pct,
        }

    def _sensors_block(self, now) -> dict[str, Any]:
        return {
            "slam": "ok" if self.slam_ok else ("lost" if self.slam_ok is False else "unknown"),
            "lidar": "ok" if not self._stale(self._scan_last, now) else "stale",
            # Extension points -- no rangefinder/camera integration exists
            # yet. Report explicitly unknown rather than fabricating a
            # reading.
            "rangefinder": "not_integrated",
            "camera": "not_integrated",
        }

    def _navigation_block(self) -> dict[str, Any]:
        return {
            "target": self.explorer_status.get("target"),
            "frontier_count": self.explorer_status.get("frontier_count"),
            "candidate_count": self.explorer_status.get("candidate_count"),
            "blacklisted_count": self.explorer_status.get("blacklisted_count"),
            # No canonical, sign-off-reviewed path follower exists yet (see
            # this module's docstring) -- these stay None, not fabricated.
            "path_progress": None,
            "brake_active": None,
            "escaping_local_minimum": None,
            "map_local_offset_m": None,
            "geofence_breached": self.geofence_breached,
        }

    def _stale(self, last_seen, now) -> bool:
        return stale(
            None if last_seen is None else last_seen.nanoseconds / 1e9,
            now.nanoseconds / 1e9,
            self.timeout,
        )

    def _tick(self) -> None:
        now = self.get_clock().now()
        st = self.mavros_state
        connected = bool(st.connected) if st is not None else False
        elapsed = (
            None if self._mission_start is None else (now - self._mission_start).nanoseconds / 1e9
        )

        contract = build_contract(
            connected=connected,
            # See module docstring: C2/GCS-heartbeat liveness isn't wired
            # to a meaningful signal on this Jetson yet.
            heartbeat_age_sec=None,
            armed=bool(st.armed) if st is not None else False,
            mode=st.mode if st is not None else None,
            system_status=st.system_status if st is not None else None,
            battery_pct=(
                self.battery.percentage * 100.0
                if self.battery is not None
                and self.battery.percentage is not None
                and 0.0 <= self.battery.percentage <= 1.0
                else None
            ),
            position=self._position_block(),
            sensors=self._sensors_block(now),
            mapping=self._mapping_block(),
            navigation=self._navigation_block(),
            autonomy=autonomy_state(
                self.mission_state,
                self.explorer_status.get("state"),
                self.explorer_status.get("target"),
            ),
            mission={
                "state": self.mission_state,
                "elapsed_sec": elapsed,
                "complete": self.mission_complete,
            },
            survivors=[],
        )
        out = String()
        out.data = json.dumps(contract)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = TelemetryBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
