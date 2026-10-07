"""GCS "RUN SIMULATION" -- rclpy node wrapper around mission_simulator.py.

Runs a `MissionSimulator` on a timer and publishes its state to a
completely separate, `/simulation/`-namespaced set of topics -- see
`topics.py`'s "Simulation-only topics" section and
CHECKPOINT/CURRENT_STATE.md for the full design record.

HARD SAFETY BOUNDARY, structural not just behavioral: this node imports
`rclpy`, `std_msgs`, `nav_msgs`, and this repo's own
`mission_simulator`/`telemetry_contract`/`topics`/`ros_conversions`
modules -- and nothing else. It never imports `mavros_msgs`,
`flight_command`, or `arming_guard`, so there is no code path from here
to `/mavros/cmd/arming`, `/mavros/set_mode`, or any real setpoint topic.
It never subscribes to or publishes the real `/gcs/command` or
`/mission/state` topics -- only their `/simulation/`-prefixed
counterparts. `command_node.py`/`mission_state_node.py` and this node can
run side by side on the same ROS graph with zero topic overlap.

The `/simulation/command` channel accepts exactly `"run"` / `"reset"` --
never `"start"`/`"abort"` (those remain the real system's exclusive
vocabulary on the real `/gcs/command` topic, handled by `command_node.py`,
untouched by this file).
"""
from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String

from .mission_simulator import MissionSimulator, SimulationSnapshot
from .ros_conversions import dict_to_occupancy_grid_msg
from .telemetry_contract import autonomy_state, build_contract
from .topics import (
    SIMULATION_COMMAND_TOPIC,
    SIMULATION_COVERAGE_GRID_TOPIC,
    SIMULATION_MAP_TOPIC,
    SIMULATION_MISSION_STATE_TOPIC,
    SIMULATION_PLANNED_PATH_TOPIC,
    SIMULATION_STATUS_TOPIC,
    SIMULATION_TELEMETRY_STATE_TOPIC,
    VALID_SIMULATION_COMMANDS,
)


class SimulationNode(Node):
    def __init__(self, simulator: MissionSimulator | None = None) -> None:
        super().__init__("simulation_node")
        self.declare_parameter("step_hz", 10.0)
        step_hz = self.get_parameter("step_hz").value

        self._simulator = simulator if simulator is not None else MissionSimulator()

        self.create_subscription(String, SIMULATION_COMMAND_TOPIC, self._on_command, 10)
        self._mission_state_pub = self.create_publisher(String, SIMULATION_MISSION_STATE_TOPIC, 10)
        self._status_pub = self.create_publisher(String, SIMULATION_STATUS_TOPIC, 10)
        self._map_pub = self.create_publisher(OccupancyGrid, SIMULATION_MAP_TOPIC, 10)
        self._coverage_pub = self.create_publisher(OccupancyGrid, SIMULATION_COVERAGE_GRID_TOPIC, 10)
        self._path_pub = self.create_publisher(Path, SIMULATION_PLANNED_PATH_TOPIC, 10)
        self._telemetry_pub = self.create_publisher(String, SIMULATION_TELEMETRY_STATE_TOPIC, 10)

        self.create_timer(1.0 / step_hz, self._tick)
        self.get_logger().info(
            f"simulation_node: ready on {SIMULATION_COMMAND_TOPIC} "
            f"('run'/'reset' only -- never the real /gcs/command), stepping @ {step_hz} Hz"
        )

    # ---------------------------------------------------------------- inputs

    def _on_command(self, msg: String) -> None:
        command = msg.data
        if command not in VALID_SIMULATION_COMMANDS:
            self.get_logger().warning(
                f"ignoring invalid simulation command {command!r} -- "
                f"expected one of {VALID_SIMULATION_COMMANDS}"
            )
            return
        if command == "run":
            self.get_logger().info("RUN SIMULATION: (re)starting a fresh simulated mission")
            snapshot = self._simulator.run()
        else:
            self.get_logger().info("simulation reset requested")
            snapshot = self._simulator.reset()
        self._publish(snapshot)

    # --------------------------------------------------------------- ticking

    def _tick(self) -> None:
        snapshot = self._simulator.step()
        self._publish(snapshot)

    # ------------------------------------------------------------ publishing

    def _publish(self, snapshot: SimulationSnapshot) -> None:
        stamp = self.get_clock().now().to_msg()

        self._mission_state_pub.publish(String(data=snapshot.mission_state))
        self._status_pub.publish(String(data=snapshot.status))

        self._map_pub.publish(dict_to_occupancy_grid_msg(snapshot.map, stamp=stamp))
        self._coverage_pub.publish(dict_to_occupancy_grid_msg(snapshot.coverage, stamp=stamp))

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = stamp
        for point in snapshot.planned_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = point["x"]
            pose.pose.position.y = point["y"]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self._path_pub.publish(path_msg)

        self._telemetry_pub.publish(String(data=json.dumps(self._telemetry_contract(snapshot))))

    def _telemetry_contract(self, snapshot: SimulationSnapshot) -> dict:
        """Build the same shape telemetry_contract.build_contract()
        produces for the real system, reusing that exact function and
        autonomy_state() -- but wrapped with an explicit
        `"source": "simulation"` tag, and everything else this dict
        carries also unambiguously says SIMULATION, so it can never be
        mistaken for real Pixhawk telemetry even by a consumer that
        forgot to check the tag (see the module docstring and
        CHECKPOINT/CURRENT_STATE.md)."""
        armed = snapshot.mission_state in ("entering", "searching", "exiting")
        contract = build_contract(
            connected=True,
            heartbeat_age_sec=0.0,
            armed=armed,
            mode="SIMULATION",
            system_status=None,
            battery_pct=None,  # not modeled -- MockFlightController has no battery
            position=snapshot.pose,
            sensors={"slam": "simulated", "lidar": "simulated",
                     "rangefinder": "not_integrated", "camera": "not_integrated"},
            mapping={
                "available": True,
                "resolution_m": snapshot.map.get("info", {}).get("resolution"),
                "width_cells": snapshot.map.get("info", {}).get("width"),
                "height_cells": snapshot.map.get("info", {}).get("height"),
                "origin_x": snapshot.map.get("info", {}).get("origin", {}).get("position", {}).get("x"),
                "origin_y": snapshot.map.get("info", {}).get("origin", {}).get("position", {}).get("y"),
                "coverage_cell_size_m": snapshot.coverage.get("info", {}).get("resolution"),
                # Deliberately the true map-completeness fraction, not
                # coverage.py's search-coverage percent (see
                # SimulationSnapshot.map_known_pct's own docstring for
                # why those two differ and which one answers "how much
                # of the map is done").
                "explored_pct": snapshot.map_known_pct,
            },
            navigation={
                "target": snapshot.target,
                "frontier_count": snapshot.frontier_count,
                "candidate_count": snapshot.candidate_count,
                "blacklisted_count": snapshot.blacklisted_count,
                "path_progress": None,
                "brake_active": None,
                "escaping_local_minimum": None,
                "map_local_offset_m": 0.0,
                "geofence_breached": False,
            },
            autonomy=autonomy_state(snapshot.mission_state, None, snapshot.target),
            mission={
                "state": snapshot.mission_state,
                "elapsed_sec": snapshot.elapsed_sim_seconds,
                "complete": snapshot.mission_state == "complete",
            },
            survivors=[],
        )
        return {
            "source": "simulation",
            "simulation_status": snapshot.status,
            "simulation_step": snapshot.step,
            "coverage_search_pct": snapshot.explored_pct,
            "error": snapshot.error,
            **contract,
        }


def main() -> None:
    rclpy.init()
    node = SimulationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
