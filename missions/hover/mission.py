#!/usr/bin/env python3
"""NIDAR hover mission -- ROS 2 node, runs on the Jetson.

    radio START (hover) -> radio_command_node -> /gcs/mission_select + /gcs/command
      -> command_node (validation) -> /nidar_autonomy/validated_command
      -> THIS NODE -> ArduCopterVehicle -> MAVROS -> Pixhawk 6X (ArduCopter 4.6.3)

This is the ONE authoritative execution path for the hover: do not run
onboard-autonomy's mission_state_node at the same time (it would arm on
the same START by itself) -- START is refused while it is running.

Decisions live in hover_logic.py (pure, unit-tested). This file only wires
it to ROS: it feeds VehicleSnapshots in at 10 Hz, executes the returned
actions through onboard-autonomy's ArduCopterVehicle (arm/disarm via the
existing FlightCommandClient), and publishes the mission status on
/flight_test/status, which the GCS already displays.

Run (after sourcing ROS 2 Humble and ~/ros2_ws/install/setup.bash):

    python3 missions/hover/mission.py
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from hover_logic import HoverConfig, HoverMission  # noqa: E402
from nidar_autonomy.ardupilot_vehicle import ArduCopterVehicle  # noqa: E402
from nidar_autonomy.flight_command import FlightCommandClient  # noqa: E402
from nidar_autonomy.topics import (  # noqa: E402
    HOVER_STATUS_TOPIC,
    MISSION_SELECT_TOPIC,
    VALIDATED_COMMAND_TOPIC,
)

MISSION_ID = "hover"

# ---------------------------------------------------------------------------
# Flight parameters for the hover. Edit here.
# TODO(hardware): review every value against the real test space and the
# installed sensors before the first flight:
#   - takeoff_altitude_m must be inside the downward rangefinder's reliable
#     range (many have a minimum range of 0.1-0.3 m) and well below the
#     ceiling/net.
#   - max_horizontal_drift_m must fit inside the test area.
#   - min_battery_voltage_v: set for the flight battery (e.g. per-cell
#     minimum x cell count). None disables the check -- only acceptable on
#     the bench without a power module.
# ---------------------------------------------------------------------------
CONFIG = HoverConfig(
    takeoff_altitude_m=0.5,
    hold_duration_s=10.0,
    altitude_reached_fraction=0.8,
    climb_timeout_s=15.0,
    max_altitude_margin_m=0.5,
    max_horizontal_drift_m=0.75,
    position_stale_s=1.0,
    min_battery_voltage_v=None,
)

# A START must follow a /gcs/mission_select naming this mission within this
# window (radio_command_node publishes both back to back).
_SELECTION_MAX_AGE_S = 5.0
_CONFLICTING_NODES = ("mission_state_node",)


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


class HoverMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("hover_mission")
        self._lock = threading.RLock()
        self._mission = HoverMission(CONFIG)
        self._vehicle = ArduCopterVehicle(self, FlightCommandClient(self, already_spinning=True))
        self._selection = None  # (mission_id, monotonic time)
        self._streams_requested = False
        self._last_logged = (self._mission.state, self._mission.detail)

        self.create_subscription(String, MISSION_SELECT_TOPIC, self._on_select, 10)
        self.create_subscription(String, VALIDATED_COMMAND_TOPIC, self._on_command, 10)
        self._status_pub = self.create_publisher(String, HOVER_STATUS_TOPIC, 10)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().warning(
            f"[{_now()}] hover mission ready (REAL FLIGHT): takeoff {CONFIG.takeoff_altitude_m} m, "
            f"hold {CONFIG.hold_duration_s} s; waiting for radio START"
        )

    # -- inputs -------------------------------------------------------------

    def _on_select(self, msg: String) -> None:
        try:
            selection = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(f"ignoring malformed mission selection {msg.data!r}")
            return
        if isinstance(selection, dict):
            self._selection = (selection.get("mission_id"), time.monotonic())

    def _on_command(self, msg: String) -> None:
        now = time.monotonic()
        snap = self._vehicle.snapshot()
        with self._lock:
            if msg.data == "abort":
                self.get_logger().error(f"[{_now()}] ABORT received (state {self._mission.state})")
                self._run(self._mission.abort(now, snap))
                return
            if msg.data != "start":
                return
            problem = self._start_problem(now)
            if problem:
                self.get_logger().error(f"[{_now()}] START refused: {problem}")
                self._mission.reject_start(problem)
                self._log_change()
                return
            self.get_logger().warning(f"[{_now()}] START received for {MISSION_ID}")
            self._run(self._mission.start(now, snap))

    def _start_problem(self, now: float):
        if self._selection is None:
            return "no mission selection received"
        mission_id, at = self._selection
        if mission_id != MISSION_ID:
            return f"selected mission is {mission_id!r}, not {MISSION_ID!r}"
        if now - at > _SELECTION_MAX_AGE_S:
            return "mission selection is stale"
        running = [n for n in _CONFLICTING_NODES if n in self.get_node_names()]
        if running:
            return f"{', '.join(running)} is running and would arm independently -- stop it first"
        return None

    # -- loop -----------------------------------------------------------------

    def _tick(self) -> None:
        snap = self._vehicle.snapshot()
        if snap.fcu_connected and not self._streams_requested:
            self._vehicle.request_streams()
            self._streams_requested = True
        elif not snap.fcu_connected:
            self._streams_requested = False
        with self._lock:
            self._run(self._mission.tick(time.monotonic(), snap))

    def _run(self, actions) -> None:
        self._log_change()
        for action in actions:
            value = "" if action.value is None else action.value
            self.get_logger().warning(f"[{_now()}] action: {action.kind} {value}")
            done = lambda ok, detail, kind=action.kind: self._on_result(kind, ok, detail)  # noqa: E731
            if action.kind == "set_mode":
                self._vehicle.set_mode(action.value, done)
            elif action.kind == "takeoff":
                self._vehicle.takeoff(action.value, done)
            elif action.kind == "arm":
                self._vehicle.arm(done)
            elif action.kind == "disarm":
                self._vehicle.disarm(done)

    def _on_result(self, kind: str, ok: bool, detail: str) -> None:
        log = self.get_logger().info if ok else self.get_logger().error
        log(f"[{_now()}] {kind} {'OK' if ok else 'FAILED'}: {detail}")
        with self._lock:
            self._run(self._mission.on_result(time.monotonic(), kind, ok, detail))

    def _log_change(self) -> None:
        current = (self._mission.state, self._mission.detail)
        if current != self._last_logged:
            self._last_logged = current
            self.get_logger().warning(f"[{_now()}] hover: {current[0]} -- {current[1]}")

    def _publish_status(self) -> None:
        with self._lock:
            status = self._mission.status(time.monotonic())
        self._status_pub.publish(String(data=json.dumps(status)))


def main() -> None:
    rclpy.init()
    node = HoverMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
