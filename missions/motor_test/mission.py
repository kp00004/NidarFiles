#!/usr/bin/env python3
"""NIDAR motor test mission -- ROS 2 node, runs on the Jetson. PROPS OFF.

    radio START (motor_test) -> radio_command_node -> /gcs/mission_select + /gcs/command
      -> command_node (validation) -> /nidar_autonomy/validated_command
      -> THIS NODE -> ArduCopterVehicle.motor_test -> MAVROS -> Pixhawk 6X

Spins each motor in turn with ArduCopter's motor test
(MAV_CMD_DO_MOTOR_TEST): low throttle, a few seconds each, so the
operator can check every motor runs, in the right order and direction.
It does not need a position estimate. ABORT stops the running motor at
once; each motor command also carries its own timeout, so the FCU stops
the motor by itself if this node stops responding.

Decisions live in motor_test_logic.py (pure, unit-tested). This file only
wires it to ROS and publishes the status on /motor_test/status, which
radio_command_node relays to the GCS.

Acts only on a START whose /gcs/mission_select names "motor_test"; STARTs
for other missions are ignored.

Run (after sourcing ROS 2 Humble and ~/nidar_ws/install/setup.bash):

    python3 missions/motor_test/mission.py
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

from motor_test_logic import MotorTestConfig, MotorTestMission  # noqa: E402
from nidar_autonomy.ardupilot_vehicle import ArduCopterVehicle  # noqa: E402
from nidar_autonomy.flight_command import FlightCommandClient  # noqa: E402
from nidar_autonomy.topics import (  # noqa: E402
    MISSION_SELECT_TOPIC,
    MOTOR_TEST_STATUS_TOPIC,
    VALIDATED_COMMAND_TOPIC,
)

MISSION_ID = "motor_test"

# ---------------------------------------------------------------------------
# Motor test parameters. Edit here. PROPS OFF for every run.
# Motors are numbered in ArduPilot's test sequence: 1 = A (front-right on a
# quad X), then clockwise B, C, D.
# ---------------------------------------------------------------------------
CONFIG = MotorTestConfig(
    motor_count=4,
    throttle_pct=8.0,
    per_motor_s=5.0,
    pause_s=1.0,
)

_SELECTION_MAX_AGE_S = 5.0


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


class MotorTestNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_test_mission")
        self._lock = threading.RLock()
        self._mission = MotorTestMission(CONFIG)
        self._vehicle = ArduCopterVehicle(self, FlightCommandClient(self, already_spinning=True))
        self._selection = None  # (mission_id, monotonic time)
        self._last_logged = (self._mission.state, self._mission.detail)

        self.create_subscription(String, MISSION_SELECT_TOPIC, self._on_select, 10)
        self.create_subscription(String, VALIDATED_COMMAND_TOPIC, self._on_command, 10)
        self._status_pub = self.create_publisher(String, MOTOR_TEST_STATUS_TOPIC, 10)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().warning(
            f"[{_now()}] motor test mission ready (PROPS OFF): {CONFIG.motor_count} motors, "
            f"{CONFIG.throttle_pct:.0f}% for {CONFIG.per_motor_s:.0f} s each; waiting for radio START"
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
                if self._mission.state not in ("idle", "complete", "aborted", "failed"):
                    self.get_logger().error(f"[{_now()}] ABORT received (state {self._mission.state})")
                self._run(self._mission.abort(now, snap))
                return
            if msg.data != "start":
                return
            if not self._selected(now):
                return  # a START for another mission
            self.get_logger().warning(f"[{_now()}] START received for {MISSION_ID}")
            self._run(self._mission.start(now, snap))

    def _selected(self, now: float) -> bool:
        if self._selection is None:
            return False
        mission_id, at = self._selection
        return mission_id == MISSION_ID and now - at <= _SELECTION_MAX_AGE_S

    # -- loop -----------------------------------------------------------------

    def _tick(self) -> None:
        snap = self._vehicle.snapshot()
        with self._lock:
            self._run(self._mission.tick(time.monotonic(), snap))

    def _run(self, actions) -> None:
        self._log_change()
        for action in actions:
            self.get_logger().warning(
                f"[{_now()}] action: {action.kind} motor {action.motor} "
                f"{action.throttle_pct:.0f}% {action.timeout_s:.1f} s"
            )
            done = lambda ok, detail, a=action: self._on_result(a, ok, detail)  # noqa: E731
            self._vehicle.motor_test(action.motor, action.throttle_pct, action.timeout_s, done)

    def _on_result(self, action, ok: bool, detail: str) -> None:
        log = self.get_logger().info if ok else self.get_logger().error
        log(f"[{_now()}] {action.kind} motor {action.motor} {'OK' if ok else 'FAILED'}: {detail}")
        with self._lock:
            self._run(self._mission.on_result(time.monotonic(), action.kind, action.motor, ok, detail))

    def _log_change(self) -> None:
        current = (self._mission.state, self._mission.detail)
        if current != self._last_logged:
            self._last_logged = current
            self.get_logger().warning(f"[{_now()}] motor test: {current[0]} -- {current[1]}")

    def _publish_status(self) -> None:
        with self._lock:
            status = self._mission.status(time.monotonic())
        self._status_pub.publish(String(data=json.dumps(status)))


def main() -> None:
    rclpy.init()
    node = MotorTestNode()
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
