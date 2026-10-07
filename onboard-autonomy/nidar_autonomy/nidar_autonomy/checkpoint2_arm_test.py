"""Manual verification harness for Checkpoint 2 -- see
CHECKPOINT/INTEGRATION_CHECKPOINTS.md. Deliberately NOT wired to
/gcs/command, command_node, or mission_state_node: this checkpoint's own
definition says the initiating command is "issued from onboard-autonomy
code directly (test-triggered, not yet wired to /gcs/command -- that
wiring is Checkpoint 3)".

By default (`execute_arm_cycle` parameter unset/false) this only checks
readiness -- waits for /mavros/state, reports the current armed state,
and checks the arming service is reachable. It does NOT call the arming
service in that mode.

The actual arm-then-immediately-disarm cycle only runs if
`execute_arm_cycle` is explicitly set to true. That is a deliberate,
separate invocation, never a default -- see this repo's CLAUDE.md Hard
Safety Rule 1.

Once ARM has actually been attempted, DISARM is issued from a `finally`
block (see _safe_disarm) so a Python-level exception anywhere in
between cannot skip it. This cannot protect against the process itself
being killed or losing power at the OS level -- that is what the
physical kill switch / battery disconnect precondition is for, and
remains required throughout regardless of this hardening.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from .arming_guard import ArmRejected
from .flight_command import FlightCommandClient


class Checkpoint2ArmTestNode(Node):
    def __init__(self) -> None:
        super().__init__("checkpoint2_arm_test")
        self.declare_parameter("execute_arm_cycle", False)
        self._client = FlightCommandClient(self)

    def _wait_for_state(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._client.is_armed is not None:
                return True
        return False

    def run(self) -> None:
        execute = (
            self.get_parameter("execute_arm_cycle").get_parameter_value().bool_value
        )

        self.get_logger().info("[checkpoint2] Waiting for /mavros/state...")
        if not self._wait_for_state():
            self.get_logger().error(
                "[checkpoint2] No /mavros/state received -- is mavros "
                "running? Aborting."
            )
            return

        self.get_logger().info(
            f"[checkpoint2] Current armed state: {self._client.is_armed}"
        )

        available = self._client.arming_service_available()
        self.get_logger().info(
            "[checkpoint2] "
            + ("/mavros/cmd/arming reachable" if available else "/mavros/cmd/arming NOT reachable")
        )

        if not execute:
            self.get_logger().info(
                "[checkpoint2] Dry run only (execute_arm_cycle:=false). "
                "Not calling the arming service. Readiness check complete."
            )
            return

        if not available:
            self.get_logger().error(
                "[checkpoint2] Cannot execute: arming service unavailable."
            )
            return

        try:
            arm_result = self._client.arm()
        except ArmRejected as exc:
            # Nothing was ever sent to the FCU -- nothing to clean up.
            self.get_logger().error(
                f"[checkpoint2] ARM refused before calling FCU: {exc}"
            )
            return

        # From here on, an arm request has actually been sent to the FCU
        # (whether or not it was accepted/confirmed) -- DISARM must be
        # attempted no matter what happens next, hence the finally block.
        try:
            self.get_logger().info(f"[checkpoint2] ARM result: {arm_result}")
        finally:
            self._safe_disarm()

    def _safe_disarm(self) -> None:
        """Always attempt to disarm and report the outcome. Called from a
        `finally` block, so it must not itself raise and skip the
        final-state check below -- this is the last line of defense
        before the process exits."""
        try:
            disarm_result = self._client.disarm()
            self.get_logger().info(f"[checkpoint2] DISARM result: {disarm_result}")
        except Exception as exc:  # last line of defense: must not propagate
            self.get_logger().error(
                f"[checkpoint2] DISARM call raised unexpectedly: {exc!r}"
            )

        final_armed = self._client.is_armed
        self.get_logger().info(f"[checkpoint2] Final armed state: {final_armed}")
        if final_armed is not False:
            self.get_logger().error(
                "[checkpoint2] *** POST-TEST CHECK FAILED: vehicle does "
                f"not confirm DISARMED (final armed state = {final_armed!r}"
                "). VERIFY PHYSICALLY AND USE THE KILL SWITCH / BATTERY "
                "DISCONNECT IMMEDIATELY IF ARMED. ***"
            )


def main() -> None:
    rclpy.init()
    node = Checkpoint2ArmTestNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
