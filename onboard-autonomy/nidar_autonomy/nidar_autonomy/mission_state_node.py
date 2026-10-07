"""Owns and publishes /mission/state. Listens ONLY to the sanitized
internal topic from command_node (never the raw /gcs/command) -- see
that node's docstring and this repo's CLAUDE.md Hard Safety Rule 5.
The actual state-transition logic lives in state_machine.py (kept
ROS-free so it's unit-testable); this node is just the ROS plumbing
around it.

IMPORTANT LIMITATION (2026-08-27, Phase 0): "start" moves the state
directly to "entering" and nothing further ever moves it to "searching"
or "exiting" -- those transitions are supposed to be driven by real
events from the exploration/path-planning subsystem (e.g. "we've left
the entry cell" -> searching, "time/coverage budget hit, heading back"
-> exiting), which doesn't exist yet. Deliberately NOT faking those
transitions with a timer here (the way custom-gcs/sim/ does, for testing
purposes) -- that would make this node lie about what the real drone is
actually doing, which defeats the point of it existing. Extend this once
the exploration subsystem exists and can report its own phase
transitions.

CHECKPOINT 3/4 WIRING (2026-09-02): this node now owns the
FlightCommandClient and is the one place that turns a validated GCS
command into a real ARM/DISARM attempt -- see arm_trigger.py for the
(pure, tested) decision logic and CHECKPOINT/INTEGRATION_CHECKPOINTS.md
Checkpoints 3/4. "start" only ever attempts ARM as a direct consequence
of the state machine actually accepting it (idle -> entering); "abort"
always attempts DISARM, regardless of current state, per this repo's
CLAUDE.md Hard Safety Rules 1/2. Neither attempt is retried here --
arm()/disarm() already own their own bounded retry policy (see
flight_command.py); this node just logs the outcome and never crashes on
a rejected/failed attempt.

Threading note: _attempt_arm()/_attempt_disarm() are dispatched onto a
background thread (never called inline from the subscription callback).
flight_command.py's arm()/disarm() block for up to several seconds
waiting on the FCU; running that inline here would stall this node's
single-threaded executor (blocking the 2Hz /mission/state publisher and
all other callbacks) for that whole window. The FlightCommandClient is
constructed with already_spinning=True for the same reason: it must not
call rclpy.spin_once/spin_until_future_complete itself, since this node
is already being spun by an executor elsewhere -- see flight_command.py's
already_spinning docstring for the deadlock/timeout hazard that avoids.
A lock serializes arm/disarm attempts so a start immediately followed by
an abort can't race two concurrent FCU requests.

MISSION RESET (2026-09-08): "aborted" used to be terminal -- the only way
to restart a mission after an abort was to restart this whole node
(recreating MissionStateMachine fresh). This node now also drives the
one and only path back to "idle": whenever a DISARM *this node itself
commanded* (from "abort", or from a forced disarm after a failed/
unconfirmed ARM -- see below) is confirmed via the real
/mavros/state.armed value, state_machine.handle_ground_reset_confirmed()
moves "aborted" -> "idle", making a subsequent "start" work again without
any process restart. An unsolicited FCU disarm (see "UNSOLICITED DISARM"
below) deliberately does NOT go through this path -- it only ever reaches
"aborted", never "idle" -- and a commanded disarm that fails to confirm
leaves the state at "aborted" indefinitely (see
state_machine.handle_ground_reset_confirmed()'s docstring for why
guessing would be unsafe). Relatedly, a failed/unconfirmed ARM attempt
(_attempt_arm()) now also drives the mission to "aborted" and forces a
DISARM, via the same confirm-and-maybe-reset path as "abort" -- an ARM
that didn't cleanly succeed must not leave /mission/state claiming
"entering" (i.e. an active, armed mission) when the vehicle never armed.

UNSOLICITED DISARM (2026-09-03, late-session): a real bench test showed the FCU can
disarm itself a few seconds after a successful ARM (observed ~5s after
arming, consistent with ArduCopter's stock ground-idle auto-disarm --
DISARM_DELAY=10 confirmed set -- since nothing here sends a throttle/
setpoint stream, per this repo's Phase 8/Checkpoint 5+ scope). Nothing
in this node previously noticed that: /mavros/state was only consulted
by FlightCommandClient during the few seconds right after a commanded
arm()/disarm() call, so mission state stayed "entering" -- falsely
implying an active armed mission -- for as long as the vehicle sat
disarmed until an operator eventually sent "abort". This node now keeps
its own independent /mavros/state subscription (separate from
FlightCommandClient's internal one -- multiple subscribers to the same
topic is normal) purely to detect an armed:true -> armed:false
transition that was NOT preceded by mission state leaving "entering"
(our own commanded disarm always transitions state to "aborted" via
"abort" first, before disarm() is even called -- see
_on_validated_command -- so any true->false transition seen while state
is still "entering" is, by construction, the FCU's own doing). See
state_machine.py's handle_fcu_disarmed() for the (pure, tested) logic --
only "entering" transitions, everything else is a no-op, so this can
never re-arm, never overrides an operator's explicit abort/state, and
never touches the FCU itself.
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from std_msgs.msg import String

from .arm_trigger import should_attempt_arm, should_attempt_disarm
from .arming_guard import ArmRejected
from .flight_command import STATE_TOPIC, ArmingResult, FlightCommandClient
from .state_machine import MissionStateMachine
from .topics import MISSION_STATE_TOPIC, VALIDATED_COMMAND_TOPIC

_PUBLISH_RATE_HZ = 2.0


class MissionStateNode(Node):
    def __init__(self, flight_client: FlightCommandClient | None = None) -> None:
        super().__init__("mission_state_node")
        self._machine = MissionStateMachine()
        self._publisher = self.create_publisher(String, MISSION_STATE_TOPIC, 10)
        self._subscription = self.create_subscription(
            String, VALIDATED_COMMAND_TOPIC, self._on_validated_command, 10
        )
        self._timer = self.create_timer(1.0 / _PUBLISH_RATE_HZ, self._publish_state)
        self._flight = (
            flight_client
            if flight_client is not None
            else FlightCommandClient(self, already_spinning=True)
        )
        self._flight_lock = threading.Lock()
        self._last_fcu_armed: Optional[bool] = None
        self._fcu_state_sub = self.create_subscription(
            State, STATE_TOPIC, self._on_fcu_state, 10
        )
        self.get_logger().info(f"Starting in state: {self._machine.state!r}")

    def _on_validated_command(self, msg: String) -> None:
        previous = self._machine.state
        command = msg.data
        new_state = self._machine.handle_command(command)

        if new_state == previous and command == "start":
            self.get_logger().warning(
                f"Got 'start' while already in state {previous!r}; ignoring "
                "(a real mission can only be started from 'idle')."
            )
        elif new_state != previous:
            self.get_logger().info(f"Mission state: {previous!r} -> {new_state!r}")

        if should_attempt_arm(command, previous, new_state):
            threading.Thread(target=self._attempt_arm, daemon=True).start()
        if should_attempt_disarm(command):
            threading.Thread(target=self._attempt_disarm, daemon=True).start()

    def _attempt_arm(self) -> None:
        result: Optional[ArmingResult]
        with self._flight_lock:
            try:
                result = self._flight.arm()
            except ArmRejected as exc:
                self.get_logger().error(
                    f"[mission_state_node] Checkpoint 3: ARM refused before "
                    f"reaching the FCU: {exc}"
                )
                result = None

        if result is not None:
            self.get_logger().info(f"[mission_state_node] Checkpoint 3: ARM attempt: {result}")
            if result.success and result.state_confirmed:
                return  # genuinely armed -- mission stays "entering"

        previous = self._machine.state
        new_state = self._machine.handle_arm_failed()
        if new_state != previous:
            self.get_logger().warning(
                f"[mission_state_node] ARM did not cleanly succeed/confirm "
                f"-- mission state {previous!r} -> {new_state!r}; forcing "
                "DISARM to reach a known-safe state."
            )
        with self._flight_lock:
            disarm_result = self._flight.disarm()
        self.get_logger().info(
            f"[mission_state_node] Forced DISARM after failed ARM: {disarm_result}"
        )
        self._maybe_reset_after_disarm(disarm_result)

    def _attempt_disarm(self) -> None:
        with self._flight_lock:
            result = self._flight.disarm()
        self.get_logger().info(f"[mission_state_node] Checkpoint 4: DISARM attempt: {result}")
        self._maybe_reset_after_disarm(result)

    def _maybe_reset_after_disarm(self, result: ArmingResult) -> None:
        """Common tail for every commanded-disarm path (operator abort,
        or a forced disarm after a failed arm): only a disarm that the
        FCU actually accepted AND that /mavros/state confirmed is treated
        as "safe to make restartable again" -- see
        state_machine.handle_ground_reset_confirmed()'s docstring. An
        ambiguous/unconfirmed result leaves the mission at "aborted" on
        purpose, matching flight_command.py's own "don't guess, escalate"
        posture for a disarm that didn't confirm."""
        if not (result.success and result.state_confirmed):
            return
        previous = self._machine.state
        new_state = self._machine.handle_ground_reset_confirmed()
        if new_state != previous:
            self.get_logger().info(
                "[mission_state_node] Commanded DISARM confirmed via "
                f"/mavros/state -- mission is restartable again (mission "
                f"state: {previous!r} -> {new_state!r})."
            )

    def _on_fcu_state(self, msg: State) -> None:
        previous_armed = self._last_fcu_armed
        self._last_fcu_armed = msg.armed
        if previous_armed is True and msg.armed is False:
            previous_mission_state = self._machine.state
            new_state = self._machine.handle_fcu_disarmed()
            if new_state != previous_mission_state:
                self.get_logger().warning(
                    "FCU disarmed on its own (not via a DISARM we "
                    f"requested) while mission state was "
                    f"{previous_mission_state!r} -- treating the mission "
                    f"as aborted (mission state: {previous_mission_state!r} "
                    f"-> {new_state!r}). Likely cause: ArduCopter's own "
                    "ground-idle auto-disarm, since no throttle/setpoint "
                    "stream exists yet -- see CHECKPOINT/CURRENT_STATE.md."
                )

    def _publish_state(self) -> None:
        self._publisher.publish(String(data=self._machine.state))


def main() -> None:
    rclpy.init()
    node = MissionStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
