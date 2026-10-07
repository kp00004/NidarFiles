"""Minimal flight-command layer for Checkpoint 2 (Jetson -> Pixhawk
ARM/DISARM via mavros). This is the only place in this repo allowed to
touch mavros's arming interface -- see
CHECKPOINT/INTEGRATION_CHECKPOINTS.md Checkpoint 2 and this repo's
CLAUDE.md Hard Safety Rules 1 and 4.

Deliberately NOT wired to /gcs/command, command_node, or
mission_state_node -- Checkpoint 2 is test-triggered only, per
INTEGRATION_CHECKPOINTS.md ("Initiating command: a command issued from
onboard-autonomy code directly ... not yet wired to /gcs/command -- that
wiring is Checkpoint 3"). This module is written so Checkpoint 3 can
import and reuse FlightCommandClient as-is instead of duplicating
ARM/DISARM logic.

The pure precondition-checking logic (ArmRejected, check_arm_preconditions)
lives in arming_guard.py, which has no rclpy dependency, so it's
unit-testable without a ROS context -- same pattern as state_machine.py
vs. mission_state_node.py. This module itself does depend on rclpy (it's
the ROS-facing half).

Never bypasses a Pixhawk prearm rejection: if the FCU says no, that is
reported (including any STATUSTEXT reason captured), never retried with
a force-arm parameter.

Hardening (added before the first real bench test): a service-level
"success" from mavros is the FCU's COMMAND_ACK, not proof the vehicle's
actual state changed -- so every arm/disarm call is followed by a short,
bounded poll of the real /mavros/state.armed value (see
_wait_for_armed_state) before being considered confirmed. disarm() is
additionally retried (bounded, never infinite) if the FCU rejects it or
the state doesn't confirm disarmed -- arm() is deliberately NOT retried;
if arming doesn't cleanly succeed the safe response is to stay disarmed,
not to keep pushing.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool

from .arming_guard import check_arm_preconditions

ARMING_SERVICE = "/mavros/cmd/arming"
STATE_TOPIC = "/mavros/state"
STATUSTEXT_TOPIC = "/mavros/statustext/recv"

# ArduPilot reports prearm-check failure reasons via STATUSTEXT, not in
# the CommandBool response (which only carries a numeric MAV_RESULT) --
# keep a short rolling history so a rejected arm attempt has something
# human-readable to report instead of just "success=False".
_STATUSTEXT_HISTORY = 10

# How long to wait for /mavros/state.armed to reflect a requested
# arm/disarm before giving up on confirmation. Kept short deliberately:
# for the post-ARM check in particular, this window is time spent with
# the vehicle actually armed, so it must not be generous "just in case".
_DEFAULT_STATE_CONFIRM_TIMEOUT_SEC = 2.0

# DISARM is retried (bounded) because leaving the vehicle armed is the
# one outcome that must not happen silently. ARM is never retried -- see
# module docstring.
_DISARM_MAX_ATTEMPTS = 3


@dataclass
class ArmingResult:
    """Outcome of a call that actually reached the mavros service (see
    ArmRejected for the case where we refused to call it at all)."""

    requested: bool  # True = arm was requested, False = disarm
    success: bool  # FCU accepted the request (COMMAND_ACK-level)
    mav_result: Optional[int]
    state_confirmed: bool  # /mavros/state.armed actually matched `requested`
    message: str


class FlightCommandClient:
    """Thin wrapper around mavros's /mavros/cmd/arming service."""

    def __init__(
        self,
        node: Node,
        service_timeout_sec: float = 5.0,
        state_confirm_timeout_sec: float = _DEFAULT_STATE_CONFIRM_TIMEOUT_SEC,
        already_spinning: bool = False,
    ) -> None:
        """`already_spinning`: set True when `node` is already being spun
        by an executor elsewhere (e.g. called from inside a subscription
        callback under `rclpy.spin(node)`, as mission_state_node.py does).
        In that case this class must NOT call rclpy.spin_once/
        spin_until_future_complete itself -- doing so creates a second,
        nested executor for the same node from inside that node's own
        callback dispatch, which is a known rclpy deadlock/timeout
        hazard (the nested wait can time out even though the FCU request
        genuinely went through, which is worse than useless for
        arm/disarm code -- it looks like a clean failure but isn't).
        Instead, this class just polls with plain time.sleep() and lets
        the *external* executor keep processing the state/service-
        response callbacks that make progress possible. Default False
        preserves the exact behavior already verified on real hardware
        for checkpoint2_arm_test.py, which calls this from a plain
        script with no executor already spinning."""
        self._node = node
        self._log = node.get_logger()
        self._service_timeout_sec = service_timeout_sec
        self._state_confirm_timeout_sec = state_confirm_timeout_sec
        self._already_spinning = already_spinning

        self._arming_client = node.create_client(CommandBool, ARMING_SERVICE)

        self._latest_state: Optional[State] = None
        self._state_sub = node.create_subscription(
            State, STATE_TOPIC, self._on_state, 10
        )

        self._statustext_history: List[str] = []
        # mavros publishes STATUSTEXT as BEST_EFFORT/VOLATILE -- a
        # default (RELIABLE) subscription is QoS-incompatible with that
        # and would silently never receive anything.
        statustext_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        self._statustext_sub = node.create_subscription(
            StatusText, STATUSTEXT_TOPIC, self._on_statustext, statustext_qos
        )

    def _on_state(self, msg: State) -> None:
        self._latest_state = msg

    def _on_statustext(self, msg: StatusText) -> None:
        self._statustext_history.append(msg.text)
        if len(self._statustext_history) > _STATUSTEXT_HISTORY:
            self._statustext_history.pop(0)

    @property
    def is_armed(self) -> Optional[bool]:
        """Last known armed state from /mavros/state, or None if no
        message has been received yet."""
        return self._latest_state.armed if self._latest_state is not None else None

    def recent_statustext(self) -> List[str]:
        """Recent STATUSTEXT strings from the FCU (e.g. prearm-check
        failure reasons), oldest first."""
        return list(self._statustext_history)

    def arming_service_available(self) -> bool:
        return self._arming_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        )

    def _wait_for_armed_state(self, expected: bool, timeout_sec: float) -> bool:
        """Bounded poll of /mavros/state.armed for the expected value.
        Returns as soon as it matches -- never sleeps longer than needed
        -- and returns False (not an exception) if it doesn't happen
        within timeout_sec, since "not yet confirmed" is a fact to report,
        not a crash."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.is_armed == expected:
                return True
            if self._already_spinning:
                time.sleep(0.05)
            else:
                rclpy.spin_once(self._node, timeout_sec=0.1)
        return self.is_armed == expected

    def arm(self) -> ArmingResult:
        """Attempt to arm, once. Raises ArmRejected without touching
        mavros at all if the local safety precondition isn't met. If the
        FCU rejects the request (prearm check failure), or the resulting
        state can't be confirmed, that is returned as-is -- never
        retried, never bypassed. Arming is deliberately not retried: if
        it doesn't cleanly succeed, staying disarmed is the safe
        response, not trying again."""
        check_arm_preconditions(self.is_armed)
        return self._set_arming(True)

    def disarm(self) -> ArmingResult:
        """Attempt to disarm. Always permitted -- disarm is the safe
        direction regardless of current or unknown state. Retried
        (bounded, _DISARM_MAX_ATTEMPTS) if the FCU rejects it or
        /mavros/state doesn't confirm disarmed -- unlike arm(), because
        leaving the vehicle armed is the one outcome that must not
        happen silently. If every attempt fails, that is logged as an
        unmistakable, repeated error, not a single quiet warning."""
        result: Optional[ArmingResult] = None
        for attempt in range(1, _DISARM_MAX_ATTEMPTS + 1):
            result = self._set_arming(False)
            if result.success and result.state_confirmed:
                if attempt > 1:
                    self._log.info(
                        f"[flight_command] DISARM confirmed on retry "
                        f"attempt {attempt}/{_DISARM_MAX_ATTEMPTS}"
                    )
                return result
            self._log.error(
                f"[flight_command] DISARM attempt {attempt}/"
                f"{_DISARM_MAX_ATTEMPTS} not confirmed: {result.message}"
            )

        self._log.error(
            "[flight_command] *** DISARM FAILED AFTER "
            f"{_DISARM_MAX_ATTEMPTS} ATTEMPTS -- VEHICLE MAY STILL BE "
            "ARMED. USE THE PHYSICAL KILL SWITCH / BATTERY DISCONNECT "
            "IMMEDIATELY AND VERIFY VISUALLY. ***"
        )
        assert result is not None  # loop always runs at least once
        return result

    def _set_arming(self, value: bool) -> ArmingResult:
        action = "ARM" if value else "DISARM"
        self._log.info(f"[flight_command] Requesting {action} via {ARMING_SERVICE}")

        if not self.arming_service_available():
            message = (
                f"{ARMING_SERVICE} not available after "
                f"{self._service_timeout_sec}s"
            )
            self._log.error(f"[flight_command] {action} failed: {message}")
            return ArmingResult(
                requested=value,
                success=False,
                mav_result=None,
                state_confirmed=False,
                message=message,
            )

        request = CommandBool.Request()
        request.value = value
        future = self._arming_client.call_async(request)
        if self._already_spinning:
            deadline = time.monotonic() + self._service_timeout_sec
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
        else:
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=self._service_timeout_sec
            )

        if not future.done():
            # A timed-out future does not prove the FCU never received or
            # acted on the request -- check the real state anyway rather
            # than assuming nothing happened.
            confirmed = self._wait_for_armed_state(
                value, self._state_confirm_timeout_sec
            )
            message = (
                f"{action} service call timed out after "
                f"{self._service_timeout_sec}s"
                + (
                    " -- but /mavros/state now shows the requested state "
                    "anyway"
                    if confirmed
                    else " -- and /mavros/state does not confirm it either"
                )
            )
            self._log.error(f"[flight_command] {message}")
            return ArmingResult(
                requested=value,
                success=False,
                mav_result=None,
                state_confirmed=confirmed,
                message=message,
            )

        response = future.result()
        if response is None:
            confirmed = self._wait_for_armed_state(
                value, self._state_confirm_timeout_sec
            )
            message = f"{action} service call raised: {future.exception()!r}"
            self._log.error(f"[flight_command] {message}")
            return ArmingResult(
                requested=value,
                success=False,
                mav_result=None,
                state_confirmed=confirmed,
                message=message,
            )

        confirmed = self._wait_for_armed_state(value, self._state_confirm_timeout_sec)

        if response.success and confirmed:
            message = (
                f"{action} accepted by FCU (result={response.result}) and "
                "confirmed via /mavros/state"
            )
            self._log.info(f"[flight_command] {message}")
        elif response.success and not confirmed:
            message = (
                f"{action} ACCEPTED by FCU (result={response.result}) but "
                f"/mavros/state.armed did NOT confirm within "
                f"{self._state_confirm_timeout_sec}s -- MISMATCH between "
                "FCU acknowledgment and observed state"
            )
            self._log.error(f"[flight_command] {message}")
        else:
            reasons = self.recent_statustext()
            message = (
                f"{action} rejected by FCU (result={response.result})"
                + (
                    f"; recent STATUSTEXT: {reasons}"
                    if reasons
                    else "; no STATUSTEXT captured"
                )
                + (
                    "; NOTE: /mavros/state nonetheless shows the requested "
                    "state -- treat with caution"
                    if confirmed
                    else ""
                )
            )
            self._log.warning(f"[flight_command] {message}")

        return ArmingResult(
            requested=value,
            success=response.success,
            mav_result=response.result,
            state_confirmed=confirmed,
            message=message,
        )
