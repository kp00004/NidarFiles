"""Motor test mission decision logic -- pure, no rclpy, driven by explicit
start()/abort()/tick()/on_result() calls with a monotonic `now`, so every
branch is unit-testable (test_motor_test_logic.py). mission.py is the ROS
node that feeds it VehicleSnapshots and executes the Actions it returns.

PROPS OFF bench check: spins each motor in turn with ArduCopter's motor
test (MAV_CMD_DO_MOTOR_TEST) at low throttle, so the operator can check
that every motor runs, in the right order and direction.

    preflight (FCU connected, disarmed)
      -> motor 1 (A) for T s -> pause -> motor 2 (B) ... -> complete

The motor test does NOT arm the vehicle the normal way and needs no
position estimate, GPS or EKF: ArduCopter only checks the hardware (safety
switch, RC calibration, not already armed) and reports a refusal in its
status text. While a motor spins, ArduCopter reports the vehicle as armed.

Safety rules encoded here:
  - Preflight refuses (nothing spins) without a connected FCU or if the
    vehicle is armed or its armed state is unknown.
  - Each motor command carries its own timeout: ArduCopter stops the motor
    by itself after T seconds even if the Jetson stops responding.
  - ABORT stops the running motor at once (throttle 0, timeout 0) and ends
    the sequence.
  - A rejected or unacknowledged motor command, or a lost FCU link, ends
    the test as failed -- with a stop command sent anyway.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from nidar_autonomy.vehicle_snapshot import VehicleSnapshot

IDLE = "idle"
PREFLIGHT = "preflight"
TESTING = "testing"
COMPLETE = "complete"
ABORTED = "aborted"
FAILED = "failed"

TERMINAL_STATES = frozenset({IDLE, COMPLETE, ABORTED, FAILED})

MOTOR_LETTERS = "ABCDEFGH"


@dataclass(frozen=True)
class MotorTestConfig:
    motor_count: int = 4
    throttle_pct: float = 8.0
    per_motor_s: float = 5.0
    pause_s: float = 1.0  # between motors, after the FCU's own timeout
    ack_timeout_s: float = 3.0
    state_stale_s: float = 3.0


@dataclass(frozen=True)
class Action:
    kind: str  # "motor_test" | "motor_stop"
    motor: int
    throttle_pct: float = 0.0
    timeout_s: float = 0.0


class MotorTestMission:
    def __init__(self, config: MotorTestConfig) -> None:
        self.config = config
        self.state = IDLE
        self.detail = "waiting for START (PROPS OFF)"
        self.motor: Optional[int] = None  # 1-based motor currently commanded
        self._motor_started = 0.0
        self._acked = False

    # -- operator commands ----------------------------------------------------

    def start(self, now: float, snap: VehicleSnapshot) -> List[Action]:
        if self.state not in TERMINAL_STATES:
            self.detail = f"START ignored: already {self.state}"
            return []
        self.__init__(self.config)
        self.state = PREFLIGHT
        problems = self._preflight_problems(snap)
        if problems:
            self.state = FAILED
            self.detail = "preflight failed: " + "; ".join(problems)
            return []
        self.state = TESTING
        return self._run_motor(now, 1)

    def reject_start(self, reason: str) -> None:
        """START refused by the node before reaching the mission. Nothing
        was commanded."""
        if self.state in TERMINAL_STATES:
            self.state = FAILED
            self.detail = reason

    def abort(self, now: float, snap: VehicleSnapshot) -> List[Action]:
        if self.state in TERMINAL_STATES:
            self.detail = f"ABORT received while {self.state}: nothing to stop"
            return []
        motor = self.motor
        self.state = ABORTED
        self.detail = "operator ABORT -- motors stopped"
        return [] if motor is None else [self._stop(motor)]

    def internal_error(self, now: float, reason: str) -> List[Action]:
        """The node hit an unexpected error (a bug): stop the running motor
        and end the test. Safe to call repeatedly."""
        if self.state in TERMINAL_STATES:
            self.detail = reason
            return []
        return self._fail(now, reason)

    # -- periodic ---------------------------------------------------------------

    def tick(self, now: float, snap: VehicleSnapshot) -> List[Action]:
        if self.state != TESTING:
            return []
        c = self.config
        if not self._link_ok(snap):
            return self._fail(now, "FCU/MAVROS link lost -- stopping motor test")
        if not self._acked and now - self._motor_started > c.ack_timeout_s:
            return self._fail(now, f"motor {self._label(self.motor)} test not acknowledged by the FCU")
        if now - self._motor_started < c.per_motor_s + c.pause_s:
            return []
        if self.motor >= c.motor_count:
            self.state = COMPLETE
            self.detail = f"all {c.motor_count} motors tested -- check order and direction"
            self.motor = None
            return []
        return self._run_motor(now, self.motor + 1)

    def on_result(self, now: float, kind: str, motor: int, ok: bool, detail: str) -> List[Action]:
        if kind == "motor_stop":
            if not ok and self.state in (ABORTED, FAILED):
                self.detail += f"; STOP NOT CONFIRMED ({detail}) -- motor stops by itself within {self.config.per_motor_s:.0f} s"
            return []
        if kind != "motor_test" or self.state != TESTING or motor != self.motor:
            return []  # late reply for an earlier motor
        if ok:
            self._acked = True
            return []
        return self._fail(
            now,
            f"motor {self._label(motor)} test REJECTED by ArduCopter ({detail}) -- "
            "see FCU status text (safety switch? RC calibration?)",
        )

    def status(self, now: float) -> dict:
        c = self.config
        return {
            "scenario": "motor_test",
            "state": self.state,
            "detail": self.detail,
            "current_motor": self.motor,
            "motor_count": c.motor_count,
            "throttle_pct": c.throttle_pct,
            "duration_s": c.per_motor_s,
            "elapsed_motor_s": (
                round(min(now - self._motor_started, c.per_motor_s), 1)
                if self.state == TESTING and self.motor is not None
                else None
            ),
            "execution_mode": "real",
        }

    # -- internals ----------------------------------------------------------------

    def _preflight_problems(self, snap: VehicleSnapshot) -> List[str]:
        problems = []
        if not self._link_ok(snap):
            problems.append("FCU not connected (/mavros/state stale or connected=false)")
        if snap.armed is not False:
            problems.append("vehicle armed or armed state unknown -- motor test needs a disarmed vehicle")
        return problems

    def _link_ok(self, snap: VehicleSnapshot) -> bool:
        return (
            snap.fcu_connected
            and snap.state_age_s is not None
            and snap.state_age_s <= self.config.state_stale_s
        )

    def _run_motor(self, now: float, motor: int) -> List[Action]:
        c = self.config
        self.motor = motor
        self._motor_started = now
        self._acked = False
        self.detail = (
            f"motor {self._label(motor)} ({motor}/{c.motor_count}) at "
            f"{c.throttle_pct:.0f}% for {c.per_motor_s:.0f} s"
        )
        return [Action("motor_test", motor, c.throttle_pct, c.per_motor_s)]

    def _fail(self, now: float, reason: str) -> List[Action]:
        motor = self.motor
        self.state = FAILED
        self.detail = reason
        return [] if motor is None else [self._stop(motor)]

    @staticmethod
    def _stop(motor: int) -> Action:
        return Action("motor_stop", motor, 0.0, 0.0)

    @staticmethod
    def _label(motor: Optional[int]) -> str:
        if motor is None:
            return "?"
        return MOTOR_LETTERS[motor - 1] if 1 <= motor <= len(MOTOR_LETTERS) else str(motor)
