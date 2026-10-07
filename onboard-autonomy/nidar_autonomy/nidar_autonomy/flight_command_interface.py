"""Abstract boundary between mission/autonomy logic and flight control.

CHECKPOINT/AUTONOMY_ROADMAP.md Phase 1: future autonomy code (path
planning, trajectory generation, the mission manager) must depend on
this interface, not on mavros or a specific implementation directly, so
the exact same autonomy code can run against `MockFlightController`,
then a SITL-backed `MAVROSFlightController`, then a real-Pixhawk-backed
one, without being rewritten. See `mock_flight_controller.py` and
`mavros_flight_controller.py` for the two implementations that exist so
far.

Uses `abc.ABC` rather than `typing.Protocol`: every existing abstraction
boundary already in this repo (`arming_guard.check_arm_preconditions`,
`flight_command.FlightCommandClient`) is a concrete, nominally-typed
contract, not a structural/duck-typed one, and `ABC` gives a hard error
at *instantiation* time if a subclass forgets to implement a method,
rather than a silent `AttributeError` the first time it's called from
deep inside mission logic -- more in keeping with this repo's "fail
loud, fail early" flight-safety posture than `Protocol` would be.

This module intentionally has no rclpy/mavros dependency, same
separation as `state_machine.py` vs `mission_state_node.py` -- it must
be importable (and the mock built on it fully testable) without a ROS
environment.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Tuple


class FlightCommandError(Exception):
    """Base class for every error raised by a FlightCommandInterface
    implementation. Callers/tests should catch this (or a specific
    subclass below) rather than a bare Exception, to distinguish flight-
    command failures from unrelated bugs."""


class NotArmedError(FlightCommandError):
    """Raised when a command that requires the vehicle to be armed
    (any setpoint/motion command) is issued while it is not armed."""


class InvalidCommandError(FlightCommandError):
    """Raised when a command's arguments are invalid or out of bounds --
    NaN/Inf, exceeds a configured velocity/altitude limit, or (for
    `arm()`) an invalid arm/disarm transition. Distinct from
    NotArmedError so a caller/test can tell "this command is malformed
    or out of bounds" from "the vehicle isn't ready for any command at
    all"."""


class FlightCommandInterface(ABC):
    """Everything mission/autonomy logic is allowed to know about
    "the flight controller", implementation-agnostic.

    Every method below that actually moves the vehicle (`takeoff`,
    `land`, `hold`, `set_position`, `set_velocity`, `set_yaw`) MUST raise
    `NotArmedError` if called while `armed` is False -- implementations
    must not silently no-op or auto-arm. `arm`/`disarm`/`abort` are the
    only methods callable regardless of armed state.
    """

    @property
    @abstractmethod
    def armed(self) -> bool:
        """Last known armed state. Implementations that cannot know this
        with confidence (e.g. no telemetry received yet) must raise
        rather than fabricate a value -- see
        `mavros_flight_controller.TelemetryUnavailableError`."""

    @property
    @abstractmethod
    def position(self) -> Tuple[float, float, float]:
        """Last known (x, y, z) position, in the implementation's local
        frame. Implementations without real position telemetry must
        raise rather than fabricate a value (e.g. all zeros) -- a
        fabricated position is worse than an explicit "not available",
        since it can silently pass a caller's sanity check."""

    @property
    @abstractmethod
    def velocity(self) -> Tuple[float, float, float]:
        """Last known (vx, vy, vz) velocity. Same "raise, don't
        fabricate" rule as `position`."""

    @abstractmethod
    def arm(self) -> None:
        """Arm the vehicle. Must not be a no-op consequence of any other
        call -- only ever invoked directly."""

    @abstractmethod
    def disarm(self) -> None:
        """Disarm the vehicle. Always permitted regardless of current
        state -- disarm is the safe direction."""

    @abstractmethod
    def takeoff(self, height: float) -> None:
        """Take off and hold at `height` metres above the current
        position. Requires `armed`."""

    @abstractmethod
    def land(self) -> None:
        """Descend to the ground at the current (x, y) and disarm on
        touchdown. Requires `armed`."""

    @abstractmethod
    def hold(self) -> None:
        """Stop all motion immediately and hold the current position --
        cancels any in-progress `set_position`/`set_velocity`. Requires
        `armed`."""

    @abstractmethod
    def set_position(
        self, x: float, y: float, z: float, yaw: Optional[float] = None
    ) -> None:
        """Command the vehicle toward (x, y, z) (and `yaw` if given).
        Requires `armed`. Implementations are not required to teleport
        to the target -- see `MockFlightController.tick`."""

    @abstractmethod
    def set_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: Optional[float] = None,
    ) -> None:
        """Command a velocity vector (and yaw rate if given). Requires
        `armed`. Implementations must reject (not silently clamp) a
        velocity that exceeds their configured bound."""

    @abstractmethod
    def set_yaw(self, yaw: float) -> None:
        """Command a target yaw (radians). Requires `armed`."""

    @abstractmethod
    def abort(self) -> None:
        """Preempt whatever the vehicle is doing right now, immediately
        and synchronously -- must not require any further call (e.g. a
        `tick()`) to take effect. Per this repo's Hard Safety Rule 2 and
        `CHECKPOINT/INTEGRATION_CHECKPOINTS.md` Checkpoint 7's design
        note, `abort()` must NOT simply mean "immediately disarm" -- an
        immediate disarm mid-flight is a crash, not a safe abort.
        Callable regardless of current armed state."""
