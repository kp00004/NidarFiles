"""`FlightCommandInterface` implementation backed by mavros, via the
existing, already-reviewed `flight_command.FlightCommandClient`.

CHECKPOINT/AUTONOMY_ROADMAP.md Phase 1 -- this class is **deliberately,
severely limited**:

- `arm()` / `disarm()` do nothing new. They delegate to
  `FlightCommandClient`, the only code in this repo permitted to call
  mavros's arming service (see this repo's CLAUDE.md Hard Safety Rule 1
  and `flight_command.py`'s own module docstring). This is wrapping
  already-approved code behind the shared interface, not adding
  capability.
- `abort()`, `takeoff`, `land`, `hold`, `set_position`, `set_velocity`,
  `set_yaw` all raise `NotImplementedError`. Real setpoint issuance
  (velocity/position commands that would actually make the vehicle fly)
  requires a dedicated, explicit safety-reviewed design pass -- control
  mode, failsafe behavior, geofence, abort-preemption mechanism -- per
  `CHECKPOINT/NEXT.md` Phase 8 and
  `CHECKPOINT/INTEGRATION_CHECKPOINTS.md` Checkpoint 5 onward. `abort()`
  specifically is unimplemented because what in-flight ABORT means is
  Checkpoint 7's own required design pass -- see that method's
  docstring -- not because setpoint issuance is involved. **No mavros
  setpoint-publishing code exists in this file at all, not even
  unreachable/never-called code** -- writing it is out of scope for this
  task regardless of how the interface is structured.

This module does not import `flight_command.py` at runtime (only under
`typing.TYPE_CHECKING`, made possible by `from __future__ import
annotations`) so that it -- and `MAVROSFlightController` itself -- stay
importable and unit-testable (with a stub in place of the real client)
without a ROS environment/rclpy/mavros_msgs available, same as every
other non-ROS module in this package. The constructor takes an
already-constructed client (dependency injection) -- this class never
creates its own mavros connection or Node, matching the existing
pattern where `mission_state_node.py` owns that construction.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Tuple

from .flight_command_interface import FlightCommandError, FlightCommandInterface

if TYPE_CHECKING:
    from .flight_command import ArmingResult, FlightCommandClient

_SETPOINT_NOT_IMPLEMENTED = (
    "{method}() is not implemented in MAVROSFlightController: real "
    "setpoint issuance requires a dedicated safety-reviewed design pass "
    "(control mode, failsafe behavior, geofence, abort-preemption) per "
    "CHECKPOINT/NEXT.md Phase 8 and "
    "CHECKPOINT/INTEGRATION_CHECKPOINTS.md Checkpoint 5 onward -- not "
    "implemented here, and not to be implemented without that separate, "
    "explicit sign-off."
)

_ABORT_NOT_IMPLEMENTED = (
    "abort() is not implemented in MAVROSFlightController: "
    "INTEGRATION_CHECKPOINTS.md Checkpoint 7 requires in-flight ABORT to "
    "NOT simply mean 'immediately disarm' -- the safe airborne response "
    "(hold, controlled descent, RTL-equivalent, etc.) must be designed "
    "and reviewed with the user before any implementation, per that "
    "checkpoint's own text and this repo's CLAUDE.md Hard Safety Rule 2. "
    "Not implemented here, and not to be implemented without that "
    "separate, explicit design pass and sign-off."
)


class ArmCommandFailed(FlightCommandError):
    """Raised when the underlying `FlightCommandClient` reports an
    arm/disarm attempt that did not both succeed (FCU accepted it) and
    get confirmed (`/mavros/state` actually reflected it) -- see
    `flight_command.ArmingResult`. Note `FlightCommandClient.arm()` can
    also raise `arming_guard.ArmRejected` directly (a local precondition
    refusal, before ever reaching mavros) -- that propagates unchanged
    through this class rather than being wrapped, since it is already an
    established, reviewed exception type in this repo."""


class TelemetryUnavailableError(FlightCommandError):
    """Raised by a read-only state property when the underlying client
    cannot answer with confidence -- either because no telemetry has
    been received yet (`armed`, transiently, before the first
    `/mavros/state` message) or because the telemetry does not exist at
    all yet (`position`/`velocity` -- `flight_command.py` only exposes
    `/mavros/state.armed` today; indoor pose depends on the SLAM ->
    `VISION_POSITION_ESTIMATE` pipeline tracked as open in
    `custom-gcs/docs/DECISIONS.md` D-11). This is a real, documented gap,
    not a stub -- callers must handle it rather than receiving a
    fabricated zero/placeholder value."""


class MAVROSFlightController(FlightCommandInterface):
    def __init__(self, client: "FlightCommandClient") -> None:
        self._client = client

    # -- FlightCommandInterface: read-only state -----------------------

    @property
    def armed(self) -> bool:
        armed = self._client.is_armed
        if armed is None:
            raise TelemetryUnavailableError(
                "armed state unknown: no /mavros/state message has been "
                "received yet by the underlying FlightCommandClient."
            )
        return armed

    @property
    def position(self) -> Tuple[float, float, float]:
        raise TelemetryUnavailableError(
            "position telemetry is not available: FlightCommandClient "
            "does not expose pose today -- see this class's docstring."
        )

    @property
    def velocity(self) -> Tuple[float, float, float]:
        raise TelemetryUnavailableError(
            "velocity telemetry is not available: FlightCommandClient "
            "does not expose velocity today -- see this class's "
            "docstring."
        )

    # -- FlightCommandInterface: commands wrapping FlightCommandClient --

    def arm(self) -> None:
        self._raise_if_not_confirmed(self._client.arm())

    def disarm(self) -> None:
        self._raise_if_not_confirmed(self._client.disarm())

    def abort(self) -> None:
        """Deliberately unimplemented -- see `INTEGRATION_CHECKPOINTS.md`
        Checkpoint 7: what in-flight ABORT actually means (ground-only
        disarm vs. an airborne-safe response) is Checkpoint 7's own
        required design pass, not something to pre-empt here by assuming
        "abort == disarm" is correct for every case this method might
        eventually need to cover."""
        raise NotImplementedError(_ABORT_NOT_IMPLEMENTED)

    def _raise_if_not_confirmed(self, result: "ArmingResult") -> None:
        if not (result.success and result.state_confirmed):
            raise ArmCommandFailed(result.message)

    # -- FlightCommandInterface: setpoint issuance -- OUT OF SCOPE ------

    def takeoff(self, height: float) -> None:
        raise NotImplementedError(_SETPOINT_NOT_IMPLEMENTED.format(method="takeoff"))

    def land(self) -> None:
        raise NotImplementedError(_SETPOINT_NOT_IMPLEMENTED.format(method="land"))

    def hold(self) -> None:
        raise NotImplementedError(_SETPOINT_NOT_IMPLEMENTED.format(method="hold"))

    def set_position(
        self, x: float, y: float, z: float, yaw: Optional[float] = None
    ) -> None:
        raise NotImplementedError(
            _SETPOINT_NOT_IMPLEMENTED.format(method="set_position")
        )

    def set_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: Optional[float] = None,
    ) -> None:
        raise NotImplementedError(
            _SETPOINT_NOT_IMPLEMENTED.format(method="set_velocity")
        )

    def set_yaw(self, yaw: float) -> None:
        raise NotImplementedError(_SETPOINT_NOT_IMPLEMENTED.format(method="set_yaw"))
