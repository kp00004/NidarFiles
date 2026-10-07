"""Fully-working, deterministic `FlightCommandInterface` implementation
against a simulated vehicle -- no hardware, no mavros, no real time.

CHECKPOINT/AUTONOMY_ROADMAP.md Phase 1/2: this is what future autonomy
code (path planning, trajectory generation, the mission manager) is
developed and tested against first, before `MAVROSFlightController`
against SITL and then a real Pixhawk -- same interface, same calling
code, no rewrite.

Driven entirely by an explicit `tick(dt)` call, never a background
thread or `time.sleep` -- required for reproducible tests (Phase 2's
"Deterministic time progression").
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from .flight_command_interface import (
    FlightCommandInterface,
    InvalidCommandError,
    NotArmedError,
)

# Arena is <=15m x 15m indoor (custom-gcs/docs/REQUIREMENTS.md) -- 2.0
# m/s is a reasonable bound for that space (crosses the whole arena in
# ~7.5s) without being fast enough that a single tick's worth of motion
# is hard to reason about in tests.
_DEFAULT_MAX_SPEED_MPS = 2.0

# 3m is a sane ceiling for indoor bench/arena testing -- comfortably
# below any real ceiling height while still exercising a real climb.
_DEFAULT_MAX_ALTITUDE_M = 3.0

# How close position must get to a target to be considered "arrived",
# both for ordinary set_position seeking and for land()'s z~=0 check.
_DEFAULT_POSITION_TOLERANCE_M = 0.02

# Float bound-comparisons (max_speed_mps, etc.) get a tiny slack so a
# value that is legitimately *at* the bound (e.g. requesting exactly
# max_speed_mps) is not rejected purely from floating-point noise.
_BOUND_EPSILON = 1e-9


def _validate_finite(**named_values: Optional[float]) -> None:
    for name, value in named_values.items():
        if value is None:
            continue
        if math.isnan(value) or math.isinf(value):
            raise InvalidCommandError(f"{name}={value!r} is not a finite number")


class MockFlightController(FlightCommandInterface):
    """Simulated vehicle: armed/disarmed, a position, a velocity, and at
    most one active target (position-seek or land). No attitude/rotational
    dynamics are modeled -- `set_yaw` is applied instantly, since nothing
    downstream of this mock yet depends on yaw ramping (Phase 2/3 scope
    note, revisit if/when trajectory generation needs it).

    Double-`arm()` raises `InvalidCommandError` rather than being a
    no-op -- this deliberately mirrors the real vehicle's behavior
    (`arming_guard.check_arm_preconditions` refuses to re-arm an already-
    armed vehicle) so autonomy code that runs correctly against the mock
    behaves the same way against `MAVROSFlightController`, which is the
    entire point of sharing one interface.
    """

    def __init__(
        self,
        max_speed_mps: float = _DEFAULT_MAX_SPEED_MPS,
        max_altitude_m: float = _DEFAULT_MAX_ALTITUDE_M,
        position_tolerance_m: float = _DEFAULT_POSITION_TOLERANCE_M,
    ) -> None:
        if max_speed_mps <= 0 or max_altitude_m <= 0 or position_tolerance_m <= 0:
            raise InvalidCommandError(
                "max_speed_mps, max_altitude_m, and position_tolerance_m "
                "must all be positive"
            )
        self._max_speed_mps = max_speed_mps
        self._max_altitude_m = max_altitude_m
        self._position_tolerance_m = position_tolerance_m

        self._armed = False
        self._position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._yaw = 0.0

        self._target_position: Optional[Tuple[float, float, float]] = None
        self._target_yaw: Optional[float] = None
        # True while set_velocity's vector should be applied verbatim
        # each tick, instead of tick() computing a chase-the-target
        # velocity from _target_position.
        self._velocity_mode = False
        # True only while executing land()'s target-seek -- distinguishes
        # "arrived at a z=0 target because of land()" (auto-disarm) from
        # "arrived at an ordinary set_position target that happens to be
        # z=0" (must NOT auto-disarm).
        self._landing = False

    # -- FlightCommandInterface: read-only state -----------------------

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def position(self) -> Tuple[float, float, float]:
        return self._position

    @property
    def velocity(self) -> Tuple[float, float, float]:
        return self._velocity

    @property
    def yaw(self) -> float:
        return self._yaw

    # -- FlightCommandInterface: commands --------------------------------

    def arm(self) -> None:
        if self._armed:
            raise InvalidCommandError(
                "Cannot arm: already armed (mirrors the real vehicle's "
                "arming_guard.check_arm_preconditions refusal to re-arm)."
            )
        self._armed = True

    def disarm(self) -> None:
        self._armed = False
        self._clear_motion()

    def takeoff(self, height: float) -> None:
        self._require_armed()
        _validate_finite(height=height)
        if height <= 0 or height > self._max_altitude_m + _BOUND_EPSILON:
            raise InvalidCommandError(
                f"takeoff height {height}m must be in (0, "
                f"{self._max_altitude_m}]m"
            )
        x, y, _z = self._position
        self.set_position(x, y, height)

    def land(self) -> None:
        self._require_armed()
        x, y, _z = self._position
        self.set_position(x, y, 0.0)
        self._landing = True

    def hold(self) -> None:
        self._require_armed()
        self._clear_motion()

    def set_position(
        self, x: float, y: float, z: float, yaw: Optional[float] = None
    ) -> None:
        self._require_armed()
        _validate_finite(x=x, y=y, z=z, yaw=yaw)
        self._target_position = (x, y, z)
        self._target_yaw = yaw
        self._velocity_mode = False
        self._landing = False

    def set_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: Optional[float] = None,
    ) -> None:
        self._require_armed()
        _validate_finite(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > self._max_speed_mps + _BOUND_EPSILON:
            raise InvalidCommandError(
                f"requested speed {speed:.3f} m/s exceeds max_speed_mps="
                f"{self._max_speed_mps} -- rejected, not clamped"
            )
        self._velocity = (vx, vy, vz)
        self._target_position = None
        self._target_yaw = None
        self._velocity_mode = True
        self._landing = False

    def set_yaw(self, yaw: float) -> None:
        self._require_armed()
        _validate_finite(yaw=yaw)
        self._yaw = yaw

    def abort(self) -> None:
        """Immediate, synchronous preempt -- does NOT require tick() and
        does NOT disarm (in-flight abort must not simply mean "disarm",
        per INTEGRATION_CHECKPOINTS.md Checkpoint 7's design note -- this
        mock has no real flight dynamics yet, but mirrors that decision
        now so autonomy code written against it doesn't learn the wrong
        habit)."""
        self._clear_motion()

    # -- deterministic time progression ---------------------------------

    def tick(self, dt: float) -> None:
        """Advance the simulated vehicle by `dt` seconds. Must be called
        explicitly and repeatedly to make any motion happen -- there is
        no background clock."""
        _validate_finite(dt=dt)
        if dt <= 0:
            raise InvalidCommandError(f"dt={dt!r} must be positive")
        if not self._armed:
            # Disarmed vehicle is stationary on the ground -- a safe
            # no-op, notably including the tick() right after land()'s
            # auto-disarm.
            return

        if self._velocity_mode:
            vx, vy, vz = self._velocity
            x, y, z = self._position
            self._position = (x + vx * dt, y + vy * dt, z + vz * dt)
            return

        if self._target_position is not None:
            self._tick_toward_target(dt)

    def _tick_toward_target(self, dt: float) -> None:
        assert self._target_position is not None
        tx, ty, tz = self._target_position
        x, y, z = self._position
        dx, dy, dz = tx - x, ty - y, tz - z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= self._position_tolerance_m:
            self._arrive_at_target()
            return

        max_step = self._max_speed_mps * dt
        step = min(max_step, distance)
        ratio = step / distance
        self._position = (x + dx * ratio, y + dy * ratio, z + dz * ratio)
        self._velocity = (dx * ratio / dt, dy * ratio / dt, dz * ratio / dt)

        if distance - step <= self._position_tolerance_m:
            self._arrive_at_target()

    def _arrive_at_target(self) -> None:
        assert self._target_position is not None
        self._position = self._target_position
        self._velocity = (0.0, 0.0, 0.0)
        if self._target_yaw is not None:
            self._yaw = self._target_yaw
        self._target_position = None
        self._target_yaw = None
        if self._landing:
            self._landing = False
            self._armed = False

    # -- internal ---------------------------------------------------------

    def _require_armed(self) -> None:
        if not self._armed:
            raise NotArmedError(
                "Cannot issue a motion command: vehicle is not armed."
            )

    def _clear_motion(self) -> None:
        self._velocity = (0.0, 0.0, 0.0)
        self._target_position = None
        self._target_yaw = None
        self._velocity_mode = False
        self._landing = False
