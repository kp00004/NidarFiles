"""Abstract boundary between mission/exploration logic and a specific
path-planning algorithm.

CHECKPOINT/AUTONOMY_ROADMAP.md Phase 7: future exploration/mission-
manager code (Phase 6, Phase 9) must depend on this interface, not on a
specific search algorithm directly, so the concrete planner can change
(grid A* today, something graph-based or hierarchical later) without
mission logic being rewritten -- same "mock/sim first, real integration
later, no rewrite" pattern as `flight_command_interface.py`
(`FlightCommandInterface` -> `MockFlightController` ->
`MAVROSFlightController`).

This module also decouples path planning from Phase 4/5 (SLAM/mapping):
`plan()` takes an environment object satisfying `IndoorEnvironment`'s
query surface (`is_within_bounds`/`is_occupied`), not a live SLAM
subscription, so `GridAStarPlanner` (`grid_astar_planner.py`) can be
built and fully tested against Phase 3's synthetic ground truth
(`indoor_environment.py`) right now. When real SLAM/mapping lands, it
only needs to produce a compatible map -- this interface and its
concrete implementations don't need to change.

Uses `abc.ABC` rather than `typing.Protocol`, same reasoning as
`flight_command_interface.py`: a hard error at instantiation time if a
concrete planner forgets `plan()`, not a silent `AttributeError` the
first time exploration logic calls it.

This module intentionally has no rclpy/mavros dependency and does not
import anything flight-adjacent -- it only ever produces geometric
waypoints from a ground-truth/mapped environment, never a
`FlightCommandInterface` call. Turning a plan into actual vehicle motion
is Phase 8 (trajectory generation) and is explicitly out of scope here.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Tuple

# TYPE_CHECKING-only would be the stricter choice, but this module is
# meant to be importable standalone (no rclpy) and IndoorEnvironment
# already is -- a plain runtime import keeps the type hint on `plan()`
# honest without creating a cycle (indoor_environment.py does not import
# this module).
from .indoor_environment import IndoorEnvironment


class PathPlanningError(Exception):
    """Base class for every error raised by a `PathPlanner`
    implementation. Callers should catch this (or a specific subclass
    below), never a bare `Exception` -- matches
    `flight_command_interface.FlightCommandError`'s role for flight
    commands."""


class EndpointOutOfBoundsError(PathPlanningError):
    """Raised when `start` or `goal` falls outside the environment's
    arena, per `IndoorEnvironment.is_within_bounds`. Distinct from
    `EndpointOccupiedError` so a caller/test can tell "not even in the
    arena" from "in the arena but sitting in an obstacle" -- same
    distinction `IndoorEnvironment` itself draws between
    `OutOfBoundsError` and `InvalidStartPoseError` for its own
    constructor."""


class EndpointOccupiedError(PathPlanningError):
    """Raised when `start` or `goal` is in-bounds but lands on an
    occupied cell, per `IndoorEnvironment.is_occupied`."""


class NoPathFoundError(PathPlanningError):
    """Raised when both endpoints are valid but no route between them
    exists in the environment (the target is enclosed/unreachable). Must
    be raised deterministically once the search space is exhausted --
    never a timeout, never a hang."""


class PathPlanner(ABC):
    """Everything mission/exploration logic is allowed to know about "a
    path planner", implementation-agnostic.

    `plan()` must never return `None` or an empty list to signal
    failure -- every failure mode has a dedicated `PathPlanningError`
    subclass above, raised instead. A successful `plan()` call always
    returns a non-empty list of `(x, y)` world-coordinate waypoints,
    inclusive of both `start` and `goal` (the first waypoint is always
    exactly `start`, the last is always exactly `goal` -- see each
    subclass's own docstring for the `start == goal` edge case).
    """

    @abstractmethod
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        environment: IndoorEnvironment,
    ) -> List[Tuple[float, float]]:
        """Return a list of `(x, y)` world-coordinate waypoints from
        `start` to `goal`, inclusive of both endpoints.

        Implementations must validate `start`/`goal` against
        `environment` (see `_validate_endpoints`) *before* attempting to
        search, and must raise a `PathPlanningError` subclass rather
        than returning `None` or `[]` on any failure."""
        raise NotImplementedError

    @staticmethod
    def _validate_endpoints(
        start: Tuple[float, float],
        goal: Tuple[float, float],
        environment: IndoorEnvironment,
    ) -> None:
        """Shared precondition check every concrete planner should call
        as the first thing `plan()` does -- a `plan()` implementation
        that skipped this and let an invalid input fall through to "no
        path found" would be actively misleading, not just incomplete.
        Lives here (not duplicated per-planner) so every implementation
        validates against `environment`'s own `is_within_bounds`/
        `is_occupied`, never a hand-rolled re-derivation that could
        drift from them.

        Bounds are checked before occupancy for both points, since
        `IndoorEnvironment.is_occupied` reports `True` for out-of-bounds
        points too -- checking occupancy first would misreport an
        out-of-bounds point as merely "occupied"."""
        for name, point in (("start", start), ("goal", goal)):
            x, y = point
            if not environment.is_within_bounds(x, y):
                raise EndpointOutOfBoundsError(
                    f"{name}={point} is outside the environment's arena bounds"
                )
        for name, point in (("start", start), ("goal", goal)):
            x, y = point
            if environment.is_occupied(x, y):
                raise EndpointOccupiedError(
                    f"{name}={point} lands on an occupied cell"
                )
