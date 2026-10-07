"""Simulated indoor arena ground truth -- no hardware, no ROS, no mavros.

CHECKPOINT/AUTONOMY_ROADMAP.md Phase 3: this is what Phase 7's path
planner and Phase 6's exploration logic will be developed and tested
against, and it is eventually what Phase 4/5's SLAM output gets checked
against for correctness -- same deterministic, mock-first pattern as
Phase 1/2's `MockFlightController` (`mock_flight_controller.py`): pure
Python, no wall-clock, no threading, fully reproducible.

This module's `IndoorEnvironment` is the *ground truth* map of a
scenario, not a partially-explored SLAM estimate -- every cell is
definitively free or occupied, there is no "unknown" here (see
`to_occupancy_grid_data`'s docstring for why that still matters for the
wire format).
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Set, Tuple

# Arena bound is <=15m x 15m (custom-gcs/docs/REQUIREMENTS.md). Defaulting
# to exactly that ceiling gives canned scenarios the largest realistic
# space to work with; callers building a smaller/different arena override
# both dimensions.
_DEFAULT_WIDTH_M = 15.0
_DEFAULT_HEIGHT_M = 15.0

# 1.0m/cell matches D-7's GCS-facing map resolution
# (custom-gcs/docs/DECISIONS.md D-7/D-7a) -- this module produces ground
# truth at that same resolution rather than SLAM's finer internal working
# resolution, since nothing here simulates SLAM itself.
_DEFAULT_RESOLUTION_M = 1.0

# Origin defaults to (0, 0), i.e. the entry/exit point sits at this
# environment's grid corner (col=0, row=0's world-space corner) rather
# than somewhere interior to the mapped area. D-7 only pins the origin to
# the entry/exit point semantically; it does not require the origin to be
# a grid corner (a real SLAM map can grow in any direction from an
# interior entry point). For a synthetic ground-truth environment there
# is no incremental exploration to grow outward from, so treating origin
# as the corner is the simplest choice that still satisfies D-7, and
# matches DATA_MODELS.md's own worked example (origin (0,0) for a 15x15
# grid).
_DEFAULT_ORIGIN = (0.0, 0.0)

# is_path_clear samples the segment at this many points per cell width --
# fine enough that a single-cell-thick wall between two clear endpoints
# cannot be stepped over, without oversampling long paths pointlessly.
_PATH_SAMPLE_STEPS_PER_CELL = 10

# Rasterizing an obstacle rectangle subtracts this from its max corner
# before quantizing, so an edge that lands exactly on a cell boundary
# (e.g. x_max=8.0 with resolution 1.0) doesn't spill into the next cell.
_RASTER_EPSILON = 1e-9


class IndoorEnvironmentError(Exception):
    """Base class for every error raised by `IndoorEnvironment`."""


class OutOfBoundsError(IndoorEnvironmentError):
    """Raised when a point that must be in-arena (e.g. `start_pose`)
    falls outside `[origin, origin + (width_m, height_m))`."""


class InvalidStartPoseError(IndoorEnvironmentError):
    """Raised when `start_pose` is in-bounds but lands on an occupied
    cell -- distinct from `OutOfBoundsError` so a caller/test can tell
    "not even in the arena" from "in the arena but sitting in a wall"."""


class IndoorEnvironment:
    """Ground-truth occupancy for a rectangular indoor arena.

    Obstacles are specified as axis-aligned rectangles in world meters
    (`(x_min, y_min, x_max, y_max)`) and rasterized into a set of
    occupied `(col, row)` cells once, at construction time -- a rectangle
    list is far easier to hand-author for a canned scenario ("a wall from
    x=7 to x=8, y=0 to y=10") than enumerating individual cell tuples,
    and construction-time rasterization means every later `is_occupied`/
    `is_path_clear` call is a plain set lookup, not a per-call geometry
    test.

    Cell quantization follows `custom-gcs/docs/DECISIONS.md` D-7 exactly:
    `cell = (floor((x - origin_x) / resolution_m), floor((y - origin_y)
    / resolution_m))`.
    """

    def __init__(
        self,
        width_m: float = _DEFAULT_WIDTH_M,
        height_m: float = _DEFAULT_HEIGHT_M,
        resolution_m: float = _DEFAULT_RESOLUTION_M,
        origin: Tuple[float, float] = _DEFAULT_ORIGIN,
        obstacles: Optional[Iterable[Tuple[float, float, float, float]]] = None,
        start_pose: Tuple[float, float] = (0.5, 0.5),
    ) -> None:
        if width_m <= 0 or height_m <= 0 or resolution_m <= 0:
            raise ValueError(
                "width_m, height_m, and resolution_m must all be positive"
            )
        self._resolution_m = resolution_m
        self._origin_x, self._origin_y = origin
        self._width_m = width_m
        self._height_m = height_m
        self._width_cells = self._cell_count(width_m, "width_m")
        self._height_cells = self._cell_count(height_m, "height_m")

        self._occupied: Set[Tuple[int, int]] = set()
        for rect in obstacles or []:
            self._rasterize_obstacle(rect)

        if not self.is_within_bounds(*start_pose):
            raise OutOfBoundsError(
                f"start_pose {start_pose} is outside the arena "
                f"[{self._origin_x}, {self._origin_x + width_m}) x "
                f"[{self._origin_y}, {self._origin_y + height_m})"
            )
        if self.is_occupied(*start_pose):
            raise InvalidStartPoseError(
                f"start_pose {start_pose} lands on an occupied cell"
            )
        self._start_pose = start_pose

    def _cell_count(self, size_m: float, name: str) -> int:
        cells = size_m / self._resolution_m
        rounded = round(cells)
        if abs(cells - rounded) > 1e-6:
            raise ValueError(
                f"{name}={size_m} is not a whole multiple of "
                f"resolution_m={self._resolution_m}"
            )
        return rounded

    def _rasterize_obstacle(self, rect: Tuple[float, float, float, float]) -> None:
        x_min, y_min, x_max, y_max = rect
        if x_min >= x_max or y_min >= y_max:
            raise ValueError(
                f"obstacle rectangle {rect} must have x_min<x_max and "
                "y_min<y_max"
            )
        col_start, row_start = self._cell_of(x_min, y_min)
        col_end, row_end = self._cell_of(
            x_max - _RASTER_EPSILON, y_max - _RASTER_EPSILON
        )
        col_start = max(col_start, 0)
        row_start = max(row_start, 0)
        col_end = min(col_end, self._width_cells - 1)
        row_end = min(row_end, self._height_cells - 1)
        for col in range(col_start, col_end + 1):
            for row in range(row_start, row_end + 1):
                self._occupied.add((col, row))

    def _cell_of(self, x: float, y: float) -> Tuple[int, int]:
        col = math.floor((x - self._origin_x) / self._resolution_m)
        row = math.floor((y - self._origin_y) / self._resolution_m)
        return col, row

    # -- public read-only state ------------------------------------------

    @property
    def start_pose(self) -> Tuple[float, float]:
        return self._start_pose

    @property
    def resolution_m(self) -> float:
        return self._resolution_m

    @property
    def width_cells(self) -> int:
        return self._width_cells

    @property
    def height_cells(self) -> int:
        return self._height_cells

    @property
    def origin(self) -> Tuple[float, float]:
        return (self._origin_x, self._origin_y)

    # -- geometry queries --------------------------------------------------

    def is_within_bounds(self, x: float, y: float) -> bool:
        return (
            self._origin_x <= x < self._origin_x + self._width_m
            and self._origin_y <= y < self._origin_y + self._height_m
        )

    def is_occupied(self, x: float, y: float) -> bool:
        """True if (x, y) is occupied -- also True if it's out of bounds,
        since a point the vehicle can't legally be at is exactly as
        un-flyable as a wall, and treating both cases identically keeps
        `is_path_clear` from needing a separate bounds check."""
        if not self.is_within_bounds(x, y):
            return True
        return self._cell_of(x, y) in self._occupied

    def is_path_clear(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """True if every point sampled along the straight segment from
        (x0, y0) to (x1, y1) -- including both endpoints -- is free and
        in-bounds. Sampled, not just endpoint-checked, at
        `resolution_m / _PATH_SAMPLE_STEPS_PER_CELL` spacing, so a wall
        between two clear endpoints is actually detected -- this is the
        collision-check primitive Phase 7's planner will use, so a stub
        that only checked endpoints would be actively wrong, not just
        incomplete."""
        dx = x1 - x0
        dy = y1 - y0
        distance = math.hypot(dx, dy)
        step = self._resolution_m / _PATH_SAMPLE_STEPS_PER_CELL
        steps = max(1, math.ceil(distance / step)) if distance > 0 else 0
        for i in range(steps + 1):
            t = i / steps if steps > 0 else 0.0
            x = x0 + dx * t
            y = y0 + dy * t
            if self.is_occupied(x, y):
                return False
        return True

    # -- export -------------------------------------------------------------

    def to_occupancy_grid_data(self) -> Dict:
        """Return this environment's ground truth in the exact
        `nav_msgs/OccupancyGrid`-as-seen-by-roslibjs shape from
        `custom-gcs/docs/DATA_MODELS.md` Section 4, so it's directly
        usable as a test fixture against a real Phase 5 map publisher's
        output later.

        `data` only ever contains 0 (free) or 100 (occupied), never -1
        (unknown) -- this is ground truth for a scenario this module
        fully constructed, not a partial SLAM estimate, so there is
        nothing left "unknown" to report. `header.stamp` is always
        `{sec: 0, nanosec: 0}` -- this module has no wall-clock (same
        determinism rule as `mock_flight_controller.py`'s `tick(dt)`), so
        a real timestamp would be fabricated; a real Phase 5 publisher
        stamps this with actual time, this ground-truth fixture does
        not.
        """
        data: List[int] = []
        for row in range(self._height_cells):
            for col in range(self._width_cells):
                data.append(100 if (col, row) in self._occupied else 0)
        return {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "info": {
                "resolution": self._resolution_m,
                "width": self._width_cells,
                "height": self._height_cells,
                "origin": {
                    "position": {
                        "x": self._origin_x,
                        "y": self._origin_y,
                        "z": 0.0,
                    },
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "data": data,
        }


# -- canned deterministic scenarios ---------------------------------------
#
# Module-level factory functions, not methods on IndoorEnvironment -- a
# scenario is a specific, named configuration of the class, not a
# capability of the class itself (same reasoning as e.g. pytest fixtures
# living beside, not inside, the thing they configure).


def empty_room() -> IndoorEnvironment:
    """15x15m arena, no obstacles at all. Baseline/control case: any
    straight line between two in-bounds free points must be clear, and
    every in-bounds cell must be free -- useful as the "nothing should
    ever fail here" comparison case for Phase 6/7 tests."""
    return IndoorEnvironment(start_pose=(0.5, 0.5))


def room_with_single_obstacle() -> IndoorEnvironment:
    """15x15m arena with one wall: cells col=7, row=0..9 (world
    x in [7.0, 8.0), y in [0.0, 10.0)) -- a 1m-wide, 10m-tall wall running
    up from the bottom edge, stopping 5m short of the top.

    `start_pose = (1.5, 1.5)` (bottom-left area). The wall sits squarely
    on the direct diagonal line from `start_pose` to the far corner
    `(13.5, 13.5)`: `is_path_clear(1.5, 1.5, 13.5, 13.5)` is `False`. A
    route around the open top of the wall exists and is genuinely clear,
    e.g. via waypoint `(7.5, 12.0)`:
    `is_path_clear(1.5, 1.5, 7.5, 12.0)` and
    `is_path_clear(7.5, 12.0, 13.5, 13.5)` are both `True`.

    This is the ground truth for Phase 7's "planner finds an alternate
    route around an obstacle" test -- the direct line must fail, a
    specific known detour must succeed, without this module doing any
    actual routing itself.
    """
    obstacles = [(7.0, 0.0, 8.0, 10.0)]
    return IndoorEnvironment(obstacles=obstacles, start_pose=(1.5, 1.5))


def unreachable_target_room() -> IndoorEnvironment:
    """15x15m arena with a solid 1-cell-thick ring of 8 occupied cells --
    (6,6) (7,6) (8,6) (6,7) (8,7) (6,8) (7,8) (8,8) -- completely
    surrounding the free center cell (7,7) (world x,y in [7.0, 8.0)),
    with no gap in the ring anywhere, including its corners.

    `start_pose = (1.5, 1.5)`. The point `(7.5, 7.5)` (the enclosed
    center cell) is provably unreachable from `start_pose`: every one of
    the 8 cells orthogonally or diagonally adjacent to (7,7) is occupied,
    so no continuous path -- straight-line or otherwise -- can reach it
    without crossing an occupied cell first.

    This is the ground truth for Phase 7's "target is unreachable, the
    planner must report failure, not hang" test.
    """
    obstacles = [
        (6.0, 6.0, 7.0, 7.0),  # (6,6)
        (7.0, 6.0, 8.0, 7.0),  # (7,6)
        (8.0, 6.0, 9.0, 7.0),  # (8,6)
        (6.0, 7.0, 7.0, 8.0),  # (6,7)
        (8.0, 7.0, 9.0, 8.0),  # (8,7)
        (6.0, 8.0, 7.0, 9.0),  # (6,8)
        (7.0, 8.0, 8.0, 9.0),  # (7,8)
        (8.0, 8.0, 9.0, 9.0),  # (8,8)
    ]
    return IndoorEnvironment(obstacles=obstacles, start_pose=(1.5, 1.5))


def simulation_maze() -> IndoorEnvironment:
    """15x15m walled arena with interior room dividers -- the ground-truth
    world for the NIDAR simulation harness (`mission_simulator.py`, see
    CHECKPOINT/CURRENT_STATE.md). Unlike this module's other canned
    scenarios (which model a fully open, implicitly-bounded rectangle),
    this one has EXPLICIT exterior walls plus interior dividers, giving a
    genuine multi-room/corridor layout deep enough to require several
    distinct exploration legs and produce more than one frontier region at
    a time -- the property the simulation needs to meaningfully exercise
    `frontier_detector.py`/`exploration_policy.py` beyond a single trivial
    frontier.

    Layout (approximate, world meters, y increasing "north"):

        14 ############### (north wall)
        13 #    |          #
         . # SW | corridor #   NE room, entered via the open corridor at
         . #room|          #   the gap in the col=10 divider (rows 8-9)
         8 #    +---+      #
         7 #        |######
         . # SE room|      #  SE room and the NE room split by the
         . #        | gap  #  col=10 wall; gap at rows 3-4 connects them
         1 #(start) |      #
         0 ############### (south wall)
            0              14

    `start_pose = (1.5, 1.5)` (bottom-left / "SE" room, matching this
    module's start_pose convention elsewhere)."""
    obstacles = [
        # Exterior walls (0..15 world, 1-cell-thick border).
        (0.0, 0.0, 15.0, 1.0),  # south
        (0.0, 14.0, 15.0, 15.0),  # north
        (0.0, 0.0, 1.0, 15.0),  # west
        (14.0, 0.0, 15.0, 15.0),  # east
        # Vertical divider at x in [7,8): splits the arena into a west
        # half (start side) and east half. Gap at y in [8,10) is the only
        # corridor connection between the two halves.
        (7.0, 1.0, 8.0, 8.0),
        (7.0, 10.0, 8.0, 14.0),
        # Horizontal divider at y in [7,8), west half only: splits the
        # west half into the start ("SW") room below and a smaller room
        # above. Gap at x in [4,6) connects them.
        (1.0, 7.0, 4.0, 8.0),
        (6.0, 7.0, 7.0, 8.0),
        # Vertical divider at x in [10,11), east half: splits the east
        # half into a south ("SE") room and a north ("NE") room. Gap at
        # y in [3,5) connects them.
        (10.0, 1.0, 11.0, 3.0),
        (10.0, 5.0, 11.0, 14.0),
        # A small interior obstacle in the NE room, so even the largest
        # open room isn't a single trivial rectangle.
        (11.5, 10.5, 13.0, 12.0),
    ]
    return IndoorEnvironment(obstacles=obstacles, start_pose=(1.5, 1.5))
