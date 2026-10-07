"""Adapter: a live `nav_msgs/OccupancyGrid`-shaped dict, queried through the
same surface `indoor_environment.IndoorEnvironment` exposes.

WHY THIS EXISTS
================
`grid_astar_planner.GridAStarPlanner` and `frontier_detector.detect_frontiers`
were both built and tested (AUTONOMY_ROADMAP.md Phase 6/7) against
`IndoorEnvironment` -- a hand-authored, fully-known *ground truth* -- on the
explicit premise that "when real mapping lands, it only needs to produce a
compatible occupancy grid; the planner itself shouldn't need to change" (see
`grid_astar_planner.py`'s module docstring). This class is that promise kept:
it wraps the same `{"info": {...}, "data": [...]}` dict shape
`frontier_detector.py` already consumes (and `to_occupancy_grid_data()`
produces) so `GridAStarPlanner.plan()` can be called against it exactly as it
is against `IndoorEnvironment`, via duck typing (Python does not enforce the
type hint at runtime).

This module was added as part of the NIDAR Autonomy Migration (folding
gps_denied/raj-dev's exploration/planning concepts into onboard-autonomy --
see CHECKPOINT/CURRENT_STATE.md) to let a migrated exploration node request a
real path from onboard-autonomy's own planner instead of gps_denied's Nav2
`ComputePathToPose` action client, which is not installed on this Jetson and
would have been a second, competing planning path.

UNKNOWN CELLS ARE TREATED AS NOT TRAVERSABLE. Unlike `IndoorEnvironment`
(pure ground truth, no "unknown" state), a live/partial map has real unknown
cells. Routing a path through unexplored space is unsafe -- there might be a
wall there -- so any cell that is not confidently free (`0 <= value <
free_below`) is treated as occupied for planning purposes, exactly like an
out-of-bounds point already is in `IndoorEnvironment.is_occupied`.

`free_below` defaults to 50 (not a strict `== 0` check) to match the same
convention this migration's `coverage_grid.py` and the source
`coverage_tracker.py` already use: a real SLAM stack (e.g. Cartographer)
reports free space as a probability range, not always exactly 0.
"""
from __future__ import annotations

import math
from typing import Any


class OccupancyGridEnvironmentError(Exception):
    """Base class for every error raised by this module."""


class InvalidOccupancyGridError(OccupancyGridEnvironmentError):
    """Raised when `grid` doesn't have the structure this class requires --
    mirrors `frontier_detector.InvalidOccupancyGridError` so a malformed
    grid fails the same way in either consumer."""


class OccupancyGridEnvironment:
    """Read-only view over a `nav_msgs/OccupancyGrid`-as-seen-by-roslibjs
    dict (`custom-gcs/docs/DATA_MODELS.md` Section 4 shape), exposing the
    same query surface `GridAStarPlanner`/`PathPlanner` need:
    `origin`, `resolution_m`, `width_cells`, `height_cells`,
    `is_within_bounds`, `is_occupied`.

    Takes a snapshot of `grid` at construction time (does not track a live
    subscription) -- a caller that wants to plan against the newest map
    should construct a new instance per planning attempt, the same way a
    fresh `IndoorEnvironment` is constructed per test scenario.
    """

    def __init__(self, grid: dict[str, Any], free_below: int = 50) -> None:
        width, height, resolution, origin_x, origin_y, data = _validate_grid(grid)
        self._width_cells = width
        self._height_cells = height
        self._resolution_m = resolution
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._data = data
        self._free_below = free_below

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
    def origin(self) -> tuple[float, float]:
        return (self._origin_x, self._origin_y)

    def is_within_bounds(self, x: float, y: float) -> bool:
        width_m = self._width_cells * self._resolution_m
        height_m = self._height_cells * self._resolution_m
        return (
            self._origin_x <= x < self._origin_x + width_m
            and self._origin_y <= y < self._origin_y + height_m
        )

    def is_occupied(self, x: float, y: float) -> bool:
        """True if (x, y) is out of bounds, unknown, or occupied -- i.e.
        not confidently traversable. See module docstring for why unknown
        counts as occupied here."""
        if not self.is_within_bounds(x, y):
            return True
        col, row = self._cell_of(x, y)
        value = self._data[row * self._width_cells + col]
        return not (0 <= value < self._free_below)

    def _cell_of(self, x: float, y: float) -> tuple[int, int]:
        col = math.floor((x - self._origin_x) / self._resolution_m)
        row = math.floor((y - self._origin_y) / self._resolution_m)
        return col, row


def _validate_grid(grid: dict[str, Any]) -> tuple[int, int, float, float, float, list[int]]:
    """Same validation `frontier_detector._validate_grid` performs --
    duplicated rather than imported so this module has no dependency on
    frontier_detector.py, keeping the two independently usable."""
    if not isinstance(grid, dict) or "info" not in grid or "data" not in grid:
        raise InvalidOccupancyGridError("grid must be a dict with 'info' and 'data' keys")
    info = grid["info"]
    if not isinstance(info, dict):
        raise InvalidOccupancyGridError("grid['info'] must be a dict")
    for key in ("resolution", "width", "height", "origin"):
        if key not in info:
            raise InvalidOccupancyGridError(f"grid['info'] is missing '{key}'")

    width = info["width"]
    height = info["height"]
    resolution = info["resolution"]
    if not isinstance(width, int) or width <= 0:
        raise InvalidOccupancyGridError(f"grid['info']['width'] must be a positive int, got {width!r}")
    if not isinstance(height, int) or height <= 0:
        raise InvalidOccupancyGridError(f"grid['info']['height'] must be a positive int, got {height!r}")
    if not isinstance(resolution, (int, float)) or resolution <= 0:
        raise InvalidOccupancyGridError(
            f"grid['info']['resolution'] must be a positive number, got {resolution!r}"
        )

    origin = info["origin"]
    if not isinstance(origin, dict) or "position" not in origin:
        raise InvalidOccupancyGridError("grid['info']['origin'] must be a dict with 'position'")
    position = origin["position"]
    if not isinstance(position, dict) or "x" not in position or "y" not in position:
        raise InvalidOccupancyGridError(
            "grid['info']['origin']['position'] must be a dict with 'x' and 'y'"
        )

    data = grid["data"]
    if not isinstance(data, list):
        raise InvalidOccupancyGridError("grid['data'] must be a list")
    if len(data) != width * height:
        raise InvalidOccupancyGridError(
            f"grid['data'] has length {len(data)}, expected width*height={width * height}"
        )

    return width, height, float(resolution), float(position["x"]), float(position["y"]), data
