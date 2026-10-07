"""Simulated LiDAR-like sensor model -- pure logic, no rclpy.

Added for the NIDAR simulation harness (`mission_simulator.py`, see
CHECKPOINT/CURRENT_STATE.md). This is the "simulated occupancy
observation" adapter the simulation architecture calls for: given
`indoor_environment.IndoorEnvironment`'s ground truth and the simulated
vehicle's current pose, decide which cells the vehicle can *currently*
see, the same job a real 2D LiDAR + SLAM front-end would do for the real
system (AUTONOMY_ROADMAP.md Phase 4, not implemented here or anywhere
else in this repo yet).

This module deliberately knows nothing about ROS, ticks, or motion -- it
answers exactly one question, "what is visible from this pose right
now", as a pure function of `(IndoorEnvironment, pose, range)`. The
caller (`mission_simulator.py`) is responsible for merging newly-visible
cells into a running "known map" over time, the same way a real
SLAM/occupancy-mapping pipeline would.

360-degree LiDAR model (no field-of-view cone, unlike
`coverage_grid.py`'s camera model) -- visibility is range + line-of-sight
only. A cell is visible if a straight ray from the sensor pose to that
cell's center is unobstructed by any *other* occupied cell; the cell
itself may be free (revealed as such) or occupied (revealed as a wall --
this is how a wall first becomes known, exactly like a real LiDAR return
stopping at the first surface it hits).
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

from .indoor_environment import IndoorEnvironment

Cell = Tuple[int, int]

# Sample the ray at this many points per cell width -- fine enough that a
# thin wall between the sensor and a farther cell is never stepped over,
# matching IndoorEnvironment.is_path_clear's own sampling density.
_SAMPLES_PER_CELL = 10


def _line_of_sight_clear(
    environment: IndoorEnvironment, x0: float, y0: float, x1: float, y1: float
) -> bool:
    """True if every sampled point along the straight segment from
    (x0, y0) to (x1, y1) is free, EXCEPT points that fall inside the
    TARGET cell itself (the cell containing (x1, y1)) -- not just the
    literal endpoint. A LiDAR ray that hits a wall reveals that wall cell;
    since the target cell has real width, several samples near the end of
    the ray can land inside it before t reaches exactly 1.0, so excluding
    only the exact final point (as IndoorEnvironment.is_path_clear does)
    is not enough -- those in-between samples would otherwise report the
    wall as blocking itself and it could never be revealed at all.
    Cells other than the target, anywhere along the ray, are still
    checked normally -- only the target cell gets this exemption."""
    dx = x1 - x0
    dy = y1 - y0
    distance = math.hypot(dx, dy)
    if distance <= 1e-9:
        return True

    resolution = environment.resolution_m
    origin_x, origin_y = environment.origin
    target_col = math.floor((x1 - origin_x) / resolution)
    target_row = math.floor((y1 - origin_y) / resolution)

    step = resolution / _SAMPLES_PER_CELL
    steps = max(1, math.ceil(distance / step))
    for i in range(steps + 1):
        t = i / steps
        x = x0 + dx * t
        y = y0 + dy * t
        col = math.floor((x - origin_x) / resolution)
        row = math.floor((y - origin_y) / resolution)
        if (col, row) == (target_col, target_row):
            continue
        if environment.is_occupied(x, y):
            return False
    return True


def reveal_cells(
    environment: IndoorEnvironment, pose: Tuple[float, float], sensor_range_m: float
) -> Dict[Cell, int]:
    """Return `{(col, row): value}` for every cell within `sensor_range_m`
    of `pose` that currently has line of sight from it, `value` being that
    cell's ground-truth occupancy (`0` free or `100` occupied, per
    `IndoorEnvironment.to_occupancy_grid_data()`'s own encoding). Cells
    outside the environment's bounds are never included. Deterministic,
    a pure function of its inputs -- no randomness, no history."""
    px, py = pose
    resolution = environment.resolution_m
    origin_x, origin_y = environment.origin
    result: Dict[Cell, int] = {}

    cell_radius = math.ceil(sensor_range_m / resolution)
    center_col = math.floor((px - origin_x) / resolution)
    center_row = math.floor((py - origin_y) / resolution)

    for drow in range(-cell_radius, cell_radius + 1):
        for dcol in range(-cell_radius, cell_radius + 1):
            col = center_col + dcol
            row = center_row + drow
            if col < 0 or row < 0 or col >= environment.width_cells or row >= environment.height_cells:
                continue
            cx = origin_x + (col + 0.5) * resolution
            cy = origin_y + (row + 0.5) * resolution
            if math.hypot(cx - px, cy - py) > sensor_range_m:
                continue
            if not _line_of_sight_clear(environment, px, py, cx, cy):
                continue
            result[(col, row)] = 100 if environment.is_occupied(cx, cy) else 0
    return result
