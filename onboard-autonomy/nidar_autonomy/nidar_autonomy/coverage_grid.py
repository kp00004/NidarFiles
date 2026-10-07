"""Coverage grid -- pure logic, no rclpy.

Migrated/adapted from gps_denied/raj-dev's coverage_tracker.py (NIDAR
Autonomy Migration, see CHECKPOINT/CURRENT_STATE.md). The ROS plumbing (TF
lookup, `OccupancyGrid`/`Float32` publishers, the update timer) was stripped
out; the actual coverage-classification/visibility algorithm is preserved
here so it is unit-testable without a ROS install, matching this repo's
existing "pure logic module + thin rclpy node" split
(`state_machine.py`/`mission_state_node.py`,
`arming_guard.py`/`flight_command.py`).

WHY THIS EXISTS
================
Frontier exploration maps where the *walls* are. But mapping a corridor's
walls is NOT the same as pointing a camera into every place a survivor could
be. This module maintains a second, coarse grid over the arena (the same
grid the mission rules want survivor locations reported in). Every cell
starts UNKNOWN; a cell becomes SEARCHED only once the vehicle has actually
been in a pose from which its camera could see that cell:
    * the cell is within camera range of the vehicle,
    * within the camera's field of view (around the vehicle's heading), and
    * has clear line of sight in the occupancy map (no wall in between).
So a cell can only be ticked off if the sensor genuinely covered it.

Consumes the same `nav_msgs/OccupancyGrid`-as-seen-by-roslibjs dict shape
(`custom-gcs/docs/DATA_MODELS.md` Section 4) that `frontier_detector.py` and
`occupancy_grid_environment.py` already use, for the same reason: it keeps
working unchanged once a real Phase 4/5 SLAM/mapping publisher produces that
shape, and is directly testable against `indoor_environment.py`'s ground
truth (via `IndoorEnvironment.to_occupancy_grid_data()`) today.
"""
from __future__ import annotations

import math
from typing import Any, Optional

UNKNOWN, UNSEARCHED, SEARCHED = -1, 0, 100

# Sample the cell center plus its four quarter-points, not just the center --
# a coarse coverage cell that straddles a thin wall has its center blocked
# from line of sight while its free part is perfectly visible, and a
# survivor there would be seen. Marking on ANY visible sub-point stops such
# cells stalling coverage forever. Mirrors coverage_tracker.py's own
# `_tick()` sampling exactly.
_SAMPLE_OFFSET_FRACTIONS: tuple[tuple[float, float], ...] = (
    (0.0, 0.0), (0.3, 0.3), (-0.3, 0.3), (0.3, -0.3), (-0.3, -0.3),
)

_CLASSIFY_OFFSET_FRACTIONS: tuple[tuple[float, float], ...] = (
    (0.0, 0.0), (0.25, 0.25), (-0.25, 0.25), (0.25, -0.25), (-0.25, -0.25),
)


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class CoverageGrid:
    """Owns the coverage state grid and the update algorithm. Mirrors
    `indoor_environment.IndoorEnvironment`'s "own its state, export in wire
    format" shape: constructed once, `update()`d repeatedly as new map data
    and vehicle poses arrive, `to_occupancy_grid_data()`/`percent_searched()`
    read out for telemetry/GCS use.
    """

    def __init__(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        cell_size: float = 1.0,
        camera_range_m: float = 2.0,
        camera_fov_deg: float = 100.0,
        free_below: int = 50,
        occupied_at: int = 65,
    ) -> None:
        if cell_size <= 0:
            raise ValueError("cell_size must be positive")
        if max_x <= min_x or max_y <= min_y:
            raise ValueError("max_x/max_y must exceed min_x/min_y")
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.cell_size = cell_size
        self.camera_range_m = camera_range_m
        self.camera_fov_rad = math.radians(camera_fov_deg)
        self.free_below = free_below
        self.occupied_at = occupied_at

        self.ncols = max(1, math.ceil((max_x - min_x) / cell_size))
        self.nrows = max(1, math.ceil((max_y - min_y) / cell_size))
        self.state: list[list[int]] = [[UNKNOWN] * self.ncols for _ in range(self.nrows)]

    # -- cell/coordinate helpers ------------------------------------------------

    def cell_center(self, col: int, row: int) -> tuple[float, float]:
        return (self.min_x + (col + 0.5) * self.cell_size,
                self.min_y + (row + 0.5) * self.cell_size)

    @staticmethod
    def _map_value(grid: dict[str, Any], wx: float, wy: float) -> Optional[int]:
        """Occupancy value at a world point from `grid` (the OccupancyGrid-
        shaped dict), or None if outside that grid's own bounds."""
        info = grid["info"]
        origin = info["origin"]["position"]
        resolution = info["resolution"]
        mx = int((wx - origin["x"]) / resolution)
        my = int((wy - origin["y"]) / resolution)
        if 0 <= mx < info["width"] and 0 <= my < info["height"]:
            return grid["data"][my * info["width"] + mx]
        return None

    def _classify(self, grid: dict[str, Any], col: int, row: int) -> str:
        """"unknown" / "free" / "occupied" for one coverage cell, by
        sampling the source map over the cell (center + quarter points) so a
        coarse coverage cell isn't judged by a single map pixel."""
        cx, cy = self.cell_center(col, row)
        free = occ = known = 0
        for fx, fy in _CLASSIFY_OFFSET_FRACTIONS:
            value = self._map_value(grid, cx + fx * self.cell_size, cy + fy * self.cell_size)
            if value is None or value == UNKNOWN:
                continue
            known += 1
            if value >= self.occupied_at:
                occ += 1
            elif 0 <= value < self.free_below:
                free += 1
        if known == 0:
            return "unknown"
        if occ > 0:
            return "occupied"
        if free > 0:
            return "free"
        return "unknown"

    def _line_of_sight_clear(self, grid: dict[str, Any], x0: float, y0: float, x1: float, y1: float) -> bool:
        """Line of sight between two world points across `grid`: step along
        the ray at the map's own resolution; blocked if any sampled cell is
        occupied."""
        info = grid["info"]
        resolution = info["resolution"]
        distance = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(distance / resolution))
        for i in range(1, steps):  # skip endpoints (drone cell, target cell)
            t = i / steps
            value = self._map_value(grid, x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if value is not None and value >= self.occupied_at:
                return False
        return True

    def _is_visible(self, grid: dict[str, Any], col: int, row: int, drone_x: float, drone_y: float, drone_yaw: float) -> bool:
        cx, cy = self.cell_center(col, row)
        half_fov = self.camera_fov_rad / 2.0
        for fx, fy in _SAMPLE_OFFSET_FRACTIONS:
            sx = cx + fx * self.cell_size
            sy = cy + fy * self.cell_size
            d = math.hypot(sx - drone_x, sy - drone_y)
            if d > self.camera_range_m:
                continue
            if d > 1e-3:
                bearing = math.atan2(sy - drone_y, sx - drone_x)
                if abs(wrap(bearing - drone_yaw)) > half_fov:
                    continue  # outside the camera cone
            if self._line_of_sight_clear(grid, drone_x, drone_y, sx, sy):
                return True
        return False

    # -- main update --------------------------------------------------------------

    def update(self, grid: dict[str, Any], drone_x: float, drone_y: float, drone_yaw: float) -> tuple[int, int]:
        """Advance coverage state given the latest source map and vehicle
        pose. Returns (free_total, searched_total) for this update -- the
        same pair `percent_searched()` derives from, exposed directly so a
        caller/test can check the raw counts without a separate call."""
        free_total = searched_total = 0
        for row in range(self.nrows):
            for col in range(self.ncols):
                state = self.state[row][col]
                if state != SEARCHED:
                    kind = self._classify(grid, col, row)
                    if kind == "free" and state == UNKNOWN:
                        self.state[row][col] = state = UNSEARCHED
                    elif kind != "free" and state == UNKNOWN:
                        continue  # wall or still-unknown: not a search cell
                if self.state[row][col] == UNKNOWN:
                    continue
                free_total += 1
                if self.state[row][col] == SEARCHED:
                    searched_total += 1
                    continue
                if self._is_visible(grid, col, row, drone_x, drone_y, drone_yaw):
                    self.state[row][col] = SEARCHED
                    searched_total += 1
        return free_total, searched_total

    def percent_searched(self) -> float:
        free_total = sum(1 for row in self.state for v in row if v != UNKNOWN)
        searched_total = sum(1 for row in self.state for v in row if v == SEARCHED)
        return (100.0 * searched_total / free_total) if free_total else 0.0

    def to_occupancy_grid_data(self) -> dict[str, Any]:
        """Export in the same `nav_msgs/OccupancyGrid`-as-seen-by-roslibjs
        shape `indoor_environment.IndoorEnvironment.to_occupancy_grid_data()`
        uses, so a `/coverage_grid` publisher wrapping this class can
        publish it with no reshaping."""
        data: list[int] = []
        for row in range(self.nrows):
            for col in range(self.ncols):
                data.append(self.state[row][col])
        return {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "info": {
                "resolution": self.cell_size,
                "width": self.ncols,
                "height": self.nrows,
                "origin": {
                    "position": {"x": self.min_x, "y": self.min_y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "data": data,
        }
