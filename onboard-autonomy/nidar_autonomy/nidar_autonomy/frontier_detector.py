"""Frontier detection over a partially-explored occupancy grid.

CHECKPOINT/AUTONOMY_ROADMAP.md Phase 6: given a map with free/occupied/
unknown cells, find and score candidate exploration targets ("frontier"
cells -- the boundary between explored-free space and unknown space).
Pure Python, no ROS/rclpy dependency, no wall-clock or randomness --
same determinism posture as `grid_astar_planner.py`.

This module deliberately consumes the generic occupancy-grid *dict*
shape directly (`custom-gcs/docs/DATA_MODELS.md` Section 4's
`nav_msgs/OccupancyGrid`-as-seen-by-roslibjs shape,
`{"info": {"resolution", "width", "height", "origin": {"position":
{"x", "y"}}}, "data": [...]}`, row-major, `-1`=unknown, `0`=free,
`100`=occupied), not `IndoorEnvironment` -- `IndoorEnvironment.
to_occupancy_grid_data()` only ever produces full ground truth (0/100,
never -1), so it is not itself usable as partially-explored test input.
Test fixtures build partial-exploration states by masking that ground
truth to -1; this module itself has no `IndoorEnvironment` import and
never will, so it keeps working unchanged once a real Phase 5 SLAM
publisher produces the same dict shape from an actual partial map.

This module does not call, import, or interact with
`FlightCommandInterface`/`MockFlightController`/`MAVROSFlightController`
in any way -- it only ever reads a map dict and returns candidate
target positions, the same category of work as `grid_astar_planner.py`.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Set, Tuple

FREE = 0
OCCUPIED = 100
UNKNOWN = -1

# 8-connected for both "does this free cell border unknown space" and
# "are these two frontier cells part of the same region" -- a cell that
# only touches unknown space diagonally is still a real vantage point a
# vehicle sitting there could see into, so 4-connectivity would
# understate the actual known/unknown boundary. The extra neighbor
# checks are trivial at this project's arena-scale grid sizes (<=15x15
# cells at 1m resolution today, per REQUIREMENTS.md's <=15m x 15m
# ceiling), so there's no real cost to the more thorough choice.
_NEIGHBOR_OFFSETS: Tuple[Tuple[int, int], ...] = tuple(
    (dcol, drow)
    for dcol in (-1, 0, 1)
    for drow in (-1, 0, 1)
    if not (dcol == 0 and drow == 0)
)

Cell = Tuple[int, int]


class FrontierDetectionError(Exception):
    """Base class for every error raised by this module."""


class InvalidOccupancyGridError(FrontierDetectionError):
    """Raised when `grid` doesn't have the structure `detect_frontiers`
    requires -- a missing key or `len(data) != width * height` --
    rather than letting a malformed grid crash on an index error deep
    inside the algorithm or silently produce garbage candidates."""


@dataclass(frozen=True)
class FrontierCandidate:
    """One candidate exploration target: a contiguous frontier region's
    representative world-coordinate point, plus a size-derived score.

    `x`/`y` are the centroid (arithmetic mean of cell centers) of the
    region's frontier cells, in the same world frame as the grid's
    `info.origin`. This is a simple, well-defined representative point,
    not necessarily itself a free cell (a concave or irregularly-shaped
    frontier region's centroid can sit outside the region) -- callers
    that need a guaranteed-free waypoint should path-plan a nearby free
    cell to this point rather than fly to `(x, y)` unchecked. Picking a
    single representative point for a whole region (rather than every
    individual frontier cell) is what keeps a long wall-of-unknown-
    boundary from producing dozens of near-duplicate candidates.

    `score` is proportional to the total size (cell count) of the
    contiguous unknown region(s) this frontier region borders -- a
    bigger unknown area behind a frontier is generally more valuable to
    explore next than a tiny unknown pocket. This is deliberately a
    single-factor score for a first version (see module docstring);
    later phases (Phase 9 Mission Manager) can layer in distance-to-
    vehicle, revisit cost, etc. without this module needing to change
    shape.

    `cell_count` is the number of frontier cells making up this
    candidate's region -- kept alongside `score` so a caller/test can
    see what the region actually looked like without recomputing it.
    """

    x: float
    y: float
    score: float
    cell_count: int


def detect_frontiers(grid: Dict) -> List[FrontierCandidate]:
    """Return candidate exploration targets for `grid`, one per
    contiguous frontier region, ordered deterministically (by each
    region's first cell in row-major scan order) and identically across
    repeated calls on the same input.

    A "frontier cell" is a free (`0`) cell with at least one unknown
    (`-1`) cell among its 8-connected neighbors. Adjacent frontier cells
    are grouped into contiguous regions (8-connected) rather than
    returned individually. Returns `[]` if `grid` has no frontier cells
    at all -- both a fully-explored grid (no unknown cells left to
    border) and a fully-unknown grid (no free cells to be a frontier
    from) hit this path. The fully-unknown case is deliberately *not*
    treated as an error or given a fallback candidate: with literally
    nothing explored yet, this module has no boundary to report --
    bootstrapping exploration from a known start pose before any map
    exists is a Mission Manager (Phase 9) concern, not this pure
    grid-analysis function's job.

    Raises `InvalidOccupancyGridError` if `grid` is missing required
    keys or `data`'s length doesn't match `width * height`.
    """
    width, height, resolution, origin_x, origin_y, data = _validate_grid(grid)

    frontier_cells = _find_frontier_cells(data, width, height)
    if not frontier_cells:
        return []

    component_of, component_size = _label_unknown_components(data, width, height)
    regions = _group_frontier_regions(frontier_cells, width, height)

    candidates: List[FrontierCandidate] = []
    for region in regions:
        sum_x = 0.0
        sum_y = 0.0
        touched_components: Set[int] = set()
        for col, row in region:
            cx, cy = _cell_center(col, row, resolution, origin_x, origin_y)
            sum_x += cx
            sum_y += cy
            for dcol, drow in _NEIGHBOR_OFFSETS:
                neighbor = (col + dcol, row + drow)
                if not _in_bounds(neighbor, width, height):
                    continue
                if data[_index(neighbor, width)] == UNKNOWN:
                    touched_components.add(component_of[neighbor])
        cell_count = len(region)
        score = float(sum(component_size[cid] for cid in touched_components))
        candidates.append(
            FrontierCandidate(
                x=sum_x / cell_count,
                y=sum_y / cell_count,
                score=score,
                cell_count=cell_count,
            )
        )
    return candidates


# -- grid validation ----------------------------------------------------------


def _validate_grid(grid: Dict) -> Tuple[int, int, float, float, float, List[int]]:
    if not isinstance(grid, dict) or "info" not in grid or "data" not in grid:
        raise InvalidOccupancyGridError(
            "grid must be a dict with 'info' and 'data' keys"
        )
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


# -- cell/coordinate helpers ---------------------------------------------------


def _index(cell: Cell, width: int) -> int:
    col, row = cell
    return row * width + col


def _in_bounds(cell: Cell, width: int, height: int) -> bool:
    col, row = cell
    return 0 <= col < width and 0 <= row < height


def _cell_center(
    col: int, row: int, resolution: float, origin_x: float, origin_y: float
) -> Tuple[float, float]:
    # Same cell-center convention as grid_astar_planner._cell_center --
    # deliberately kept in sync so a frontier candidate's world position
    # means the same thing a planner waypoint's does.
    return origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution


# -- frontier-cell identification -----------------------------------------------


def _find_frontier_cells(data: List[int], width: int, height: int) -> List[Cell]:
    """Free cells with >=1 unknown 8-connected neighbor, in row-major
    scan order -- the fixed order this whole module's determinism rests
    on for region discovery below."""
    frontier: List[Cell] = []
    for row in range(height):
        for col in range(width):
            if data[_index((col, row), width)] != FREE:
                continue
            for dcol, drow in _NEIGHBOR_OFFSETS:
                neighbor = (col + dcol, row + drow)
                if _in_bounds(neighbor, width, height) and data[_index(neighbor, width)] == UNKNOWN:
                    frontier.append((col, row))
                    break
    return frontier


def _group_frontier_regions(
    frontier_cells: List[Cell], width: int, height: int
) -> List[List[Cell]]:
    """Group `frontier_cells` (already in row-major order) into
    8-connected contiguous regions. Regions are returned in the order
    their first (row-major-earliest) cell was discovered; each region's
    own cell list is in BFS-visit order -- fixed given `_NEIGHBOR_OFFSETS`'
    fixed iteration order, so grouping is deterministic without relying
    on any `set`/`dict` iteration order for output shape."""
    frontier_set: Set[Cell] = set(frontier_cells)
    visited: Set[Cell] = set()
    regions: List[List[Cell]] = []

    for start in frontier_cells:
        if start in visited:
            continue
        region: List[Cell] = []
        queue: deque = deque([start])
        visited.add(start)
        while queue:
            current = queue.popleft()
            region.append(current)
            col, row = current
            for dcol, drow in _NEIGHBOR_OFFSETS:
                neighbor = (col + dcol, row + drow)
                if neighbor in frontier_set and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        regions.append(region)
    return regions


# -- unknown-region connected-component labeling --------------------------------


def _label_unknown_components(
    data: List[int], width: int, height: int
) -> Tuple[Dict[Cell, int], Dict[int, int]]:
    """Flood-fill every unknown (`-1`) cell into 8-connected connected
    components, in row-major scan order for the same fixed-order
    determinism reason as `_find_frontier_cells`. Labeling every unknown
    cell once up front (rather than flood-filling per frontier region on
    demand) means each region's score is just a lookup-and-sum over
    already-known component sizes -- cheap, and avoids re-deriving the
    same flood fill repeatedly when several frontier regions border the
    same unknown blob."""
    component_of: Dict[Cell, int] = {}
    component_size: Dict[int, int] = {}
    next_id = 0

    for row in range(height):
        for col in range(width):
            start = (col, row)
            if data[_index(start, width)] != UNKNOWN or start in component_of:
                continue
            component_id = next_id
            next_id += 1
            size = 0
            queue: deque = deque([start])
            component_of[start] = component_id
            while queue:
                current = queue.popleft()
                size += 1
                ccol, crow = current
                for dcol, drow in _NEIGHBOR_OFFSETS:
                    neighbor = (ccol + dcol, crow + drow)
                    if not _in_bounds(neighbor, width, height):
                        continue
                    if neighbor in component_of:
                        continue
                    if data[_index(neighbor, width)] != UNKNOWN:
                        continue
                    component_of[neighbor] = component_id
                    queue.append(neighbor)
            component_size[component_id] = size

    return component_of, component_size
