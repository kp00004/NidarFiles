"""Deterministic grid-based A* implementation of `PathPlanner`.

CHECKPOINT/AUTONOMY_ROADMAP.md Phase 7: this is the concrete planner
`path_planner_interface.PathPlanner` describes, built and tested against
Phase 3's `indoor_environment.IndoorEnvironment` ground truth. Pure
Python, no ROS/rclpy dependency, no wall-clock or randomness -- same
determinism posture as `mock_flight_controller.py`'s `tick(dt)`.
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Set, Tuple

from .indoor_environment import IndoorEnvironment
from .path_planner_interface import NoPathFoundError, PathPlanner

# Fixed, deterministic 8-connected neighbor iteration order -- the same
# (dcol, drow) enumeration test_indoor_environment.py's independent BFS
# reachability helper uses, so both this planner and that test's oracle
# walk the grid the same way. What matters for determinism is only that
# the order is fixed, not which fixed order it is.
_NEIGHBOR_OFFSETS: Tuple[Tuple[int, int], ...] = tuple(
    (dcol, drow)
    for dcol in (-1, 0, 1)
    for drow in (-1, 0, 1)
    if not (dcol == 0 and drow == 0)
)

# Path-simplification collinearity tolerance. Cell-center coordinates
# are exact `origin + (index + 0.5) * resolution` sums, so genuinely
# collinear triplets should cross-product to exactly 0.0, but this
# leaves a hair of slack for float accumulation rather than requiring
# bit-exact equality.
_COLLINEAR_EPSILON = 1e-9


class GridAStarPlanner(PathPlanner):
    """8-connected A* over `environment`'s own occupied-cell grid.

    A* (over Dijkstra, greedy best-first, or a sampling-based planner
    like RRT) fits this problem specifically: the search space is
    already `IndoorEnvironment`'s small (<=15x15 cells at the default 1m
    resolution), fully-known, discretized occupancy grid, so an
    exhaustive-but-heuristic-guided search is cheap and finds a
    cost-optimal path under this planner's own edge-cost model, and
    (unlike RRT) is deterministic by construction rather than needing a
    seeded RNG -- the "same query twice -> identical output" requirement
    this planner is tested against rules out any sampling-based
    alternative outright.

    Determinism rests on two things, both required, neither optional:
    (1) `_NEIGHBOR_OFFSETS` is a fixed tuple, iterated in the same order
    every time, never a `set`/`dict` whose iteration order isn't
    part of its contract; (2) the open-set priority queue breaks ties
    between equal-`f_score` entries by a monotonically increasing
    insertion counter, never by comparing cell tuples directly (which
    would work but isn't the actual tie-break driving determinism here)
    and never by `dict`/`set` iteration order.

    Cell quantization re-implements `IndoorEnvironment`'s own documented
    D-7 formula (`cell = (floor((x - origin_x) / resolution_m),
    floor((y - origin_y) / resolution_m))`) against its public `origin`/
    `resolution_m` properties -- this is the contract `IndoorEnvironment`
    itself documents for exactly this purpose, not a re-derivation that
    could drift from it. Occupancy is always re-checked live via
    `environment.is_occupied`, never cached into a second grid structure
    that could go stale relative to it.

    `start == goal` returns the single-waypoint path `[start]` -- there
    is no motion to plan, and returning an empty list would violate
    `PathPlanner.plan`'s "never empty on success" contract.
    """

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        environment: IndoorEnvironment,
    ) -> List[Tuple[float, float]]:
        self._validate_endpoints(start, goal, environment)

        if start == goal:
            return [start]

        start_cell = self._cell_of(start, environment)
        goal_cell = self._cell_of(goal, environment)

        if start_cell == goal_cell:
            # Distinct start/goal that quantize to the same free cell --
            # the straight segment between them never leaves that one
            # free cell, so no grid search is needed.
            return [start, goal]

        cell_path = self._search(start_cell, goal_cell, environment)
        waypoints = [self._cell_center(cell, environment) for cell in cell_path]
        waypoints = self._simplify_collinear(waypoints)
        # The grid search operates on cell centers, but `plan()` promises
        # the first/last waypoints are exactly `start`/`goal` -- swap
        # them in after simplification so simplification only ever sees
        # grid-aligned points (keeps its exact-collinearity check clean).
        waypoints[0] = start
        waypoints[-1] = goal
        return waypoints

    # -- grid quantization --------------------------------------------------

    @staticmethod
    def _cell_of(
        point: Tuple[float, float], environment: IndoorEnvironment
    ) -> Tuple[int, int]:
        origin_x, origin_y = environment.origin
        resolution = environment.resolution_m
        x, y = point
        return (
            math.floor((x - origin_x) / resolution),
            math.floor((y - origin_y) / resolution),
        )

    @staticmethod
    def _cell_center(
        cell: Tuple[int, int], environment: IndoorEnvironment
    ) -> Tuple[float, float]:
        origin_x, origin_y = environment.origin
        resolution = environment.resolution_m
        col, row = cell
        return (
            origin_x + (col + 0.5) * resolution,
            origin_y + (row + 0.5) * resolution,
        )

    @staticmethod
    def _cell_within_grid(cell: Tuple[int, int], environment: IndoorEnvironment) -> bool:
        col, row = cell
        return 0 <= col < environment.width_cells and 0 <= row < environment.height_cells

    @classmethod
    def _cell_is_occupied(cls, cell: Tuple[int, int], environment: IndoorEnvironment) -> bool:
        return environment.is_occupied(*cls._cell_center(cell, environment))

    # -- search ---------------------------------------------------------------

    @classmethod
    def _neighbors(
        cls, cell: Tuple[int, int], environment: IndoorEnvironment
    ) -> List[Tuple[Tuple[int, int], float]]:
        col, row = cell
        results: List[Tuple[Tuple[int, int], float]] = []
        for dcol, drow in _NEIGHBOR_OFFSETS:
            neighbor = (col + dcol, row + drow)
            if not cls._cell_within_grid(neighbor, environment):
                continue
            if cls._cell_is_occupied(neighbor, environment):
                continue
            if dcol != 0 and drow != 0:
                # Refuse to cut a diagonal move between two orthogonally
                # occupied cells, even though a real vehicle needs
                # footprint clearance IndoorEnvironment's own point-based
                # is_path_clear doesn't model at the exact shared corner
                # -- a diagonal "squeeze" is not a move this planner
                # should offer, even if the ground-truth model wouldn't
                # itself flag it.
                orthogonal_a = (col + dcol, row)
                orthogonal_b = (col, row + drow)
                if cls._cell_is_occupied(orthogonal_a, environment) or cls._cell_is_occupied(
                    orthogonal_b, environment
                ):
                    continue
            step_cost = math.hypot(dcol, drow)
            results.append((neighbor, step_cost))
        return results

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        # Octile distance -- admissible and consistent for this planner's
        # own edge-cost model (1.0 orthogonal, sqrt(2) diagonal), so A*
        # remains cost-optimal under that model, not just "a" heuristic.
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    @classmethod
    def _search(
        cls,
        start_cell: Tuple[int, int],
        goal_cell: Tuple[int, int],
        environment: IndoorEnvironment,
    ) -> List[Tuple[int, int]]:
        counter = 0
        open_heap: List[Tuple[float, int, Tuple[int, int]]] = [
            (cls._heuristic(start_cell, goal_cell), counter, start_cell)
        ]
        g_score: Dict[Tuple[int, int], float] = {start_cell: 0.0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        closed: Set[Tuple[int, int]] = set()

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                # Stale queue entry -- a cheaper route to `current` was
                # already settled (lazy-deletion, no decrease-key).
                continue
            if current == goal_cell:
                return cls._reconstruct_path(came_from, current)
            closed.add(current)

            for neighbor, step_cost in cls._neighbors(current, environment):
                if neighbor in closed:
                    continue
                tentative_g = g_score[current] + step_cost
                if tentative_g < g_score.get(neighbor, math.inf):
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    counter += 1
                    f_score = tentative_g + cls._heuristic(neighbor, goal_cell)
                    heapq.heappush(open_heap, (f_score, counter, neighbor))

        raise NoPathFoundError(
            f"no path exists from cell {start_cell} to cell {goal_cell}"
        )

    @staticmethod
    def _reconstruct_path(
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # -- path simplification ---------------------------------------------------

    @staticmethod
    def _is_collinear(
        a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]
    ) -> bool:
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        return abs(cross) < _COLLINEAR_EPSILON

    @classmethod
    def _simplify_collinear(
        cls, waypoints: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """Drop exactly-collinear intermediate waypoints. Each dropped
        point sits exactly on the straight segment between the last kept
        point and its own immediate successor, so removing it doesn't
        change the polyline's actual geometry -- deliberately not
        smoothing (that's a later, separate concern, see this module's
        docstring and AUTONOMY_ROADMAP.md Phase 7/8)."""
        if len(waypoints) <= 2:
            return list(waypoints)
        simplified = [waypoints[0]]
        for i in range(1, len(waypoints) - 1):
            prev = simplified[-1]
            current = waypoints[i]
            nxt = waypoints[i + 1]
            if not cls._is_collinear(prev, current, nxt):
                simplified.append(current)
        simplified.append(waypoints[-1])
        return simplified
