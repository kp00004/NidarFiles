"""Deterministic end-to-end mission simulator -- pure logic, no rclpy.

Built to answer a specific requirement: the custom GCS needs a "RUN
SIMULATION" path that exercises the REAL canonical autonomy decision-
making components end-to-end (frontier detection, exploration selection,
A* planning, mission-state progression) driving a REAL (simulated)
vehicle motion model, without ever touching the real Pixhawk/mavros path
or the real `/mission/state` topic. See CHECKPOINT/CURRENT_STATE.md for
the full design record.

This module is deliberately NOT a second autonomy implementation -- every
decision is made by already-canonical, already-tested modules, unchanged:

    indoor_environment.simulation_maze()   -- ground-truth world
    sensor_model.reveal_cells()            -- simulated LiDAR observation
    frontier_detector.detect_frontiers()   -- frontier detection (Phase 6)
    exploration_policy.score_candidates()  -- target selection (migrated)
    grid_astar_planner.GridAStarPlanner    -- path planning (Phase 7)
    occupancy_grid_environment             -- live-grid adapter for the
        .OccupancyGridEnvironment             planner (NIDAR Autonomy
                                               Migration)
    mock_flight_controller.MockFlightController -- deterministic vehicle
                                               motion (Phase 1/2)
    coverage_grid.CoverageGrid             -- search-coverage tracking
                                               (migrated)
    state_machine.MissionStateMachine      -- mission lifecycle (this
                                               module's own, separate
                                               instance -- never
                                               mission_state_node.py's)

This class only orchestrates calls between them and owns the translation
between the vehicle's local motion frame and the environment's world
frame (see `_to_world`/`_to_local`) -- the same "two frames related by a
fixed offset" concept `geometry.RigidTransform2D` generalizes for a real
SLAM<->FCU pairing, simplified here to a pure translation since the
simulated vehicle never yaws relative to world frame.

HARD SAFETY BOUNDARY: this module never imports `rclpy`, `mavros_msgs`,
`flight_command`, or `arming_guard` -- there is no import path from here
to the real Pixhawk. `MockFlightController.arm()`/`disarm()` only ever
mutate this class's own in-memory simulated state; they have no
side effect outside this process.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from .coverage_grid import CoverageGrid
from .exploration_policy import (
    Blacklist,
    VisitedMemory,
    apply_sweep_radius,
    clear_failure,
    record_failure,
    record_visit,
    score_candidates,
)
from .frontier_detector import detect_frontiers
from .grid_astar_planner import GridAStarPlanner
from .indoor_environment import IndoorEnvironment, simulation_maze
from .mock_flight_controller import MockFlightController
from .occupancy_grid_environment import OccupancyGridEnvironment
from .path_planner_interface import PathPlanningError
from .sensor_model import reveal_cells
from .state_machine import MissionStateMachine

Cell = tuple[int, int]

# Simulation lifecycle status -- distinct from MissionStateMachine's
# mission-lifecycle states (idle/entering/searching/exiting/complete/
# aborted). This answers "is the simulation itself running", the same
# question the GCS's RUN SIMULATION button needs answered; mission_state
# answers "what is the simulated mission doing right now".
_STATUS_IDLE = "idle"
_STATUS_RUNNING = "running"
_STATUS_COMPLETED = "completed"
_STATUS_FAILED = "failed"

_ARRIVAL_TOLERANCE_M = 0.05
_NEAREST_KNOWN_FREE_SEARCH_RADIUS_CELLS = 3


@dataclass(frozen=True)
class SimulationSnapshot:
    """Everything the GCS needs to render one moment of the simulation --
    deliberately the same *kind* of information the real
    telemetry_contract.py contract carries (position, map, coverage,
    navigation, autonomy, mission), reusing those field names/shapes so a
    frontend consumer can treat this uniformly with a light relabeling,
    without conflating it with the real contract (see main.py's
    SimulationStatusResponse, a distinct Pydantic type)."""

    status: str
    mission_state: str
    step: int
    elapsed_sim_seconds: float
    pose: dict[str, float]  # {x, y, z, yaw_deg}
    map: dict[str, Any]  # nav_msgs/OccupancyGrid-shaped dict (known cells only)
    coverage: dict[str, Any]  # same shape, coverage semantics
    planned_path: list[dict[str, float]]  # [{x, y}, ...], world frame
    target: Optional[list[float]]
    frontier_count: int
    candidate_count: int
    blacklisted_count: int
    # Two DIFFERENT metrics, deliberately both exposed rather than
    # collapsed into one -- they answer different questions and can
    # diverge sharply (verified: `explored_pct` reaches 100 almost
    # immediately in this simulation's own runs, because a coverage cell
    # only needs to be observed once with a clear enough view to count as
    # "searched" relative to what's *currently known* -- it is NOT a
    # measure of how much of the whole arena has been discovered yet.
    # `map_known_pct` is that measure):
    #   explored_pct   -- coverage.py's search-coverage percent, matching
    #                     the real telemetry contract's mapping.explored_pct
    #                     naming (searched / currently-known-free cells).
    #   map_known_pct  -- fraction of the WHOLE arena's cells (free or
    #                     occupied) this simulation has revealed so far.
    #                     This is the number that actually answers "how
    #                     much of the map is done" and grows visibly
    #                     step by step -- render THIS for a progress
    #                     indicator, not explored_pct.
    explored_pct: float
    map_known_pct: float
    error: Optional[str] = None


class MissionSimulator:
    """Owns one complete, resettable simulated mission. Call `run()` to
    (re)start from a clean world, then `step()` repeatedly (each step is
    one fixed-`dt` tick of simulated time) until `snapshot().status` is
    "completed" or "failed". `run_to_completion()` does that loop
    in-process, for tests and for any caller that doesn't need to
    visualize intermediate steps.
    """

    def __init__(
        self,
        *,
        sensor_range_m: float = 2.0,
        takeoff_height_m: float = 1.0,
        dt_s: float = 0.5,
        max_steps: int = 5000,
        min_frontier_cells: int = 3,
        coverage_cell_size_m: float = 1.0,
        camera_range_m: float = 2.5,
        camera_fov_deg: float = 360.0,
        min_goal_distance_m: float = 0.05,
        empty_cycles_to_finish: int = 4,
    ) -> None:
        self._sensor_range_m = sensor_range_m
        self._takeoff_height_m = takeoff_height_m
        self._dt_s = dt_s
        self._max_steps = max_steps
        self._min_frontier_cells = min_frontier_cells
        self._coverage_cell_size_m = coverage_cell_size_m
        self._camera_range_m = camera_range_m
        self._camera_fov_deg = camera_fov_deg
        # exploration_policy.py's own default (0.7m) was tuned for
        # gps_denied's finer real-world SLAM resolution; at this
        # simulation's 1m grid cells and a modest sensor range, a
        # newly-formed frontier often sits well under 0.7m from the
        # vehicle's own position right after a move, which wrongly
        # excludes it as "already here" and can prematurely end
        # exploration. Empirically tuned smaller for this grid scale --
        # see test_mission_simulator.py's coverage/premature-completion
        # tests, which would fail if this regressed back toward 0.7.
        self._min_goal_distance_m = min_goal_distance_m
        self._empty_cycles_to_finish = empty_cycles_to_finish

        self._environment: Optional[IndoorEnvironment] = None
        self._known_map: dict[Cell, int] = {}
        self._coverage: Optional[CoverageGrid] = None
        self._controller: Optional[MockFlightController] = None
        self._machine: Optional[MissionStateMachine] = None
        self._planner = GridAStarPlanner()

        self._visited: VisitedMemory = {}
        self._blacklist: Blacklist = {}
        self._goal: Optional[tuple[float, float]] = None
        self._path: list[tuple[float, float]] = []
        self._entry_offset: tuple[float, float] = (0.0, 0.0)

        self._status = _STATUS_IDLE
        self._steps = 0
        self._error: Optional[str] = None
        self._last_frontier_count = 0
        self._last_candidate_count = 0
        self._empty_scored_streak = 0

    # -- lifecycle -----------------------------------------------------------

    def run(self) -> SimulationSnapshot:
        """(Re)start a fresh simulated mission -- always resets the world
        first, so repeated calls are the "click RUN SIMULATION again"
        repeatability the GCS needs, never a continuation of a stale
        run."""
        self._environment = simulation_maze()
        self._known_map = {}
        min_x, min_y = self._environment.origin
        max_x = min_x + self._environment.width_cells * self._environment.resolution_m
        max_y = min_y + self._environment.height_cells * self._environment.resolution_m
        self._coverage = CoverageGrid(
            min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y,
            cell_size=self._coverage_cell_size_m,
            camera_range_m=self._camera_range_m,
            camera_fov_deg=self._camera_fov_deg,
        )
        self._controller = MockFlightController()
        self._machine = MissionStateMachine()
        self._visited = {}
        self._blacklist = {}
        self._goal = None
        self._path = []
        self._entry_offset = self._environment.start_pose
        self._status = _STATUS_RUNNING
        self._steps = 0
        self._error = None
        self._last_frontier_count = 0
        self._last_candidate_count = 0
        self._empty_scored_streak = 0

        self._machine.handle_command("start")  # idle -> entering
        self._controller.arm()
        self._controller.takeoff(self._takeoff_height_m)

        return self.snapshot()

    def reset(self) -> SimulationSnapshot:
        """Stop wherever the simulation is and return to a fresh idle
        snapshot, without starting a new run. Distinct from `run()`
        (which both resets AND starts) so the GCS can offer a plain
        "stop and clear" action if it wants one, per the simulation API
        design."""
        self._environment = None
        self._known_map = {}
        self._coverage = None
        self._controller = None
        self._machine = None
        self._visited = {}
        self._blacklist = {}
        self._goal = None
        self._path = []
        self._entry_offset = (0.0, 0.0)
        self._status = _STATUS_IDLE
        self._steps = 0
        self._error = None
        self._last_frontier_count = 0
        self._last_candidate_count = 0
        self._empty_scored_streak = 0
        return self.snapshot()

    # -- per-step advance ---------------------------------------------------

    def step(self) -> SimulationSnapshot:
        """Advance the simulation by exactly one `dt_s`-second tick. A
        no-op (returns the current snapshot unchanged) once `status` is
        anything other than "running". Never raises for a normal planning
        failure (blacklists and retries next step instead) -- only an
        unexpected/unrecoverable condition sets `status = "failed"`."""
        if self._status != _STATUS_RUNNING:
            return self.snapshot()
        assert self._environment is not None and self._controller is not None and self._machine is not None

        world_x, world_y, world_z = self._to_world(self._controller.position)
        self._observe(world_x, world_y)
        record_visit(self._visited, world_x, world_y, cell_size=0.75)

        state = self._machine.state
        if state == "entering":
            self._step_entering(world_z)
        elif state == "searching":
            self._step_searching(world_x, world_y)
        elif state == "exiting":
            self._step_exiting(world_x, world_y)
        # "complete"/"aborted": nothing left to advance.

        self._steps += 1
        if self._status == _STATUS_RUNNING and self._steps >= self._max_steps:
            self._status = _STATUS_FAILED
            self._error = (
                f"exceeded max_steps safety bound ({self._max_steps}) without reaching "
                "'complete' -- this is a circuit breaker, not the intended completion path"
            )

        return self.snapshot()

    def run_to_completion(self, max_steps: Optional[int] = None) -> SimulationSnapshot:
        """Convenience for tests and non-realtime callers: `run()` then
        `step()` repeatedly until `status` leaves "running". `max_steps`
        here only bounds THIS call's loop (defaults to the instance's own
        safety bound) -- it does not change the instance's own
        `max_steps` circuit breaker."""
        bound = max_steps if max_steps is not None else self._max_steps
        snapshot = self.run()
        while snapshot.status == _STATUS_RUNNING and snapshot.step < bound:
            snapshot = self.step()
        return snapshot

    # -- per-state step logic -------------------------------------------------

    def _step_entering(self, world_z: float) -> None:
        assert self._controller is not None and self._machine is not None
        if world_z >= self._takeoff_height_m - _ARRIVAL_TOLERANCE_M:
            self._machine.handle_exploration_started()  # entering -> searching
            return
        self._controller.tick(self._dt_s)

    def _step_searching(self, world_x: float, world_y: float) -> None:
        assert self._controller is not None and self._machine is not None
        if self._path:
            self._advance_along_path(world_x, world_y)
            return

        grid = self._known_grid_dict()
        candidates = [
            c for c in detect_frontiers(grid) if c.cell_count >= self._min_frontier_cells
        ]
        self._last_frontier_count = len(candidates)
        scored = score_candidates(
            candidates,
            pose=(world_x, world_y),
            current_goal=self._goal,
            visited=self._visited,
            blacklist=self._blacklist,
            min_goal_distance=self._min_goal_distance_m,
        )
        self._last_candidate_count = len(scored)

        if not scored:
            # A single empty-scored tick is NOT reliable evidence
            # exploration is actually done -- it commonly just means the
            # only currently-known frontier region's centroid happens to
            # sit within min_goal_distance of wherever the vehicle just
            # arrived (the known-free area is still one connected blob
            # early on, so its whole boundary is one frontier region
            # whose centroid tracks close behind the vehicle). Require
            # several CONSECUTIVE empty ticks -- during which the vehicle
            # has nothing better to do but re-observe from where it is,
            # which can reveal a fresh, farther-away piece of the same
            # frontier -- before concluding there is truly nothing left
            # reachable. This is a plain topological debounce, not a
            # port of gps_denied's flight-tuned coverage-plateau/stuck-
            # detection heuristics (deliberately not migrated -- see the
            # module docstring).
            self._empty_scored_streak += 1
            if self._empty_scored_streak < self._empty_cycles_to_finish:
                self._goal = None
                return
            self._goal = None
            self._machine.handle_exploration_complete()  # searching -> exiting
            self._plan_path_to(self._entry_offset, on_failure=self._fail)
            return

        self._empty_scored_streak = 0
        target = apply_sweep_radius(scored)[0]
        self._goal = (target.x, target.y)
        self._plan_path_to(
            (target.x, target.y),
            on_failure=lambda reason: self._abandon_goal(target.x, target.y, reason),
            snap_to_frontier=True,
        )

    def _step_exiting(self, world_x: float, world_y: float) -> None:
        assert self._controller is not None and self._machine is not None
        if self._path:
            self._advance_along_path(world_x, world_y)
            return
        # Path to entry is empty -- either just arrived, or landing is
        # already in progress from a previous step.
        if self._controller.armed:
            self._controller.land()
            self._controller.tick(self._dt_s)
            return
        self._machine.handle_exit_complete()  # exiting -> complete
        self._status = _STATUS_COMPLETED

    # -- motion / planning helpers --------------------------------------------

    def _advance_along_path(self, world_x: float, world_y: float) -> None:
        assert self._controller is not None
        next_wp = self._path[0]
        if math.hypot(world_x - next_wp[0], world_y - next_wp[1]) <= _ARRIVAL_TOLERANCE_M:
            self._path.pop(0)
            if self._goal is not None and not self._path:
                clear_failure(self._blacklist, self._goal[0], self._goal[1])
            if self._path:
                self._command_local(self._path[0])
            return
        self._controller.tick(self._dt_s)

    def _plan_path_to(
        self, world_goal: tuple[float, float], *, on_failure, snap_to_frontier: bool = False
    ) -> None:
        assert self._controller is not None and self._environment is not None
        world_x, world_y, _ = self._to_world(self._controller.position)
        grid = self._known_grid_dict()
        env = OccupancyGridEnvironment(grid)
        if snap_to_frontier:
            # See _nearest_frontier_cell's docstring: a frontier REGION's
            # centroid (what score_candidates/apply_sweep_radius select)
            # can sit near the geometric center of an already-explored
            # disc when the region is ring-shaped around the vehicle
            # (typical for a 360-degree sensor growing outward from one
            # point) -- snapping to "the nearest KNOWN-FREE cell" would
            # then just find a cell right next to the vehicle, making no
            # real progress. Snapping to the nearest cell that is itself
            # ACTUALLY ON the frontier (free AND bordering unknown space)
            # finds a point genuinely at the boundary instead -- the same
            # "nearest real cell of the cluster nearest the centroid"
            # snap gps_denied's own frontier_explorer.py used, for the
            # same reason.
            plan_goal = self._nearest_frontier_cell(world_goal, grid)
        else:
            plan_goal = self._nearest_known_free(world_goal, grid, exclude_world_xy=(world_x, world_y))
        if plan_goal is None:
            on_failure("no known-free cell near the goal to plan toward yet (other than where the vehicle already is)")
            return
        try:
            waypoints = self._planner.plan((world_x, world_y), plan_goal, env)
        except PathPlanningError as exc:
            on_failure(str(exc))
            return
        self._path = list(waypoints[1:])  # drop the current position itself
        if self._path:
            self._command_local(self._path[0])
        else:
            # start == goal after snapping to the same cell the vehicle is
            # already standing in (can happen despite excluding the exact
            # current cell above, e.g. a goal that snaps to a cell within
            # the planner's own quantization of "here") -- nothing to
            # travel to. Do NOT leave a lingering self._goal with an empty
            # path (that would spin forever re-selecting the same
            # candidate every step without ever moving, observing nothing
            # new, and never converging) -- treat it as effectively
            # already visited and let the next step re-detect for real.
            if self._goal is not None:
                clear_failure(self._blacklist, self._goal[0], self._goal[1])
            self._goal = None

    def _abandon_goal(self, goal_x: float, goal_y: float, reason: str) -> None:
        record_failure(self._blacklist, goal_x, goal_y)
        self._goal = None
        self._path = []

    def _fail(self, reason: str) -> None:
        self._status = _STATUS_FAILED
        self._error = f"could not plan a return path to the entry point: {reason}"

    def _command_local(self, world_xy: tuple[float, float]) -> None:
        assert self._controller is not None
        lx, ly = self._to_local(world_xy)
        self._controller.set_position(lx, ly, self._takeoff_height_m)

    def _nearest_known_free(
        self,
        world_xy: tuple[float, float],
        grid: dict[str, Any],
        *,
        exclude_world_xy: Optional[tuple[float, float]] = None,
    ) -> Optional[tuple[float, float]]:
        """`FrontierCandidate`'s own docstring: its (x, y) is a region
        centroid, "not necessarily itself a free cell" -- callers that
        need a guaranteed-free waypoint should path-plan a nearby free
        cell instead. This does exactly that: search a small radius
        around `world_xy` for the nearest cell this simulation has
        already observed as free, and plan to that cell's center instead
        of the raw (possibly-unknown-or-occupied) centroid. Returns None
        if nothing free is found within the search radius (should be rare
        -- a real frontier region necessarily has free cells adjacent to
        it).

        `exclude_world_xy`, when given, skips the cell containing that
        point -- callers plan FROM their current pose, and picking the
        vehicle's own current cell as the snapped goal would produce a
        zero-length "path" (planner's own documented `start == goal`
        contract) with nothing to actually travel, silently stalling
        forever since no new observation would ever happen. Excluding it
        forces a real, if possibly very short, move."""
        assert self._environment is not None
        info = grid["info"]
        resolution = info["resolution"]
        origin_x = info["origin"]["position"]["x"]
        origin_y = info["origin"]["position"]["y"]
        width = info["width"]
        height = info["height"]
        data = grid["data"]

        center_col = math.floor((world_xy[0] - origin_x) / resolution)
        center_row = math.floor((world_xy[1] - origin_y) / resolution)

        exclude_cell: Optional[Cell] = None
        if exclude_world_xy is not None:
            exclude_cell = (
                math.floor((exclude_world_xy[0] - origin_x) / resolution),
                math.floor((exclude_world_xy[1] - origin_y) / resolution),
            )

        best: Optional[tuple[float, tuple[float, float]]] = None
        r = _NEAREST_KNOWN_FREE_SEARCH_RADIUS_CELLS
        for drow in range(-r, r + 1):
            for dcol in range(-r, r + 1):
                col, row = center_col + dcol, center_row + drow
                if not (0 <= col < width and 0 <= row < height):
                    continue
                if (col, row) == exclude_cell:
                    continue
                if data[row * width + col] != 0:
                    continue
                cx = origin_x + (col + 0.5) * resolution
                cy = origin_y + (row + 0.5) * resolution
                dist = math.hypot(cx - world_xy[0], cy - world_xy[1])
                if best is None or dist < best[0]:
                    best = (dist, (cx, cy))
        return best[1] if best is not None else None

    def _nearest_frontier_cell(
        self, world_xy: tuple[float, float], grid: dict[str, Any]
    ) -> Optional[tuple[float, float]]:
        """Like `_nearest_known_free`, but restricted to cells that are
        themselves genuine frontier cells (free AND bordering at least
        one unknown 8-connected neighbor) -- the same "known-free cell"
        constraint plus the extra property that actually matters for a
        navigation target: a point genuinely on the boundary of explored
        space, not merely a free cell somewhere nearby (which, near the
        center of a growing sensed disc, is often the vehicle's own
        neighborhood -- see `_plan_path_to`'s `snap_to_frontier` note for
        why that degenerates target selection).

        Searches a radius sized to this simulation's own sensor range
        (plus slack), since a frontier region's actual boundary cells can
        sit that far from its centroid for a roughly disc-shaped known
        area."""
        info = grid["info"]
        resolution = info["resolution"]
        origin_x = info["origin"]["position"]["x"]
        origin_y = info["origin"]["position"]["y"]
        width = info["width"]
        height = info["height"]
        data = grid["data"]

        center_col = math.floor((world_xy[0] - origin_x) / resolution)
        center_row = math.floor((world_xy[1] - origin_y) / resolution)
        r = math.ceil(self._sensor_range_m / resolution) + 2

        best: Optional[tuple[float, tuple[float, float]]] = None
        for drow in range(-r, r + 1):
            for dcol in range(-r, r + 1):
                col, row = center_col + dcol, center_row + drow
                if not (0 <= col < width and 0 <= row < height):
                    continue
                if data[row * width + col] != 0:
                    continue
                if not self._has_unknown_neighbor(data, width, height, col, row):
                    continue
                cx = origin_x + (col + 0.5) * resolution
                cy = origin_y + (row + 0.5) * resolution
                dist = math.hypot(cx - world_xy[0], cy - world_xy[1])
                if best is None or dist < best[0]:
                    best = (dist, (cx, cy))
        return best[1] if best is not None else None

    @staticmethod
    def _has_unknown_neighbor(data: list[int], width: int, height: int, col: int, row: int) -> bool:
        """Same frontier-cell predicate `frontier_detector.py` uses
        internally (free cell, >=1 unknown 8-connected neighbor) --
        duplicated here as a small, single-purpose primitive rather than
        importing frontier_detector's private helpers, so that module's
        public contract stays exactly what AUTONOMY_ROADMAP.md Phase 6
        already reviewed it as."""
        for drow in (-1, 0, 1):
            for dcol in (-1, 0, 1):
                if dcol == 0 and drow == 0:
                    continue
                ncol, nrow = col + dcol, row + drow
                if 0 <= ncol < width and 0 <= nrow < height and data[nrow * width + ncol] == -1:
                    return True
        return False

    # -- observation ----------------------------------------------------------

    def _observe(self, world_x: float, world_y: float) -> None:
        assert self._environment is not None and self._coverage is not None
        visible = reveal_cells(self._environment, (world_x, world_y), self._sensor_range_m)
        self._known_map.update(visible)
        self._coverage.update(self._known_grid_dict(), world_x, world_y, drone_yaw=0.0)

    def _known_grid_dict(self) -> dict[str, Any]:
        assert self._environment is not None
        env = self._environment
        data = [-1] * (env.width_cells * env.height_cells)
        for (col, row), value in self._known_map.items():
            data[row * env.width_cells + col] = value
        return {
            "header": {"frame_id": "map"},
            "info": {
                "resolution": env.resolution_m,
                "width": env.width_cells,
                "height": env.height_cells,
                "origin": {"position": {"x": env.origin[0], "y": env.origin[1]}},
            },
            "data": data,
        }

    # -- frame translation ------------------------------------------------------

    def _to_world(self, local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        lx, ly, lz = local_xyz
        ox, oy = self._entry_offset
        return (lx + ox, ly + oy, lz)

    def _to_local(self, world_xy: tuple[float, float]) -> tuple[float, float]:
        wx, wy = world_xy
        ox, oy = self._entry_offset
        return (wx - ox, wy - oy)

    # -- snapshot ---------------------------------------------------------------

    def snapshot(self) -> SimulationSnapshot:
        if self._environment is None or self._controller is None or self._machine is None or self._coverage is None:
            return SimulationSnapshot(
                status=self._status,
                mission_state="idle",
                step=self._steps,
                elapsed_sim_seconds=0.0,
                pose={"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
                map={"header": {"frame_id": "map"},
                     "info": {"resolution": 1.0, "width": 0, "height": 0,
                              "origin": {"position": {"x": 0.0, "y": 0.0}}}, "data": []},
                coverage={"header": {"frame_id": "map"},
                          "info": {"resolution": 1.0, "width": 0, "height": 0,
                                   "origin": {"position": {"x": 0.0, "y": 0.0}}}, "data": []},
                planned_path=[],
                target=None,
                frontier_count=0,
                candidate_count=0,
                blacklisted_count=0,
                explored_pct=0.0,
                map_known_pct=0.0,
                error=self._error,
            )

        world_x, world_y, world_z = self._to_world(self._controller.position)
        path_world = [{"x": x, "y": y} for x, y in self._path]
        return SimulationSnapshot(
            status=self._status,
            mission_state=self._machine.state,
            step=self._steps,
            elapsed_sim_seconds=self._steps * self._dt_s,
            pose={"x": world_x, "y": world_y, "z": world_z, "yaw_deg": 0.0},
            map=self._known_grid_dict(),
            coverage=self._coverage.to_occupancy_grid_data(),
            planned_path=path_world,
            target=list(self._goal) if self._goal is not None else None,
            frontier_count=self._last_frontier_count,
            candidate_count=self._last_candidate_count,
            blacklisted_count=len(self._blacklist),
            explored_pct=round(self._coverage.percent_searched(), 1),
            map_known_pct=round(
                100.0 * len(self._known_map) / (self._environment.width_cells * self._environment.height_cells), 1
            ),
            error=self._error,
        )
