"""Frontier target-selection policy -- pure logic, no rclpy.

Migrated/adapted from gps_denied/raj-dev's frontier_explorer.py (NIDAR
Autonomy Migration, see CHECKPOINT/CURRENT_STATE.md). That file did BOTH
frontier detection AND target selection inline, coupled to a Nav2 action
client. Detection is deliberately NOT migrated from there -- onboard-autonomy
already has a better-tested, independently-reviewed detector
(`frontier_detector.detect_frontiers()`, AUTONOMY_ROADMAP.md Phase 6, 14
tests, zero flight-command coupling) that this migration keeps as canonical,
per the migration's "prefer the better-tested implementation" rule (see
CHECKPOINT/CURRENT_STATE.md's migration report, item 16).

What IS migrated here is the SELECTION POLICY gps_denied's frontier_explorer.py
proved out on real (simulated) flights, which `frontier_detector.py`'s plain
"biggest bordering-unknown-region wins" score doesn't have:
  * distance-vs-information-gain cost, not region size alone
  * hysteresis toward the goal already being pursued (stops flip-flopping
    between two similar candidates as the drone moves)
  * a "visited" memory that penalizes re-covering the same ground
  * a local-first sweep radius (finish the neighbourhood before a long hop)
  * blacklisting a goal that repeatedly fails to be reached

This module operates purely on `FrontierCandidate` objects
(`frontier_detector.py`) plus plain dicts/tuples/floats for state -- no ROS,
no `FlightCommandInterface` import, same "pure decision, ROS plumbing
elsewhere" pattern as `arm_trigger.py`/`state_machine.py`. It answers "what
should we explore next", never "how do we get there" (that's
`grid_astar_planner.py`/`occupancy_grid_environment.py`) or "should the
vehicle fly there at all" (the mission manager, AUTONOMY_ROADMAP.md Phase 9)
-- this module is a target *selector*, not the mission authority, so its
output is a suggestion a caller decides whether to act on.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .frontier_detector import FrontierCandidate

# Defaults mirror gps_denied/raj-dev's frontier_explorer.py declared ROS
# parameters -- same tuned values, now plain Python defaults.
_DEFAULT_MIN_GOAL_DISTANCE_M = 0.7
_DEFAULT_INFO_WEIGHT = 0.4
_DEFAULT_HYSTERESIS_BONUS = 2.0
_DEFAULT_VISIT_CELL_SIZE_M = 0.75
_DEFAULT_VISIT_WEIGHT = 0.6
_DEFAULT_VISIT_RADIUS_M = 1.5
_DEFAULT_VISIT_PENALTY_CAP = 20.0
_DEFAULT_MAX_FAILURES = 3
_DEFAULT_SWEEP_RADIUS_M = 5.0

# A goal within this distance of the current goal is treated as "the same
# goal" for hysteresis purposes (the map/centroid can shift slightly between
# ticks without that meaning a genuinely different target was chosen).
_SAME_GOAL_TOLERANCE_M = 1.0

VisitedMemory = dict[tuple[int, int], int]
Blacklist = dict[tuple[float, float], int]


@dataclass(frozen=True)
class ScoredTarget:
    """One scored, reachable-looking candidate target."""

    x: float
    y: float
    cost: float
    cell_count: int
    distance: float


# -- visited memory -----------------------------------------------------------


def visit_key(x: float, y: float, cell_size: float) -> tuple[int, int]:
    return (round(x / cell_size), round(y / cell_size))


def record_visit(visited: VisitedMemory, x: float, y: float, cell_size: float) -> None:
    """Record that the vehicle's current position has been near (x, y),
    quantized to a coarse cell -- the exploration policy's memory of where
    it has already been. Call once per pose update."""
    key = visit_key(x, y, cell_size)
    visited[key] = visited.get(key, 0) + 1


def visit_penalty(
    visited: VisitedMemory,
    x: float,
    y: float,
    cell_size: float,
    radius: float,
    weight: float,
    cap: float = _DEFAULT_VISIT_PENALTY_CAP,
) -> float:
    """How much time the vehicle has already spent near (x, y). Candidates
    in well-trodden areas score worse (higher cost), pushing selection
    toward genuinely new ground."""
    r = int(radius / cell_size) + 1
    cx, cy = visit_key(x, y, cell_size)
    count = 0
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            count += visited.get((cx + dx, cy + dy), 0)
    return weight * min(count, cap)


# -- blacklist ------------------------------------------------------------------


def blacklist_key(x: float, y: float) -> tuple[float, float]:
    return (round(x, 1), round(y, 1))


def is_blacklisted(blacklist: Blacklist, x: float, y: float, max_failures: int) -> bool:
    return blacklist.get(blacklist_key(x, y), 0) >= max_failures


def record_failure(blacklist: Blacklist, x: float, y: float, weight: int = 1) -> int:
    """Record that a goal at (x, y) failed to be reached (no path found, or
    abandoned as stuck/timed out). Returns the new failure count."""
    key = blacklist_key(x, y)
    blacklist[key] = blacklist.get(key, 0) + weight
    return blacklist[key]


def clear_failure(blacklist: Blacklist, x: float, y: float) -> None:
    """Clear a goal's failure record -- call when a goal is successfully
    reached, so a transient earlier failure doesn't linger forever."""
    blacklist.pop(blacklist_key(x, y), None)


# -- scoring / selection ---------------------------------------------------------


def score_candidates(
    candidates: list[FrontierCandidate],
    *,
    pose: tuple[float, float],
    current_goal: Optional[tuple[float, float]],
    visited: VisitedMemory,
    blacklist: Blacklist,
    min_goal_distance: float = _DEFAULT_MIN_GOAL_DISTANCE_M,
    info_weight: float = _DEFAULT_INFO_WEIGHT,
    hysteresis_bonus: float = _DEFAULT_HYSTERESIS_BONUS,
    visit_cell_size: float = _DEFAULT_VISIT_CELL_SIZE_M,
    visit_weight: float = _DEFAULT_VISIT_WEIGHT,
    visit_radius: float = _DEFAULT_VISIT_RADIUS_M,
    max_failures: int = _DEFAULT_MAX_FAILURES,
) -> list[ScoredTarget]:
    """Score every candidate not excluded by the blacklist or minimum
    distance, ascending by cost (best first). Lower cost = more preferred:
    closer, larger unknown region behind it, less-visited, and (if within
    `_SAME_GOAL_TOLERANCE_M` of `current_goal`) discounted by
    `hysteresis_bonus` so a committed goal is seen through rather than
    flip-flopping to a marginally cheaper alternative."""
    px, py = pose
    scored: list[ScoredTarget] = []
    for c in candidates:
        if is_blacklisted(blacklist, c.x, c.y, max_failures):
            continue
        distance = math.hypot(c.x - px, c.y - py)
        if distance < min_goal_distance:
            continue  # already here; nothing new revealed
        cost = distance - info_weight * c.score + visit_penalty(
            visited, c.x, c.y, visit_cell_size, visit_radius, visit_weight
        )
        if current_goal is not None and math.hypot(
            c.x - current_goal[0], c.y - current_goal[1]
        ) < _SAME_GOAL_TOLERANCE_M:
            cost -= hysteresis_bonus
        scored.append(
            ScoredTarget(x=c.x, y=c.y, cost=cost, cell_count=c.cell_count, distance=distance)
        )
    scored.sort(key=lambda t: t.cost)
    return scored


def apply_sweep_radius(
    scored: list[ScoredTarget], sweep_radius: float = _DEFAULT_SWEEP_RADIUS_M
) -> list[ScoredTarget]:
    """Local-first sweep: if anything scored is within `sweep_radius`, drop
    the far ones so a long (SLAM-stressing, per gps_denied's own tuning
    notes) traverse is only taken once nothing nearer remains. Returns
    `scored` unchanged if nothing is within radius (so callers never lose
    all candidates because everything happens to be far away)."""
    near = [t for t in scored if t.distance <= sweep_radius]
    return near if near else scored


def select_target(
    candidates: list[FrontierCandidate],
    *,
    pose: tuple[float, float],
    current_goal: Optional[tuple[float, float]],
    visited: VisitedMemory,
    blacklist: Blacklist,
    min_goal_distance: float = _DEFAULT_MIN_GOAL_DISTANCE_M,
    info_weight: float = _DEFAULT_INFO_WEIGHT,
    hysteresis_bonus: float = _DEFAULT_HYSTERESIS_BONUS,
    visit_cell_size: float = _DEFAULT_VISIT_CELL_SIZE_M,
    visit_weight: float = _DEFAULT_VISIT_WEIGHT,
    visit_radius: float = _DEFAULT_VISIT_RADIUS_M,
    max_failures: int = _DEFAULT_MAX_FAILURES,
    sweep_radius: float = _DEFAULT_SWEEP_RADIUS_M,
) -> Optional[ScoredTarget]:
    """The single entry point: score every candidate, apply the local-first
    sweep, and return the best surviving target -- or `None` if nothing
    survives (every candidate blacklisted, too close, or the list was empty
    to begin with). Callers must treat `None` as "no target right now", not
    an error -- exactly how `frontier_detector.detect_frontiers` treats an
    empty result."""
    scored = score_candidates(
        candidates,
        pose=pose,
        current_goal=current_goal,
        visited=visited,
        blacklist=blacklist,
        min_goal_distance=min_goal_distance,
        info_weight=info_weight,
        hysteresis_bonus=hysteresis_bonus,
        visit_cell_size=visit_cell_size,
        visit_weight=visit_weight,
        visit_radius=visit_radius,
        max_failures=max_failures,
    )
    if not scored:
        return None
    return apply_sweep_radius(scored, sweep_radius)[0]
