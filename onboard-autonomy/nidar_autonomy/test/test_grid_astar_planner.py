import math

import pytest

from nidar_autonomy.indoor_environment import (
    empty_room,
    room_with_single_obstacle,
    unreachable_target_room,
)
from nidar_autonomy.grid_astar_planner import GridAStarPlanner
from nidar_autonomy.path_planner_interface import (
    EndpointOccupiedError,
    EndpointOutOfBoundsError,
    NoPathFoundError,
)


def _assert_path_independently_collision_free(env, path):
    """Real, independent collision check against the environment --
    every consecutive waypoint pair's straight segment must be
    is_path_clear, using is_path_clear itself (the module's own
    collision-check primitive), not anything the planner's internal grid
    search already assumed. Mirrors test_indoor_environment.py's own
    non-circularity discipline for the unreachable-target case."""
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        assert env.is_path_clear(x0, y0, x1, y1) is True


def _path_length(path):
    total = 0.0
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


# -- empty_room ---------------------------------------------------------------


def test_empty_room_plans_straight_line_to_far_point():
    env = empty_room()
    planner = GridAStarPlanner()
    goal = (14.5, 14.5)
    path = planner.plan(env.start_pose, goal, env)

    assert len(path) >= 2
    assert path[0] == env.start_pose
    assert path[-1] == goal
    _assert_path_independently_collision_free(env, path)


def test_empty_room_path_is_a_straight_line_since_nothing_blocks_it():
    # a fully open room has no reason to detour -- the returned path
    # should collapse (via collinear simplification) to exactly the two
    # endpoints.
    env = empty_room()
    planner = GridAStarPlanner()
    goal = (14.5, 14.5)
    path = planner.plan(env.start_pose, goal, env)
    assert path == [env.start_pose, goal]


# -- room_with_single_obstacle --------------------------------------------------


def test_room_with_single_obstacle_routes_around_and_stays_clear():
    env = room_with_single_obstacle()
    planner = GridAStarPlanner()
    goal = (13.5, 13.5)  # the far corner the module's own docstring says is blocked

    # independently confirm this scenario's premise: the direct line is
    # genuinely blocked, so any valid path here must not be a straight line.
    assert env.is_path_clear(*env.start_pose, *goal) is False

    path = planner.plan(env.start_pose, goal, env)

    assert path[0] == env.start_pose
    assert path[-1] == goal
    _assert_path_independently_collision_free(env, path)

    # (a) the path is not a straight line through the obstacle: it must
    # actually detour, which a genuinely obstacle-routing path reveals by
    # being longer than the straight-line distance between the endpoints.
    straight_line_distance = math.hypot(
        goal[0] - env.start_pose[0], goal[1] - env.start_pose[1]
    )
    assert _path_length(path) > straight_line_distance

    # every intermediate waypoint must itself be off the direct diagonal
    # line the obstacle sits on (a concrete, independent geometric check
    # beyond just "path has more than 2 points").
    assert len(path) > 2


def test_room_with_single_obstacle_no_segment_crosses_the_wall_column():
    # concrete geometric check: no consecutive-waypoint segment may pass
    # through the wall's occupied column (world x in [7.0, 8.0), y in
    # [0.0, 10.0)) -- independently re-derived from the scenario's own
    # documented obstacle rectangle, not from the planner's internal grid.
    env = room_with_single_obstacle()
    planner = GridAStarPlanner()
    goal = (13.5, 13.5)
    path = planner.plan(env.start_pose, goal, env)

    steps = 200
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        for i in range(steps + 1):
            t = i / steps
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            in_wall_column = 7.0 <= x < 8.0 and 0.0 <= y < 10.0
            assert not in_wall_column


# -- unreachable_target_room ----------------------------------------------------


def test_unreachable_target_raises_no_path_found_deterministically():
    env = unreachable_target_room()
    planner = GridAStarPlanner()
    target = (7.5, 7.5)

    with pytest.raises(NoPathFoundError):
        planner.plan(env.start_pose, target, env)
    # deterministic, not flaky -- raises again identically.
    with pytest.raises(NoPathFoundError):
        planner.plan(env.start_pose, target, env)


# -- invalid inputs -------------------------------------------------------------


def test_start_out_of_bounds_raises_typed_error():
    env = empty_room()
    planner = GridAStarPlanner()
    with pytest.raises(EndpointOutOfBoundsError):
        planner.plan((-1.0, -1.0), (5.0, 5.0), env)


def test_goal_out_of_bounds_raises_typed_error():
    env = empty_room()
    planner = GridAStarPlanner()
    with pytest.raises(EndpointOutOfBoundsError):
        planner.plan(env.start_pose, (100.0, 100.0), env)


def test_start_occupied_raises_typed_error():
    env = room_with_single_obstacle()
    planner = GridAStarPlanner()
    with pytest.raises(EndpointOccupiedError):
        planner.plan((7.5, 5.0), (1.5, 1.5), env)


def test_goal_occupied_raises_typed_error():
    env = room_with_single_obstacle()
    planner = GridAStarPlanner()
    with pytest.raises(EndpointOccupiedError):
        planner.plan(env.start_pose, (7.5, 5.0), env)


def test_invalid_input_errors_are_distinguishable():
    env = empty_room()
    planner = GridAStarPlanner()
    with pytest.raises(EndpointOutOfBoundsError) as out_of_bounds_exc_info:
        planner.plan((-1.0, -1.0), (5.0, 5.0), env)
    assert not issubclass(out_of_bounds_exc_info.type, EndpointOccupiedError)


# -- start == goal edge case ------------------------------------------------------


def test_start_equals_goal_returns_single_waypoint_path():
    env = empty_room()
    planner = GridAStarPlanner()
    path = planner.plan(env.start_pose, env.start_pose, env)
    assert path == [env.start_pose]


def test_start_and_goal_in_same_free_cell_but_not_equal():
    env = empty_room()
    planner = GridAStarPlanner()
    start = (0.2, 0.2)
    goal = (0.8, 0.8)  # same cell (0, 0) as start, at 1m resolution
    path = planner.plan(start, goal, env)
    assert path[0] == start
    assert path[-1] == goal
    _assert_path_independently_collision_free(env, path)


# -- determinism ------------------------------------------------------------------


def test_determinism_same_query_twice_yields_identical_path():
    env = room_with_single_obstacle()
    planner = GridAStarPlanner()
    goal = (13.5, 13.5)

    path_a = planner.plan(env.start_pose, goal, env)
    path_b = planner.plan(env.start_pose, goal, env)
    assert path_a == path_b


def test_determinism_across_separate_planner_instances():
    env = room_with_single_obstacle()
    goal = (13.5, 13.5)

    path_a = GridAStarPlanner().plan(env.start_pose, goal, env)
    path_b = GridAStarPlanner().plan(env.start_pose, goal, env)
    assert path_a == path_b


def test_determinism_empty_room():
    env = empty_room()
    goal = (14.5, 0.5)
    planner = GridAStarPlanner()
    path_a = planner.plan(env.start_pose, goal, env)
    path_b = planner.plan(env.start_pose, goal, env)
    assert path_a == path_b
