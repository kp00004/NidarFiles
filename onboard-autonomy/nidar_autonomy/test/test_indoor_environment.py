from collections import deque

import pytest

from nidar_autonomy.indoor_environment import (
    IndoorEnvironment,
    InvalidStartPoseError,
    OutOfBoundsError,
    empty_room,
    room_with_single_obstacle,
    simulation_maze,
    unreachable_target_room,
)


def _reachable_cells(env: IndoorEnvironment, start_xy) -> set:
    """BFS flood-fill over free cells, 8-connected, starting from the
    cell containing start_xy. Test-only helper -- the module itself
    deliberately doesn't implement general reachability (that's Phase
    7's job); this exists only to prove unreachable_target_room()'s
    target really is unreachable, independent of is_path_clear."""
    start_col, start_row = env._cell_of(*start_xy)  # noqa: SLF001
    assert (start_col, start_row) not in env._occupied  # noqa: SLF001

    seen = {(start_col, start_row)}
    queue = deque([(start_col, start_row)])
    while queue:
        col, row = queue.popleft()
        for dcol in (-1, 0, 1):
            for drow in (-1, 0, 1):
                if dcol == 0 and drow == 0:
                    continue
                neighbor = (col + dcol, row + drow)
                if neighbor in seen:
                    continue
                if not (0 <= neighbor[0] < env.width_cells):
                    continue
                if not (0 <= neighbor[1] < env.height_cells):
                    continue
                if neighbor in env._occupied:  # noqa: SLF001
                    continue
                seen.add(neighbor)
                queue.append(neighbor)
    return seen


# -- bounds checking --------------------------------------------------------


def test_within_bounds_interior_point():
    env = empty_room()
    assert env.is_within_bounds(7.5, 7.5) is True


def test_within_bounds_lower_boundary_included():
    env = empty_room()
    assert env.is_within_bounds(0.0, 0.0) is True


def test_within_bounds_upper_boundary_excluded():
    env = empty_room()
    # arena is [0, 15) x [0, 15) -- exactly 15.0 is one past the last cell
    assert env.is_within_bounds(15.0, 5.0) is False
    assert env.is_within_bounds(5.0, 15.0) is False


def test_within_bounds_just_inside_upper_boundary():
    env = empty_room()
    assert env.is_within_bounds(14.999, 14.999) is True


def test_within_bounds_negative_point():
    env = empty_room()
    assert env.is_within_bounds(-0.5, 5.0) is False


def test_within_bounds_far_outside():
    env = empty_room()
    assert env.is_within_bounds(100.0, 100.0) is False


# -- occupancy checks --------------------------------------------------------


def test_occupied_at_known_wall_cell():
    env = room_with_single_obstacle()
    # wall covers x in [7,8), y in [0,10)
    assert env.is_occupied(7.5, 5.0) is True


def test_free_at_known_open_cell():
    env = room_with_single_obstacle()
    assert env.is_occupied(1.5, 1.5) is False


def test_free_just_past_wall_top():
    env = room_with_single_obstacle()
    # wall stops at y=10.0 -- one cell above should be open
    assert env.is_occupied(7.5, 10.5) is False


def test_out_of_bounds_point_is_occupied():
    env = empty_room()
    assert env.is_occupied(-1.0, -1.0) is True
    assert env.is_occupied(15.0, 15.0) is True


# -- is_path_clear ------------------------------------------------------------


def test_path_clear_direct_line_no_obstacles():
    env = empty_room()
    assert env.is_path_clear(0.5, 0.5, 14.5, 14.5) is True


def test_path_blocked_by_known_obstacle():
    env = room_with_single_obstacle()
    assert env.is_path_clear(1.5, 1.5, 13.5, 13.5) is False


def test_path_clear_route_around_obstacle():
    env = room_with_single_obstacle()
    assert env.is_path_clear(1.5, 1.5, 7.5, 12.0) is True
    assert env.is_path_clear(7.5, 12.0, 13.5, 13.5) is True


def test_path_out_of_bounds_returns_false():
    env = empty_room()
    assert env.is_path_clear(0.5, 0.5, 20.0, 20.0) is False


def test_path_same_point_free_is_clear():
    env = empty_room()
    assert env.is_path_clear(1.0, 1.0, 1.0, 1.0) is True


def test_path_same_point_occupied_is_blocked():
    env = room_with_single_obstacle()
    assert env.is_path_clear(7.5, 5.0, 7.5, 5.0) is False


def test_path_fine_sampling_detects_thin_wall_between_clear_endpoints():
    # both endpoints free, straight line still crosses the 1m-wide wall
    env = room_with_single_obstacle()
    assert env.is_occupied(6.5, 5.0) is False
    assert env.is_occupied(8.5, 5.0) is False
    assert env.is_path_clear(6.5, 5.0, 8.5, 5.0) is False


# -- constructor start_pose validation ---------------------------------------


def test_start_pose_out_of_bounds_raises():
    with pytest.raises(OutOfBoundsError):
        IndoorEnvironment(start_pose=(100.0, 100.0))


def test_start_pose_negative_out_of_bounds_raises():
    with pytest.raises(OutOfBoundsError):
        IndoorEnvironment(start_pose=(-1.0, 1.0))


def test_start_pose_on_occupied_cell_raises():
    with pytest.raises(InvalidStartPoseError):
        IndoorEnvironment(obstacles=[(7.0, 0.0, 8.0, 10.0)], start_pose=(7.5, 5.0))


def test_start_pose_valid_is_accepted():
    env = IndoorEnvironment(
        obstacles=[(7.0, 0.0, 8.0, 10.0)], start_pose=(1.5, 1.5)
    )
    assert env.start_pose == (1.5, 1.5)


# -- to_occupancy_grid_data ----------------------------------------------------


def test_occupancy_grid_shape_matches_data_models_schema():
    env = empty_room()
    grid = env.to_occupancy_grid_data()
    assert set(grid.keys()) == {"header", "info", "data"}
    assert set(grid["header"].keys()) == {"stamp", "frame_id"}
    assert set(grid["header"]["stamp"].keys()) == {"sec", "nanosec"}
    assert grid["header"]["frame_id"] == "map"
    assert set(grid["info"].keys()) == {"resolution", "width", "height", "origin"}
    assert set(grid["info"]["origin"].keys()) == {"position", "orientation"}
    assert set(grid["info"]["origin"]["position"].keys()) == {"x", "y", "z"}
    assert set(grid["info"]["origin"]["orientation"].keys()) == {"x", "y", "z", "w"}


def test_occupancy_grid_dimensions_match_constructor_args():
    env = IndoorEnvironment(width_m=6.0, height_m=4.0, resolution_m=1.0)
    grid = env.to_occupancy_grid_data()
    assert grid["info"]["width"] == 6
    assert grid["info"]["height"] == 4
    assert grid["info"]["resolution"] == 1.0
    assert len(grid["data"]) == 6 * 4


def test_occupancy_grid_never_contains_unknown():
    env = room_with_single_obstacle()
    grid = env.to_occupancy_grid_data()
    assert set(grid["data"]) <= {0, 100}


def test_occupancy_grid_spot_check_known_cells():
    env = room_with_single_obstacle()
    grid = env.to_occupancy_grid_data()
    width = grid["info"]["width"]

    # wall cell (col=7, row=5) must be occupied
    occupied_index = 5 * width + 7
    assert grid["data"][occupied_index] == 100

    # start_pose cell (col=1, row=1) must be free
    free_index = 1 * width + 1
    assert grid["data"][free_index] == 0

    # a cell just past the wall's top (col=7, row=10) must be free
    past_wall_index = 10 * width + 7
    assert grid["data"][past_wall_index] == 0


def test_occupancy_grid_origin_matches_constructor_origin():
    env = IndoorEnvironment(origin=(2.0, 3.0), start_pose=(2.5, 3.5))
    grid = env.to_occupancy_grid_data()
    assert grid["info"]["origin"]["position"]["x"] == 2.0
    assert grid["info"]["origin"]["position"]["y"] == 3.0
    assert grid["info"]["origin"]["position"]["z"] == 0.0


# -- canned scenarios ---------------------------------------------------------


def test_empty_room_has_no_occupied_cells():
    env = empty_room()
    grid = env.to_occupancy_grid_data()
    assert all(value == 0 for value in grid["data"])


def test_empty_room_any_direct_path_is_clear():
    env = empty_room()
    assert env.is_path_clear(*env.start_pose, 14.5, 0.5) is True
    assert env.is_path_clear(*env.start_pose, 0.5, 14.5) is True


def test_room_with_single_obstacle_blocks_direct_line_to_far_corner():
    env = room_with_single_obstacle()
    assert env.is_path_clear(*env.start_pose, 13.5, 13.5) is False


def test_room_with_single_obstacle_has_a_clear_route_around():
    env = room_with_single_obstacle()
    waypoint = (7.5, 12.0)
    assert env.is_path_clear(*env.start_pose, *waypoint) is True
    assert env.is_path_clear(*waypoint, 13.5, 13.5) is True


def test_unreachable_target_room_target_not_in_flood_fill():
    env = unreachable_target_room()
    target = (7.5, 7.5)
    assert env.is_occupied(target[0], target[1]) is False  # the cell itself is free

    reachable = _reachable_cells(env, env.start_pose)
    target_cell = env._cell_of(*target)  # noqa: SLF001
    assert target_cell not in reachable


def test_unreachable_target_room_direct_path_also_blocked():
    env = unreachable_target_room()
    assert env.is_path_clear(*env.start_pose, 7.5, 7.5) is False


# -- simulation_maze() (NIDAR simulation harness ground truth) ---------------


def test_simulation_maze_start_pose_is_valid():
    env = simulation_maze()
    assert env.start_pose == (1.5, 1.5)
    assert not env.is_occupied(*env.start_pose)


def test_simulation_maze_every_room_is_reachable_from_start():
    """Proves the corridor gaps actually connect all four rooms -- a
    maze where a room is sealed off would silently make the simulation's
    exploration-completion criterion (no reachable frontiers) trigger far
    too early, without any test catching it."""
    env = simulation_maze()
    reachable = _reachable_cells(env, env.start_pose)

    # One representative free point inside each of the four rooms
    # described in simulation_maze()'s docstring.
    sw_room = (2.5, 10.5)
    se_room = (2.5, 2.5)  # same room as start_pose
    ne_room = (12.5, 12.5)
    se_east_room = (12.5, 2.5)

    for label, point in (("SW", sw_room), ("SE/start", se_room), ("NE", ne_room), ("east/SE", se_east_room)):
        assert not env.is_occupied(*point), f"{label} sample point {point} should be free"
        cell = env._cell_of(*point)  # noqa: SLF001
        assert cell in reachable, f"{label} room at {point} is not reachable from start_pose"


def test_simulation_maze_has_substantial_free_area():
    """A sanity bound, not a precise figure -- catches an obstacle list
    that accidentally walls off most of the arena."""
    env = simulation_maze()
    reachable = _reachable_cells(env, env.start_pose)
    total_cells = env.width_cells * env.height_cells
    assert len(reachable) > total_cells * 0.4


def test_simulation_maze_produces_multiple_frontier_regions_when_partially_explored():
    """The whole reason this scenario exists: the sim needs >1 distinct
    frontier region available at once, not just a single corridor to
    follow -- otherwise exploration_policy's selection logic (hysteresis,
    sweep radius, blacklisting) is never meaningfully exercised."""
    from nidar_autonomy.frontier_detector import detect_frontiers
    from nidar_autonomy.sensor_model import reveal_cells

    env = simulation_maze()
    # Reveal only a modest radius around start -- enough to have crossed
    # into the SE room but nowhere near having explored the whole maze.
    visible = reveal_cells(env, env.start_pose, sensor_range_m=3.0)
    data = [-1] * (env.width_cells * env.height_cells)
    for (col, row), value in visible.items():
        data[row * env.width_cells + col] = value
    grid = {
        "info": {
            "resolution": env.resolution_m,
            "width": env.width_cells,
            "height": env.height_cells,
            "origin": {"position": {"x": env.origin[0], "y": env.origin[1]}},
        },
        "data": data,
    }
    candidates = detect_frontiers(grid)
    assert len(candidates) >= 1
