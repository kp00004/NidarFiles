"""Tests for occupancy_grid_environment.py -- new module added by the NIDAR
Autonomy Migration to let GridAStarPlanner/detect_frontiers plan against a
live occupancy-grid dict instead of only IndoorEnvironment's ground truth."""
import pytest

from nidar_autonomy.grid_astar_planner import GridAStarPlanner
from nidar_autonomy.occupancy_grid_environment import (
    InvalidOccupancyGridError,
    OccupancyGridEnvironment,
)
from nidar_autonomy.path_planner_interface import EndpointOccupiedError, NoPathFoundError


def _grid(width, height, data, resolution=1.0, origin=(0.0, 0.0)):
    return {
        "info": {
            "resolution": resolution,
            "width": width,
            "height": height,
            "origin": {"position": {"x": origin[0], "y": origin[1]}},
        },
        "data": data,
    }


def _all_free(width, height):
    return _grid(width, height, [0] * (width * height))


class TestConstructionAndValidation:
    def test_basic_properties(self):
        env = OccupancyGridEnvironment(_grid(5, 4, [0] * 20, resolution=0.5, origin=(1.0, 2.0)))
        assert env.width_cells == 5
        assert env.height_cells == 4
        assert env.resolution_m == 0.5
        assert env.origin == (1.0, 2.0)

    def test_missing_info_raises(self):
        with pytest.raises(InvalidOccupancyGridError):
            OccupancyGridEnvironment({"data": []})

    def test_data_length_mismatch_raises(self):
        with pytest.raises(InvalidOccupancyGridError):
            OccupancyGridEnvironment(_grid(3, 3, [0] * 5))


class TestOccupancyAndBounds:
    def test_free_cell_not_occupied(self):
        env = OccupancyGridEnvironment(_all_free(5, 5))
        assert not env.is_occupied(2.5, 2.5)

    def test_occupied_cell_is_occupied(self):
        data = [0] * 25
        data[2 * 5 + 2] = 100  # row 2, col 2
        env = OccupancyGridEnvironment(_grid(5, 5, data))
        assert env.is_occupied(2.5, 2.5)

    def test_unknown_cell_is_treated_as_occupied(self):
        """Unlike IndoorEnvironment (pure ground truth), a live map has real
        unknown cells -- routing through them is unsafe, so they must count
        as not-traversable, same as a wall."""
        data = [-1] * 25
        env = OccupancyGridEnvironment(_grid(5, 5, data))
        assert env.is_occupied(2.5, 2.5)

    def test_out_of_bounds_is_occupied(self):
        env = OccupancyGridEnvironment(_all_free(5, 5))
        assert env.is_occupied(-1.0, -1.0)
        assert env.is_occupied(100.0, 100.0)

    def test_out_of_bounds_is_not_within_bounds(self):
        env = OccupancyGridEnvironment(_all_free(5, 5))
        assert not env.is_within_bounds(5.0, 2.5)  # exactly at the far edge
        assert env.is_within_bounds(4.999, 2.5)

    def test_free_below_threshold_matches_coverage_grid_convention(self):
        # Cartographer-style: free space is 0..49, not just exactly 0.
        data = [30] * 25
        env = OccupancyGridEnvironment(_grid(5, 5, data), free_below=50)
        assert not env.is_occupied(2.5, 2.5)


class TestPlannerDuckTyping:
    """Proves GridAStarPlanner (built and tested only against
    IndoorEnvironment) works unmodified against this adapter -- the
    compatibility promise grid_astar_planner.py's own docstring makes."""

    def test_planner_finds_a_path_through_a_live_grid(self):
        width, height = 10, 10
        data = [0] * (width * height)
        env = OccupancyGridEnvironment(_grid(width, height, data))
        planner = GridAStarPlanner()
        path = planner.plan((0.5, 0.5), (8.5, 8.5), env)
        assert path[0] == (0.5, 0.5)
        assert path[-1] == (8.5, 8.5)

    def test_planner_routes_around_a_live_obstacle(self):
        width, height = 10, 10
        data = [0] * (width * height)
        for row in range(0, 8):  # a wall at col=5, rows 0..7 -- gap at the top
            data[row * width + 5] = 100
        env = OccupancyGridEnvironment(_grid(width, height, data))
        planner = GridAStarPlanner()
        path = planner.plan((1.5, 1.5), (8.5, 1.5), env)
        assert path[0] == (1.5, 1.5)
        assert path[-1] == (8.5, 1.5)
        # The direct straight line would cross the wall -- confirm the
        # returned path actually detours (visits a cell with row >= 8).
        assert any(y >= 8.0 for _x, y in path)

    def test_planner_raises_no_path_found_for_unreachable_goal(self):
        width, height = 10, 10
        data = [0] * (width * height)
        for col in range(0, 10):
            data[5 * width + col] = 100  # a full wall across row 5
        env = OccupancyGridEnvironment(_grid(width, height, data))
        planner = GridAStarPlanner()
        with pytest.raises(NoPathFoundError):
            planner.plan((1.5, 1.5), (8.5, 8.5), env)

    def test_planner_raises_for_unknown_goal_cell(self):
        width, height = 10, 10
        data = [-1] * (width * height)
        data[1 * width + 1] = 0  # only the start cell is known-free
        env = OccupancyGridEnvironment(_grid(width, height, data))
        planner = GridAStarPlanner()
        with pytest.raises(EndpointOccupiedError):
            planner.plan((1.5, 1.5), (8.5, 8.5), env)
