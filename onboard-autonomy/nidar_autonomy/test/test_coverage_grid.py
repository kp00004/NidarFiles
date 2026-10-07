"""Tests for coverage_grid.py -- migrated/adapted from gps_denied/raj-dev's
coverage_tracker.py (NIDAR Autonomy Migration). Pure logic, no ROS
required."""
import pytest
from pytest import approx as pytest_approx

from nidar_autonomy.coverage_grid import SEARCHED, UNKNOWN, UNSEARCHED, CoverageGrid


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


def _all_free_map(width, height):
    return _grid(width, height, [0] * (width * height))


class TestConstruction:
    def test_grid_sized_from_bounds_and_cell_size(self):
        cg = CoverageGrid(min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, cell_size=1.0)
        assert cg.ncols == 5
        assert cg.nrows == 5
        assert all(v == UNKNOWN for row in cg.state for v in row)

    def test_rejects_degenerate_bounds(self):
        with pytest.raises(ValueError):
            CoverageGrid(min_x=5.0, max_x=5.0, min_y=0.0, max_y=5.0)


class TestClassifyAndUpdate:
    def test_free_map_cell_becomes_unsearched_before_being_seen(self):
        cg = CoverageGrid(min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, cell_size=1.0,
                           camera_range_m=0.0)  # camera can see nothing but its own cell
        grid = _all_free_map(5, 5)
        cg.update(grid, drone_x=2.5, drone_y=2.5, drone_yaw=0.0)
        # Cells away from the drone's own position should be UNSEARCHED
        # (known-free, not yet seen) -- the drone's own cell is a distance-0
        # special case (always "visible" to itself) and excluded here.
        far_cell = cg.state[0][0]
        assert far_cell == UNSEARCHED

    def test_cell_within_range_and_fov_becomes_searched(self):
        cg = CoverageGrid(min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, cell_size=1.0,
                           camera_range_m=10.0, camera_fov_deg=360.0)
        grid = _all_free_map(5, 5)
        free_total, searched_total = cg.update(grid, drone_x=2.5, drone_y=2.5, drone_yaw=0.0)
        assert free_total == 25
        assert searched_total == 25  # wide FOV, long range, all free -> all seen

    def test_cell_outside_range_stays_unsearched(self):
        cg = CoverageGrid(min_x=0.0, max_x=10.0, min_y=0.0, max_y=1.0, cell_size=1.0,
                           camera_range_m=2.0, camera_fov_deg=360.0)
        grid = _all_free_map(10, 1)
        cg.update(grid, drone_x=0.5, drone_y=0.5, drone_yaw=0.0)
        # Far cell (col=9, ~9m away) is well outside camera_range_m=2.0.
        assert cg.state[0][9] == UNSEARCHED
        # Near cell (col=0, the drone's own cell) should be searched.
        assert cg.state[0][0] == SEARCHED

    def test_cell_outside_fov_cone_stays_unsearched(self):
        cg = CoverageGrid(min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, cell_size=1.0,
                           camera_range_m=10.0, camera_fov_deg=10.0)  # very narrow cone
        grid = _all_free_map(5, 5)
        # Drone at center facing +x (yaw=0); a cell behind it (-x direction)
        # should be outside a 10-degree cone.
        cg.update(grid, drone_x=2.5, drone_y=2.5, drone_yaw=0.0)
        behind_col, behind_row = 0, 2  # west of the drone
        assert cg.state[behind_row][behind_col] != SEARCHED

    def test_cell_behind_wall_stays_unsearched_even_in_range_and_fov(self):
        # Fine map resolution (0.2m) and a thick (1.0m) wall so the LOS
        # ray-march (which steps at the map's own resolution) cannot skip
        # over it regardless of which of the coverage cell's 5 visibility
        # sample offsets is used.
        resolution = 0.2
        width, height = 25, 5  # 5m x 1m
        data = [0] * (width * height)
        wall_cols = range(10, 15)  # x in [2.0, 3.0)
        for row in range(height):
            for col in wall_cols:
                data[row * width + col] = 100
        grid = _grid(width, height, data, resolution=resolution)
        cg = CoverageGrid(min_x=0.0, max_x=5.0, min_y=0.0, max_y=1.0, cell_size=1.0,
                           camera_range_m=10.0, camera_fov_deg=360.0)
        cg.update(grid, drone_x=0.5, drone_y=0.5, drone_yaw=0.0)
        # Coverage cell col=4 (world x in [4.0, 5.0)) is beyond the wall --
        # LOS must be blocked.
        assert cg.state[0][4] != SEARCHED
        # Coverage cell col=1 (world x in [1.0, 2.0)) is in front of the
        # wall -- LOS must be clear, proving the block isn't just "nothing
        # is ever visible."
        assert cg.state[0][1] == SEARCHED

    def test_unknown_map_cell_stays_unknown_in_coverage(self):
        width, height = 3, 3
        data = [-1] * (width * height)
        grid = _grid(width, height, data)
        cg = CoverageGrid(min_x=0.0, max_x=3.0, min_y=0.0, max_y=3.0, cell_size=1.0)
        free_total, searched_total = cg.update(grid, drone_x=1.5, drone_y=1.5, drone_yaw=0.0)
        assert free_total == 0
        assert searched_total == 0
        assert all(v == UNKNOWN for row in cg.state for v in row)

    def test_searched_cell_stays_searched_across_updates(self):
        cg = CoverageGrid(min_x=0.0, max_x=3.0, min_y=0.0, max_y=3.0, cell_size=1.0,
                           camera_range_m=10.0, camera_fov_deg=360.0)
        grid = _all_free_map(3, 3)
        cg.update(grid, drone_x=1.5, drone_y=1.5, drone_yaw=0.0)
        assert cg.state[1][1] == SEARCHED
        # Even if the drone moves far away next tick, the cell must remain
        # SEARCHED -- coverage is a one-way ratchet, not a live camera view.
        cg.update(grid, drone_x=100.0, drone_y=100.0, drone_yaw=0.0)
        assert cg.state[1][1] == SEARCHED


class TestPercentAndExport:
    def test_percent_searched_zero_when_nothing_known(self):
        cg = CoverageGrid(min_x=0.0, max_x=3.0, min_y=0.0, max_y=3.0, cell_size=1.0)
        assert cg.percent_searched() == 0.0

    def test_percent_searched_reflects_ratio(self):
        cg = CoverageGrid(min_x=0.0, max_x=4.0, min_y=0.0, max_y=1.0, cell_size=1.0,
                           camera_range_m=0.5, camera_fov_deg=360.0)
        grid = _all_free_map(4, 1)
        # Camera range 0.5m: only the drone's own cell (col=0, distance 0)
        # is close enough; cols 1-3 (distance 1.0/2.0/3.0m) are clearly
        # beyond range and stay unsearched. 1 of 4 free cells searched.
        cg.update(grid, drone_x=0.5, drone_y=0.5, drone_yaw=0.0)
        assert cg.percent_searched() == pytest_approx(25.0)

    def test_to_occupancy_grid_data_shape(self):
        cg = CoverageGrid(min_x=1.0, max_x=3.0, min_y=2.0, max_y=4.0, cell_size=1.0)
        exported = cg.to_occupancy_grid_data()
        assert exported["info"]["width"] == 2
        assert exported["info"]["height"] == 2
        assert exported["info"]["origin"]["position"]["x"] == 1.0
        assert exported["info"]["origin"]["position"]["y"] == 2.0
        assert len(exported["data"]) == 4
        assert all(v == UNKNOWN for v in exported["data"])
