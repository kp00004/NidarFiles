"""Tests for sensor_model.py -- the simulated LiDAR visibility model used
by the simulation harness. Pure logic, no ROS required."""
from nidar_autonomy.indoor_environment import empty_room, room_with_single_obstacle
from nidar_autonomy.sensor_model import reveal_cells


class TestEmptyRoom:
    def test_reveals_free_cells_within_range(self):
        env = empty_room()
        visible = reveal_cells(env, pose=(0.5, 0.5), sensor_range_m=2.0)
        assert len(visible) > 0
        assert all(value == 0 for value in visible.values())  # empty room -- everything free

    def test_reveals_nothing_beyond_range(self):
        env = empty_room()
        visible = reveal_cells(env, pose=(0.5, 0.5), sensor_range_m=2.0)
        far_cell = (14, 14)  # near the far corner of the 15x15 arena
        assert far_cell not in visible

    def test_center_cell_always_visible(self):
        env = empty_room()
        visible = reveal_cells(env, pose=(0.5, 0.5), sensor_range_m=0.5)
        assert (0, 0) in visible
        assert visible[(0, 0)] == 0

    def test_out_of_bounds_never_included(self):
        env = empty_room()
        visible = reveal_cells(env, pose=(0.5, 0.5), sensor_range_m=50.0)
        assert all(0 <= c < env.width_cells and 0 <= r < env.height_cells for c, r in visible)

    def test_deterministic(self):
        env = empty_room()
        v1 = reveal_cells(env, pose=(5.5, 5.5), sensor_range_m=3.0)
        v2 = reveal_cells(env, pose=(5.5, 5.5), sensor_range_m=3.0)
        assert v1 == v2

    def test_larger_range_reveals_more(self):
        env = empty_room()
        small = reveal_cells(env, pose=(7.5, 7.5), sensor_range_m=1.0)
        large = reveal_cells(env, pose=(7.5, 7.5), sensor_range_m=4.0)
        assert len(large) > len(small)
        assert set(small.keys()).issubset(set(large.keys()))


class TestWallOcclusion:
    """room_with_single_obstacle(): a 1m-wide, 10m-tall wall at col=7,
    rows 0..9 (world x in [7.0, 8.0)), start_pose=(1.5, 1.5)."""

    def test_wall_cell_itself_is_revealed_as_occupied(self):
        env = room_with_single_obstacle()
        # Sensor close to the wall, looking straight at it.
        visible = reveal_cells(env, pose=(6.5, 5.5), sensor_range_m=2.0)
        assert (7, 5) in visible
        assert visible[(7, 5)] == 100

    def test_cell_directly_behind_wall_is_not_visible(self):
        env = room_with_single_obstacle()
        # Sensor on the near side of the wall; a cell on the far side,
        # directly behind it, must not be revealed even though it's within
        # range -- the wall blocks line of sight.
        visible = reveal_cells(env, pose=(6.5, 5.5), sensor_range_m=5.0)
        far_side_cell = (9, 5)  # world x=9.5, directly behind the wall at col=7
        assert far_side_cell not in visible

    def test_cell_reachable_around_the_open_top_is_visible(self):
        env = room_with_single_obstacle()
        # The wall only spans rows 0..9 (world y in [0,10)); a sensor
        # positioned above the wall's open top should see cells on both
        # sides at that row.
        visible = reveal_cells(env, pose=(7.5, 11.5), sensor_range_m=2.5)
        assert (6, 11) in visible
        assert (8, 11) in visible

    def test_no_line_of_sight_means_nothing_revealed_beyond_wall_from_start(self):
        env = room_with_single_obstacle()
        visible = reveal_cells(env, pose=env.start_pose, sensor_range_m=3.0)
        # start_pose is (1.5, 1.5) -- well clear of the wall at x in
        # [7,8) -- confirm no far-side cell leaks through despite being
        # within nominal range of a much larger radius.
        assert (9, 1) not in visible
