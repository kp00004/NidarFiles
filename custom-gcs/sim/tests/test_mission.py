import pytest

from rosbridge_sim.mission import MissionConfig, MissionSimulator


def make_sim(**overrides) -> MissionSimulator:
    defaults = dict(width=8, height=8, resolution=1.0, duration_s=60.0, num_survivors=3, seed=5)
    defaults.update(overrides)
    return MissionSimulator(MissionConfig(**defaults))


def test_config_rejects_too_many_survivors():
    with pytest.raises(ValueError):
        MissionConfig(num_survivors=7)


def test_config_rejects_nonpositive_duration():
    with pytest.raises(ValueError):
        MissionConfig(duration_s=0)


def test_progress_phase_transitions_in_order():
    sim = make_sim()
    d = sim.config.duration_s
    assert sim.progress_phase_at(0) == "entering"
    assert sim.progress_phase_at(d * 0.5) == "searching"
    assert sim.progress_phase_at(d * 0.95) == "exiting"
    assert sim.progress_phase_at(d) == "complete"
    assert sim.progress_phase_at(d * 2) == "complete"


def test_revealed_cells_grow_monotonically_and_reach_full_coverage():
    sim = make_sim()
    total = sim.config.width * sim.config.height
    prev = 0
    for t in (0, 10, 20, 30, 45, 60):
        cells = sim.revealed_cells_at(t)
        assert len(cells) >= prev
        prev = len(cells)
    assert prev == total


def test_revealed_cells_stay_full_past_duration():
    sim = make_sim()
    total = sim.config.width * sim.config.height
    assert len(sim.revealed_cells_at(sim.config.duration_s * 3)) == total


def test_occupancy_grid_at_known_count_matches_revealed_count():
    sim = make_sim()
    for t in (5, 25, 55):
        msg = sim.occupancy_grid_at(t)
        revealed = sim.revealed_cells_at(t)
        known = sum(1 for v in msg["data"] if v != -1)
        assert known == len(revealed)


def test_battery_drains_monotonically_within_bounds():
    sim = make_sim()
    prev_pct = None
    for t in (0, 15, 30, 45, 60):
        b = sim.battery_at(t)
        assert 0.0 <= b["percentage"] <= 1.0
        assert 10.0 < b["voltage"] < 20.0  # sane 4S LiPo range
        if prev_pct is not None:
            assert b["percentage"] <= prev_pct
        prev_pct = b["percentage"]


def test_pose_stays_within_arena_bounds():
    sim = make_sim()
    max_x = sim.config.width * sim.config.resolution
    max_y = sim.config.height * sim.config.resolution
    for t in (0, 10, 30, 60):
        pose = sim.pose_at(t)["pose"]["position"]
        assert 0.0 <= pose["x"] <= max_x
        assert 0.0 <= pose["y"] <= max_y


def test_pose_matches_current_reveal_frontier():
    sim = make_sim()
    t = 20
    revealed = sim.revealed_cells_at(t)
    frontier_row, frontier_col = sim._reveal_order[len(revealed) - 1]
    expected_x = (frontier_col + 0.5) * sim.config.resolution
    expected_y = (frontier_row + 0.5) * sim.config.resolution
    pose = sim.pose_at(t)["pose"]["position"]
    assert pose["x"] == pytest.approx(expected_x)
    assert pose["y"] == pytest.approx(expected_y)


def test_survivors_detected_grows_monotonically_and_caps_at_config():
    sim = make_sim(num_survivors=3)
    counts = [len(sim.survivors_detected_at(t)) for t in (0, 15, 30, 45, 60)]
    assert counts == sorted(counts)
    assert counts[-1] == 3


def test_survivors_have_valid_fields_and_unique_ids():
    sim = make_sim(num_survivors=3)
    detections = sim.survivors_detected_at(sim.config.duration_s)
    ids = [d["survivor_id"] for d in detections]
    assert len(ids) == len(set(ids)) == 3
    for d in detections:
        assert 0.0 <= d["confidence"] <= 1.0
        assert 0.0 <= d["x"] <= sim.config.width * sim.config.resolution
        assert 0.0 <= d["y"] <= sim.config.height * sim.config.resolution


def test_same_seed_is_fully_reproducible():
    a = make_sim(seed=99)
    b = make_sim(seed=99)
    t = 33
    assert a.occupancy_grid_at(t) == b.occupancy_grid_at(t)
    assert a.pose_at(t) == b.pose_at(t)
    assert a.survivors_detected_at(t) == b.survivors_detected_at(t)
