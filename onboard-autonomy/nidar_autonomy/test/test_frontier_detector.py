import copy

import pytest

from nidar_autonomy.frontier_detector import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    FrontierCandidate,
    InvalidOccupancyGridError,
    detect_frontiers,
)
from nidar_autonomy.indoor_environment import empty_room, room_with_single_obstacle


# -- fixture builders -----------------------------------------------------------


def _mask_to_unknown(grid, keep_free):
    """Return a deep copy of `grid` with every cell forced to -1 (unknown)
    except the `(col, row)` cells in `keep_free`, which stay whatever
    `grid` already had them as. Simulates "not yet explored" over a
    ground-truth grid -- the standard way to build partial-exploration
    test input without needing real SLAM output (see this module's own
    module docstring)."""
    new_grid = copy.deepcopy(grid)
    info = new_grid["info"]
    width, height = info["width"], info["height"]
    data = list(new_grid["data"])
    for row in range(height):
        for col in range(width):
            if (col, row) not in keep_free:
                data[row * width + col] = UNKNOWN
    new_grid["data"] = data
    return new_grid


def _all_unknown(grid):
    new_grid = copy.deepcopy(grid)
    new_grid["data"] = [UNKNOWN] * len(new_grid["data"])
    return new_grid


def _grid_from_column_pattern(pattern, height=5, resolution=1.0, origin=(0.0, 0.0)):
    """Build a grid dict whose cell value only depends on column (every
    row identical) -- lets a test spell out a 1D layout
    (free/unknown/occupied per column) and get a full 2D grid dict out,
    used by the grouping/scoring tests below where the exact size of
    isolated unknown pockets needs precise, hand-picked control that the
    canned `indoor_environment` scenarios don't offer."""
    width = len(pattern)
    data = []
    for _row in range(height):
        data.extend(pattern)
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
        "info": {
            "resolution": resolution,
            "width": width,
            "height": height,
            "origin": {
                "position": {"x": origin[0], "y": origin[1], "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
        "data": data,
    }


# -- fixture 1: small known region near start, rest unknown ---------------------


def test_small_known_region_produces_frontier_along_its_boundary():
    base = empty_room().to_occupancy_grid_data()
    keep_free = {(col, row) for col in range(3) for row in range(3)}
    grid = _mask_to_unknown(base, keep_free)

    candidates = detect_frontiers(grid)

    # the 3x3 known block sits at the grid's own (0,0) corner, so only
    # its right column and bottom row actually border unknown space --
    # its top/left sides border the true grid edge (out of bounds, not
    # unknown), and its (1,0)/(0,1) edge-but-not-corner cells only touch
    # free/out-of-bounds neighbors too. That leaves exactly 5 frontier
    # cells: (2,0), (2,1), (2,2), (0,2), (1,2) -- 8-connected into one
    # region, not 5 separate single-cell candidates.
    assert len(candidates) == 1
    candidate = candidates[0]
    assert candidate.cell_count == 5
    # candidate position must land within/near the known 3x3 block
    # (world x,y in [0,3)), not somewhere arbitrary in the 15x15 arena.
    assert 0.0 <= candidate.x <= 3.0
    assert 0.0 <= candidate.y <= 3.0
    # score is the size of the single contiguous unknown region
    # surrounding the block: the whole rest of the 15x15 grid.
    assert candidate.score == 225 - 9


# -- fixture 2: fully explored, no unknown cells at all --------------------------


def test_fully_explored_grid_has_no_frontiers():
    grid = room_with_single_obstacle().to_occupancy_grid_data()
    assert UNKNOWN not in grid["data"]
    assert detect_frontiers(grid) == []


# -- fixture 3: entirely unknown, nothing explored yet ---------------------------


def test_fully_unknown_grid_has_no_frontiers():
    # No free cells exist yet to be a frontier *from* -- see
    # detect_frontiers' own docstring for why this is the deliberate,
    # documented behavior rather than a fallback/bootstrap candidate.
    grid = _all_unknown(empty_room().to_occupancy_grid_data())
    assert detect_frontiers(grid) == []


# -- determinism ------------------------------------------------------------------


def test_determinism_same_grid_twice_yields_identical_candidates():
    base = empty_room().to_occupancy_grid_data()
    keep_free = {(col, row) for col in range(3) for row in range(3)}
    grid = _mask_to_unknown(base, keep_free)

    result_a = detect_frontiers(grid)
    result_b = detect_frontiers(grid)
    assert result_a == result_b


def test_determinism_two_region_grid_preserves_order():
    pattern = [FREE, FREE, FREE, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, FREE, FREE, FREE]
    grid = _grid_from_column_pattern(pattern)

    result_a = detect_frontiers(grid)
    result_b = detect_frontiers(grid)
    assert result_a == result_b
    assert [c.x for c in result_a] == [c.x for c in result_b]


# -- malformed input --------------------------------------------------------------


def test_missing_info_key_raises_typed_error():
    with pytest.raises(InvalidOccupancyGridError):
        detect_frontiers({"data": []})


def test_missing_data_key_raises_typed_error():
    grid = empty_room().to_occupancy_grid_data()
    del grid["data"]
    with pytest.raises(InvalidOccupancyGridError):
        detect_frontiers(grid)


def test_missing_info_subfield_raises_typed_error():
    grid = empty_room().to_occupancy_grid_data()
    del grid["info"]["width"]
    with pytest.raises(InvalidOccupancyGridError):
        detect_frontiers(grid)


def test_missing_origin_position_raises_typed_error():
    grid = empty_room().to_occupancy_grid_data()
    del grid["info"]["origin"]["position"]["x"]
    with pytest.raises(InvalidOccupancyGridError):
        detect_frontiers(grid)


def test_data_length_mismatch_raises_typed_error():
    grid = empty_room().to_occupancy_grid_data()
    grid["data"] = grid["data"][:-1]  # one short of width*height
    with pytest.raises(InvalidOccupancyGridError):
        detect_frontiers(grid)


def test_non_positive_dimensions_raise_typed_error():
    grid = empty_room().to_occupancy_grid_data()
    grid["info"]["width"] = 0
    grid["data"] = []
    with pytest.raises(InvalidOccupancyGridError):
        detect_frontiers(grid)


# -- frontier grouping --------------------------------------------------------------


def test_two_well_separated_boundaries_produce_two_distinct_regions_not_a_flood():
    # cols 0-2 free, cols 3-7 unknown (5-wide gap), cols 8-10 free -- two
    # known blocks with a wide unknown gap between them, far enough apart
    # (distance 6 columns) that their frontier cells cannot be
    # 8-connected to each other.
    pattern = [FREE, FREE, FREE, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, FREE, FREE, FREE]
    grid = _grid_from_column_pattern(pattern)

    candidates = detect_frontiers(grid)

    assert len(candidates) == 2
    # each region is the single boundary column (5 cells, one per row),
    # grouped into one candidate each -- not five separate single-cell
    # candidates per wall.
    assert [c.cell_count for c in candidates] == [5, 5]
    xs = sorted(c.x for c in candidates)
    assert xs[0] < 4.0  # left region's centroid near col=2
    assert xs[1] > 7.0  # right region's centroid near col=8


# -- scoring ------------------------------------------------------------------------


def test_frontier_bordering_larger_unknown_region_scores_higher():
    # col1: frontier bordering a 1-column-wide (5-cell) isolated unknown
    # pocket at col2, walled off at col3 (occupied) from anything beyond.
    # col5: frontier bordering a 7-column-wide (35-cell) unknown region at
    # cols 6-12, open to the grid edge. Same frontier region size (5
    # cells each) on both sides -- only the adjacent unknown area differs
    # -- isolates the property under test (score ordering) from
    # incidental frontier-size differences.
    pattern = [
        FREE, FREE, UNKNOWN, OCCUPIED, FREE, FREE,
        UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN,
    ]
    grid = _grid_from_column_pattern(pattern)

    candidates = detect_frontiers(grid)

    assert len(candidates) == 2
    small_pocket_candidate = min(candidates, key=lambda c: c.score)
    large_pocket_candidate = max(candidates, key=lambda c: c.score)
    assert small_pocket_candidate.score == 5.0
    assert large_pocket_candidate.score == 35.0
    assert large_pocket_candidate.score > small_pocket_candidate.score
    assert small_pocket_candidate.x < large_pocket_candidate.x


# -- FrontierCandidate shape --------------------------------------------------------


def test_frontier_candidate_is_frozen_dataclass_with_expected_fields():
    candidate = FrontierCandidate(x=1.0, y=2.0, score=3.0, cell_count=4)
    assert candidate.x == 1.0
    assert candidate.y == 2.0
    assert candidate.score == 3.0
    assert candidate.cell_count == 4
    with pytest.raises(Exception):
        candidate.x = 5.0  # frozen -- immutable once constructed
