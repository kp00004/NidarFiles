from rosbridge_sim.grid import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    build_occupancy_grid_msg,
    compute_reveal_order,
    generate_ground_truth,
)


def test_ground_truth_deterministic():
    a = generate_ground_truth(10, 10, seed=1)
    b = generate_ground_truth(10, 10, seed=1)
    assert a == b


def test_ground_truth_different_seeds_can_differ():
    a = generate_ground_truth(12, 12, seed=1)
    b = generate_ground_truth(12, 12, seed=2)
    assert a != b


def test_ground_truth_border_is_always_occupied():
    width, height = 6, 6
    grid = generate_ground_truth(width, height, seed=7)
    for c in range(width):
        assert grid[0][c] == OCCUPIED
        assert grid[height - 1][c] == OCCUPIED
    for r in range(height):
        assert grid[r][0] == OCCUPIED
        assert grid[r][width - 1] == OCCUPIED


def test_ground_truth_entry_cell_is_free():
    grid = generate_ground_truth(8, 8, seed=3)
    assert grid[1][1] == FREE


def test_reveal_order_visits_every_cell_exactly_once():
    width, height = 8, 8
    order = compute_reveal_order(width, height, seed=3)
    assert len(order) == width * height
    assert len(set(order)) == width * height
    assert set(order) == {(r, c) for r in range(height) for c in range(width)}


def test_reveal_order_deterministic():
    a = compute_reveal_order(8, 8, seed=3)
    b = compute_reveal_order(8, 8, seed=3)
    assert a == b


def test_reveal_order_starts_at_entry_cell():
    order = compute_reveal_order(9, 9, seed=11)
    assert order[0] == (1, 1)


def test_occupancy_grid_message_shape_and_content():
    width, height = 5, 5
    ground_truth = generate_ground_truth(width, height, seed=2)
    revealed = {(0, 0), (0, 1)}

    msg = build_occupancy_grid_msg(
        ground_truth, revealed, resolution=1.0, width=width, height=height, stamp_sec=100
    )

    assert msg["info"]["width"] == width
    assert msg["info"]["height"] == height
    assert msg["info"]["resolution"] == 1.0
    assert msg["info"]["origin"]["position"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert len(msg["data"]) == width * height

    # Unrevealed cell reports UNKNOWN regardless of ground truth.
    unrevealed_index = 2 * width + 2
    assert msg["data"][unrevealed_index] == UNKNOWN

    # Revealed cells report their ground-truth value.
    for row, col in revealed:
        assert msg["data"][row * width + col] == ground_truth[row][col]
