"""Deterministic synthetic arena generation and OccupancyGrid message
building, matching the 1 m x 1 m grid convention in docs/DECISIONS.md
D-7/D-7a and the message shape in docs/DATA_MODELS.md §4.

Everything here is a pure function of (dimensions, seed) — no I/O, no
clock — so it's fully unit-testable and produces the same arena for the
same seed every run (useful for reproducible manual testing too).
"""

from __future__ import annotations

import random

FREE = 0
OCCUPIED = 100
UNKNOWN = -1

Cell = tuple[int, int]  # (row, col)


def generate_ground_truth(width: int, height: int, seed: int) -> list[list[int]]:
    """Ground-truth occupancy for the synthetic arena: FREE (0) or
    OCCUPIED (100) per cell, border always occupied (arena walls), with
    some deterministic interior walls scattered in based on `seed`."""
    if width < 2 or height < 2:
        raise ValueError("arena must be at least 2x2")

    rng = random.Random(seed)
    grid = [[FREE for _ in range(width)] for _ in range(height)]

    for c in range(width):
        grid[0][c] = OCCUPIED
        grid[height - 1][c] = OCCUPIED
    for r in range(height):
        grid[r][0] = OCCUPIED
        grid[r][width - 1] = OCCUPIED

    # Scatter some interior walls, keeping the entry cell (1,1) clear so
    # the reveal walk always has somewhere to start.
    interior = [
        (r, c)
        for r in range(1, height - 1)
        for c in range(1, width - 1)
        if (r, c) != (1, 1)
    ]
    wall_count = max(0, len(interior) // 6)
    for r, c in rng.sample(interior, k=min(wall_count, len(interior))):
        grid[r][c] = OCCUPIED

    return grid


def compute_reveal_order(width: int, height: int, seed: int) -> list[Cell]:
    """Deterministic order in which cells are "explored", starting from
    the entry cell (1,1) and randomly walking the frontier until every
    cell has been visited exactly once. Used to simulate the map filling
    in progressively over the course of a mission, and to derive the
    drone's simulated position (it's always at/near the exploration
    frontier)."""
    rng = random.Random(seed)
    start = (1, 1) if width > 2 and height > 2 else (0, 0)

    visited: set[Cell] = {start}
    order: list[Cell] = [start]
    frontier: list[Cell] = [start]

    all_cells = {(r, c) for r in range(height) for c in range(width)}

    while len(visited) < len(all_cells):
        if not frontier:
            # Disconnected remainder (can't happen with current wall
            # density, but don't hang forever if it ever does): pull in
            # any unvisited cell to keep the order total.
            remaining = sorted(all_cells - visited)
            next_cell = remaining[0]
            visited.add(next_cell)
            order.append(next_cell)
            frontier.append(next_cell)
            continue

        current = frontier[-1]
        r, c = current
        neighbors = [
            (r + dr, c + dc)
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1))
            if (r + dr, c + dc) in all_cells and (r + dr, c + dc) not in visited
        ]
        if not neighbors:
            frontier.pop()
            continue

        nxt = neighbors[rng.randrange(len(neighbors))]
        visited.add(nxt)
        order.append(nxt)
        frontier.append(nxt)

    return order


def build_occupancy_grid_msg(
    ground_truth: list[list[int]],
    revealed: set[Cell],
    *,
    resolution: float,
    width: int,
    height: int,
    stamp_sec: int,
    stamp_nanosec: int = 0,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
) -> dict:
    """Build a nav_msgs/OccupancyGrid-shaped dict (docs/DATA_MODELS.md §4).
    Cells not yet in `revealed` are UNKNOWN; revealed cells report their
    ground-truth value. Row-major, matching real OccupancyGrid semantics."""
    data = [UNKNOWN] * (width * height)
    for r, c in revealed:
        data[r * width + c] = ground_truth[r][c]

    return {
        "header": {"stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec}, "frame_id": "map"},
        "info": {
            "resolution": resolution,
            "width": width,
            "height": height,
            "origin": {
                "position": {"x": origin_x, "y": origin_y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
        "data": data,
    }
