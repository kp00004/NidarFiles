"""Integration tests for frontier_explorer_node.py -- requires a real rclpy
context (message types, Node construction), gated with
`pytest.importorskip("rclpy")` so this file skips cleanly without ROS
sourced and actually runs when it is, same pattern as this repo's existing
test_mission_state_node.py.

Node internals (`map_msg`, `pose_xy`) are set directly rather than published
over a real topic + spun, to keep these tests fast and deterministic (no
discovery latency) -- a legitimate white-box test of `_tick()`'s logic,
which is what actually matters here (the pub/sub wiring itself is exercised
by the node-construction smoke test in test_telemetry_bridge_node.py's
sibling checks, and manually against a real ROS graph -- see the migration
report for what has and hasn't been run that way).
"""
import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402


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


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    from nidar_autonomy.frontier_explorer_node import FrontierExplorerNode

    node = FrontierExplorerNode()
    return node, node.destroy_node


def _occupancy_grid_msg(grid: dict):
    from nidar_autonomy.ros_conversions import dict_to_occupancy_grid_msg

    return dict_to_occupancy_grid_msg({"header": {"frame_id": "map"}, **grid})


class TestTickPublishesAPathWhenAFrontierExists:
    def test_frontier_found_and_path_planned(self):
        node, cleanup = _make_node()
        try:
            # 12x12 free grid with an unexplored band on the far edge -- a
            # real frontier for detect_frontiers to find.
            width, height = 12, 12
            data = [0] * (width * height)
            for row in range(height):
                for col in range(width):
                    if col >= 10:
                        data[row * width + col] = -1  # unknown
            node.map_msg = _occupancy_grid_msg(_grid(width, height, data))
            node.pose_xy = (1.5, 1.5)

            published = {}
            node.path_pub.publish = lambda msg: published.setdefault("path", msg)
            node.marker_pub.publish = lambda msg: published.setdefault("markers", msg)

            node._tick()

            assert node._last_frontier_count > 0
            assert node.goal is not None
            assert "path" in published
            assert len(published["path"].poses) >= 1
        finally:
            cleanup()

    def test_no_frontier_publishes_no_path_and_clears_goal(self):
        node, cleanup = _make_node()
        try:
            width, height = 5, 5
            data = [0] * (width * height)  # fully known, fully free -- no frontiers
            node.map_msg = _occupancy_grid_msg(_grid(width, height, data))
            node.pose_xy = (1.5, 1.5)

            published = {}
            node.path_pub.publish = lambda msg: published.setdefault("path", msg)
            node.marker_pub.publish = lambda msg: published.setdefault("markers", msg)

            node._tick()

            assert node._last_frontier_count == 0
            assert node.goal is None
            assert "path" not in published
        finally:
            cleanup()

    def test_status_reports_waiting_before_map_or_pose_arrive(self):
        node, cleanup = _make_node()
        try:
            captured = {}
            node.status_pub.publish = lambda msg: captured.setdefault("status", msg)
            node._publish_status()
            import json

            payload = json.loads(captured["status"].data)
            assert payload["state"] == "waiting"
        finally:
            cleanup()

    def test_unreachable_goal_gets_blacklisted_not_replanned_forever(self):
        """A frontier the planner can never route to (behind a solid,
        floor-to-ceiling wall) must be recorded as a failure, not re-picked
        immediately next tick."""
        node, cleanup = _make_node()
        try:
            node.min_frontier_cells = 1  # isolate the reachability behavior under test

            width, height = 12, 12
            data = [0] * (width * height)
            # Unknown region beyond col=9, rows 7-11 -- creates frontier
            # cells at col=8 (free, bordering unknown) in those rows.
            for row in range(7, height):
                for col in range(9, width):
                    data[row * width + col] = -1
            # Full-width wall at row=6 -- the ONLY way from the start
            # (row 1) to the frontier region (rows 7+) would have to cross
            # it, so it must be genuinely unreachable, not just far.
            for col in range(width):
                data[6 * width + col] = 100
            node.map_msg = _occupancy_grid_msg(_grid(width, height, data))
            node.pose_xy = (1.5, 1.5)

            node.path_pub.publish = lambda msg: None
            node.marker_pub.publish = lambda msg: None

            node._tick()

            assert node._last_frontier_count >= 1  # frontier WAS detected...
            assert node.goal is None  # ...but correctly abandoned as unreachable
            assert len(node.blacklist) >= 1
        finally:
            cleanup()
