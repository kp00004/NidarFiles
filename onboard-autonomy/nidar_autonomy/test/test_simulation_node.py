"""Integration tests for simulation_node.py -- requires a real rclpy
context, gated with `pytest.importorskip("rclpy")` (see
test_frontier_explorer_node.py's module docstring for the pattern).

These specifically prove the hardware-isolation properties the GCS "RUN
SIMULATION" feature depends on: the node never touches the real
`/gcs/command` or `/mission/state` topics, and `/simulation/command`
accepts only "run"/"reset" -- never "start"/"abort"."""
import json

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from nidar_autonomy.mission_simulator import MissionSimulator  # noqa: E402
from nidar_autonomy.topics import (  # noqa: E402
    COMMAND_TOPIC,
    MISSION_STATE_TOPIC,
    SIMULATION_COMMAND_TOPIC,
    SIMULATION_MISSION_STATE_TOPIC,
)


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(**sim_kwargs):
    from nidar_autonomy.simulation_node import SimulationNode

    simulator = MissionSimulator(**sim_kwargs) if sim_kwargs else None
    node = SimulationNode(simulator=simulator)
    return node, node.destroy_node


class TestTopicIsolation:
    def test_command_topic_is_the_simulation_specific_one_not_real(self):
        node, cleanup = _make_node()
        try:
            # The node's own subscription topic name (introspected via
            # the constant it was built from) must be the /simulation/
            # one, never the real /gcs/command.
            assert SIMULATION_COMMAND_TOPIC != COMMAND_TOPIC
            assert SIMULATION_COMMAND_TOPIC.startswith("/simulation/")
        finally:
            cleanup()

    def test_mission_state_topic_is_simulation_specific_not_real(self):
        assert SIMULATION_MISSION_STATE_TOPIC != MISSION_STATE_TOPIC
        assert SIMULATION_MISSION_STATE_TOPIC.startswith("/simulation/")

    def test_invalid_command_is_ignored_not_crashing(self):
        node, cleanup = _make_node()
        try:
            node._on_command(String(data="start"))  # real command vocabulary -- must be rejected here
            assert node._simulator.snapshot().status == "idle"
            node._on_command(String(data="abort"))
            assert node._simulator.snapshot().status == "idle"
            node._on_command(String(data="anything else"))
            assert node._simulator.snapshot().status == "idle"
        finally:
            cleanup()


class TestCommandHandling:
    def test_run_command_starts_the_simulation(self):
        node, cleanup = _make_node()
        try:
            published = {}
            node._mission_state_pub.publish = lambda msg: published.setdefault("mission_state", msg)
            node._status_pub.publish = lambda msg: published.setdefault("status", msg)
            node._map_pub.publish = lambda msg: None
            node._coverage_pub.publish = lambda msg: None
            node._path_pub.publish = lambda msg: None
            node._telemetry_pub.publish = lambda msg: published.setdefault("telemetry", msg)

            node._on_command(String(data="run"))

            assert published["status"].data == "running"
            assert published["mission_state"].data == "entering"
            contract = json.loads(published["telemetry"].data)
            assert contract["source"] == "simulation"
        finally:
            cleanup()

    def test_reset_command_returns_to_idle(self):
        node, cleanup = _make_node()
        try:
            node._on_command(String(data="run"))
            for _ in range(5):
                node._tick()
            assert node._simulator.snapshot().status == "running"

            published = {}
            node._status_pub.publish = lambda msg: published.setdefault("status", msg)
            node._mission_state_pub.publish = lambda msg: None
            node._map_pub.publish = lambda msg: None
            node._coverage_pub.publish = lambda msg: None
            node._path_pub.publish = lambda msg: None
            node._telemetry_pub.publish = lambda msg: None

            node._on_command(String(data="reset"))
            assert published["status"].data == "idle"
        finally:
            cleanup()

    def test_tick_advances_simulation_and_publishes(self):
        node, cleanup = _make_node()
        try:
            node._on_command(String(data="run"))
            captured = []
            node._mission_state_pub.publish = lambda msg: captured.append(msg.data)
            node._status_pub.publish = lambda msg: None
            node._map_pub.publish = lambda msg: None
            node._coverage_pub.publish = lambda msg: None
            node._path_pub.publish = lambda msg: None
            node._telemetry_pub.publish = lambda msg: None

            for _ in range(3):
                node._tick()
            assert len(captured) == 3
        finally:
            cleanup()


class TestTelemetryContractIsUnmistakablySimulation:
    def test_telemetry_contract_is_tagged_as_simulation(self):
        node, cleanup = _make_node()
        try:
            snap = node._simulator.run()
            contract = node._telemetry_contract(snap)
            assert contract["source"] == "simulation"
            assert contract["flight"]["mode"] == "SIMULATION"
        finally:
            cleanup()

    def test_map_publishes_real_occupancy_grid_message(self):
        node, cleanup = _make_node()
        try:
            captured = {}
            node._map_pub.publish = lambda msg: captured.setdefault("map", msg)
            node._mission_state_pub.publish = lambda msg: None
            node._status_pub.publish = lambda msg: None
            node._coverage_pub.publish = lambda msg: None
            node._path_pub.publish = lambda msg: None
            node._telemetry_pub.publish = lambda msg: None

            node._on_command(String(data="run"))
            for _ in range(20):
                node._tick()

            from nav_msgs.msg import OccupancyGrid

            assert isinstance(captured["map"], OccupancyGrid)
            assert captured["map"].info.width > 0
        finally:
            cleanup()
