"""Integration tests for telemetry_bridge_node.py -- requires a real rclpy
context, gated with `pytest.importorskip("rclpy")` (see
test_frontier_explorer_node.py's module docstring for the pattern/rationale)."""
import json

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from mavros_msgs.msg import State  # noqa: E402
from sensor_msgs.msg import BatteryState  # noqa: E402
from std_msgs.msg import String  # noqa: E402


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    from nidar_autonomy.telemetry_bridge_node import TelemetryBridgeNode

    node = TelemetryBridgeNode()
    return node, node.destroy_node


class TestTick:
    def test_tick_with_no_data_yet_publishes_a_valid_contract(self):
        node, cleanup = _make_node()
        try:
            captured = {}
            node.pub.publish = lambda msg: captured.setdefault("out", msg)
            node._tick()
            contract = json.loads(captured["out"].data)
            assert contract["schema_version"] == 1
            assert contract["connection"]["connected"] is False
            assert contract["flight"]["armed"] is False
            assert contract["mapping"]["available"] is False
            assert contract["survivors"] == []
        finally:
            cleanup()

    def test_tick_reflects_real_fcu_state_message(self):
        node, cleanup = _make_node()
        try:
            state = State()
            state.connected = True
            state.armed = True
            state.mode = "GUIDED"
            state.system_status = 4
            node._on_state(state)

            captured = {}
            node.pub.publish = lambda msg: captured.setdefault("out", msg)
            node._tick()
            contract = json.loads(captured["out"].data)
            assert contract["connection"]["connected"] is True
            assert contract["flight"]["armed"] is True
            assert contract["flight"]["mode"] == "GUIDED"
        finally:
            cleanup()

    def test_tick_reflects_real_battery_message(self):
        node, cleanup = _make_node()
        try:
            battery = BatteryState()
            battery.percentage = 0.55
            node._on_battery(battery)

            captured = {}
            node.pub.publish = lambda msg: captured.setdefault("out", msg)
            node._tick()
            contract = json.loads(captured["out"].data)
            assert contract["flight"]["battery_pct"] == pytest.approx(55.0)
        finally:
            cleanup()

    def test_mission_state_flows_through_to_autonomy_block(self):
        node, cleanup = _make_node()
        try:
            node._on_mission_state(String(data="searching"))

            captured = {}
            node.pub.publish = lambda msg: captured.setdefault("out", msg)
            node._tick()
            contract = json.loads(captured["out"].data)
            assert contract["mission"]["state"] == "searching"
            assert contract["autonomy"]["objective"] == "Explore unexplored region"
        finally:
            cleanup()

    def test_malformed_explorer_status_does_not_crash(self):
        node, cleanup = _make_node()
        try:
            node._on_explorer_status(String(data="not valid json"))
            node._tick()  # must not raise
        finally:
            cleanup()
