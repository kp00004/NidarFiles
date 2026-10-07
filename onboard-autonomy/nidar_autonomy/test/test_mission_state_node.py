"""Node-level tests for mission_state_node.MissionStateNode's ARM/DISARM/
reset wiring, against a fake flight_command.FlightCommandClient (same
duck-typed-fake pattern as test_mavros_flight_controller.py's
FakeFlightCommandClient) -- no real mavros, no real Pixhawk.

Unlike the rest of this package's tests, this file needs a real rclpy
context (MissionStateNode subclasses rclpy.node.Node) -- gated with
pytest.importorskip so `pytest test/` still collects and runs everything
else standalone (without ROS sourced), exactly as it does today. Run with
ROS sourced (`source /opt/ros/humble/setup.bash`) to actually exercise
this file instead of skipping it.

The background-thread dispatch mission_state_node.py normally uses for
ARM/DISARM attempts (see that module's "Threading note") is replaced here
with a synchronous stand-in (ImmediateThread) so these tests are
deterministic -- the thing under test is the state-machine/dispatch
wiring, not the threading-vs-nested-executor hazard that dispatch exists
to solve (that was found and fixed against real hardware, not something
a unit test can reproduce -- see CHECKPOINT/CURRENT_STATE.md's rclpy-bug
writeup).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import pytest

rclpy = pytest.importorskip("rclpy")

from std_msgs.msg import String  # noqa: E402

from nidar_autonomy import mission_state_node as msn_module  # noqa: E402
from nidar_autonomy.arming_guard import ArmRejected  # noqa: E402
from nidar_autonomy.mission_state_node import MissionStateNode  # noqa: E402


@dataclass
class FakeArmingResult:
    requested: bool
    success: bool
    state_confirmed: bool
    message: str = "fake result"


class FakeFlightCommandClient:
    """Stands in for flight_command.FlightCommandClient -- records every
    arm()/disarm() call so tests can assert on call counts (e.g. no
    duplicate ARM on a repeated "start")."""

    def __init__(self) -> None:
        self.calls: List[str] = []
        self.next_arm_result: Optional[FakeArmingResult] = None
        self.next_arm_raises: Optional[Exception] = None
        self.next_disarm_result: Optional[FakeArmingResult] = None

    def arm(self) -> FakeArmingResult:
        self.calls.append("arm")
        if self.next_arm_raises is not None:
            raise self.next_arm_raises
        return self.next_arm_result or FakeArmingResult(
            requested=True, success=True, state_confirmed=True
        )

    def disarm(self) -> FakeArmingResult:
        self.calls.append("disarm")
        return self.next_disarm_result or FakeArmingResult(
            requested=False, success=True, state_confirmed=True
        )


class ImmediateThread:
    """threading.Thread stand-in whose start() runs target() synchronously
    in the caller's thread instead of a background one."""

    def __init__(self, target, daemon: bool = True) -> None:
        self._target = target

    def start(self) -> None:
        self._target()


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(msn_module.threading, "Thread", ImmediateThread)
    rclpy.init()
    fake = FakeFlightCommandClient()
    n = MissionStateNode(flight_client=fake)
    n.fake_flight = fake  # convenience handle for assertions
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _send(n: MissionStateNode, command: str) -> None:
    n._on_validated_command(String(data=command))


def test_fresh_startup_is_idle(node):
    assert node._machine.state == "idle"


def test_start_arms_and_reaches_entering(node):
    _send(node, "start")
    assert node._machine.state == "entering"
    assert node.fake_flight.calls == ["arm"]


def test_repeated_start_while_entering_does_not_duplicate_arm(node):
    _send(node, "start")
    _send(node, "start")
    _send(node, "start")
    assert node._machine.state == "entering"
    assert node.fake_flight.calls == ["arm"]


def test_abort_while_active_disarms_and_resets_to_idle(node):
    _send(node, "start")
    _send(node, "abort")
    assert node._machine.state == "idle"
    assert node.fake_flight.calls == ["arm", "disarm"]


def test_start_works_again_after_abort_reset(node):
    _send(node, "start")
    _send(node, "abort")
    _send(node, "start")
    assert node._machine.state == "entering"
    assert node.fake_flight.calls == ["arm", "disarm", "arm"]


def test_abort_while_idle_is_safe_and_idempotent(node):
    _send(node, "abort")
    assert node._machine.state == "idle"
    assert node.fake_flight.calls == ["disarm"]


def test_abort_while_already_aborted_is_idempotent(node):
    """An unconfirmed disarm leaves the mission at 'aborted' -- a second
    abort in that state must not crash, re-arm, or fabricate a
    transition, just try DISARM again."""
    node.fake_flight.next_disarm_result = FakeArmingResult(
        requested=False, success=False, state_confirmed=False
    )
    _send(node, "start")
    _send(node, "abort")
    assert node._machine.state == "aborted"
    _send(node, "abort")
    assert node._machine.state == "aborted"
    assert node.fake_flight.calls == ["arm", "disarm", "disarm"]


def test_failed_arm_forces_disarm_and_reaches_aborted_not_idle(node):
    """An ARM that the FCU didn't confirm must not leave /mission/state
    claiming 'entering' (an active, armed mission); it also must not
    reach 'idle' unless the forced DISARM afterward actually confirms."""
    node.fake_flight.next_arm_result = FakeArmingResult(
        requested=True, success=False, state_confirmed=False
    )
    node.fake_flight.next_disarm_result = FakeArmingResult(
        requested=False, success=False, state_confirmed=False
    )
    _send(node, "start")
    assert node._machine.state == "aborted"
    assert node.fake_flight.calls == ["arm", "disarm"]


def test_failed_arm_with_confirmed_disarm_resets_to_idle(node):
    node.fake_flight.next_arm_result = FakeArmingResult(
        requested=True, success=False, state_confirmed=False
    )
    _send(node, "start")
    assert node._machine.state == "idle"
    assert node.fake_flight.calls == ["arm", "disarm"]


def test_arm_rejected_before_fcu_forces_disarm_and_reaches_aborted(node):
    node.fake_flight.next_arm_raises = ArmRejected("already armed")
    node.fake_flight.next_disarm_result = FakeArmingResult(
        requested=False, success=False, state_confirmed=False
    )
    _send(node, "start")
    assert node._machine.state == "aborted"
    assert node.fake_flight.calls == ["arm", "disarm"]


def test_unsolicited_fcu_disarm_does_not_reset_to_idle(node):
    from mavros_msgs.msg import State

    _send(node, "start")
    assert node._machine.state == "entering"

    armed_msg = State()
    armed_msg.armed = True
    node._on_fcu_state(armed_msg)

    disarmed_msg = State()
    disarmed_msg.armed = False
    node._on_fcu_state(disarmed_msg)

    assert node._machine.state == "aborted"
    # No disarm() was called by this node -- the FCU disarmed itself.
    assert node.fake_flight.calls == ["arm"]
