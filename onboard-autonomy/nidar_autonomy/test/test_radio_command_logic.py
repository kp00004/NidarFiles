"""Unit tests for radio_command_logic.CommandGate (pure, no ROS)."""
import pytest

from nidar_autonomy.radio_command_logic import CommandGate, Readiness
from nidar_autonomy.telem_command_codec import (
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_DENIED,
    MAV_RESULT_TEMPORARILY_REJECTED,
    MISSION_HOVER,
    MISSION_MOTOR_TEST,
    REASON_BAD_PROTOCOL_VERSION,
    REASON_FCU_NOT_CONNECTED,
    REASON_MISSION_BUSY,
    REASON_MISSION_NOT_READY,
    REASON_OK,
    REASON_UNKNOWN_MISSION,
    RadioCommand,
)

GCS = (255, 190)
READY = Readiness(fcu_connected=True, missions={"hover": "idle", "motor_test": "idle"})


def _start(nonce=1, mission=MISSION_HOVER, version=2):
    return RadioCommand("start", nonce, mission, version)


def test_valid_hover_start_is_forwarded_with_mission_id():
    d = CommandGate().decide(_start(), GCS, READY)
    assert (d.forward, d.mission_id, d.result, d.reason) == ("start", "hover", MAV_RESULT_ACCEPTED, REASON_OK)


@pytest.mark.parametrize(
    "cmd,readiness,result,reason",
    [
        (_start(mission=7), READY, MAV_RESULT_DENIED, REASON_UNKNOWN_MISSION),
        (_start(mission=0), READY, MAV_RESULT_DENIED, REASON_UNKNOWN_MISSION),
        (_start(version=1), READY, MAV_RESULT_DENIED, REASON_BAD_PROTOCOL_VERSION),
        (_start(), Readiness(True, {}), MAV_RESULT_TEMPORARILY_REJECTED, REASON_MISSION_NOT_READY),
        (_start(), Readiness(False, {"hover": "idle"}), MAV_RESULT_TEMPORARILY_REJECTED, REASON_FCU_NOT_CONNECTED),
        (_start(), Readiness(True, {"hover": "hovering"}), MAV_RESULT_TEMPORARILY_REJECTED, REASON_MISSION_BUSY),
    ],
)
def test_start_rejections_never_forward(cmd, readiness, result, reason):
    d = CommandGate().decide(cmd, GCS, readiness)
    assert d.forward is None
    assert (d.result, d.reason) == (result, reason)


@pytest.mark.parametrize("state", ["complete", "aborted", "failed", "pilot_override"])
def test_start_allowed_after_a_finished_run(state):
    assert CommandGate().decide(_start(), GCS, Readiness(True, {"hover": state})).forward == "start"


@pytest.mark.parametrize(
    "readiness",
    [Readiness(False, {}), Readiness(True, {"hover": "hovering"}), Readiness(False, {"hover": "landing"})],
)
def test_abort_is_always_forwarded(readiness):
    d = CommandGate().decide(RadioCommand("abort", 3, 0, 1), GCS, readiness)
    assert (d.forward, d.result) == ("abort", MAV_RESULT_ACCEPTED)


def test_resend_returns_original_decision_and_is_not_forwarded_again():
    gate = CommandGate()
    first = gate.decide(_start(nonce=9), GCS, READY)
    again = gate.decide(_start(nonce=9), GCS, Readiness(False, {}))
    assert first.forward == "start"
    assert again.duplicate and again.forward is None
    assert (again.result, again.reason) == (first.result, first.reason)


def test_rejected_start_resend_stays_rejected():
    gate = CommandGate()
    gate.decide(_start(nonce=4), GCS, Readiness(False, {"hover": "idle"}))
    again = gate.decide(_start(nonce=4), GCS, READY)
    assert again.duplicate and again.reason == REASON_FCU_NOT_CONNECTED


def test_same_nonce_from_another_sender_is_a_new_command():
    gate = CommandGate()
    gate.decide(_start(nonce=5), GCS, READY)
    assert gate.decide(_start(nonce=5), (254, 190), READY).forward == "start"


# -- several missions ----------------------------------------------------------


def test_motor_test_start_is_forwarded_with_its_mission_id():
    d = CommandGate().decide(_start(mission=MISSION_MOTOR_TEST), GCS, READY)
    assert (d.forward, d.mission_id) == ("start", "motor_test")


def test_start_for_a_mission_whose_node_is_not_running_is_not_ready():
    d = CommandGate().decide(_start(mission=MISSION_MOTOR_TEST), GCS, Readiness(True, {"hover": "idle"}))
    assert (d.forward, d.reason) == (None, REASON_MISSION_NOT_READY)


@pytest.mark.parametrize(
    "requested,missions",
    [
        (MISSION_MOTOR_TEST, {"hover": "hovering", "motor_test": "idle"}),
        (MISSION_HOVER, {"hover": "idle", "motor_test": "testing"}),
        (MISSION_HOVER, {"hover": "idle", "motor_test": "preflight"}),
    ],
)
def test_no_start_while_another_mission_is_running(requested, missions):
    d = CommandGate().decide(_start(mission=requested), GCS, Readiness(True, missions))
    assert (d.forward, d.reason) == (None, REASON_MISSION_BUSY)


def test_abort_accepted_while_another_mission_runs():
    d = CommandGate().decide(RadioCommand("abort", 5, 0, 2), GCS, Readiness(True, {"motor_test": "testing"}))
    assert (d.forward, d.result) == ("abort", MAV_RESULT_ACCEPTED)
