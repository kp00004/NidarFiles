"""Unit tests for motor_test_logic.py -- pure decision logic, no ROS, no
vehicle. They prove the decisions, not the motors: real behaviour must
still be verified on the bench, PROPS OFF (see README.md at the NidarFiles root).

Run from the NidarFiles root:  python -m pytest missions/motor_test -q
"""
from motor_test_logic import (
    ABORTED, COMPLETE, FAILED, IDLE, TESTING, Action, MotorTestConfig, MotorTestMission,
)
from nidar_autonomy.vehicle_snapshot import VehicleSnapshot

CFG = MotorTestConfig(motor_count=4, throttle_pct=8.0, per_motor_s=5.0, pause_s=1.0)
STEP = CFG.per_motor_s + CFG.pause_s


def snap(armed=False, connected=True, state_age=0.2):
    return VehicleSnapshot(connected, state_age, armed, "STABILIZE", None, None, None, None)


def started(t=0.0):
    m = MotorTestMission(CFG)
    actions = m.start(t, snap())
    return m, actions


def test_idle_until_start():
    m = MotorTestMission(CFG)
    assert m.state == IDLE
    assert m.tick(10.0, snap()) == []


def test_start_spins_motor_1_with_configured_throttle_and_timeout():
    m, actions = started()
    assert m.state == TESTING
    assert actions == [Action("motor_test", 1, 8.0, 5.0)]
    assert "motor A (1/4)" in m.detail


def test_runs_all_motors_in_order_then_completes():
    m, _ = started()
    commanded = [1]
    t = 0.0
    m.on_result(t, "motor_test", 1, True, "ok")
    for _ in range(3):
        t += STEP
        actions = m.tick(t, snap(armed=True))  # FCU reports armed while a motor spins
        assert len(actions) == 1 and actions[0].kind == "motor_test"
        commanded.append(actions[0].motor)
        m.on_result(t, "motor_test", actions[0].motor, True, "ok")
    assert commanded == [1, 2, 3, 4]
    assert m.tick(t + STEP, snap()) == []
    assert m.state == COMPLETE


def test_no_next_motor_before_time_and_pause():
    m, _ = started()
    m.on_result(0.0, "motor_test", 1, True, "ok")
    assert m.tick(CFG.per_motor_s, snap()) == []  # FCU timeout reached, pause not over
    assert m.motor == 1


def test_preflight_refuses_when_armed_or_unknown_or_disconnected():
    for s in (snap(armed=True), snap(armed=None), snap(connected=False), snap(state_age=10.0)):
        m = MotorTestMission(CFG)
        assert m.start(0.0, s) == []
        assert m.state == FAILED and "preflight failed" in m.detail


def test_abort_stops_the_running_motor_immediately():
    m, _ = started()
    actions = m.abort(2.0, snap(armed=True))
    assert actions == [Action("motor_stop", 1, 0.0, 0.0)]
    assert m.state == ABORTED
    assert m.tick(100.0, snap()) == []  # no further motors


def test_abort_when_idle_does_nothing():
    m = MotorTestMission(CFG)
    assert m.abort(0.0, snap()) == []
    assert m.state == IDLE


def test_rejected_motor_command_fails_and_sends_stop():
    m, _ = started()
    actions = m.on_result(0.1, "motor_test", 1, False, "success=True result=4")
    assert m.state == FAILED and "REJECTED" in m.detail
    assert actions == [Action("motor_stop", 1, 0.0, 0.0)]


def test_unacknowledged_motor_command_fails():
    m, _ = started()
    actions = m.tick(CFG.ack_timeout_s + 0.1, snap())
    assert m.state == FAILED and "not acknowledged" in m.detail
    assert actions[0].kind == "motor_stop"


def test_link_loss_fails_and_stops():
    m, _ = started()
    m.on_result(0.0, "motor_test", 1, True, "ok")
    actions = m.tick(1.0, snap(connected=False))
    assert m.state == FAILED and "link lost" in m.detail
    assert actions == [Action("motor_stop", 1, 0.0, 0.0)]


def test_late_result_for_an_earlier_motor_is_ignored():
    m, _ = started()
    m.on_result(0.0, "motor_test", 1, True, "ok")
    m.tick(STEP, snap())
    assert m.on_result(STEP + 0.1, "motor_test", 1, False, "late") == []
    assert m.state == TESTING and m.motor == 2


def test_failed_stop_is_reported():
    m, _ = started()
    m.abort(1.0, snap())
    m.on_result(1.1, "motor_stop", 1, False, "service call failed")
    assert "STOP NOT CONFIRMED" in m.detail


def test_start_while_testing_is_ignored():
    m, _ = started()
    assert m.start(1.0, snap()) == []
    assert m.state == TESTING and "ignored" in m.detail


def test_can_run_again_after_completion():
    m, _ = started()
    m.abort(1.0, snap())
    actions = m.start(2.0, snap())
    assert m.state == TESTING and actions[0].motor == 1


def test_status_shape():
    m, _ = started()
    m.on_result(0.0, "motor_test", 1, True, "ok")
    status = m.status(2.0)
    assert status["scenario"] == "motor_test" and status["state"] == TESTING
    assert status["current_motor"] == 1 and status["motor_count"] == 4
    assert status["elapsed_motor_s"] == 2.0 and status["execution_mode"] == "real"


def test_internal_error_stops_the_running_motor():
    m, _ = started()
    actions = m.internal_error(1.0, "internal error in tick: ValueError()")
    assert actions == [Action("motor_stop", 1, 0.0, 0.0)]
    assert m.state == FAILED
    assert m.internal_error(1.1, "again") == []  # repeated: nothing new
