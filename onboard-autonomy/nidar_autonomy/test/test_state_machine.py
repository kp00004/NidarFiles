import pytest

from nidar_autonomy.state_machine import InvalidCommandError, MissionStateMachine


def test_starts_idle():
    assert MissionStateMachine().state == "idle"


def test_start_from_idle_goes_to_entering():
    m = MissionStateMachine()
    assert m.handle_command("start") == "entering"
    assert m.state == "entering"


def test_start_when_not_idle_is_a_noop():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_command("start") == "entering"  # unchanged, not re-triggered


def test_abort_from_idle_goes_to_aborted():
    m = MissionStateMachine()
    assert m.handle_command("abort") == "aborted"


def test_abort_from_entering_goes_to_aborted():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_command("abort") == "aborted"


def test_abort_after_abort_stays_aborted():
    m = MissionStateMachine()
    m.handle_command("abort")
    assert m.handle_command("abort") == "aborted"


def test_start_while_aborted_is_a_noop():
    """A bare 'start' can never leave 'aborted' by itself -- the only way
    out is a confirmed ground reset (handle_ground_reset_confirmed), which
    is driven by real FCU telemetry via mission_state_node.py, never by
    the /gcs/command channel directly. This is deliberate: re-arming a
    live vehicle should never be a side effect of a stray command."""
    m = MissionStateMachine()
    m.handle_command("abort")
    assert m.handle_command("start") == "aborted"


def test_repeated_start_while_aborted_stays_aborted():
    m = MissionStateMachine()
    m.handle_command("abort")
    m.handle_command("start")
    assert m.handle_command("start") == "aborted"


@pytest.mark.parametrize("bad_command", ["", "STOP", "waypoint", "abort ", None])
def test_invalid_command_raises(bad_command):
    m = MissionStateMachine()
    with pytest.raises(InvalidCommandError):
        m.handle_command(bad_command)


def test_fcu_disarm_while_entering_goes_to_aborted():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_fcu_disarmed() == "aborted"
    assert m.state == "aborted"


def test_fcu_disarm_while_idle_is_a_noop():
    m = MissionStateMachine()
    assert m.handle_fcu_disarmed() == "idle"


def test_fcu_disarm_while_already_aborted_is_a_noop():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    assert m.handle_fcu_disarmed() == "aborted"


def test_arm_failed_while_entering_goes_to_aborted():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_arm_failed() == "aborted"
    assert m.state == "aborted"


def test_arm_failed_while_idle_is_a_noop():
    m = MissionStateMachine()
    assert m.handle_arm_failed() == "idle"


def test_arm_failed_while_already_aborted_is_a_noop():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    assert m.handle_arm_failed() == "aborted"


def test_safety_trigger_while_entering_goes_to_aborted():
    """Migrated from gps_denied/raj-dev's mission_fsm.py -- a failsafe
    trigger (geofence breach, critical battery, C2-link loss, time budget,
    operator e-stop) must abort an active mission, same as an unsolicited
    FCU disarm or a failed arm attempt."""
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_safety_trigger() == "aborted"
    assert m.state == "aborted"


def test_safety_trigger_while_idle_is_a_noop():
    m = MissionStateMachine()
    assert m.handle_safety_trigger() == "idle"


def test_safety_trigger_while_already_aborted_is_a_noop():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    assert m.handle_safety_trigger() == "aborted"


def test_safety_trigger_never_reaches_idle_by_itself():
    """Same guarantee as handle_fcu_disarmed()/handle_arm_failed(): a
    safety trigger alone can never make the mission look restartable
    again -- only a confirmed commanded ground reset can."""
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_safety_trigger()
    assert m.state == "aborted"
    assert m.handle_safety_trigger() == "aborted"  # repeated trigger stays aborted


def test_ground_reset_confirmed_after_abort_goes_to_idle():
    """The whole point of this task: a completed ground abort must be
    able to make the mission restartable again without restarting the
    node."""
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    assert m.handle_ground_reset_confirmed() == "idle"
    assert m.state == "idle"


def test_start_works_again_after_ground_reset():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    m.handle_ground_reset_confirmed()
    assert m.handle_command("start") == "entering"


def test_ground_reset_confirmed_while_idle_is_a_noop():
    m = MissionStateMachine()
    assert m.handle_ground_reset_confirmed() == "idle"


def test_ground_reset_confirmed_while_entering_is_a_noop():
    """A disarm confirmation must never fabricate a transition out of a
    state other than 'aborted' -- in particular it must never appear to
    "complete" an in-progress arm."""
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_ground_reset_confirmed() == "entering"


def test_unsolicited_fcu_disarm_does_not_reach_idle_by_itself():
    """An unsolicited FCU disarm (e.g. ground-idle auto-disarm) must only
    ever reach 'aborted', never 'idle' -- only a DISARM this node itself
    commanded and then confirmed (handle_ground_reset_confirmed) may do
    that. Reaching 'idle' from here would require a separate, explicit
    ground-reset confirmation, not a side effect of the FCU disarming
    itself."""
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_fcu_disarmed() == "aborted"
    assert m.state == "aborted"


def test_full_lifecycle_start_abort_reset_restart():
    m = MissionStateMachine()
    assert m.state == "idle"
    assert m.handle_command("start") == "entering"
    assert m.handle_command("abort") == "aborted"
    assert m.handle_ground_reset_confirmed() == "idle"
    assert m.handle_command("start") == "entering"


# -- searching/exiting/complete transitions (added for the simulation
# harness -- see mission_simulator.py and state_machine.py's module
# docstring) ------------------------------------------------------------


def test_exploration_started_while_entering_goes_to_searching():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_exploration_started() == "searching"
    assert m.state == "searching"


def test_exploration_started_while_idle_is_a_noop():
    m = MissionStateMachine()
    assert m.handle_exploration_started() == "idle"


def test_exploration_started_while_already_searching_is_a_noop():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_exploration_started()
    assert m.handle_exploration_started() == "searching"


def test_exploration_complete_while_searching_goes_to_exiting():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_exploration_started()
    assert m.handle_exploration_complete() == "exiting"
    assert m.state == "exiting"


def test_exploration_complete_while_entering_is_a_noop():
    """Must not skip 'searching' -- exploration can't be "complete" before
    it started."""
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_exploration_complete() == "entering"


def test_exit_complete_while_exiting_goes_to_complete():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_exploration_started()
    m.handle_exploration_complete()
    assert m.handle_exit_complete() == "complete"
    assert m.state == "complete"


def test_exit_complete_while_searching_is_a_noop():
    """Must not skip 'exiting' -- can't finish exiting before exploration
    handed off to it."""
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_exploration_started()
    assert m.handle_exit_complete() == "searching"


def test_abort_preempts_searching():
    """Hard Safety Rule 2 (abort must preempt everything) still holds once
    searching/exiting/complete are reachable -- handle_command("abort") is
    unconditional regardless of which of these new states is active."""
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_exploration_started()
    assert m.handle_command("abort") == "aborted"


def test_abort_preempts_exiting():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_exploration_started()
    m.handle_exploration_complete()
    assert m.handle_command("abort") == "aborted"


def test_full_lifecycle_start_search_exit_complete():
    m = MissionStateMachine()
    assert m.state == "idle"
    assert m.handle_command("start") == "entering"
    assert m.handle_exploration_started() == "searching"
    assert m.handle_exploration_complete() == "exiting"
    assert m.handle_exit_complete() == "complete"
    # "complete" is terminal via this path -- no bare command reopens it;
    # a fresh mission still requires a real ground-reset-equivalent event,
    # mirroring "aborted"'s own restart discipline. Today, nothing calls
    # handle_ground_reset_confirmed() from "complete" (only from
    # "aborted") -- a repeated "start" is a no-op, same as it already is
    # from any non-idle state.
    assert m.handle_command("start") == "complete"
