"""Pure mission state machine logic, with no ROS/rclpy dependency -- kept
separate from mission_state_node.py so it's testable without a running
ROS context, mirroring the pattern custom-gcs/sim/rosbridge_sim/mission.py
already uses for the same reason.

See mission_state_node.py's module docstring for the current limitation:
against the REAL vehicle, only idle/entering/aborted are reachable right
now, because nothing upstream exists yet to report the
searching->exiting->complete transitions for real (no real exploration
subsystem, no real flight control -- AUTONOMY_ROADMAP.md Phase 8/9).
`handle_exploration_started()`/`handle_exploration_complete()`/
`handle_exit_complete()` exist and are exercised for real by the
simulation harness (`mission_simulator.py`, its own separate
`MissionStateMachine` instance -- never `mission_state_node.py`'s), which
is exactly the kind of caller this class was always meant to support
once one existed.
"""

from __future__ import annotations

from .topics import VALID_COMMANDS


class InvalidCommandError(ValueError):
    pass


class MissionStateMachine:
    def __init__(self) -> None:
        self._state = "idle"

    @property
    def state(self) -> str:
        return self._state

    def handle_command(self, command: str) -> str:
        """Apply a command (must already be validated as "start" or
        "abort" -- see command_node.py). Returns the resulting state.
        Raises InvalidCommandError for anything else, as a defense-in-
        depth check, not the primary validation point."""
        if command not in VALID_COMMANDS:
            raise InvalidCommandError(f"{command!r} is not a valid command")

        if command == "abort":
            self._state = "aborted"
            return self._state

        # command == "start"
        if self._state != "idle":
            # No-op: a mission can only be started from idle. Caller decides
            # whether/how to log this; we just refuse the transition.
            return self._state

        self._state = "entering"
        return self._state

    def handle_fcu_disarmed(self) -> str:
        """Called when /mavros/state reports the FCU has disarmed on its
        own (e.g. ArduCopter's ground-idle auto-disarm after arming with
        no throttle/setpoint stream ever sent) -- NOT as a result of our
        own commanded DISARM, which already moves this state machine via
        handle_command("abort") before the disarm is even requested (see
        mission_state_node.py). Only "entering" implies an arm attempt
        this node itself made and is still relying on being armed, so
        only "entering" transitions here (to "aborted"); every other
        state is a no-op. Without this, an unsolicited FCU disarm would
        leave the reported state stuck on "entering" (i.e. an
        active/armed mission) indefinitely, which is exactly the kind of
        state machine lying about drone reality this module's docstring
        already says not to do.

        Deliberately does NOT go any further than "aborted" -- see
        handle_ground_reset_confirmed() for why an *unsolicited* disarm
        must never, by itself, make the mission restartable again."""
        return self._to_aborted_if_entering()

    def handle_safety_trigger(self) -> str:
        """Called when an independent safety monitor (geofence breach,
        critical battery, C2/GCS-link loss, mission time budget, operator
        e-stop -- Rulebook Section 10 failsafes) fires. Added by the NIDAR
        Autonomy Migration (folding gps_denied/raj-dev's `mission_fsm.py`
        failsafe *triggers* into this state machine -- see
        CHECKPOINT/CURRENT_STATE.md's migration report -- while keeping
        this the ONE and ONLY `/mission/state` publisher; `mission_fsm.py`
        itself, which also independently published `/mission/state`, was
        NOT migrated).

        Same reasoning and same no-op-elsewhere shape as
        handle_fcu_disarmed()/handle_arm_failed(): only "entering" implies
        an active mission relying on the vehicle staying armed, so only
        "entering" transitions here (to "aborted"). Deliberately does NOT
        go any further than "aborted" -- same as those two methods -- so a
        safety trigger can never, by itself, make the mission look
        restartable again; only handle_ground_reset_confirmed() (a
        confirmed DISARM this repo itself commanded) can do that.

        SCOPE NOTE (mirrors handle_ground_reset_confirmed()'s own): today
        this can only ever mean "force a ground disarm" -- there is no
        real flight/setpoint capability yet (AUTONOMY_ROADMAP.md Phase 8
        is blocked pending human sign-off), so "abort" is the only safe
        response available. gps_denied's mission_fsm.py distinguished a
        graceful RECALL (fly home first) from an immediate LAND_NOW --
        that distinction requires real flight control and does not exist
        here; every trigger collapses to the same ground-abort response
        until Checkpoint 5+ and a real in-flight-abort design (Checkpoint
        7) exist. Revisit this method when they do."""
        return self._to_aborted_if_entering()

    def handle_arm_failed(self) -> str:
        """Called when an ARM attempt this node itself made did not
        cleanly succeed -- refused before ever reaching the FCU
        (arming_guard.ArmRejected), rejected by the FCU, or accepted but
        not confirmed via /mavros/state (see flight_command.ArmingResult).
        Same reasoning as handle_fcu_disarmed(): "entering" must not keep
        claiming an active/armed mission when the vehicle never actually
        armed, so this moves "entering" -> "aborted" and is a no-op
        everywhere else. mission_state_node.py always follows this with a
        forced DISARM attempt, so the vehicle ends up in a confirmed,
        known-safe (disarmed) state either way."""
        return self._to_aborted_if_entering()

    def _to_aborted_if_entering(self) -> str:
        if self._state == "entering":
            self._state = "aborted"
        return self._state

    def handle_exploration_started(self) -> str:
        """Called when the exploration subsystem reports it has actually
        taken over (e.g. the vehicle is airborne/entered the arena and
        has begun searching). Added for the NIDAR simulation harness
        (`mission_simulator.py`) -- this is the "extend this once the
        exploration subsystem exists" promise this module's own header
        docstring already made; a real exploration subsystem can call
        this exactly the same way once it exists.

        Only "entering" transitions here (to "searching") -- a mission
        that was never started, or that already left "entering" for any
        reason (aborted, or already searching), is a no-op. This mirrors
        `handle_fcu_disarmed()`/`handle_arm_failed()`'s "only the state
        that implies this transition makes sense reacts, everything else
        is inert" shape."""
        if self._state == "entering":
            self._state = "searching"
        return self._state

    def handle_exploration_complete(self) -> str:
        """Called when the exploration subsystem reports no reachable
        unexplored region remains (e.g. `frontier_detector.detect_frontiers`
        returns nothing new to chase) and the mission should head for the
        exit. Only "searching" transitions here (to "exiting") -- no-op
        everywhere else, same defensive shape as every other transition in
        this class."""
        if self._state == "searching":
            self._state = "exiting"
        return self._state

    def handle_exit_complete(self) -> str:
        """Called when the vehicle has actually reached the exit/entry
        point and the mission is finished. Only "exiting" transitions
        here (to "complete") -- no-op everywhere else. Deliberately a
        distinct terminal state from "aborted": "complete" means the
        mission finished on its own terms, "aborted" means it didn't."""
        if self._state == "exiting":
            self._state = "complete"
        return self._state

    def handle_ground_reset_confirmed(self) -> str:
        """Called when a DISARM *this node itself commanded* (as a direct
        consequence of "abort", or of handle_arm_failed()'s forced
        disarm) is confirmed via the real /mavros/state.armed value --
        never from a timer, a guess, or the unsolicited-disarm path
        (handle_fcu_disarmed() never calls this). This is the ONLY way
        "aborted" ever becomes "idle" again -- a bare "start" command
        can't do it (see test_start_while_aborted_is_a_noop), and an
        unconfirmed/failed commanded disarm leaves the state at "aborted"
        indefinitely, forcing a human/physical intervention rather than
        guessing the vehicle is safe. No-op in every state but "aborted",
        so a stray or duplicate disarm confirmation while already
        idle/entering can never fabricate a transition.

        Scope note: today, "aborted" is only ever reached while the
        vehicle is disarmed-or-being-disarmed on the bench (no takeoff/
        setpoint code exists yet -- see this repo's CLAUDE.md "Current
        Project Phase"), so a confirmed disarm here is also, in practice,
        a confirmed *ground* state. Once real flight exists (Phase 8 /
        Checkpoint 7+), this assumption must be revisited: an in-flight
        abort must not simply mean "immediately disarm" (see
        flight_command_interface.FlightCommandInterface.abort()'s
        docstring), and this reset logic will need a real
        airborne/grounded distinction, not just "is it disarmed"."""
        if self._state == "aborted":
            self._state = "idle"
        return self._state
