"""Pure decision logic for when a validated GCS command should trigger a
real ARM/DISARM attempt, kept free of rclpy so it's unit-testable -- same
split as state_machine.py vs. mission_state_node.py.

This is Checkpoint 3/4's wiring (see CHECKPOINT/INTEGRATION_CHECKPOINTS.md):
GCS "start" may lead to a real ARM only as a direct, traceable consequence
of the mission state machine actually accepting it (idle -> entering) --
see this repo's CLAUDE.md Hard Safety Rule 1. GCS "abort" always triggers
a DISARM attempt, regardless of current mission state, per Hard Safety
Rule 2 (abort must preempt everything) -- disarm is always safe to
attempt even if the vehicle is already disarmed.
"""

from __future__ import annotations


def should_attempt_arm(command: str, previous_state: str, new_state: str) -> bool:
    """True only when "start" actually moved the state machine idle ->
    entering -- i.e. never on a no-op start (already past idle)."""
    return command == "start" and previous_state == "idle" and new_state == "entering"


def should_attempt_disarm(command: str) -> bool:
    return command == "abort"
