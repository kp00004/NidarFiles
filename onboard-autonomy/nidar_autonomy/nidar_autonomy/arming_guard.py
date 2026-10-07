"""Pure ARM-precondition logic for Checkpoint 2, deliberately free of any
rclpy/ROS dependency so it's unit-testable without a running ROS context
-- same separation as state_machine.py vs. mission_state_node.py.
"""

from __future__ import annotations

from typing import Optional


class ArmRejected(Exception):
    """Raised when an ARM attempt is refused before ever calling the
    mavros service -- a local safety precondition failed. Distinct from
    the FCU itself rejecting an arm request (a prearm-check failure),
    which is reported instead as ArmingResult.success=False."""


def check_arm_preconditions(current_armed: Optional[bool]) -> None:
    """May an ARM attempt be issued at all, given the last known
    /mavros/state.armed value? Raises ArmRejected if not.

    Deliberately conservative: only proceeds if the vehicle is
    affirmatively known to be disarmed. An unknown state (None -- no
    /mavros/state received yet) is treated the same as "already armed":
    refuse, don't guess.
    """
    if current_armed is None:
        raise ArmRejected(
            "Refusing to arm: no /mavros/state received yet, current "
            "armed state is unknown."
        )
    if current_armed:
        raise ArmRejected("Refusing to arm: vehicle already reports armed.")
