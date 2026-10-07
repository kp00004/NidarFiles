"""Pure accept/reject logic for commands arriving over the MicroLR900
radio, kept free of rclpy so it's unit-testable -- radio_command_node.py
is the ROS/serial wrapper around it.

Rules:
  - ABORT is always forwarded and ACCEPTED, whatever the vehicle or
    mission state (Hard Safety Rule 2: abort must preempt everything).
  - START is forwarded only if every check passes, otherwise it is
    rejected with a REASON_* code the GCS displays. The requested
    mission's node must be alive, and NO mission may be mid-run (a motor
    test must never start while a hover is flying, and vice versa). Nothing is armed or
    moved for a rejected START -- it never reaches /gcs/command.
  - A resend of an already-handled (sender, nonce) gets the ORIGINAL
    decision again and is never forwarded twice.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, replace
from typing import Mapping, Optional, Tuple

from .telem_command_codec import (
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_DENIED,
    MAV_RESULT_TEMPORARILY_REJECTED,
    MISSION_NAMES,
    PROTOCOL_VERSION,
    REASON_BAD_PROTOCOL_VERSION,
    REASON_FCU_NOT_CONNECTED,
    REASON_MISSION_BUSY,
    REASON_MISSION_NOT_READY,
    REASON_OK,
    REASON_UNKNOWN_MISSION,
    RadioCommand,
)

# Mission states from which a new START may begin. Everything else means
# a run is in progress. (A run that ended in pilot_override is startable
# again; the mission's own preflight still refuses if the vehicle is armed.)
STARTABLE_MISSION_STATES = frozenset({"idle", "complete", "aborted", "failed", "pilot_override"})

_MEMORY = 64


@dataclass(frozen=True)
class Readiness:
    fcu_connected: bool  # fresh /mavros/state with connected=True
    # mission id -> state, for every mission node whose status is fresh
    # (a mission missing here is not running / not alive)
    missions: Mapping[str, str]


@dataclass(frozen=True)
class Decision:
    forward: Optional[str]  # "start" / "abort" to publish on /gcs/command, or None
    mission_id: Optional[str]  # e.g. "hover" when forwarding a START
    result: int  # MAV_RESULT_*
    reason: int  # REASON_*
    duplicate: bool = False


class CommandGate:
    def __init__(self) -> None:
        self._decisions: "OrderedDict[Tuple[int, int, int], Decision]" = OrderedDict()

    def decide(
        self, cmd: RadioCommand, sender: Tuple[int, int], readiness: Readiness
    ) -> Decision:
        key = (sender[0], sender[1], cmd.nonce)
        previous = self._decisions.get(key)
        if previous is not None:
            return replace(previous, forward=None, duplicate=True)

        decision = self._evaluate(cmd, readiness)
        self._decisions[key] = decision
        while len(self._decisions) > _MEMORY:
            self._decisions.popitem(last=False)
        return decision

    @staticmethod
    def _evaluate(cmd: RadioCommand, readiness: Readiness) -> Decision:
        if cmd.command == "abort":
            return Decision("abort", None, MAV_RESULT_ACCEPTED, REASON_OK)

        if cmd.protocol_version != PROTOCOL_VERSION:
            return Decision(None, None, MAV_RESULT_DENIED, REASON_BAD_PROTOCOL_VERSION)
        mission_id = MISSION_NAMES.get(cmd.mission_code)
        if mission_id is None:
            return Decision(None, None, MAV_RESULT_DENIED, REASON_UNKNOWN_MISSION)
        if mission_id not in readiness.missions:
            return Decision(None, mission_id, MAV_RESULT_TEMPORARILY_REJECTED, REASON_MISSION_NOT_READY)
        if not readiness.fcu_connected:
            return Decision(None, mission_id, MAV_RESULT_TEMPORARILY_REJECTED, REASON_FCU_NOT_CONNECTED)
        if any(state not in STARTABLE_MISSION_STATES for state in readiness.missions.values()):
            return Decision(None, mission_id, MAV_RESULT_TEMPORARILY_REJECTED, REASON_MISSION_BUSY)
        return Decision("start", mission_id, MAV_RESULT_ACCEPTED, REASON_OK)
