"""The normalized GCS telemetry contract -- pure logic, no rclpy.

Migrated from gps_denied/raj-dev's scripts/lib/telemetry.py (NIDAR Autonomy
Migration, see CHECKPOINT/CURRENT_STATE.md and
CHECKPOINT/docs/gcs_telemetry_contract.md, ported from
gps_denied/docs/gcs_telemetry_contract.md). The mission-state vocabulary
below is adapted from that source: gps_denied's own `mission_fsm.py`
(WAIT_FIRST_POSE/EXPLORE/RECALL/LAND_NOW) is explicitly NOT migrated -- see
the migration report -- so this module maps onboard-autonomy's own
`topics.MISSION_STATES` (idle/entering/searching/exiting/complete/aborted,
owned solely by state_machine.MissionStateMachine) instead.

The custom GCS should not need to know Cartographer/Nav2 topic names,
OccupancyGrid semantics, or this repo's mission_state_node internals. A
future telemetry_bridge_node.py (the rclpy node) collects raw ROS state and
calls build_contract() here to turn it into one stable, versioned JSON shape
that custom-gcs's FastAPI/rosbridge backend can forward to the browser
unchanged.

Keeping the shape-building logic here (instead of inline in a node) means it
can be unit tested without a ROS install, and it is the one place to edit
when a field is added, instead of hunting through publisher callbacks.

This module intentionally does NOT touch the heavy payloads (the occupancy
grid's cell data, the full planned path). Those already have well-formed ROS
message types (nav_msgs/OccupancyGrid, nav_msgs/Path) that rosbridge
forwards natively -- duplicating them into this JSON blob on every tick
would be the exact "blindly serialize a huge OccupancyGrid into JSON every
render cycle" mistake this contract is designed to avoid. Only lightweight
*summaries* (resolution/width/height/origin, percentages, counts) belong
here.
"""
from __future__ import annotations

import time
from typing import Any, Optional

from .topics import MISSION_STATES

SCHEMA_VERSION = 1

# mission_state_node's /mission/state value -> (objective, next_action) shown
# on the GCS "Active Thinking" panel. This is STRUCTURED operator telemetry,
# not free-form reasoning -- no chain-of-thought, just a fixed vocabulary of
# operational states (see the workspace mission brief's "Active Thinking"
# requirement).
_MISSION_OBJECTIVES: dict[str, tuple[str, str]] = {
    "idle": ("Awaiting mission start", "Wait for GCS start"),
    "entering": ("Arm and enter the arena", "Hold for arming confirmation"),
    "searching": ("Explore unexplored region", "Navigate to frontier"),
    "exiting": ("Return to entry/exit point", "Navigate to entry"),
    "complete": ("Mission complete", "Land and disarm"),
    "aborted": ("Mission aborted", "Hold -- awaiting ground reset"),
}
assert set(_MISSION_OBJECTIVES) == set(MISSION_STATES)

# A future frontier-exploration node's own status 'state' field -> the label
# shown to the operator. Takes precedence over the raw mission_state because
# it is more specific (e.g. distinguishes "exploring" from "returning" while
# mission_state is still just "searching" for both, pending a real
# exploration-state-machine integration -- see AUTONOMY_ROADMAP.md Phase 9).
_EXPLORER_STATE_LABEL: dict[str, str] = {
    "waiting": "WAITING_FOR_MAP",
    "exploring": "SEARCHING_FRONTIER",
    "returning": "RETURNING_TO_ENTRY",
    "done": "MISSION_COMPLETE",
}


def autonomy_state(
    mission_state: Optional[str],
    explorer_state: Optional[str],
    target: Optional[list[float]],
) -> dict[str, Any]:
    """Build the {state, objective, target, next_action} block.

    mission_state: raw string from /mission/state
        (state_machine.MissionStateMachine), or None.
    explorer_state: raw string from an exploration node's own status, or
        None if that node isn't up yet.
    target: [x, y] the explorer is currently driving toward, or None.
    """
    label = _EXPLORER_STATE_LABEL.get(explorer_state, (mission_state or "UNKNOWN").upper())
    objective, next_action = _MISSION_OBJECTIVES.get(mission_state, ("Unknown", "Unknown"))
    if explorer_state == "returning":
        objective, next_action = _MISSION_OBJECTIVES["exiting"]
    elif explorer_state == "done":
        objective, next_action = _MISSION_OBJECTIVES["complete"]
    return {
        "state": label,
        "objective": objective,
        "target": target,
        "next_action": next_action,
    }


def build_contract(
    *,
    connected: bool,
    heartbeat_age_sec: Optional[float],
    armed: bool,
    mode: Optional[str],
    system_status: Optional[int],
    battery_pct: Optional[float],
    position: dict[str, Any],
    sensors: dict[str, Any],
    mapping: dict[str, Any],
    navigation: dict[str, Any],
    autonomy: dict[str, Any],
    mission: dict[str, Any],
    survivors: list[Any],
) -> dict[str, Any]:
    """Assemble the full telemetry contract. Every argument is a plain dict
    or primitive already extracted from ROS messages by the caller -- this
    function does no ROS-message handling itself, only shaping the dict a
    frontend will json-parse.

    'autonomy' is the structured Active-Thinking block (state/objective/
    target/next_action from autonomy_state()); 'mission' is the raw mission
    progress block (state/elapsed/complete) -- kept separate because they
    answer different operator questions ("what is it doing right now" vs
    "how far through the mission are we").
    """
    return {
        "schema_version": SCHEMA_VERSION,
        "stamp": time.time(),
        "connection": {
            "connected": connected,
            "heartbeat_age_sec": heartbeat_age_sec,
        },
        "flight": {
            "armed": armed,
            "mode": mode,
            "system_status": system_status,
            "battery_pct": battery_pct,
        },
        "position": position,
        "sensors": sensors,
        "mapping": mapping,
        "navigation": navigation,
        "autonomy": autonomy,
        "mission": mission,
        # Extension point: no survivor-detection subsystem exists yet (see
        # AUTONOMY_ROADMAP.md Phase 10). Always a list (possibly empty),
        # never fabricated entries.
        "survivors": survivors,
    }


def stale(last_seen_wall_time: Optional[float], now_wall_time: float, timeout_sec: float) -> bool:
    """True if last_seen_wall_time is None or older than timeout_sec. Small
    helper so every "is this topic still alive" check in telemetry_bridge_node.py
    uses the same rule."""
    if last_seen_wall_time is None:
        return True
    return (now_wall_time - last_seen_wall_time) > timeout_sec
