"""The missions the GCS can start -- what the Mission dropdown shows.

Static, in-process data so app/main.py can validate a mission-start
request without trusting anything the frontend sends. Each mission has a
`radio_code`: the number sent in the radio START (COMMAND_LONG param4),
which the Jetson's radio_command_node maps back to the mission it runs
(onboard-autonomy telem_command_codec.MISSION_NAMES). The two tables must
agree -- an unknown code is rejected by the Jetson (UNKNOWN_MISSION),
never mapped to some other mission.

Adding a mission = one entry here plus the matching code and mission node
on the Jetson (telem_command_codec.MISSION_NAMES, missions/<id>/mission.py,
radio_command_node.MISSION_STATUS_TOPICS).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MissionDefinition:
    id: str
    name: str
    description: str
    radio_code: int


MISSION_REGISTRY: tuple[MissionDefinition, ...] = (
    MissionDefinition(
        id="hover",
        name="Hover",
        description=(
            "Real ArduCopter hover: GUIDED, arm, take off to a low altitude, "
            "hold, land. Altitude/duration are set on the Jetson "
            "(missions/hover/mission.py)."
        ),
        radio_code=1,
    ),
    MissionDefinition(
        id="motor_test",
        name="Motor Test",
        description=(
            "PROPS OFF bench check: ArduCopter's motor test spins each motor "
            "in turn (A, B, C, D) at low throttle for a few seconds. No "
            "position estimate needed. Throttle/duration are set on the "
            "Jetson (missions/motor_test/mission.py)."
        ),
        radio_code=2,
    ),
)


class MissionNotFoundError(KeyError):
    """No MissionDefinition in MISSION_REGISTRY has this id."""


def get_mission(mission_id: str) -> MissionDefinition:
    for mission in MISSION_REGISTRY:
        if mission.id == mission_id:
            return mission
    raise MissionNotFoundError(mission_id)
