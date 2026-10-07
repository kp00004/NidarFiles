"""The missions the GCS can start -- what the Mission dropdown shows.

Static, in-process data so app/main.py can validate a mission-start
request without trusting anything the frontend sends. Each mission has a
`radio_code`: the number sent in the radio START (COMMAND_LONG param4),
which the Jetson's radio_command_node maps back to the mission it runs
(onboard-autonomy telem_command_codec.MISSION_NAMES). The two tables must
agree -- an unknown code is rejected by the Jetson (UNKNOWN_MISSION),
never mapped to some other mission.

Only Hover exists for now. Adding a mission later = one entry here plus
the matching code and mission node on the Jetson.
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
)


class MissionNotFoundError(KeyError):
    """No MissionDefinition in MISSION_REGISTRY has this id."""


def get_mission(mission_id: str) -> MissionDefinition:
    for mission in MISSION_REGISTRY:
        if mission.id == mission_id:
            return mission
    raise MissionNotFoundError(mission_id)
