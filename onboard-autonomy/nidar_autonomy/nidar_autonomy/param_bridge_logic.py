"""Rules for bench parameter access over the radio (GCS Setup page ->
radio -> Jetson -> MAVROS -> Pixhawk) -- pure, no rclpy, unit-tested.
radio_command_node.py does the MAVROS calls.

Bench setup only; the operator panel used in a mission never offers it
(custom-gcs: GCS_SETUP_ENABLED, off by default).

  - Reads are always allowed while the FCU is connected.
  - Writes need allow_param_write (start_jetson.sh --setup), a DISARMED
    vehicle (armed state known), and no mission mid-run.
  - Names must look like ArduPilot parameter names; integer parameters
    only take whole numbers; NaN/inf are refused.
"""

from __future__ import annotations

import math
import re
from typing import Mapping, Optional, Tuple

from .radio_command_logic import STARTABLE_MISSION_STATES
from .telem_command_codec import JETSON_COMPONENT_ID, JETSON_SYSTEM_ID, ParamRequest

_NAME = re.compile(r"^[A-Z0-9_]{1,16}$")


def addressed_to_jetson(req: ParamRequest) -> bool:
    return (req.target_system, req.target_component) == (JETSON_SYSTEM_ID, JETSON_COMPONENT_ID)


def refusal(
    req: ParamRequest,
    writes_enabled: bool,
    fcu_connected: bool,
    armed: Optional[bool],
    missions: Mapping[str, str],
) -> Optional[str]:
    """Why the request must be refused, or None if it may go ahead."""
    if not _NAME.match(req.name):
        return "invalid parameter name"
    if not fcu_connected:
        return "FCU not connected"
    if req.kind == "read":
        return None
    if not writes_enabled:
        return "writes disabled (start the Jetson with start_jetson.sh --setup)"
    if armed is not False:
        return "vehicle armed (or armed state unknown) -- writes refused"
    busy = [f"{m} is {s}" for m, s in missions.items() if s not in STARTABLE_MISSION_STATES]
    if busy:
        return "mission running (" + ", ".join(busy) + ") -- writes refused"
    if req.value is None or not math.isfinite(req.value):
        return "invalid value"
    return None


def typed_value(value: float, is_integer: bool) -> Tuple[Optional[float], Optional[str]]:
    """The value to write for a parameter of this type, or an error."""
    if not is_integer:
        return float(value), None
    if abs(value - round(value)) > 1e-6:
        return None, "integer parameter: whole numbers only"
    return float(int(round(value))), None
