"""Telemetry from the Jetson over the MicroLR900 radio -- the GCS's only
view of the drone (there is no Wi-Fi link).

RadioLink (app/radio_link.py) owns the serial port and hands every MAVLink
message from the Jetson to on_message(). This class turns them back into
the same ROS-shaped dicts RosBridgeClient caches, so every route in
app/main.py reads them through the same latest()/age_s()/
statustext_history() interface and can't tell the two apart.

What arrives (wire format: app/radio_protocol.py and onboard-autonomy's
telem_command_codec.py / radio_telemetry.py):

    Jetson 1/191  HEARTBEAT            -> link up, mission state, /gcs/heartbeat age
                  STATUSTEXT           -> hover mission `detail`
                  NAMED_VALUE_FLOAT    -> hover progress values
    relay  1/1    HEARTBEAT            -> /mavros/state
                  SYS_STATUS           -> /mavros/battery
                  LOCAL_POSITION_NED   -> /mavros/local_position/pose (+ velocity_local)
                  ATTITUDE_QUATERNION  -> /mavros/imu/data orientation
                  STATUSTEXT           -> /mavros/statustext/recv history

Nothing is reported that didn't just arrive: each value is dropped once
it's older than its stale limit, and the FCU counts as connected only
while its relayed HEARTBEAT keeps arriving. This client can't publish
anything -- START/ABORT go through RadioLink.send_command().
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Optional

from .radio_protocol import (
    ARDUCOPTER_MODE_NAMES,
    FCU_RELAY_COMPONENT_ID,
    HOVER_VALUE_KEYS,
    JETSON_COMPONENT_ID,
    JETSON_SYSTEM_ID,
    MAV_MODE_FLAG_GUIDED_ENABLED,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MISSION_STATE_NAMES,
    STATUSTEXT_CHUNK_LEN,
)
from .ros_client import (
    BATTERY_TOPIC,
    FCU_STATE_TOPIC,
    FLIGHT_TEST_STATUS_TOPIC,
    HEARTBEAT_TOPIC,
    IMU_TOPIC,
    POSE_TOPIC,
    VALID_COMMANDS,
    VALID_SIMULATION_COMMANDS,
    VELOCITY_TOPIC,
)

JETSON_LINK_TIMEOUT_S = 3.0
FCU_STALE_S = 3.0
BATTERY_STALE_S = 5.0
POSE_STALE_S = 3.0
ATTITUDE_STALE_S = 3.0
HOVER_VALUES_STALE_S = 3.0
# The Jetson resends the mission detail every 5 s while there is one.
MISSION_DETAIL_STALE_S = 12.0
STATUSTEXT_HISTORY = 10
_PARTIAL_TEXT_TIMEOUT_S = 5.0

_NO_PUBLISH = "telemetry comes over the command radio -- there is no ROS link to publish on"

def _rounded(values: dict, digits: int) -> dict:
    """Values arrive as float32 -- drop the noise digits (0.49000000953...)."""
    return {k: round(v, digits) for k, v in values.items()}


_JETSON = (JETSON_SYSTEM_ID, JETSON_COMPONENT_ID)
_FCU = (JETSON_SYSTEM_ID, FCU_RELAY_COMPONENT_ID)


class RadioTelemetryClient:
    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._values: dict[str, Any] = {}
        self._at: dict[str, float] = {}
        self._hover: dict[str, tuple[float, float]] = {}  # field -> (value, received at)
        self._statustext_history: list[dict] = []
        self._partial_texts: dict[tuple, dict] = {}

    # -- input (RadioLink reader thread) -------------------------------------------

    def on_message(self, msg) -> None:
        source = (msg.get_srcSystem(), msg.get_srcComponent())
        kind = msg.get_type()
        now = self._clock()
        with self._lock:
            if source == _JETSON:
                if kind == "HEARTBEAT":
                    self._store("jetson", MISSION_STATE_NAMES.get(msg.custom_mode, "unknown"), now)
                elif kind == "NAMED_VALUE_FLOAT" and msg.name in HOVER_VALUE_KEYS:
                    self._hover[HOVER_VALUE_KEYS[msg.name]] = (round(msg.value, 2), now)
                elif kind == "STATUSTEXT":
                    text = self._reassemble(source, msg, now)
                    if text is not None:
                        self._store("mission_detail", text, now)
            elif source == _FCU:
                if kind == "HEARTBEAT":
                    self._store("fcu", {
                        "armed": bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
                        "guided": bool(msg.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED),
                        "mode": ARDUCOPTER_MODE_NAMES.get(msg.custom_mode, "UNKNOWN"),
                        "system_status": msg.system_status,
                    }, now)
                elif kind == "SYS_STATUS":
                    self._store("battery", {
                        "voltage": None if msg.voltage_battery == 0xFFFF else msg.voltage_battery / 1000.0,
                        "current": None if msg.current_battery == -1 else msg.current_battery / 100.0,
                        "percentage": None if msg.battery_remaining < 0 else msg.battery_remaining / 100.0,
                    }, now)
                elif kind == "LOCAL_POSITION_NED":
                    self._store("position", _rounded({"x": msg.x, "y": msg.y, "z": msg.z}, 3), now)
                    velocity = (msg.vx, msg.vy, msg.vz)
                    if all(math.isfinite(v) for v in velocity):
                        self._store("velocity", _rounded(dict(zip("xyz", velocity)), 3), now)
                elif kind == "ATTITUDE_QUATERNION":
                    orientation = {"x": msg.q2, "y": msg.q3, "z": msg.q4, "w": msg.q1}
                    self._store("orientation", _rounded(orientation, 4), now)
                elif kind == "STATUSTEXT":
                    text = self._reassemble(source, msg, now)
                    if text is not None:
                        self._statustext_history.append({"severity": msg.severity, "text": text})
                        del self._statustext_history[:-STATUSTEXT_HISTORY]

    def _store(self, key: str, value: Any, now: float) -> None:
        self._values[key] = value
        self._at[key] = now

    def _reassemble(self, source: tuple, msg, now: float) -> Optional[str]:
        """MAVLink2 STATUSTEXT chunking: id 0 is a whole text; otherwise
        chunks share an id and the one shorter than 50 chars is the last.
        Returns the full text once complete, or None."""
        text_id = getattr(msg, "id", 0) or 0
        if text_id == 0:
            return msg.text
        for key in [k for k, p in self._partial_texts.items() if now - p["at"] > _PARTIAL_TEXT_TIMEOUT_S]:
            del self._partial_texts[key]
        key = (source, text_id)
        partial = self._partial_texts.setdefault(key, {"at": now, "chunks": {}})
        partial["chunks"][msg.chunk_seq] = msg.text
        if len(msg.text) >= STATUSTEXT_CHUNK_LEN:
            return None
        del self._partial_texts[key]
        chunks = partial["chunks"]
        if sorted(chunks) != list(range(len(chunks))):
            return None  # a chunk was lost
        return "".join(chunks[i] for i in range(len(chunks)))

    # -- ros_client interface ----------------------------------------------------

    def _fresh(self, key: str, limit: float) -> Any:
        at = self._at.get(key)
        if at is None or self._clock() - at >= limit:
            return None
        return self._values.get(key)

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._fresh("jetson", JETSON_LINK_TIMEOUT_S) is not None

    def connect(self) -> None:
        """Nothing to connect: RadioLink owns the serial port."""

    def disconnect(self) -> None:
        """Symmetric with connect()."""

    def latest(self, topic: str) -> Any:
        with self._lock:
            if topic == FCU_STATE_TOPIC:
                return self._fcu_state()
            if topic == BATTERY_TOPIC:
                return self._fresh("battery", BATTERY_STALE_S)
            if topic == POSE_TOPIC:
                position = self._fresh("position", POSE_STALE_S)
                return None if position is None else {"pose": {"position": position}}
            if topic == VELOCITY_TOPIC:
                velocity = self._fresh("velocity", POSE_STALE_S)
                return None if velocity is None else {"twist": {"linear": velocity}}
            if topic == IMU_TOPIC:
                orientation = self._fresh("orientation", ATTITUDE_STALE_S)
                return None if orientation is None else {"orientation": orientation}
            if topic == FLIGHT_TEST_STATUS_TOPIC:
                return self._hover_status()
            return None

    def _fcu_state(self) -> Optional[dict]:
        if "jetson" not in self._at and "fcu" not in self._at:
            return None  # never heard from the Jetson: no data, not "disconnected"
        fcu = self._fresh("fcu", FCU_STALE_S)
        if fcu is None:
            return {"connected": False, "armed": None, "guided": None, "mode": None, "system_status": None}
        return {"connected": True, **fcu}

    def _hover_status(self) -> Optional[dict]:
        state = self._fresh("jetson", JETSON_LINK_TIMEOUT_S)
        if state is None or state == "unknown":
            return None
        now = self._clock()
        values = {k: v for k, (v, at) in self._hover.items() if now - at < HOVER_VALUES_STALE_S}
        fcu = self._fresh("fcu", FCU_STALE_S) or {}
        position = self._fresh("position", POSE_STALE_S)
        return {
            "scenario": "hover",
            "state": state,
            "detail": self._fresh("mission_detail", MISSION_DETAIL_STALE_S),
            "target_altitude_m": values.get("target_altitude_m"),
            "current_altitude_m": values.get("current_altitude_m"),
            "current_position": None if position is None else [round(position[k], 2) for k in "xyz"],
            "duration_s": values.get("duration_s"),
            "elapsed_hover_s": values.get("elapsed_hover_s"),
            "armed": fcu.get("armed"),
            "flight_mode": fcu.get("mode"),
            "execution_mode": "real",
        }

    def age_s(self, topic: str) -> float | None:
        key = {
            HEARTBEAT_TOPIC: "jetson",
            FCU_STATE_TOPIC: "fcu",
            BATTERY_TOPIC: "battery",
            POSE_TOPIC: "position",
            VELOCITY_TOPIC: "velocity",
            IMU_TOPIC: "orientation",
        }.get(topic)
        with self._lock:
            at = None if key is None else self._at.get(key)
            return None if at is None else self._clock() - at

    def survivors(self) -> list[dict]:
        return []

    def statustext_history(self) -> list[dict]:
        with self._lock:
            return list(self._statustext_history)

    def publish_command(self, command: str) -> None:
        if command not in VALID_COMMANDS:
            raise ValueError(f"invalid command: {command!r}; must be one of {VALID_COMMANDS}")
        raise RuntimeError(_NO_PUBLISH)

    def publish_simulation_command(self, command: str) -> None:
        if command not in VALID_SIMULATION_COMMANDS:
            raise ValueError(
                f"invalid simulation command: {command!r}; must be one of {VALID_SIMULATION_COMMANDS}"
            )
        raise RuntimeError(_NO_PUBLISH)

    def publish_mission_select(self, mission_id: str, scenario_id: str) -> None:
        if not mission_id or not scenario_id:
            raise ValueError("mission_id and scenario_id must be non-empty")
        raise RuntimeError(_NO_PUBLISH)
