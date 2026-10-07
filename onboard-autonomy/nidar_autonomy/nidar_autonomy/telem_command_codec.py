"""Pure MAVLink2 encode/decode for the MicroLR900 command radio, kept free
of rclpy (and of pymavlink, which isn't installed on the Jetson) so it's
unit-testable -- same split as state_machine.py vs. mission_state_node.py.

The link (the radio terminates at the Jetson, NOT at the Pixhawk):

    GCS laptop --USB--> MicroLR900 ))) MicroLR900 --USB serial--> Jetson
    radio_command_node.py (this module's only runtime user)

It is a private two-node MAVLink network: the GCS is 255/190 and the
Jetson is 1/191 (MAV_COMP_ID_ONBOARD_COMPUTER). The Pixhawk is not on it.

Request: COMMAND_LONG(MAV_CMD_USER_1) addressed to the Jetson:

    param1  NIDAR_START (1.0) or NIDAR_ABORT (2.0)
    param2  nonce -- the GCS resends the same nonce until it gets an ACK;
            the Jetson acts on each nonce once and re-ACKs resends
    param3  NIDAR_MAGIC -- rejects stray MAV_CMD_USER_1 traffic
    param4  mission code (START only): MISSION_HOVER = 1
    param7  PROTOCOL_VERSION (2)

Reply: COMMAND_ACK(MAV_CMD_USER_1). `result` is MAV_RESULT_ACCEPTED or a
rejection, `progress` carries a REASON_* code saying why, and
`result_param2` echoes the request's nonce so the GCS can match each ACK
to its command (an ABORT may be sent while a START is still waiting).

The Jetson also sends a HEARTBEAT at 1 Hz whose custom_mode is the
mission state code (MISSION_STATE_CODES), so the GCS can show radio link
health and mission state.

Telemetry (there is no Wi-Fi link; the radio is the only Jetson <-> GCS
path). Sent by the Jetson, see radio_telemetry.py:

  from 1/191 (the Jetson itself)
    HEARTBEAT            mission state, as above
    STATUSTEXT           the hover mission's `detail` line, on change
    NAMED_VALUE_FLOAT    hover progress: "hv_alt", "hv_tgt", "hv_dur", "hv_elap"
  from 1/1 (the Pixhawk's state as MAVROS reports it, relayed -- the
  Pixhawk itself is still not on the radio)
    HEARTBEAT            armed / guided (base_mode), ArduCopter mode number
                         (custom_mode), system_status; only sent while
                         MAVROS is connected to the FCU
    SYS_STATUS           battery voltage / current / remaining
    LOCAL_POSITION_NED   x y z vx vy vz
    ATTITUDE_QUATERNION  q1..q4 = w x y z
    STATUSTEXT           FCU status text (/mavros/statustext/recv)

Position, velocity and attitude are sent in the ROS frame MAVROS
publishes them in (ENU world, FLU body), NOT converted to NED: both ends
of this link are ours and the GCS shows exactly the ROS values.

custom-gcs/gcs/backend/app/radio_protocol.py mirrors these constants and
must be kept in sync with this file.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Optional

MSG_ID_HEARTBEAT = 0
MSG_ID_SYS_STATUS = 1
MSG_ID_ATTITUDE_QUATERNION = 31
MSG_ID_LOCAL_POSITION_NED = 32
MSG_ID_COMMAND_LONG = 76
MSG_ID_COMMAND_ACK = 77
MSG_ID_NAMED_VALUE_FLOAT = 251
MSG_ID_STATUSTEXT = 253
CRC_EXTRA = {
    MSG_ID_HEARTBEAT: 50,
    MSG_ID_SYS_STATUS: 124,
    MSG_ID_ATTITUDE_QUATERNION: 246,
    MSG_ID_LOCAL_POSITION_NED: 185,
    MSG_ID_COMMAND_LONG: 152,
    MSG_ID_COMMAND_ACK: 143,
    MSG_ID_NAMED_VALUE_FLOAT: 170,
    MSG_ID_STATUSTEXT: 83,
}

MAV_CMD_USER_1 = 31010
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2

MAV_TYPE_QUADROTOR = 2
MAV_TYPE_ONBOARD_CONTROLLER = 18
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_GUIDED_ENABLED = 8
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_SEVERITY_INFO = 6

MAVLINK1_MAGIC = 0xFE
MAVLINK2_MAGIC = 0xFD
_MAVLINK2_HEADER_LEN = 10
_MAVLINK2_SIGNATURE_LEN = 13
_MAVLINK_IFLAG_SIGNED = 0x01

JETSON_SYSTEM_ID = 1
JETSON_COMPONENT_ID = 191
FCU_RELAY_COMPONENT_ID = 1  # MAV_COMP_ID_AUTOPILOT1: FCU state relayed by the Jetson
GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = 190

NIDAR_START = 1.0
NIDAR_ABORT = 2.0
NIDAR_MAGIC = 4242.0
PROTOCOL_VERSION = 2

MISSION_HOVER = 1
MISSION_NAMES = {MISSION_HOVER: "hover"}

REASON_OK = 0
REASON_UNKNOWN_MISSION = 1
REASON_BAD_PROTOCOL_VERSION = 2
REASON_MISSION_NOT_READY = 3
REASON_FCU_NOT_CONNECTED = 4
REASON_MISSION_BUSY = 5
REASON_NAMES = {
    REASON_OK: "OK",
    REASON_UNKNOWN_MISSION: "UNKNOWN_MISSION",
    REASON_BAD_PROTOCOL_VERSION: "BAD_PROTOCOL_VERSION",
    REASON_MISSION_NOT_READY: "MISSION_NOT_READY",
    REASON_FCU_NOT_CONNECTED: "FCU_NOT_CONNECTED",
    REASON_MISSION_BUSY: "MISSION_BUSY",
}

# Heartbeat custom_mode values -- the hover mission's states
# (missions/hover/hover_logic.py). 255 = unknown / no mission status.
MISSION_STATE_CODES = {
    "idle": 0,
    "preflight": 1,
    "setting_guided": 2,
    "arming": 3,
    "taking_off": 4,
    "hovering": 5,
    "landing": 6,
    "complete": 7,
    "aborted": 8,
    "failed": 9,
    "pilot_override": 10,
}
MISSION_STATE_UNKNOWN = 255

# ArduCopter custom_mode numbers, by the mode name MAVROS reports in
# mavros_msgs/State.mode. A name not in this table is sent as
# ARDUCOPTER_MODE_UNKNOWN.
ARDUCOPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
    "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11, "SPORT": 13, "FLIP": 14,
    "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17, "THROW": 18, "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20, "SMART_RTL": 21, "FLOWHOLD": 22, "FOLLOW": 23, "ZIGZAG": 24,
    "SYSTEMID": 25, "AUTOROTATE": 26, "AUTO_RTL": 27, "TURTLE": 28,
}
ARDUCOPTER_MODE_UNKNOWN = 0xFFFFFFFF

# NAMED_VALUE_FLOAT names for the hover mission's progress (max 10 chars).
HOVER_VALUE_NAMES = {
    "current_altitude_m": "hv_alt",
    "target_altitude_m": "hv_tgt",
    "duration_s": "hv_dur",
    "elapsed_hover_s": "hv_elap",
}

STATUSTEXT_CHUNK_LEN = 50

_PARAM_TO_COMMAND = {NIDAR_START: "start", NIDAR_ABORT: "abort"}

_COMMAND_LONG_FMT = "<7fHBBB"  # 33 bytes, MAVLink wire order
_COMMAND_LONG_LEN = struct.calcsize(_COMMAND_LONG_FMT)
_COMMAND_ACK_FMT = "<HBBiBB"  # command, result, progress, result_param2, target_sys, target_comp
_HEARTBEAT_FMT = "<IBBBBB"  # custom_mode, type, autopilot, base_mode, system_status, mavlink_version
# sensors present/enabled/health, load, voltage_battery (mV), current_battery (cA),
# drop_rate_comm, errors_comm, errors_count1..4, battery_remaining (%)
_SYS_STATUS_FMT = "<IIIHHhHHHHHHb"
_LOCAL_POSITION_FMT = "<I6f"  # time_boot_ms, x, y, z, vx, vy, vz
_ATTITUDE_QUATERNION_FMT = "<I7f"  # time_boot_ms, q1..q4, rollspeed, pitchspeed, yawspeed
_NAMED_VALUE_FLOAT_FMT = "<If10s"  # time_boot_ms, value, name
_STATUSTEXT_FMT = "<B50sHB"  # severity, text, id, chunk_seq

_MAX_BUFFER = 4096


@dataclass(frozen=True)
class Frame:
    msgid: int
    sysid: int
    compid: int
    seq: int
    payload: bytes


@dataclass(frozen=True)
class CommandLong:
    params: tuple
    command: int
    target_system: int
    target_component: int
    confirmation: int


@dataclass(frozen=True)
class RadioCommand:
    command: str  # exactly "start" or "abort"
    nonce: int
    mission_code: int
    protocol_version: int


def x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class MavlinkStreamParser:
    """Turns a raw serial byte stream into checksum-verified MAVLink2
    frames. Only message ids in CRC_EXTRA can be verified, so only those
    are returned; a frame with a bad checksum costs one byte and the
    parser resyncs. MAVLink1 frames are skipped."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[Frame]:
        self._buf.extend(data)
        if len(self._buf) > _MAX_BUFFER:
            del self._buf[: len(self._buf) - _MAX_BUFFER]
        frames: List[Frame] = []
        while True:
            start = _find_magic(self._buf)
            if start < 0:
                self._buf.clear()
                return frames
            if start:
                del self._buf[:start]
            if self._buf[0] == MAVLINK1_MAGIC:
                if len(self._buf) < 2:
                    return frames
                total = 6 + self._buf[1] + 2
                if len(self._buf) < total:
                    return frames
                del self._buf[:total]
                continue
            if len(self._buf) < _MAVLINK2_HEADER_LEN:
                return frames
            length = self._buf[1]
            total = _MAVLINK2_HEADER_LEN + length + 2
            if self._buf[2] & _MAVLINK_IFLAG_SIGNED:
                total += _MAVLINK2_SIGNATURE_LEN
            if len(self._buf) < total:
                return frames
            msgid = int.from_bytes(self._buf[7:10], "little")
            crc_extra = CRC_EXTRA.get(msgid)
            if crc_extra is None:
                del self._buf[:total]
                continue
            body = bytes(self._buf[1 : _MAVLINK2_HEADER_LEN + length])
            expected = x25_crc(bytes([crc_extra]), x25_crc(body))
            received = int.from_bytes(
                self._buf[_MAVLINK2_HEADER_LEN + length : _MAVLINK2_HEADER_LEN + length + 2],
                "little",
            )
            if expected != received:
                del self._buf[:1]
                continue
            frames.append(
                Frame(
                    msgid=msgid,
                    sysid=self._buf[5],
                    compid=self._buf[6],
                    seq=self._buf[4],
                    payload=bytes(self._buf[_MAVLINK2_HEADER_LEN : _MAVLINK2_HEADER_LEN + length]),
                )
            )
            del self._buf[:total]


def _find_magic(buf: bytearray) -> int:
    positions = [p for p in (buf.find(MAVLINK2_MAGIC), buf.find(MAVLINK1_MAGIC)) if p >= 0]
    return min(positions) if positions else -1


def encode_frame(msgid: int, payload: bytes, seq: int, sysid: int, compid: int) -> bytes:
    """A finished MAVLink2 frame (unsigned), with MAVLink2 trailing-zero
    payload truncation."""
    payload = payload.rstrip(b"\x00") or b"\x00"
    header = struct.pack(
        "<BBBBBB3s",
        len(payload),
        0,  # incompat_flags
        0,  # compat_flags
        seq & 0xFF,
        sysid,
        compid,
        msgid.to_bytes(3, "little"),
    )
    crc = x25_crc(bytes([CRC_EXTRA[msgid]]), x25_crc(header + payload))
    return bytes([MAVLINK2_MAGIC]) + header + payload + struct.pack("<H", crc)


def decode_command_long(payload: bytes) -> CommandLong:
    # MAVLink2 truncates trailing zero bytes -- restore them before unpacking.
    padded = payload[:_COMMAND_LONG_LEN].ljust(_COMMAND_LONG_LEN, b"\x00")
    *params, command, target_system, target_component, confirmation = struct.unpack(
        _COMMAND_LONG_FMT, padded
    )
    return CommandLong(tuple(params), command, target_system, target_component, confirmation)


def parse_radio_command(
    cmd: CommandLong, own_system: int, own_component: int
) -> Optional[RadioCommand]:
    """Map a decoded COMMAND_LONG to a RadioCommand, or None if it isn't a
    NIDAR command addressed to us. Mission code and protocol version are
    returned as-is; whether they're acceptable is decided by
    radio_command_logic.CommandGate, so it can NACK with a reason."""
    if cmd.command != MAV_CMD_USER_1:
        return None
    if cmd.target_system != own_system or cmd.target_component != own_component:
        return None
    if cmd.params[2] != NIDAR_MAGIC:
        return None
    command = _PARAM_TO_COMMAND.get(cmd.params[0])
    if command is None:
        return None
    return RadioCommand(
        command=command,
        nonce=int(cmd.params[1]),
        mission_code=int(cmd.params[3]),
        protocol_version=int(cmd.params[6]),
    )


def encode_command_long(
    command: str,
    nonce: int,
    mission_code: int,
    seq: int,
    target_system: int = JETSON_SYSTEM_ID,
    target_component: int = JETSON_COMPONENT_ID,
    sysid: int = GCS_SYSTEM_ID,
    compid: int = GCS_COMPONENT_ID,
    protocol_version: int = PROTOCOL_VERSION,
) -> bytes:
    """The GCS side of the request. Used by tests and bench tools. The
    nonce must stay below 2**24 so it survives the float32 param exactly."""
    param1 = NIDAR_START if command == "start" else NIDAR_ABORT
    payload = struct.pack(
        _COMMAND_LONG_FMT,
        param1, float(nonce), NIDAR_MAGIC, float(mission_code), 0.0, 0.0, float(protocol_version),
        MAV_CMD_USER_1, target_system, target_component, 0,
    )
    return encode_frame(MSG_ID_COMMAND_LONG, payload, seq, sysid, compid)


def encode_command_ack(
    result: int,
    reason: int,
    nonce: int,
    target_system: int,
    target_component: int,
    seq: int,
    sysid: int = JETSON_SYSTEM_ID,
    compid: int = JETSON_COMPONENT_ID,
) -> bytes:
    payload = struct.pack(
        _COMMAND_ACK_FMT, MAV_CMD_USER_1, result, reason, nonce, target_system, target_component
    )
    return encode_frame(MSG_ID_COMMAND_ACK, payload, seq, sysid, compid)


def encode_heartbeat(
    mission_state_code: int,
    seq: int,
    sysid: int = JETSON_SYSTEM_ID,
    compid: int = JETSON_COMPONENT_ID,
) -> bytes:
    payload = struct.pack(
        _HEARTBEAT_FMT,
        mission_state_code,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0,
        MAV_STATE_ACTIVE,
        3,
    )
    return encode_frame(MSG_ID_HEARTBEAT, payload, seq, sysid, compid)


# -- telemetry (Jetson -> GCS) ---------------------------------------------------

_UINT16_UNKNOWN = 0xFFFF


def _finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value)


def arducopter_mode_number(mode: Optional[str]) -> int:
    return ARDUCOPTER_MODES.get((mode or "").upper(), ARDUCOPTER_MODE_UNKNOWN)


def encode_fcu_heartbeat(
    armed: bool,
    guided: bool,
    mode: Optional[str],
    system_status: int,
    seq: int,
    sysid: int = JETSON_SYSTEM_ID,
    compid: int = FCU_RELAY_COMPONENT_ID,
) -> bytes:
    """The FCU's state as MAVROS reports it (mavros_msgs/State)."""
    base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
    if guided:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED
    payload = struct.pack(
        _HEARTBEAT_FMT,
        arducopter_mode_number(mode),
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        system_status & 0xFF,
        3,
    )
    return encode_frame(MSG_ID_HEARTBEAT, payload, seq, sysid, compid)


def encode_sys_status(
    voltage_v: Optional[float],
    current_a: Optional[float],
    remaining_fraction: Optional[float],
    seq: int,
    sysid: int = JETSON_SYSTEM_ID,
    compid: int = FCU_RELAY_COMPONENT_ID,
) -> bytes:
    """Battery only (sensor bitmasks are 0). Takes sensor_msgs/BatteryState
    values: volts, amps (ROS sign, unchanged), percentage as 0..1. Unknown
    (None/NaN) is sent as MAVLink's "unknown": voltage UINT16_MAX,
    current -1, remaining -1."""
    voltage_mv = (
        min(max(int(round(voltage_v * 1000)), 0), _UINT16_UNKNOWN - 1)
        if _finite(voltage_v)
        else _UINT16_UNKNOWN
    )
    current_ca = (
        min(max(int(round(current_a * 100)), -32768), 32767) if _finite(current_a) else -1
    )
    remaining = (
        min(max(int(round(remaining_fraction * 100)), 0), 100)
        if _finite(remaining_fraction)
        else -1
    )
    payload = struct.pack(
        _SYS_STATUS_FMT, 0, 0, 0, 0, voltage_mv, current_ca, 0, 0, 0, 0, 0, 0, remaining
    )
    return encode_frame(MSG_ID_SYS_STATUS, payload, seq, sysid, compid)


def encode_local_position(
    time_boot_ms: int,
    position: tuple,
    velocity: Optional[tuple],
    seq: int,
    sysid: int = JETSON_SYSTEM_ID,
    compid: int = FCU_RELAY_COMPONENT_ID,
) -> bytes:
    """LOCAL_POSITION_NED carrying the ROS ENU position/velocity unchanged
    (see the module docstring). Unknown velocity is sent as NaN."""
    vx, vy, vz = velocity if velocity is not None else (math.nan, math.nan, math.nan)
    payload = struct.pack(
        _LOCAL_POSITION_FMT, time_boot_ms & 0xFFFFFFFF, *position, vx, vy, vz
    )
    return encode_frame(MSG_ID_LOCAL_POSITION_NED, payload, seq, sysid, compid)


def encode_attitude_quaternion(
    time_boot_ms: int,
    w: float,
    x: float,
    y: float,
    z: float,
    seq: int,
    sysid: int = JETSON_SYSTEM_ID,
    compid: int = FCU_RELAY_COMPONENT_ID,
) -> bytes:
    """ATTITUDE_QUATERNION with q1..q4 = w x y z of the ROS orientation
    (sensor_msgs/Imu), unchanged. Body rates are not sent (0)."""
    payload = struct.pack(
        _ATTITUDE_QUATERNION_FMT, time_boot_ms & 0xFFFFFFFF, w, x, y, z, 0.0, 0.0, 0.0
    )
    return encode_frame(MSG_ID_ATTITUDE_QUATERNION, payload, seq, sysid, compid)


def encode_named_value_float(
    time_boot_ms: int,
    name: str,
    value: float,
    seq: int,
    sysid: int = JETSON_SYSTEM_ID,
    compid: int = JETSON_COMPONENT_ID,
) -> bytes:
    payload = struct.pack(
        _NAMED_VALUE_FLOAT_FMT, time_boot_ms & 0xFFFFFFFF, value, name.encode("ascii")[:10]
    )
    return encode_frame(MSG_ID_NAMED_VALUE_FLOAT, payload, seq, sysid, compid)


def statustext_chunks(text: str) -> List[bytes]:
    """Split text into MAVLink2 STATUSTEXT chunks of 50 bytes. A text that
    fills its last chunk exactly gets an empty terminating chunk, so the
    receiver can tell where it ends (a chunk shorter than 50 bytes)."""
    data = text.encode("ascii", errors="replace")
    if len(data) <= STATUSTEXT_CHUNK_LEN:
        return [data]
    chunks = [
        data[i : i + STATUSTEXT_CHUNK_LEN] for i in range(0, len(data), STATUSTEXT_CHUNK_LEN)
    ]
    if len(chunks[-1]) == STATUSTEXT_CHUNK_LEN:
        chunks.append(b"")
    return chunks


def encode_statustext(
    severity: int,
    chunk: bytes,
    seq: int,
    text_id: int = 0,
    chunk_seq: int = 0,
    sysid: int = JETSON_SYSTEM_ID,
    compid: int = JETSON_COMPONENT_ID,
) -> bytes:
    """One STATUSTEXT frame. text_id 0 = a single, unchunked text; a
    chunked text uses the same nonzero text_id on every chunk."""
    payload = struct.pack(
        _STATUSTEXT_FMT, severity & 0xFF, chunk[:STATUSTEXT_CHUNK_LEN], text_id & 0xFFFF, chunk_seq & 0xFF
    )
    return encode_frame(MSG_ID_STATUSTEXT, payload, seq, sysid, compid)
