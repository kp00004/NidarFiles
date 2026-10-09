"""SLAMTEC RPLIDAR (A1/A2) serial protocol -- standard SCAN mode -- and the
72-sector reduction sent over the radio. Pure Python (unit-tested); the
serial I/O is in lidar_node.py.

Protocol (SLAMTEC RPLIDAR interface protocol, standard scan):
  request   0xA5 <cmd> [<size> <payload..> <checksum>]   checksum = XOR of all bytes
  response  0xA5 0x5A <30-bit length | 2-bit mode, LE> <data type>
  GET_INFO  0x50 -> 20 bytes: model, fw minor, fw major, hardware, serial[16]
  SCAN      0x20 -> stream of 5-byte nodes:
              b0 = quality<<2 | !S<<1 | S   (S = first node of a new 360° turn)
              b1 = angle_q6[6:0]<<1 | C     (C must be 1)
              b2 = angle_q6[14:7]           angle = angle_q6 / 64 degrees, CLOCKWISE
              b3, b4 = distance_q2 (LE)     distance = distance_q2 / 4 mm (0 = no return)
  A2 motor: the USB adapter's DTR line drives MOTOCTL (DTR low = motor on),
            plus SET_MOTOR_PWM 0xF0 <uint16 pwm>.

Radio reduction: 72 sectors of 5°, sector i centred on i*5° clockwise from
the LiDAR's front, nearest return per sector in cm; no return = 65535
("unknown" in MAVLink OBSTACLE_DISTANCE).
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

SYNC = 0xA5
SYNC2 = 0x5A
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_SET_MOTOR_PWM = 0xF0

DESCRIPTOR_LEN = 7
INFO_LEN = 20
INFO_TYPE = 0x04
SCAN_TYPE = 0x81
NODE_LEN = 5
DEFAULT_MOTOR_PWM = 660

SECTORS = 72
SECTOR_DEG = 5
UNKNOWN_CM = 0xFFFF


def command(cmd: int, payload: bytes = b"") -> bytes:
    if not payload:
        return bytes([SYNC, cmd])
    body = bytes([SYNC, cmd, len(payload)]) + payload
    checksum = 0
    for b in body:
        checksum ^= b
    return body + bytes([checksum])


def motor_pwm_command(pwm: int) -> bytes:
    return command(CMD_SET_MOTOR_PWM, struct.pack("<H", pwm))


@dataclass(frozen=True)
class Descriptor:
    length: int
    mode: int
    data_type: int


def parse_descriptor(data: bytes) -> Optional[Descriptor]:
    if len(data) < DESCRIPTOR_LEN or data[0] != SYNC or data[1] != SYNC2:
        return None
    raw = int.from_bytes(data[2:6], "little")
    return Descriptor(raw & 0x3FFFFFFF, raw >> 30, data[6])


@dataclass(frozen=True)
class DeviceInfo:
    model: int
    firmware: str
    hardware: int
    serial: str


def parse_info(data: bytes) -> Optional[DeviceInfo]:
    """GET_INFO answer: descriptor + 20 data bytes."""
    desc = parse_descriptor(data)
    if desc is None or desc.data_type != INFO_TYPE or desc.length != INFO_LEN:
        return None
    body = data[DESCRIPTOR_LEN : DESCRIPTOR_LEN + INFO_LEN]
    if len(body) < INFO_LEN:
        return None
    return DeviceInfo(body[0], f"{body[2]}.{body[1]:02d}", body[3], body[4:].hex().upper())


@dataclass(frozen=True)
class Point:
    angle_deg: float  # clockwise from the LiDAR's front
    distance_mm: float  # 0 = no return
    quality: int


def _node_valid(b: bytes) -> bool:
    start = b[0] & 0x01
    inverse = (b[0] >> 1) & 0x01
    return start != inverse and (b[1] & 0x01) == 1


class ScanParser:
    """Turns the SCAN byte stream into complete 360° turns (lists of
    Points). A corrupt node costs one byte and the parser resyncs."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._turn: List[Point] = []

    def feed(self, data: bytes) -> List[List[Point]]:
        self._buf.extend(data)
        turns: List[List[Point]] = []
        while len(self._buf) >= NODE_LEN:
            node = bytes(self._buf[:NODE_LEN])
            if not _node_valid(node):
                del self._buf[0]
                continue
            del self._buf[:NODE_LEN]
            if node[0] & 0x01 and self._turn:
                turns.append(self._turn)
                self._turn = []
            angle = ((node[1] >> 1) | (node[2] << 7)) / 64.0
            distance = (node[3] | (node[4] << 8)) / 4.0
            self._turn.append(Point(angle % 360.0, distance, node[0] >> 2))
        return turns


def encode_node(angle_deg: float, distance_mm: float, quality: int = 47, start: bool = False) -> bytes:
    """One SCAN node (for tests and bench tools)."""
    s = 1 if start else 0
    angle_q6 = int(round(angle_deg * 64)) & 0x7FFF
    dist_q2 = int(round(distance_mm * 4)) & 0xFFFF
    return bytes([
        (quality << 2) | ((1 - s) << 1) | s,
        ((angle_q6 & 0x7F) << 1) | 1,
        angle_q6 >> 7,
        dist_q2 & 0xFF,
        dist_q2 >> 8,
    ])


# -- LaserScan (ROS) <-> radio sectors ----------------------------------------------


def laserscan_bins(points: Sequence[Point], yaw_offset_deg: float = 0.0, bins: int = 360) -> List[float]:
    """ROS sensor_msgs/LaserScan ranges (metres, inf = no return) for
    angle_min = -pi, increment = 2*pi/bins, COUNTER-clockwise from the
    vehicle's front. RPLIDAR angles are clockwise; yaw_offset_deg is the
    LiDAR's 0° direction relative to the vehicle's nose (clockwise)."""
    ranges = [math.inf] * bins
    step = 360.0 / bins
    for p in points:
        if p.distance_mm <= 0 or p.quality == 0:
            continue
        ccw = -(p.angle_deg + yaw_offset_deg)  # vehicle frame, counter-clockwise
        index = int(math.floor(((ccw + 180.0) % 360.0) / step)) % bins
        metres = p.distance_mm / 1000.0
        if metres < ranges[index]:
            ranges[index] = metres
    return ranges


def sectors_from_laserscan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> List[int]:
    """72 nearest-return distances in cm for 5° sectors CLOCKWISE from the
    vehicle's front (sector i centred on i*5°) -- the OBSTACLE_DISTANCE /
    MAV_FRAME_BODY_FRD convention. 65535 = no return in that sector."""
    out = [UNKNOWN_CM] * SECTORS
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        ccw_deg = math.degrees(angle_min + i * angle_increment)
        cw_deg = (-ccw_deg) % 360.0
        sector = int(math.floor((cw_deg + SECTOR_DEG / 2) / SECTOR_DEG)) % SECTORS
        cm = min(int(round(r * 100)), UNKNOWN_CM - 1)
        if cm < out[sector]:
            out[sector] = cm
    return out
