"""The SLAM map over the radio -- pure Python (unit-tested), no rclpy.

Cartographer publishes a fine nav_msgs/OccupancyGrid on /map (0..100 = %
occupied, -1 = unknown). The radio can't carry that, so radio_command_node
reduces it to a coarse grid (default 0.25 m cells) with three states and
sends only rows that changed, inside standard MAVLink TUNNEL messages
(payload type MAP_ROWS). The SLAM pose of the LiDAR goes in a small TUNNEL
of type SLAM_POSE. custom-gcs/gcs/backend/app/radio_protocol.py mirrors the
formats.

Coarse grid: aligned to whole multiples of the cell size in the map frame,
so cells don't shift when Cartographer grows its map. Cell = OCCUPIED if any
fine cell in it is occupied (>= 60), else FREE if any is free (<= 45), else
UNKNOWN. Packed 2 bits per cell (4 cells per byte, first cell in the low
bits).

MAP_ROWS payload (<= 128 bytes):
    <H cell_cm> <h origin_x_cm> <h origin_y_cm> <B width> <B height>
    <B row0> <B nrows> <packed cells: nrows * width cells, row-major>
SLAM_POSE payload:
    <h x_cm> <h y_cm> <h yaw_cdeg>     (map frame; yaw counter-clockwise from +x)
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
# Cartographer's grid starts a cell at 55 after one hit / 49 after one miss
# and moves it a little per scan; 65 hid walls that were passed quickly
# (seen live 2026-10-09: dotted walls).
OCCUPIED_MIN = 60
FREE_MAX = 45

MAP_ROWS_TYPE = 0x8001
SLAM_POSE_TYPE = 0x8002
TUNNEL_PAYLOAD_MAX = 128

_HEADER = struct.Struct("<HhhBBBB")
_POSE = struct.Struct("<hhh")
CELLS_PER_PAYLOAD = (TUNNEL_PAYLOAD_MAX - _HEADER.size) * 4
MAX_DIM = 255


@dataclass(frozen=True)
class GridMeta:
    cell_cm: int
    origin_x_cm: int
    origin_y_cm: int
    width: int
    height: int


def coarsen(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    cell_m: float,
    block: int = 8,
) -> Tuple[GridMeta, bytearray]:
    """Fine OccupancyGrid -> (meta, cells) with UNKNOWN/FREE/OCCUPIED per cell.

    The coarse bounds are rounded outwards to whole `block`s of cells (2 m at
    0.25 m), so the geometry sent to the GCS changes only when the map grows
    past a block edge -- not every time Cartographer extends its map by a
    cell, which made the GCS keep restarting its map. Caps at 248 x 248
    coarse cells (62 m at 0.25 m)."""
    cx0 = (math.floor(origin_x / cell_m) // block) * block
    cy0 = (math.floor(origin_y / cell_m) // block) * block
    cx1 = (math.floor((origin_x + width * resolution - 1e-9) / cell_m) // block + 1) * block - 1
    cy1 = (math.floor((origin_y + height * resolution - 1e-9) / cell_m) // block + 1) * block - 1
    cap = (MAX_DIM // block) * block
    w = max(1, min(cap, cx1 - cx0 + 1))
    h = max(1, min(cap, cy1 - cy0 + 1))
    cells = bytearray(w * h)  # all UNKNOWN
    # column index of each fine column, computed once
    col_index = [math.floor((origin_x + (c + 0.5) * resolution) / cell_m) - cx0 for c in range(width)]
    for r in range(height):
        ri = math.floor((origin_y + (r + 0.5) * resolution) / cell_m) - cy0
        if not 0 <= ri < h:
            continue
        base = r * width
        row_out = ri * w
        for c in range(width):
            v = data[base + c]
            if v < 0:
                continue
            ci = col_index[c]
            if not 0 <= ci < w:
                continue
            i = row_out + ci
            if v >= OCCUPIED_MIN:
                cells[i] = OCCUPIED
            elif v <= FREE_MAX and cells[i] == UNKNOWN:
                cells[i] = FREE
    meta = GridMeta(
        int(round(cell_m * 100)), int(round(cx0 * cell_m * 100)), int(round(cy0 * cell_m * 100)), w, h
    )
    return meta, cells


def pack(cells: Sequence[int]) -> bytes:
    out = bytearray((len(cells) + 3) // 4)
    for i, v in enumerate(cells):
        out[i >> 2] |= (v & 0x3) << ((i & 3) * 2)
    return bytes(out)


def unpack(data: bytes, count: int) -> List[int]:
    return [(data[i >> 2] >> ((i & 3) * 2)) & 0x3 for i in range(count)]


def encode_rows(meta: GridMeta, cells: Sequence[int], row0: int, nrows: int) -> bytes:
    w = meta.width
    body = pack(cells[row0 * w : (row0 + nrows) * w])
    return _HEADER.pack(meta.cell_cm, meta.origin_x_cm, meta.origin_y_cm, w, meta.height, row0, nrows) + body


def decode_rows(payload: bytes) -> Optional[Tuple[GridMeta, int, int, List[int]]]:
    if len(payload) < _HEADER.size:
        return None
    cell_cm, ox, oy, w, h, row0, nrows = _HEADER.unpack_from(payload)
    count = w * nrows
    body = payload[_HEADER.size :]
    if w == 0 or len(body) < (count + 3) // 4 or row0 + nrows > h:
        return None
    return GridMeta(cell_cm, ox, oy, w, h), row0, nrows, unpack(body, count)


def encode_pose(x_m: float, y_m: float, yaw_rad: float) -> bytes:
    def clamp(v):
        return max(-32768, min(32767, int(round(v))))

    yaw_cdeg = math.degrees(math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))) * 100
    return _POSE.pack(clamp(x_m * 100), clamp(y_m * 100), clamp(yaw_cdeg))


def decode_pose(payload: bytes) -> Optional[Tuple[float, float, float]]:
    if len(payload) < _POSE.size:
        return None
    x, y, yaw = _POSE.unpack_from(payload)
    return x / 100.0, y / 100.0, yaw / 100.0  # metres, metres, degrees


def rows_per_payload(width: int) -> int:
    return max(1, CELLS_PER_PAYLOAD // max(1, width))


class MapSender:
    """Decides which rows to send. New/changed rows go first (lowest row
    first, as many consecutive dirty rows as fit one payload); when nothing
    changed, one chunk is re-sent every refresh_s in turn so a GCS that
    started late (or missed a packet) fills in. A new grid geometry (size,
    origin, cell size) makes every row dirty."""

    def __init__(self, refresh_s: float = 2.0) -> None:
        self.refresh_s = refresh_s
        self.meta: Optional[GridMeta] = None
        self._cells: Optional[bytearray] = None
        self._sent: Optional[bytearray] = None
        self._dirty: set = set()
        self._refresh_row = 0
        self._last_refresh: Optional[float] = None

    def update(self, meta: GridMeta, cells: bytearray) -> None:
        if meta != self.meta:
            self.meta = meta
            self._sent = None
            self._dirty = set(range(meta.height))
            self._refresh_row = 0
        else:
            w = meta.width
            for r in range(meta.height):
                if self._sent is None or cells[r * w : (r + 1) * w] != self._sent[r * w : (r + 1) * w]:
                    self._dirty.add(r)
        self._cells = bytearray(cells)
        if self._sent is None:
            self._sent = bytearray(len(cells))

    def next_payload(self, now: float) -> Optional[bytes]:
        if self.meta is None or self._cells is None:
            return None
        meta, per = self.meta, rows_per_payload(self.meta.width)
        if self._dirty:
            row0 = min(self._dirty)
            nrows = 1
            while nrows < per and row0 + nrows < meta.height and (row0 + nrows) in self._dirty:
                nrows += 1
        elif self._last_refresh is None or now - self._last_refresh >= self.refresh_s:
            row0 = self._refresh_row
            nrows = min(per, meta.height - row0)
            self._refresh_row = 0 if row0 + nrows >= meta.height else row0 + nrows
        else:
            return None
        self._last_refresh = now  # refresh timer counts from the last map packet of any kind
        w = meta.width
        for r in range(row0, row0 + nrows):
            self._dirty.discard(r)
            self._sent[r * w : (r + 1) * w] = self._cells[r * w : (r + 1) * w]
        return encode_rows(meta, self._cells, row0, nrows)
