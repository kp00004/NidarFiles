"""SLAM map over the radio: coarsening, packing, row selection, TUNNEL frames."""
import math

import pytest

from nidar_autonomy.map_grid import (
    FREE,
    MAP_ROWS_TYPE,
    OCCUPIED,
    SLAM_POSE_TYPE,
    UNKNOWN,
    GridMeta,
    MapSender,
    coarsen,
    decode_pose,
    decode_rows,
    encode_pose,
    encode_rows,
    pack,
    rows_per_payload,
    unpack,
)
from nidar_autonomy.radio_telemetry import TelemetryRelay
from nidar_autonomy.telem_command_codec import MSG_ID_TUNNEL, MavlinkStreamParser, encode_tunnel


def room(fine_res=0.1, size_m=4.0, origin=(-2.0, -2.0)):
    """Fine grid of a size_m square room: walls occupied, inside free, and a
    0.6 m unknown margin outside (wider than a coarse cell)."""
    pad = 6
    inner = int(round(size_m / fine_res))
    n = inner + 2 * pad
    data = [-1] * (n * n)
    for r in range(pad, pad + inner):
        for c in range(pad, pad + inner):
            edge = r in (pad, pad + inner - 1) or c in (pad, pad + inner - 1)
            data[r * n + c] = 100 if edge else 0
    return data, n, n, fine_res, origin[0] - pad * fine_res, origin[1] - pad * fine_res


def test_coarsen_marks_walls_free_and_unknown():
    data, w, h, res, ox, oy = room()
    meta, cells = coarsen(data, w, h, res, ox, oy, 0.25)
    assert meta.cell_cm == 25 and meta.width == meta.height
    assert set(cells) == {UNKNOWN, FREE, OCCUPIED}
    centre = (meta.height // 2) * meta.width + meta.width // 2
    assert cells[centre] == FREE


def test_coarse_grid_is_aligned_to_cell_multiples():
    data, w, h, res, ox, oy = room()
    meta, _ = coarsen(data, w, h, res, ox, oy, 0.25)
    assert meta.origin_x_cm % 25 == 0 and meta.origin_y_cm % 25 == 0
    # growing the fine map by one column on the left keeps cell boundaries
    meta2, _ = coarsen([-1] * ((w + 1) * h), w + 1, h, res, ox - res, oy, 0.25)
    assert meta2.origin_x_cm % 25 == 0


def test_occupied_wins_over_free_in_a_cell():
    meta, cells = coarsen([0, 100, 0, 0], 2, 2, 0.1, 0.0, 0.0, 0.25)
    assert cells[0] == OCCUPIED


def test_pack_unpack_round_trip():
    cells = [0, 1, 2, 1, 2, 2, 0]
    assert unpack(pack(cells), len(cells)) == cells
    assert len(pack(cells)) == 2


def test_rows_payload_round_trip_and_size():
    meta = GridMeta(25, -200, -150, 60, 40)
    cells = bytearray((i * 7) % 3 for i in range(60 * 40))
    per = rows_per_payload(60)
    payload = encode_rows(meta, cells, 3, per)
    assert len(payload) <= 128
    m2, row0, nrows, got = decode_rows(payload)
    assert m2 == meta and (row0, nrows) == (3, per)
    assert got == list(cells[3 * 60 : (3 + per) * 60])


def test_pose_round_trip():
    x, y, yaw_deg = decode_pose(encode_pose(1.234, -0.5, math.radians(-90)))
    assert (x, y) == (1.23, -0.5) and yaw_deg == -90.0


def test_sender_sends_everything_then_only_changes():
    sender = MapSender(refresh_s=1000)
    meta = GridMeta(25, 0, 0, 20, 10)
    cells = bytearray(200)
    sender.update(meta, cells)
    first = sender.next_payload(0.0)
    _, row0, nrows, _ = decode_rows(first)
    assert (row0, nrows) == (0, 10)  # 20-wide rows: all 10 fit one payload
    assert sender.next_payload(0.1) is None  # nothing changed, refresh far away
    cells[5 * 20 + 3] = OCCUPIED
    sender.update(meta, cells)
    _, row0, nrows, got = decode_rows(sender.next_payload(0.2))
    assert (row0, nrows) == (5, 1) and got[3] == OCCUPIED


def test_sender_refreshes_round_robin_when_idle():
    sender = MapSender(refresh_s=2.0)
    meta = GridMeta(25, 0, 0, 200, 6)  # 200 wide -> 2 rows per payload
    sender.update(meta, bytearray(1200))
    sent = []
    t = 0.0
    while True:
        p = sender.next_payload(t)
        if p is None:
            break
        sent.append(decode_rows(p)[1])
    assert sent == [0, 2, 4]  # initial full send
    refresh = [decode_rows(sender.next_payload(t)) for t in (10.0, 12.0, 14.0)]
    assert [r[1] for r in refresh] == [0, 2, 4]
    assert sender.next_payload(14.5) is None


def test_new_geometry_resends_everything():
    sender = MapSender(refresh_s=1000)
    sender.update(GridMeta(25, 0, 0, 10, 4), bytearray(40))
    sender.next_payload(0.0)
    sender.update(GridMeta(25, -25, 0, 11, 4), bytearray(44))
    _, row0, nrows, _ = decode_rows(sender.next_payload(0.1))
    assert (row0, nrows) == (0, 4)


def test_tunnel_parses_in_pymavlink():
    mav = pytest.importorskip("pymavlink.dialects.v20.common")
    payload = encode_pose(1.0, 2.0, 0.5)
    msg = mav.MAVLink(None).parse_char(encode_tunnel(SLAM_POSE_TYPE, payload, seq=1))
    assert msg.get_type() == "TUNNEL" and msg.payload_type == SLAM_POSE_TYPE
    assert bytes(msg.payload[: msg.payload_length]) == payload
    assert (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 191)


class Seq:
    def __init__(self):
        self.n = 0

    def __call__(self):
        self.n = (self.n + 1) & 0xFF
        return self.n


def test_relay_sends_map_rows_at_map_rate_and_pose_every_tick():
    relay = TelemetryRelay(Seq(), 0.0, lidar_rate_hz=0, map_rate_hz=1.0)
    meta = GridMeta(25, 0, 0, 60, 60)
    relay.update_map(0.0, meta, bytearray(3600))
    tunnels = {"map": 0, "pose": 0}
    nbytes = 0
    for k in range(20):  # 10 s at 2 Hz ticks
        now = k * 0.5
        relay.update_slam_pose(now, 1.0, 2.0, 0.0)
        relay.update_map(now, meta, bytearray(3600))
        raw = relay.tick(now)
        nbytes += sum(len(f) for f in raw)
        for f in MavlinkStreamParser().feed(b"".join(raw)):
            if f.msgid == MSG_ID_TUNNEL:
                ptype = int.from_bytes(f.payload[:2], "little")
                tunnels["map" if ptype == MAP_ROWS_TYPE else "pose"] += 1
    assert tunnels["pose"] == 20
    assert 9 <= tunnels["map"] <= 11
    assert nbytes / 10 < 450  # map + pose at these rates, nothing else fresh


def test_relay_sends_no_map_when_stale():
    relay = TelemetryRelay(Seq(), 0.0, lidar_rate_hz=0, map_rate_hz=2.0)
    relay.update_map(0.0, GridMeta(25, 0, 0, 10, 10), bytearray(100))
    assert all(f.msgid != MSG_ID_TUNNEL for f in MavlinkStreamParser().feed(b"".join(relay.tick(30.0))))
