"""RPLIDAR protocol and the 72-sector radio reduction (rplidar_protocol.py)."""
import math

from nidar_autonomy.rplidar_protocol import (
    CMD_GET_INFO,
    SECTORS,
    UNKNOWN_CM,
    Point,
    ScanParser,
    command,
    encode_node,
    laserscan_bins,
    motor_pwm_command,
    parse_descriptor,
    parse_info,
    sectors_from_laserscan,
)


def test_simple_and_payload_commands():
    assert command(CMD_GET_INFO) == bytes([0xA5, 0x50])
    pwm = motor_pwm_command(660)
    assert pwm[:3] == bytes([0xA5, 0xF0, 0x02]) and pwm[3:5] == (660).to_bytes(2, "little")
    checksum = 0
    for b in pwm[:-1]:
        checksum ^= b
    assert pwm[-1] == checksum


def test_scan_descriptor():
    d = parse_descriptor(bytes([0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81]))
    assert (d.length, d.mode, d.data_type) == (5, 1, 0x81)


def test_info_answer():
    raw = bytes([0xA5, 0x5A, 0x14, 0, 0, 0, 0x04]) + bytes([0x28, 0x1A, 0x01, 0x05]) + bytes(range(16))
    info = parse_info(raw)
    assert info.model == 0x28 and info.firmware == "1.26" and info.hardware == 5
    assert parse_info(raw[:10]) is None
    assert parse_info(bytes([0xA5, 0x5A, 0x05, 0, 0, 0x40, 0x81]) + bytes(20)) is None  # a scan descriptor


def test_parser_splits_turns_on_start_flag():
    stream = b"".join(
        [encode_node(0.0, 1000, start=True), encode_node(90.0, 2000), encode_node(180.0, 3000),
         encode_node(0.5, 1100, start=True), encode_node(45.0, 500)]
    )
    turns = ScanParser().feed(stream)
    assert len(turns) == 1
    assert [round(p.angle_deg) for p in turns[0]] == [0, 90, 180]
    assert [p.distance_mm for p in turns[0]] == [1000, 2000, 3000]


def test_parser_resyncs_after_garbage_and_split_reads():
    parser = ScanParser()
    data = b"\x00\xff\x13" + encode_node(10.0, 750, start=True) + encode_node(20.0, 760) + encode_node(5.0, 700, start=True)
    turns = []
    for i in range(0, len(data), 3):  # arrive in small pieces
        turns += parser.feed(data[i:i + 3])
    assert len(turns) == 1 and [round(p.angle_deg) for p in turns[0]] == [10, 20]


def test_laserscan_bins_convert_clockwise_to_ros_counterclockwise():
    ranges = laserscan_bins([Point(90.0, 2000, 40)])  # 90° clockwise = vehicle's right
    # ROS: angle_min=-pi, 1° bins counter-clockwise; right side = -90° -> index 90
    assert ranges[90] == 2.0
    assert sum(math.isfinite(r) for r in ranges) == 1


def test_no_return_and_zero_quality_are_ignored():
    ranges = laserscan_bins([Point(0.0, 0, 40), Point(10.0, 1500, 0)])
    assert all(math.isinf(r) for r in ranges)


def test_sectors_round_trip_front_right_back_left():
    points = [Point(0.0, 1000, 40), Point(90.0, 2000, 40), Point(180.0, 3000, 40), Point(270.0, 4000, 40)]
    ranges = laserscan_bins(points)
    sectors = sectors_from_laserscan(ranges, -math.pi, 2 * math.pi / 360, 0.15, 12.0)
    assert len(sectors) == SECTORS
    assert sectors[0] == 100      # front
    assert sectors[18] == 200     # 90° clockwise = right
    assert sectors[36] == 300     # behind
    assert sectors[54] == 400     # left
    assert sum(s != UNKNOWN_CM for s in sectors) == 4


def test_sector_keeps_nearest_return():
    ranges = laserscan_bins([Point(1.0, 3000, 40), Point(2.0, 800, 40)])
    sectors = sectors_from_laserscan(ranges, -math.pi, 2 * math.pi / 360, 0.15, 12.0)
    assert sectors[0] == 80


def test_out_of_range_is_unknown():
    ranges = laserscan_bins([Point(0.0, 50, 40), Point(90.0, 20000, 40)])
    sectors = sectors_from_laserscan(ranges, -math.pi, 2 * math.pi / 360, 0.15, 12.0)
    assert sectors[0] == UNKNOWN_CM and sectors[18] == UNKNOWN_CM


def test_yaw_offset_rotates_into_vehicle_frame():
    # LiDAR mounted with its 0° pointing to the vehicle's right (offset 90° clockwise)
    ranges = laserscan_bins([Point(0.0, 1000, 40)], yaw_offset_deg=90.0)
    sectors = sectors_from_laserscan(ranges, -math.pi, 2 * math.pi / 360, 0.15, 12.0)
    assert sectors[18] == 100


def test_bin_keeps_wall_behind_close_clutter():
    # same 1-degree bin: clutter at 0.25 m and a wall at 3 m
    ranges = laserscan_bins([Point(10.2, 250, 40), Point(10.6, 3000, 40)], range_min_m=0.4)
    assert 3.0 in ranges


def test_bin_with_only_close_returns_is_never_no_return():
    """Only clutter inside the cutoff: keep the close value (dropped later as
    below range_min) -- never inf, which SLAM would treat as clear space."""
    ranges = laserscan_bins([Point(10.2, 250, 40)], range_min_m=0.4)
    assert 0.25 in ranges
    assert sum(math.isfinite(r) for r in ranges) == 1


def test_default_cutoff_keeps_old_behaviour():
    ranges = laserscan_bins([Point(10.2, 250, 40), Point(10.6, 3000, 40)])
    assert 0.25 in ranges and 3.0 not in ranges
