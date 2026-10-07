"""Unit tests for telem_command_codec.py. The pymavlink cross-checks are
skipped where pymavlink isn't installed (it isn't on the Jetson); run them
on a dev machine with `pip install pymavlink`."""
import struct

import pytest

from nidar_autonomy.telem_command_codec import (
    GCS_COMPONENT_ID,
    GCS_SYSTEM_ID,
    JETSON_COMPONENT_ID,
    JETSON_SYSTEM_ID,
    MAV_CMD_USER_1,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_DENIED,
    MISSION_HOVER,
    MSG_ID_COMMAND_ACK,
    MSG_ID_COMMAND_LONG,
    MSG_ID_HEARTBEAT,
    NIDAR_MAGIC,
    PROTOCOL_VERSION,
    REASON_UNKNOWN_MISSION,
    CommandLong,
    MavlinkStreamParser,
    RadioCommand,
    decode_command_long,
    encode_command_ack,
    encode_command_long,
    encode_heartbeat,
    parse_radio_command,
    x25_crc,
)

OWN = (JETSON_SYSTEM_ID, JETSON_COMPONENT_ID)


def _cmd(param1=1.0, nonce=7.0, magic=NIDAR_MAGIC, mission=1.0, version=2.0,
         command=MAV_CMD_USER_1, target=OWN):
    return CommandLong(
        (param1, nonce, magic, mission, 0.0, 0.0, version), command, target[0], target[1], 0
    )


class TestParseRadioCommand:
    def test_start_with_mission_and_version(self):
        assert parse_radio_command(_cmd(), *OWN) == RadioCommand("start", 7, MISSION_HOVER, 2)

    def test_abort(self):
        assert parse_radio_command(_cmd(param1=2.0, mission=0.0), *OWN).command == "abort"

    def test_unknown_mission_and_version_are_passed_through_for_the_gate_to_nack(self):
        parsed = parse_radio_command(_cmd(mission=9.0, version=1.0), *OWN)
        assert (parsed.mission_code, parsed.protocol_version) == (9, 1)

    @pytest.mark.parametrize(
        "kwargs",
        [
            {"param1": 3.0},
            {"magic": 0.0},
            {"command": 31011},
            {"target": (2, JETSON_COMPONENT_ID)},
            {"target": (JETSON_SYSTEM_ID, 1)},  # addressed to the autopilot, not the Jetson
        ],
    )
    def test_anything_not_for_us_is_ignored(self, kwargs):
        assert parse_radio_command(_cmd(**kwargs), *OWN) is None


def test_x25_known_vector():
    assert x25_crc(b"123456789") == 0x6F91  # CRC-16/MCRF4XX check value


class TestStreamParser:
    def test_round_trip_command_long(self):
        frame = MavlinkStreamParser().feed(encode_command_long("start", 42, MISSION_HOVER, seq=3))[0]
        assert (frame.msgid, frame.sysid, frame.compid, frame.seq) == (
            MSG_ID_COMMAND_LONG, GCS_SYSTEM_ID, GCS_COMPONENT_ID, 3
        )
        assert parse_radio_command(decode_command_long(frame.payload), *OWN) == RadioCommand(
            "start", 42, MISSION_HOVER, PROTOCOL_VERSION
        )

    def test_frame_split_across_reads_and_surrounded_by_noise(self):
        parser = MavlinkStreamParser()
        data = b"\x00\x13garbage" + encode_command_long("abort", 5, 0, seq=1) + b"\xff\x01"
        frames = []
        for i in range(0, len(data), 3):
            frames += parser.feed(data[i : i + 3])
        assert [f.msgid for f in frames] == [MSG_ID_COMMAND_LONG]

    def test_two_frames_in_one_read(self):
        data = encode_heartbeat(5, 0) + encode_command_ack(MAV_RESULT_ACCEPTED, 0, 77, 255, 190, 1)
        assert [f.msgid for f in MavlinkStreamParser().feed(data)] == [MSG_ID_HEARTBEAT, MSG_ID_COMMAND_ACK]

    def test_corrupted_frame_is_dropped_and_parser_resyncs(self):
        bad = bytearray(encode_command_long("start", 1, 1, seq=0))
        bad[12] ^= 0xFF
        good = encode_command_long("start", 2, 1, seq=1)
        frames = MavlinkStreamParser().feed(bytes(bad) + good)
        assert len(frames) == 1
        assert parse_radio_command(decode_command_long(frames[0].payload), *OWN).nonce == 2

    def test_unknown_message_id_is_skipped(self):
        unknown = bytes([0xFD, 1, 0, 0, 0, 1, 1]) + (999).to_bytes(3, "little") + b"\x07" + b"\x00\x00"
        frames = MavlinkStreamParser().feed(unknown + encode_heartbeat(0, 0))
        assert [f.msgid for f in frames] == [MSG_ID_HEARTBEAT]


class TestAgainstPymavlink:
    @pytest.fixture
    def mav(self):
        return pytest.importorskip("pymavlink.dialects.v20.common")

    def test_pymavlink_command_long_parses_here(self, mav):
        sender = mav.MAVLink(None, srcSystem=GCS_SYSTEM_ID, srcComponent=GCS_COMPONENT_ID)
        raw = mav.MAVLink_command_long_message(
            *OWN, MAV_CMD_USER_1, 0, 1.0, 123, NIDAR_MAGIC, MISSION_HOVER, 0, 0, PROTOCOL_VERSION
        ).pack(sender)
        frame = MavlinkStreamParser().feed(raw)[0]
        assert parse_radio_command(decode_command_long(frame.payload), *OWN) == RadioCommand(
            "start", 123, MISSION_HOVER, PROTOCOL_VERSION
        )

    def test_our_command_long_is_byte_identical_to_pymavlink(self, mav):
        sender = mav.MAVLink(None, srcSystem=GCS_SYSTEM_ID, srcComponent=GCS_COMPONENT_ID)
        sender.seq = 9
        ref = mav.MAVLink_command_long_message(
            *OWN, MAV_CMD_USER_1, 0, 1.0, 55, NIDAR_MAGIC, MISSION_HOVER, 0, 0, PROTOCOL_VERSION
        ).pack(sender)
        assert encode_command_long("start", 55, MISSION_HOVER, seq=9) == bytes(ref)

    def test_ack_with_reason_is_byte_identical_to_pymavlink(self, mav):
        sender = mav.MAVLink(None, srcSystem=JETSON_SYSTEM_ID, srcComponent=JETSON_COMPONENT_ID)
        sender.seq = 42
        ref = mav.MAVLink_command_ack_message(
            MAV_CMD_USER_1, MAV_RESULT_DENIED, REASON_UNKNOWN_MISSION, 123456, 255, 190
        ).pack(sender)
        assert encode_command_ack(MAV_RESULT_DENIED, REASON_UNKNOWN_MISSION, 123456, 255, 190, seq=42) == bytes(ref)

    def test_heartbeat_parses_in_pymavlink(self, mav):
        parsed = mav.MAVLink(None).parse_char(encode_heartbeat(5, 7))
        assert parsed.get_type() == "HEARTBEAT"
        assert parsed.custom_mode == 5
        assert (parsed.get_srcSystem(), parsed.get_srcComponent()) == OWN

    def test_ack_reason_and_nonce_read_back_in_pymavlink(self, mav):
        raw = encode_command_ack(MAV_RESULT_DENIED, REASON_UNKNOWN_MISSION, 16777215, 255, 190, seq=0)
        parsed = mav.MAVLink(None).parse_char(raw)
        assert (parsed.command, parsed.result, parsed.progress, parsed.result_param2) == (
            MAV_CMD_USER_1, MAV_RESULT_DENIED, REASON_UNKNOWN_MISSION, 16777215
        )


def test_command_long_layout_is_33_bytes_before_truncation():
    assert struct.calcsize("<7fHBBB") == 33
