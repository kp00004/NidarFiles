"""Bench parameter access over the radio: codec (vs pymavlink where
installed) and the allow/refuse rules (param_bridge_logic.py)."""
import math

import pytest

from nidar_autonomy.param_bridge_logic import addressed_to_jetson, refusal, typed_value
from nidar_autonomy.telem_command_codec import (
    MSG_ID_PARAM_VALUE,
    MavlinkStreamParser,
    ParamRequest,
    decode_param_request,
    encode_param_request_read,
    encode_param_set,
    encode_param_value,
)

IDLE = {"hover": "idle", "motor_test": "idle"}


def _req(kind="set", name="RNGFND1_MAX_CM", value=800.0, target=(1, 191)):
    return ParamRequest(kind, name, value, *target)


def _frame(raw):
    return MavlinkStreamParser().feed(raw)[0]


# -- codec ------------------------------------------------------------------------


def test_read_request_round_trip():
    req = decode_param_request(_frame(encode_param_request_read("BATT_ARM_VOLT", seq=1)))
    assert (req.kind, req.name, req.value) == ("read", "BATT_ARM_VOLT", None)
    assert addressed_to_jetson(req)


def test_set_request_round_trip_16_char_name():
    req = decode_param_request(_frame(encode_param_set("EK3_SRC1_POSXY__", 2.0, seq=1)))
    assert (req.kind, req.name, req.value) == ("set", "EK3_SRC1_POSXY__", 2.0)


def test_pymavlink_param_set_decodes_here():
    mav = pytest.importorskip("pymavlink.dialects.v20.common")
    sender = mav.MAVLink(None, srcSystem=255, srcComponent=190)
    raw = sender.param_set_encode(1, 191, b"RNGFND1_MAX_CM", 800.0, mav.MAV_PARAM_TYPE_REAL32).pack(sender)
    req = decode_param_request(_frame(raw))
    assert (req.kind, req.name, req.value) == ("set", "RNGFND1_MAX_CM", 800.0)


def test_pymavlink_param_request_read_decodes_here():
    mav = pytest.importorskip("pymavlink.dialects.v20.common")
    sender = mav.MAVLink(None, srcSystem=255, srcComponent=190)
    raw = sender.param_request_read_encode(1, 191, b"GPS2_TYPE", -1).pack(sender)
    assert decode_param_request(_frame(raw)).name == "GPS2_TYPE"


def test_read_by_index_is_not_supported():
    mav = pytest.importorskip("pymavlink.dialects.v20.common")
    sender = mav.MAVLink(None, srcSystem=255, srcComponent=190)
    raw = sender.param_request_read_encode(1, 191, b"", 5).pack(sender)
    assert decode_param_request(_frame(raw)) is None


def test_param_value_parses_in_pymavlink():
    mav = pytest.importorskip("pymavlink.dialects.v20.common")
    msg = mav.MAVLink(None).parse_char(encode_param_value("RNGFND1_MAX_CM", 800.0, True, seq=3))
    assert (msg.param_id, msg.param_value, msg.param_type) == ("RNGFND1_MAX_CM", 800.0, mav.MAV_PARAM_TYPE_INT32)
    assert (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 191)
    assert _frame(encode_param_value("X", 1.5, False, seq=0)).msgid == MSG_ID_PARAM_VALUE


# -- rules ------------------------------------------------------------------------


def test_read_allowed_even_when_armed_and_writes_disabled():
    assert refusal(_req("read", value=None), False, True, True, {"hover": "hovering"}) is None


def test_write_allowed_when_enabled_disarmed_and_idle():
    assert refusal(_req(), True, True, False, IDLE) is None


@pytest.mark.parametrize(
    "writes_enabled,fcu,armed,missions,expected",
    [
        (False, True, False, IDLE, "writes disabled"),
        (True, True, True, IDLE, "armed"),
        (True, True, None, IDLE, "armed"),
        (True, True, False, {"hover": "hovering"}, "mission running"),
        (True, True, False, {"motor_test": "testing"}, "mission running"),
        (True, False, False, IDLE, "FCU not connected"),
    ],
)
def test_write_refusals(writes_enabled, fcu, armed, missions, expected):
    assert expected in refusal(_req(), writes_enabled, fcu, armed, missions)


@pytest.mark.parametrize("name", ["", "rngfnd1_max", "BAD-NAME", "A" * 17, "X Y"])
def test_bad_names_refused(name):
    assert refusal(_req(name=name), True, True, False, IDLE) == "invalid parameter name"


@pytest.mark.parametrize("value", [math.nan, math.inf])
def test_non_finite_values_refused(value):
    assert refusal(_req(value=value), True, True, False, IDLE) == "invalid value"


def test_not_addressed_to_jetson():
    assert not addressed_to_jetson(_req(target=(1, 1)))


def test_typed_values():
    assert typed_value(800.0, True) == (800.0, None)
    assert typed_value(14.7, False) == (14.7, None)
    assert typed_value(25.5, True)[1] == "integer parameter: whole numbers only"
