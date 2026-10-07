import json

import pytest

from rosbridge_sim.protocol import (
    COMMAND_TOPIC,
    ProtocolError,
    encode_publish,
    is_valid_command,
    parse_incoming,
)


def test_encode_publish_produces_expected_envelope():
    raw = encode_publish("/gcs/heartbeat", {"seq": 1})
    assert json.loads(raw) == {"op": "publish", "topic": "/gcs/heartbeat", "msg": {"seq": 1}}


def test_parse_subscribe():
    incoming = parse_incoming(json.dumps({"op": "subscribe", "topic": "/mission/state"}))
    assert incoming.op == "subscribe"
    assert incoming.topic == "/mission/state"


def test_parse_unsubscribe():
    incoming = parse_incoming(json.dumps({"op": "unsubscribe", "topic": "/mission/state"}))
    assert incoming.op == "unsubscribe"


def test_parse_publish_command():
    incoming = parse_incoming(json.dumps({"op": "publish", "topic": COMMAND_TOPIC, "msg": {"data": "start"}}))
    assert incoming.op == "publish"
    assert incoming.topic == COMMAND_TOPIC
    assert incoming.msg == {"data": "start"}


def test_parse_rejects_malformed_json():
    with pytest.raises(ProtocolError):
        parse_incoming("not json")


def test_parse_rejects_non_object_json():
    with pytest.raises(ProtocolError):
        parse_incoming(json.dumps([1, 2, 3]))


def test_parse_rejects_missing_op():
    with pytest.raises(ProtocolError):
        parse_incoming(json.dumps({"topic": "/foo"}))


def test_parse_rejects_unknown_op():
    with pytest.raises(ProtocolError):
        parse_incoming(json.dumps({"op": "delete_everything", "topic": "/foo"}))


def test_is_valid_command():
    assert is_valid_command("start")
    assert is_valid_command("abort")
    assert not is_valid_command("waypoint")
    assert not is_valid_command(None)
