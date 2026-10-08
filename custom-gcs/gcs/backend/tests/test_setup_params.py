"""Bench Setup page: Pixhawk parameter read/write over the radio.
Routes exist only with GCS_SETUP_ENABLED; RadioLink speaks real MAVLink
(pymavlink) to a fake Jetson."""

from __future__ import annotations

import queue
import time

from fastapi.testclient import TestClient
from pymavlink.dialects.v20 import common as mavlink

import app.radio_link as radio_link
from app.config import Settings
from app.main import create_app
from app.radio_link import ParamResult, RadioLink

from .fakes import FakeRadioLink, FakeRosBridgeClient


# -- routes ----------------------------------------------------------------------


def _app(setup: bool, radio=None):
    radio = radio or FakeRadioLink()
    settings = Settings(setup_enabled=setup, telemetry_source="rosbridge")
    return TestClient(create_app(client=FakeRosBridgeClient(), settings=settings, radio=radio)), radio


def test_setup_routes_do_not_exist_by_default():
    client, radio = _app(setup=False)
    assert client.get("/api/setup/param/BATT_ARM_VOLT").status_code == 404
    assert client.post("/api/setup/param", json={"name": "BATT_ARM_VOLT", "value": 13}).status_code in (404, 405)
    assert radio.param_calls == []
    assert client.get("/health").json()["setup_enabled"] is False


def test_read_and_write_when_enabled():
    client, radio = _app(setup=True)
    radio.params["RNGFND1_MAX_CM"] = 8.0
    assert client.get("/health").json()["setup_enabled"] is True
    assert client.get("/api/setup/param/RNGFND1_MAX_CM").json() == {
        "name": "RNGFND1_MAX_CM", "value": 8.0, "attempts": 1,
    }
    resp = client.post("/api/setup/param", json={"name": "RNGFND1_MAX_CM", "value": 800})
    assert resp.status_code == 200 and resp.json()["value"] == 800.0
    assert radio.param_calls == [("read", "RNGFND1_MAX_CM", None), ("set", "RNGFND1_MAX_CM", 800.0)]


def test_bad_name_never_reaches_the_radio():
    client, radio = _app(setup=True)
    assert client.get("/api/setup/param/rngfnd").status_code == 422
    assert client.post("/api/setup/param", json={"name": "BAD-NAME", "value": 1}).status_code == 422
    assert radio.param_calls == []


def test_refusal_is_409_with_the_jetson_reason():
    client, radio = _app(setup=True)
    radio.param_reply = ParamResult(
        name="BATT_ARM_VOLT", replied=True, error="vehicle armed (or armed state unknown) -- writes refused", attempts=1
    )
    resp = client.post("/api/setup/param", json={"name": "BATT_ARM_VOLT", "value": 13})
    assert resp.status_code == 409 and "vehicle armed" in resp.json()["detail"]


def test_no_reply_is_504():
    client, radio = _app(setup=True)
    radio.param_reply = ParamResult(name="BATT_ARM_VOLT", replied=False, attempts=3)
    assert client.get("/api/setup/param/BATT_ARM_VOLT").status_code == 504


def test_radio_unavailable_is_503():
    client, radio = _app(setup=True)
    radio.unavailable = "radio port COM5 is not open"
    assert client.get("/api/setup/param/BATT_ARM_VOLT").status_code == 503


# -- RadioLink over real MAVLink ---------------------------------------------------


class ParamJetson:
    """Answers PARAM_REQUEST_READ / PARAM_SET like radio_command_node."""

    def __init__(self, writes_enabled=True):
        self.mav = mavlink.MAVLink(None, srcSystem=1, srcComponent=191)
        self.parser = mavlink.MAVLink(None)
        self.params = {"RNGFND1_MAX_CM": 8.0}
        self.writes_enabled = writes_enabled
        self.silent = False

    def handle(self, data: bytes) -> list[bytes]:
        out = []
        for msg in self.parser.parse_buffer(data) or []:
            kind = msg.get_type()
            if self.silent or kind not in ("PARAM_REQUEST_READ", "PARAM_SET"):
                continue
            name = msg.param_id
            if kind == "PARAM_SET":
                if not self.writes_enabled:
                    text = f"PARAM: {name}: writes disabled (start the Jetson with start_jetson.sh --setup)"
                    chunks = [text[i:i + 50] for i in range(0, len(text), 50)]
                    if len(chunks[-1]) == 50:
                        chunks.append("")
                    for seq, chunk in enumerate(chunks):
                        out.append(self.mav.statustext_encode(4, chunk.encode(), 0x8001, seq).pack(self.mav))
                    continue
                self.params[name] = msg.param_value
            out.append(self.mav.param_value_encode(name.encode(), self.params[name], 6, 1, 65535).pack(self.mav))
        return out


class Serial:
    def __init__(self, jetson):
        self.jetson = jetson
        self.inbox: "queue.Queue[bytes]" = queue.Queue()

    def read(self, _n):
        try:
            return self.inbox.get(timeout=0.05)
        except queue.Empty:
            return b""

    def write(self, data):
        for reply in self.jetson.handle(data):
            self.inbox.put(reply)
        return len(data)

    def close(self):
        pass


def _link(jetson):
    serial = Serial(jetson)
    link = RadioLink("COM_TEST", 115200, serial_factory=lambda p, b: serial)
    link.start()
    deadline = time.monotonic() + 2
    while not link.status()["port_open"] and time.monotonic() < deadline:
        time.sleep(0.01)
    return link


def test_radio_link_read_and_write():
    jetson = ParamJetson()
    link = _link(jetson)
    try:
        read = link.read_param("RNGFND1_MAX_CM")
        written = link.set_param("RNGFND1_MAX_CM", 800)
    finally:
        link.stop()
    assert (read.replied, read.value, read.error) == (True, 8.0, None)
    assert (written.replied, written.value) == (True, 800.0)
    assert jetson.params["RNGFND1_MAX_CM"] == 800.0


def test_radio_link_reports_refusal_reason_from_chunked_text():
    link = _link(ParamJetson(writes_enabled=False))
    try:
        result = link.set_param("RNGFND1_MAX_CM", 800)
    finally:
        link.stop()
    assert result.replied and result.value is None
    assert result.error == "writes disabled (start the Jetson with start_jetson.sh --setup)"


def test_radio_link_no_reply_retries_then_gives_up(monkeypatch):
    monkeypatch.setattr(radio_link, "PARAM_REPLY_TIMEOUT_S", 0.1)
    jetson = ParamJetson()
    jetson.silent = True
    link = _link(jetson)
    try:
        result = link.read_param("RNGFND1_MAX_CM")
    finally:
        link.stop()
    assert not result.replied and result.attempts == radio_link.PARAM_ATTEMPTS
