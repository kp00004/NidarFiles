"""Tests for app/radio_link.RadioLink -- the real class, driven through a
fake serial port by a fake Jetson that speaks real MAVLink (pymavlink).
Proves the GCS side of the protocol; the real RF link and the real Jetson
still need the hardware test."""

from __future__ import annotations

import queue
import threading
import time

import pytest
from pymavlink.dialects.v20 import common as mavlink

import app.radio_link as radio_link
from app.radio_link import RadioLink, RadioUnavailable


class FakeJetson:
    """Answers each COMMAND_LONG with a COMMAND_ACK (nonce echoed in
    result_param2, reason in progress) and can emit heartbeats."""

    def __init__(self) -> None:
        self.mav = mavlink.MAVLink(None, srcSystem=1, srcComponent=191)
        self.parser = mavlink.MAVLink(None)
        self.received: list = []
        self.reply = True
        self.result = 0
        self.reason = 0
        self.ack_after_n = 1  # ACK only from the n-th copy of a nonce
        self._copies: dict[int, int] = {}

    def handle(self, data: bytes) -> list[bytes]:
        out = []
        for msg in self.parser.parse_buffer(data) or []:
            if msg.get_type() != "COMMAND_LONG":
                continue
            self.received.append(msg)
            nonce = int(msg.param2)
            self._copies[nonce] = self._copies.get(nonce, 0) + 1
            if self.reply and self._copies[nonce] >= self.ack_after_n:
                out.append(
                    self.mav.command_ack_encode(
                        31010, self.result, self.reason, nonce, msg.get_srcSystem(), msg.get_srcComponent()
                    ).pack(self.mav)
                )
        return out

    def heartbeat(self, state_code: int) -> bytes:
        return self.mav.heartbeat_encode(18, 8, 0, state_code, 4).pack(self.mav)


class FakeSerial:
    def __init__(self, jetson: FakeJetson) -> None:
        self.jetson = jetson
        self.inbox: "queue.Queue[bytes]" = queue.Queue()
        self.closed = False

    def read(self, _n: int) -> bytes:
        try:
            return self.inbox.get(timeout=0.05)
        except queue.Empty:
            return b""

    def write(self, data: bytes) -> int:
        for reply in self.jetson.handle(data):
            self.inbox.put(reply)
        return len(data)

    def close(self) -> None:
        self.closed = True


@pytest.fixture
def fast_retries(monkeypatch):
    monkeypatch.setattr(radio_link, "START_RETRY_INTERVAL_S", 0.15)
    monkeypatch.setattr(radio_link, "ABORT_RETRY_INTERVAL_S", 0.15)


def _link(jetson: FakeJetson):
    serial = FakeSerial(jetson)
    link = RadioLink("COM_TEST", 115200, serial_factory=lambda port, baud: serial)
    link.start()
    deadline = time.monotonic() + 2
    while not link.status()["port_open"] and time.monotonic() < deadline:
        time.sleep(0.01)
    return link, serial


def test_start_carries_mission_code_magic_version_and_is_accepted(fast_retries):
    jetson = FakeJetson()
    link, _ = _link(jetson)
    try:
        result = link.send_command("start", mission_code=1)
    finally:
        link.stop()
    assert result.accepted and result.attempts == 1
    msg = jetson.received[0]
    assert (msg.target_system, msg.target_component, msg.command) == (1, 191, 31010)
    assert (msg.param1, msg.param3, msg.param4, msg.param7) == (1.0, 4242.0, 1.0, 2.0)
    assert int(msg.param2) == result.nonce


def test_rejection_reports_reason_and_is_not_accepted(fast_retries):
    jetson = FakeJetson()
    jetson.result, jetson.reason = 1, 4
    link, _ = _link(jetson)
    try:
        result = link.send_command("start", mission_code=1)
    finally:
        link.stop()
    assert result.acked and not result.accepted
    assert result.reason == "FCU_NOT_CONNECTED"


def test_no_ack_retries_same_nonce_then_reports_not_acked(fast_retries):
    jetson = FakeJetson()
    jetson.reply = False
    link, _ = _link(jetson)
    try:
        result = link.send_command("abort")
    finally:
        link.stop()
    assert not result.acked and not result.accepted
    assert result.attempts == radio_link.ABORT_ATTEMPTS
    assert len({int(m.param2) for m in jetson.received}) == 1  # one nonce, resent


def test_ack_after_a_resend_is_accepted(fast_retries):
    jetson = FakeJetson()
    jetson.ack_after_n = 3
    link, _ = _link(jetson)
    try:
        result = link.send_command("start", mission_code=1)
    finally:
        link.stop()
    assert result.accepted and result.attempts == 3


def test_abort_is_matched_by_nonce_while_a_start_is_pending(fast_retries, monkeypatch):
    """START waits for an ACK that never comes; ABORT sent meanwhile must
    get its own ACK, and must not be confused with START's."""
    monkeypatch.setattr(radio_link, "START_RETRY_INTERVAL_S", 0.3)
    jetson = FakeJetson()
    original = jetson.handle

    def only_ack_abort(data):
        replies = original(data)
        return [r for r, m in zip(replies, jetson.received[-len(replies):] if replies else []) if m.param1 == 2.0]

    jetson.handle = only_ack_abort
    link, _ = _link(jetson)
    results = {}
    try:
        t = threading.Thread(target=lambda: results.setdefault("start", link.send_command("start", 1)))
        t.start()
        time.sleep(0.1)
        results["abort"] = link.send_command("abort")
        t.join()
    finally:
        link.stop()
    assert results["abort"].accepted
    assert not results["start"].acked


def test_heartbeat_marks_link_up_and_reports_mission_state():
    jetson = FakeJetson()
    link, serial = _link(jetson)
    try:
        assert link.status()["jetson_link_up"] is False
        serial.inbox.put(jetson.heartbeat(5))
        deadline = time.monotonic() + 2
        while not link.status()["jetson_link_up"] and time.monotonic() < deadline:
            time.sleep(0.02)
        status = link.status()
    finally:
        link.stop()
    assert status["jetson_link_up"] is True
    assert status["jetson_mission_state"] == "hovering"


def test_missing_port_raises_unavailable_never_pretends_to_send():
    def no_port(port, baud):
        raise OSError("could not open port 'COM5'")

    link = RadioLink("COM5", 115200, serial_factory=no_port)
    link.start()
    try:
        time.sleep(0.1)
        with pytest.raises(RadioUnavailable):
            link.send_command("start", 1)
        status = link.status()
    finally:
        link.stop()
    assert status["port_open"] is False
    assert "COM5" in status["port_error"]
