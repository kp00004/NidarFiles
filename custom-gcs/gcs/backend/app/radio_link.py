"""The GCS end of the MicroLR900 radio -- the ONLY link between the GCS
and the drone: START and ABORT go out on it, and the Jetson's telemetry
comes back on it (handed to app/radio_telemetry.py). There is no Wi-Fi
link.

    FastAPI route -> RadioLink.send_command() -> COM5 -> MicroLR900 )))
      -> MicroLR900 on the Jetson -> radio_command_node -> hover mission

RadioLink owns the serial port: a reader thread parses MAVLink (pymavlink)
for the Jetson's COMMAND_ACKs, heartbeats and telemetry, a heartbeat thread sends the
GCS heartbeat at 1 Hz, and the port is reopened automatically if it
disappears (radio unplugged, wrong COM port at startup).

send_command() resends the same nonce until the Jetson ACKs or the
attempts run out, and reports exactly what happened -- it never reports
success without an ACK from the Jetson.
"""

from __future__ import annotations

import logging
import random
import threading
import time
from dataclasses import asdict, dataclass
from typing import Callable, Optional

from pymavlink.dialects.v20 import common as mavlink

from .radio_protocol import (
    GCS_COMPONENT_ID,
    FCU_RELAY_COMPONENT_ID,
    GCS_SYSTEM_ID,
    JETSON_COMPONENT_ID,
    JETSON_SYSTEM_ID,
    MAV_CMD_USER_1,
    MAV_RESULT_ACCEPTED,
    MAX_NONCE,
    MISSION_STATE_NAMES,
    NIDAR_ABORT,
    NIDAR_MAGIC,
    NIDAR_START,
    PROTOCOL_VERSION,
    REASON_NAMES,
)

logger = logging.getLogger(__name__)

# TODO(hardware): tune after the first RF test (round-trip time of the
# MicroLR900 pair at HIGH rate is not measured yet).
START_ATTEMPTS = 5
START_RETRY_INTERVAL_S = 0.6
ABORT_ATTEMPTS = 5
ABORT_RETRY_INTERVAL_S = 0.4

JETSON_LINK_TIMEOUT_S = 3.0
_REOPEN_INTERVAL_S = 2.0


class RadioUnavailable(RuntimeError):
    """The command could not be sent at all (port closed / radio disabled)."""


@dataclass
class RadioResult:
    command: str
    mission_code: int
    nonce: int
    attempts: int
    acked: bool
    result: Optional[int] = None
    reason: Optional[str] = None
    time: str = ""

    @property
    def accepted(self) -> bool:
        return self.acked and self.result == MAV_RESULT_ACCEPTED


def _open_serial(port: str, baud: int):
    import serial  # pyserial

    return serial.Serial(port, baud, timeout=0.1, write_timeout=1.0)


class RadioLink:
    def __init__(
        self,
        port: str,
        baud: int,
        serial_factory: Callable[[str, int], object] = _open_serial,
        telemetry=None,
    ) -> None:
        """telemetry: optional app.radio_telemetry.RadioTelemetryClient that
        receives every message from the Jetson (1/191) and its FCU relay (1/1)."""
        self._port = port
        self._telemetry = telemetry
        self._baud = baud
        self._serial_factory = serial_factory
        self._serial = None
        self._port_error: Optional[str] = None
        self._write_lock = threading.Lock()
        self._mav = mavlink.MAVLink(None, srcSystem=GCS_SYSTEM_ID, srcComponent=GCS_COMPONENT_ID)
        self._mav.robust_parsing = True
        self._pending: dict[int, dict] = {}
        self._pending_lock = threading.Lock()
        self._jetson_heartbeat_at: Optional[float] = None
        self._jetson_state_code: Optional[int] = None
        self._last_command: Optional[RadioResult] = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # -- lifecycle --------------------------------------------------------------

    def start(self) -> None:
        for target in (self._read_loop, self._heartbeat_loop):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._close(None)

    def _ensure_open(self) -> bool:
        if self._serial is not None:
            return True
        try:
            self._serial = self._serial_factory(self._port, self._baud)
        except Exception as exc:  # noqa: BLE001 -- missing/busy port
            if self._port_error != str(exc):
                logger.error("radio: cannot open %s @ %s: %s", self._port, self._baud, exc)
            self._port_error = str(exc)
            return False
        self._port_error = None
        logger.info("radio: %s open @ %s", self._port, self._baud)
        return True

    def _close(self, why: Optional[str]) -> None:
        handle, self._serial = self._serial, None
        if handle is not None:
            try:
                handle.close()
            except Exception:  # noqa: BLE001
                pass
        if why:
            self._port_error = why
            logger.error("radio: %s closed: %s", self._port, why)

    # -- threads ----------------------------------------------------------------

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            if not self._ensure_open():
                self._stop.wait(_REOPEN_INTERVAL_S)
                continue
            try:
                data = self._serial.read(256)
            except Exception as exc:  # noqa: BLE001 -- unplugged
                self._close(repr(exc))
                continue
            if not data:
                continue
            for msg in self._mav.parse_buffer(data) or []:
                self._on_message(msg)

    def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._write(
                    self._mav.heartbeat_encode(
                        mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE
                    )
                )
            except RadioUnavailable:
                pass
            self._stop.wait(1.0)

    def _on_message(self, msg) -> None:
        source = (msg.get_srcSystem(), msg.get_srcComponent())
        if source not in (
            (JETSON_SYSTEM_ID, JETSON_COMPONENT_ID),
            (JETSON_SYSTEM_ID, FCU_RELAY_COMPONENT_ID),
        ):
            return
        if self._telemetry is not None:
            try:
                self._telemetry.on_message(msg)
            except Exception:  # noqa: BLE001 -- a telemetry bug must never stop ACK handling
                logger.exception("radio: telemetry message %s not handled", msg.get_type())
        if source[1] != JETSON_COMPONENT_ID:
            return
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            self._jetson_heartbeat_at = time.monotonic()
            self._jetson_state_code = msg.custom_mode
        elif kind == "COMMAND_ACK" and msg.command == MAV_CMD_USER_1:
            with self._pending_lock:
                waiter = self._pending.get(msg.result_param2)
                if waiter is not None and not waiter["event"].is_set():
                    waiter["result"] = msg.result
                    waiter["reason"] = REASON_NAMES.get(msg.progress, f"REASON_{msg.progress}")
                    waiter["event"].set()

    def _write(self, message) -> None:
        with self._write_lock:
            if self._serial is None:
                raise RadioUnavailable(
                    f"radio port {self._port} is not open ({self._port_error or 'not connected'})"
                )
            frame = message.pack(self._mav)
            try:
                self._serial.write(frame)
            except Exception as exc:  # noqa: BLE001
                self._close(repr(exc))
                raise RadioUnavailable(f"radio write failed: {exc}") from exc

    # -- commands ---------------------------------------------------------------

    def send_command(self, command: str, mission_code: int = 0) -> RadioResult:
        if command not in ("start", "abort"):
            raise ValueError(f"invalid command {command!r}")
        attempts, interval = (
            (START_ATTEMPTS, START_RETRY_INTERVAL_S)
            if command == "start"
            else (ABORT_ATTEMPTS, ABORT_RETRY_INTERVAL_S)
        )
        nonce = random.randint(1, MAX_NONCE)
        waiter = {"event": threading.Event(), "result": None, "reason": None}
        with self._pending_lock:
            self._pending[nonce] = waiter
        message = self._mav.command_long_encode(
            JETSON_SYSTEM_ID, JETSON_COMPONENT_ID, MAV_CMD_USER_1, 0,
            NIDAR_START if command == "start" else NIDAR_ABORT,
            float(nonce), NIDAR_MAGIC, float(mission_code), 0.0, 0.0, float(PROTOCOL_VERSION),
        )
        sent = 0
        try:
            for sent in range(1, attempts + 1):
                self._write(message)
                logger.warning(
                    "radio: %s mission_code=%s nonce=%s sent (attempt %s/%s)",
                    command.upper(), mission_code, nonce, sent, attempts,
                )
                if waiter["event"].wait(interval):
                    break
        finally:
            with self._pending_lock:
                self._pending.pop(nonce, None)
        result = RadioResult(
            command=command,
            mission_code=mission_code,
            nonce=nonce,
            attempts=sent,
            acked=waiter["event"].is_set(),
            result=waiter["result"],
            reason=waiter["reason"],
            time=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )
        logger.warning("radio: %s result %s", command.upper(), result)
        self._last_command = result
        return result

    def status(self) -> dict:
        age = (
            None
            if self._jetson_heartbeat_at is None
            else round(time.monotonic() - self._jetson_heartbeat_at, 1)
        )
        return {
            "enabled": True,
            "port": self._port,
            "baud": self._baud,
            "port_open": self._serial is not None,
            "port_error": self._port_error,
            "jetson_link_up": age is not None and age < JETSON_LINK_TIMEOUT_S,
            "jetson_heartbeat_age_s": age,
            "jetson_mission_state": (
                None
                if self._jetson_state_code is None
                else MISSION_STATE_NAMES.get(self._jetson_state_code, "unknown")
            ),
            "last_command": None if self._last_command is None else asdict(self._last_command),
        }


class DisabledRadioLink:
    """GCS_RADIO_ENABLED=false (local UI development without a radio):
    every command fails loudly -- it never pretends to have sent anything."""

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def send_command(self, command: str, mission_code: int = 0) -> RadioResult:
        raise RadioUnavailable("command radio is disabled (GCS_RADIO_ENABLED=false)")

    def status(self) -> dict:
        return {
            "enabled": False,
            "port": None,
            "baud": None,
            "port_open": False,
            "port_error": "disabled",
            "jetson_link_up": False,
            "jetson_heartbeat_age_s": None,
            "jetson_mission_state": None,
            "last_command": None,
        }
