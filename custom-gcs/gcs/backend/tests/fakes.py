"""Test double for RosBridgeClient -- same interface, no real networking.
Lets the route/schema layer be tested fast and in isolation; see
test_backend_against_sim.py for the real end-to-end counterpart."""

from __future__ import annotations

from typing import Any

from app.ros_client import (
    FLIGHT_TEST_STATUS_TOPIC,
    MULTI_STEP_TEST_STATUS_TOPIC,
    PERCEPTION_DETECTIONS_TOPIC,
    PERCEPTION_STATUS_TOPIC,
    VALID_COMMANDS,
    VALID_SIMULATION_COMMANDS,
)


class FakeRosBridgeClient:
    def __init__(self, connected: bool = True) -> None:
        self.is_connected = connected
        self._latest: dict[str, Any] = {}
        self._ages: dict[str, float] = {}
        self._survivors: list[dict] = []
        self._statustext: list[dict] = []
        self.published_commands: list[str] = []
        self.published_simulation_commands: list[str] = []
        self.published_mission_selects: list[tuple[str, str]] = []
        self._publish_exception: Exception | None = None
        self._publish_simulation_exception: Exception | None = None
        self._publish_mission_select_exception: Exception | None = None

    def set_latest(self, topic: str, message: dict) -> None:
        self._latest[topic] = message

    def latest(self, topic: str) -> Any:
        return self._latest.get(topic)

    def set_perception_detections(self, data: dict) -> None:
        self._latest[PERCEPTION_DETECTIONS_TOPIC] = data

    def set_perception_status(self, data: dict) -> None:
        self._latest[PERCEPTION_STATUS_TOPIC] = data

    def set_flight_test_status(self, data: dict) -> None:
        self._latest[FLIGHT_TEST_STATUS_TOPIC] = data

    def set_multi_step_flight_test_status(self, data: dict) -> None:
        self._latest[MULTI_STEP_TEST_STATUS_TOPIC] = data

    def set_age_s(self, topic: str, age_s: float | None) -> None:
        self._ages[topic] = age_s

    def age_s(self, topic: str) -> float | None:
        return self._ages.get(topic)

    def set_survivors(self, survivors: list[dict]) -> None:
        self._survivors = survivors

    def survivors(self) -> list[dict]:
        return self._survivors

    def set_statustext_history(self, history: list[dict]) -> None:
        self._statustext = history

    def statustext_history(self) -> list[dict]:
        return self._statustext

    def fail_publish_with(self, exc: Exception) -> None:
        """Make subsequent publish_command calls raise `exc` instead of
        recording the command -- mirrors how the real RosBridgeClient
        raises RuntimeError("not connected to rosbridge") when its
        connection to rosbridge is unavailable (see
        app/ros_client.py:publish_command), so API-level tests can
        exercise that downstream-failure path without a real dead
        connection."""
        self._publish_exception = exc

    def publish_command(self, command: str) -> None:
        if command not in VALID_COMMANDS:
            raise ValueError(f"invalid command: {command!r}")
        if self._publish_exception is not None:
            raise self._publish_exception
        self.published_commands.append(command)

    def fail_publish_simulation_with(self, exc: Exception) -> None:
        self._publish_simulation_exception = exc

    def publish_simulation_command(self, command: str) -> None:
        """The fake's mirror of the real client's simulation-only publish
        method -- deliberately separate from publish_command()/
        published_commands above, so a test can assert a simulation route
        never touched the real command path."""
        if command not in VALID_SIMULATION_COMMANDS:
            raise ValueError(f"invalid simulation command: {command!r}")
        if self._publish_simulation_exception is not None:
            raise self._publish_simulation_exception
        self.published_simulation_commands.append(command)

    def fail_publish_mission_select_with(self, exc: Exception) -> None:
        self._publish_mission_select_exception = exc

    def publish_mission_select(self, mission_id: str, scenario_id: str) -> None:
        """The fake's mirror of the real client's
        publish_mission_select() -- routing metadata, deliberately
        tracked separately from published_commands above so tests can
        assert the exact publish order (mission_select before start)."""
        if not mission_id or not scenario_id:
            raise ValueError("mission_id and scenario_id must be non-empty")
        if self._publish_mission_select_exception is not None:
            raise self._publish_mission_select_exception
        self.published_mission_selects.append((mission_id, scenario_id))


class FakeRadioLink:
    """Test double for app/radio_link.RadioLink -- records commands instead
    of writing to a serial port, and answers with a configurable Jetson
    response (accepted by default)."""

    def __init__(self) -> None:
        from app.radio_link import RadioResult, RadioUnavailable

        self._result_cls = RadioResult
        self._unavailable_cls = RadioUnavailable
        self.sent: list[tuple[str, int]] = []
        self.acked = True
        self.result = 0  # MAV_RESULT_ACCEPTED
        self.reason = "OK"
        self.unavailable: str | None = None
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def send_command(self, command: str, mission_code: int = 0):
        if self.unavailable is not None:
            raise self._unavailable_cls(self.unavailable)
        self.sent.append((command, mission_code))
        return self._result_cls(
            command=command,
            mission_code=mission_code,
            nonce=1234,
            attempts=1 if self.acked else 5,
            acked=self.acked,
            result=self.result if self.acked else None,
            reason=self.reason if self.acked else None,
        )

    def status(self) -> dict:
        return {
            "enabled": True,
            "port": "COM_TEST",
            "baud": 115200,
            "port_open": True,
            "port_error": None,
            "jetson_link_up": True,
            "jetson_heartbeat_age_s": 0.4,
            "jetson_mission_state": "idle",
            "last_command": None,
        }
