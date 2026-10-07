"""Unit tests for RosBridgeClient's own pure logic -- the cache/handler
behavior in app/ros_client.py that test_api_with_fake_client.py can't
reach (those tests exercise the API layer against FakeRosBridgeClient,
which re-implements this logic rather than running it) and that
test_backend_against_sim.py only reaches indirectly through a live sim
process.

Constructing `RosBridgeClient(host, port)` does not open a socket or
touch the network: `roslibpy.Ros.__init__` calls `factory.connect()`,
but that only schedules a connection attempt on a Twisted reactor that
is never started unless `RosBridgeClient.connect()` (which calls
`self._ros.run(...)`) is called. None of the tests below call
`.connect()`, so they have no ROS/network dependency, matching this
repo's pure-logic-test pattern (see onboard-autonomy/test_state_machine.py).
"""

from __future__ import annotations

import json

import pytest

from app.ros_client import (
    PERCEPTION_DETECTIONS_TOPIC,
    PERCEPTION_STATUS_TOPIC,
    STATUSTEXT_TOPIC,
    SURVIVORS_TOPIC,
    TELEMETRY_STATE_TOPIC,
    VALID_COMMANDS,
    VALID_SIMULATION_COMMANDS,
    DisabledRosBridgeClient,
    RosBridgeClient,
)


def make_client() -> RosBridgeClient:
    # Port is never dialed (see module docstring), so any value is fine.
    return RosBridgeClient(host="127.0.0.1", port=9999)


def feed(client: RosBridgeClient, topic: str, message: dict) -> None:
    """Directly invoke the same per-topic handler `connect()` would wire
    up via `sub.subscribe(...)` -- this is the seam a real rosbridge
    message arrives through, without needing an actual subscription."""
    client._make_handler(topic)(message)


class TestPublishCommandGuards:
    def test_rejects_a_command_outside_start_abort(self):
        client = make_client()
        client._command_topic = object()  # pretend "advertised" so the connection check passes
        with pytest.raises(ValueError, match="invalid command"):
            client.publish_command("land")

    def test_rejects_empty_string(self):
        client = make_client()
        client._command_topic = object()
        with pytest.raises(ValueError):
            client.publish_command("")

    def test_raises_runtime_error_when_never_connected(self):
        """A never-connected (or disconnected-before-advertise) client
        has `_command_topic is None` -- publish_command must fail loudly
        rather than silently drop the command. This is the one error path
        this method can raise for an otherwise-valid command."""
        client = make_client()
        assert client._command_topic is None
        with pytest.raises(RuntimeError, match="not connected"):
            client.publish_command("start")

    def test_valid_commands_constant_matches_what_publish_command_accepts(self):
        # Guards against VALID_COMMANDS drifting out of sync with the
        # actual accept-list logic above.
        assert set(VALID_COMMANDS) == {"start", "abort"}


class TestCacheBeforeAnyMessage:
    def test_latest_is_none_for_an_unseen_topic(self):
        client = make_client()
        assert client.latest("/mavros/battery") is None

    def test_age_s_is_none_for_an_unseen_topic(self):
        client = make_client()
        assert client.age_s("/gcs/heartbeat") is None

    def test_survivors_is_empty_before_any_detection(self):
        client = make_client()
        assert client.survivors() == []

    def test_statustext_history_is_empty_before_any_message(self):
        client = make_client()
        assert client.statustext_history() == []


class TestGenericTopicCaching:
    def test_latest_message_overwrites_the_previous_one_on_the_same_topic(self):
        client = make_client()
        feed(client, "/mavros/battery", {"voltage": 11.0})
        feed(client, "/mavros/battery", {"voltage": 12.5})
        assert client.latest("/mavros/battery") == {"voltage": 12.5}

    def test_different_topics_are_cached_independently(self):
        client = make_client()
        feed(client, "/mavros/battery", {"voltage": 12.5})
        feed(client, "/mission/state", {"data": "searching"})
        assert client.latest("/mavros/battery") == {"voltage": 12.5}
        assert client.latest("/mission/state") == {"data": "searching"}

    def test_age_s_becomes_a_small_nonnegative_number_immediately_after_a_message(self):
        client = make_client()
        feed(client, "/gcs/heartbeat", {})
        age = client.age_s("/gcs/heartbeat")
        assert age is not None
        assert 0 <= age < 1.0


class TestSurvivorCaching:
    def test_survivors_are_keyed_by_survivor_id_not_appended(self):
        """A repeated detection for the same survivor_id (e.g. the
        drone re-observes survivor 1 on a later pass) must update that
        survivor's record in place, not create a duplicate entry --
        otherwise /api/survivors would grow unboundedly and show stale
        duplicate rows for the same person."""
        client = make_client()
        feed(client, SURVIVORS_TOPIC, {"survivor_id": 1, "x": 1.0, "y": 1.0, "confidence": 0.5})
        feed(client, SURVIVORS_TOPIC, {"survivor_id": 1, "x": 1.2, "y": 1.1, "confidence": 0.9})

        survivors = client.survivors()

        assert len(survivors) == 1
        assert survivors[0] == {"survivor_id": 1, "x": 1.2, "y": 1.1, "confidence": 0.9}

    def test_survivors_are_returned_sorted_by_survivor_id(self):
        client = make_client()
        feed(client, SURVIVORS_TOPIC, {"survivor_id": 3, "x": 0.0, "y": 0.0, "confidence": 0.5})
        feed(client, SURVIVORS_TOPIC, {"survivor_id": 1, "x": 0.0, "y": 0.0, "confidence": 0.5})
        feed(client, SURVIVORS_TOPIC, {"survivor_id": 2, "x": 0.0, "y": 0.0, "confidence": 0.5})

        ids = [s["survivor_id"] for s in client.survivors()]

        assert ids == [1, 2, 3]


class TestJsonStringTopicConversion:
    """RosBridgeClient._make_handler special-cases std_msgs/String topics
    that carry a JSON-encoded contract (see app/ros_client.py's handler
    docstring) -- caching the *parsed dict*, not the raw {"data": "..."}
    String wrapper, and never letting a malformed payload crash the
    roslibpy callback thread. This is the actual conversion the perception
    endpoints' response shape depends on (app/main.py just does
    ros_client.latest(topic) and expects a dict, not a String wrapper) --
    FakeRosBridgeClient in tests/fakes.py bypasses this logic entirely
    (its set_perception_detections()/set_perception_status() set the
    parsed dict directly), so this is the only place it's exercised."""

    def test_perception_detections_message_is_parsed_from_the_json_string_payload(self):
        client = make_client()
        payload = {
            "schema_version": 1,
            "frame_width": 640,
            "frame_height": 480,
            "timestamp": 1.0,
            "detections": [],
        }
        feed(client, PERCEPTION_DETECTIONS_TOPIC, {"data": json.dumps(payload)})

        assert client.latest(PERCEPTION_DETECTIONS_TOPIC) == payload

    def test_perception_status_message_is_parsed_from_the_json_string_payload(self):
        client = make_client()
        payload = {"camera_connected": True, "person_count": 3, "detector_backend": "mock"}
        feed(client, PERCEPTION_STATUS_TOPIC, {"data": json.dumps(payload)})

        assert client.latest(PERCEPTION_STATUS_TOPIC) == payload

    def test_telemetry_state_message_is_parsed_from_the_json_string_payload(self):
        client = make_client()
        payload = {"autonomy": {"state": "searching"}, "mapping": {"available": True}}
        feed(client, TELEMETRY_STATE_TOPIC, {"data": json.dumps(payload)})

        assert client.latest(TELEMETRY_STATE_TOPIC) == payload

    def test_malformed_json_on_perception_detections_is_dropped_not_cached(self):
        client = make_client()
        feed(client, PERCEPTION_DETECTIONS_TOPIC, {"data": "{not valid json"})

        assert client.latest(PERCEPTION_DETECTIONS_TOPIC) is None

    def test_malformed_json_does_not_clobber_a_previously_cached_good_value(self):
        client = make_client()
        good = {"frame_width": 640, "frame_height": 480, "timestamp": 1.0, "detections": []}
        feed(client, PERCEPTION_DETECTIONS_TOPIC, {"data": json.dumps(good)})
        feed(client, PERCEPTION_DETECTIONS_TOPIC, {"data": "{not valid json"})

        assert client.latest(PERCEPTION_DETECTIONS_TOPIC) == good

    def test_missing_data_key_on_perception_status_is_dropped_not_crashed(self):
        client = make_client()
        feed(client, PERCEPTION_STATUS_TOPIC, {"unexpected_shape": True})  # KeyError guarded

        assert client.latest(PERCEPTION_STATUS_TOPIC) is None

    def test_non_string_data_field_on_perception_status_is_dropped_not_crashed(self):
        client = make_client()
        feed(client, PERCEPTION_STATUS_TOPIC, {"data": 12345})  # TypeError guarded (json.loads(int))

        assert client.latest(PERCEPTION_STATUS_TOPIC) is None


class TestDisabledRosBridgeClient:
    """DisabledRosBridgeClient (app/ros_client.py) -- what
    app/main.py's create_app() wires in when GCS_ROS_ENABLED=false. See
    tests/test_ros_disabled.py for the API-level behavior this drives;
    these are the class's own pure-logic guarantees: never connected,
    never touches the network, every read degrades to empty/None, every
    write raises rather than pretending to succeed."""

    def test_is_connected_is_always_false(self):
        client = DisabledRosBridgeClient()
        assert client.is_connected is False

    def test_connect_and_disconnect_are_harmless_no_ops(self):
        client = DisabledRosBridgeClient()
        client.connect()  # must not raise, must not touch the network
        client.disconnect()
        assert client.is_connected is False

    def test_latest_is_always_none(self):
        client = DisabledRosBridgeClient()
        assert client.latest("/mavros/battery") is None

    def test_age_s_is_always_none(self):
        client = DisabledRosBridgeClient()
        assert client.age_s("/gcs/heartbeat") is None

    def test_survivors_is_always_empty(self):
        client = DisabledRosBridgeClient()
        assert client.survivors() == []

    def test_statustext_history_is_always_empty(self):
        client = DisabledRosBridgeClient()
        assert client.statustext_history() == []

    def test_publish_command_raises_runtime_error_not_a_fake_success(self):
        """The core safety property: a disabled client must never let a
        caller believe "start" reached anything real."""
        client = DisabledRosBridgeClient()
        with pytest.raises(RuntimeError, match="ROS is disabled"):
            client.publish_command("start")

    def test_publish_command_still_validates_the_command_first(self):
        client = DisabledRosBridgeClient()
        with pytest.raises(ValueError, match="invalid command"):
            client.publish_command("land")

    def test_publish_simulation_command_raises_runtime_error(self):
        client = DisabledRosBridgeClient()
        with pytest.raises(RuntimeError, match="ROS is disabled"):
            client.publish_simulation_command("run")

    def test_publish_simulation_command_still_validates_first(self):
        client = DisabledRosBridgeClient()
        with pytest.raises(ValueError, match="invalid simulation command"):
            client.publish_simulation_command("start")

    def test_publish_mission_select_raises_runtime_error(self):
        client = DisabledRosBridgeClient()
        with pytest.raises(RuntimeError, match="ROS is disabled"):
            client.publish_mission_select("flight_test", "hover")

    def test_publish_mission_select_still_validates_first(self):
        client = DisabledRosBridgeClient()
        with pytest.raises(ValueError, match="must be non-empty"):
            client.publish_mission_select("", "hover")

    def test_valid_simulation_commands_constant_still_applies(self):
        assert set(VALID_SIMULATION_COMMANDS) == {"run", "reset"}


class TestStatusTextHistory:
    def test_statustext_messages_accumulate_in_receipt_order(self):
        client = make_client()
        feed(client, STATUSTEXT_TOPIC, {"severity": 6, "text": "first"})
        feed(client, STATUSTEXT_TOPIC, {"severity": 4, "text": "second"})

        history = client.statustext_history()

        assert [m["text"] for m in history] == ["first", "second"]

    def test_statustext_history_is_capped_at_ten_dropping_the_oldest(self):
        """Mirrors onboard-autonomy/flight_command.py's own bounded
        STATUSTEXT history -- must not grow without bound over a long
        mission, and must keep the *most recent* ten, not the first ten."""
        client = make_client()
        for i in range(15):
            feed(client, STATUSTEXT_TOPIC, {"severity": 6, "text": f"msg-{i}"})

        history = client.statustext_history()

        assert len(history) == 10
        assert [m["text"] for m in history] == [f"msg-{i}" for i in range(5, 15)]
