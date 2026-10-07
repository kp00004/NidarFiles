"""Minimal rosbridge v2 wire protocol: just enough of it to be a drop-in
stand-in for `rosbridge_server` from a client's (roslibjs's) point of view.

Reference: docs/COMMUNICATION.md §2.2 for the topic table this protocol
carries. Only the operations this project's clients actually need are
implemented (subscribe / unsubscribe / publish) — no service calls, no
compression, no `id` request/response matching. If a real integration
later needs one of those, that's a deliberate protocol extension to make,
not something to guess at here.
"""

from __future__ import annotations

import json
from dataclasses import dataclass

# Topic names, matching docs/DATA_MODELS.md exactly.
MISSION_STATE_TOPIC = "/mission/state"
BATTERY_TOPIC = "/mavros/battery"
POSE_TOPIC = "/mavros/local_position/pose"
# `/map` supersedes the never-implemented `/slam/map` placeholder -- see
# docs/DATA_MODELS.md's changelog note (NIDAR Autonomy Migration).
MAP_TOPIC = "/map"
SURVIVORS_TOPIC = "/vision/survivors"
HEARTBEAT_TOPIC = "/gcs/heartbeat"
COMMAND_TOPIC = "/gcs/command"

# Mapping/exploration/telemetry topics, added alongside the migration --
# see CHECKPOINT/docs/gcs_telemetry_contract.md for the full shape.
COVERAGE_GRID_TOPIC = "/coverage_grid"
PLANNED_PATH_TOPIC = "/planned_path"
TELEMETRY_STATE_TOPIC = "/telemetry/state"

# Drone -> GCS topics this simulator publishes. Video is deliberately
# excluded (docs/DECISIONS.md D-6 — not sent over rosbridge).
PUBLISHED_TOPICS = (
    MISSION_STATE_TOPIC,
    BATTERY_TOPIC,
    POSE_TOPIC,
    MAP_TOPIC,
    SURVIVORS_TOPIC,
    HEARTBEAT_TOPIC,
    COVERAGE_GRID_TOPIC,
    PLANNED_PATH_TOPIC,
    TELEMETRY_STATE_TOPIC,
)

_VALID_OPS = frozenset({"subscribe", "unsubscribe", "publish", "advertise", "unadvertise"})
_VALID_COMMANDS = frozenset({"start", "abort"})


class ProtocolError(ValueError):
    """Raised for a malformed or unsupported incoming rosbridge message."""


@dataclass(frozen=True)
class IncomingMessage:
    op: str
    topic: str | None = None
    msg: dict | None = None
    msg_type: str | None = None


def parse_incoming(raw: str) -> IncomingMessage:
    """Parse a client->server rosbridge message. Raises ProtocolError on
    anything malformed, rather than letting a bad client message crash the
    connection handler."""
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError) as exc:
        raise ProtocolError(f"invalid JSON: {exc}") from exc

    if not isinstance(data, dict):
        raise ProtocolError("top-level message must be a JSON object")

    op = data.get("op")
    if op not in _VALID_OPS:
        raise ProtocolError(f"unknown or missing op: {op!r}")

    return IncomingMessage(
        op=op,
        topic=data.get("topic"),
        msg=data.get("msg"),
        msg_type=data.get("type"),
    )


def encode_publish(topic: str, msg: dict) -> str:
    """Encode a server->client `publish` message (the only op this
    simulator ever sends)."""
    return json.dumps({"op": "publish", "topic": topic, "msg": msg})


def is_valid_command(value: object) -> bool:
    return value in _VALID_COMMANDS
