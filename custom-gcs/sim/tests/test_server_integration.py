"""End-to-end tests: real WebSocket client against a real SimServer, over
a real (loopback) socket. This is the test that actually proves the
simulator is a valid stand-in for rosbridge_server from a client's point
of view — everything else in this test suite is a supporting unit test.
"""

import asyncio
import json

import pytest
import websockets

from rosbridge_sim.mission import MissionConfig, MissionSimulator
from rosbridge_sim.server import SimServer

FAST_RATES = dict(
    heartbeat_interval=0.05,
    battery_interval=0.05,
    pose_interval=0.05,
    map_interval=0.05,
    check_interval=0.05,
)


async def _running_server(**overrides) -> tuple[SimServer, int]:
    config = MissionConfig(width=5, height=5, duration_s=1.0, num_survivors=2, seed=1)
    mission = MissionSimulator(config)
    server = SimServer(mission, host="127.0.0.1", port=0, **{**FAST_RATES, **overrides})
    port = await server.start()
    return server, port


async def _recv_json(ws, timeout: float = 2.0) -> dict:
    raw = await asyncio.wait_for(ws.recv(), timeout=timeout)
    return json.loads(raw)


async def _subscribe(ws, topic: str) -> None:
    await ws.send(json.dumps({"op": "subscribe", "topic": topic}))


async def _publish_command(ws, command: str) -> None:
    await ws.send(json.dumps({"op": "publish", "topic": "/gcs/command", "msg": {"data": command}}))


@pytest.mark.asyncio
async def test_heartbeat_reaches_a_subscribed_client():
    server, port = await _running_server()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await _subscribe(ws, "/gcs/heartbeat")
            msg = await _recv_json(ws)
            assert msg["op"] == "publish"
            assert msg["topic"] == "/gcs/heartbeat"
            assert "stamp" in msg["msg"]
    finally:
        await server.stop()


@pytest.mark.asyncio
async def test_client_receives_nothing_without_subscribing():
    server, port = await _running_server()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            with pytest.raises(asyncio.TimeoutError):
                await _recv_json(ws, timeout=0.3)
    finally:
        await server.stop()


@pytest.mark.asyncio
async def test_unsubscribe_stops_delivery():
    server, port = await _running_server()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await _subscribe(ws, "/gcs/heartbeat")
            await _recv_json(ws)  # at least one heartbeat arrives

            await ws.send(json.dumps({"op": "unsubscribe", "topic": "/gcs/heartbeat"}))
            # drain any in-flight message from before the unsubscribe took effect
            try:
                await _recv_json(ws, timeout=0.2)
            except asyncio.TimeoutError:
                pass

            with pytest.raises(asyncio.TimeoutError):
                await _recv_json(ws, timeout=0.3)
    finally:
        await server.stop()


@pytest.mark.asyncio
async def test_new_subscriber_immediately_gets_current_mission_state():
    server, port = await _running_server()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await _subscribe(ws, "/mission/state")
            first = await _recv_json(ws)
            assert first["msg"]["data"] == "idle"
    finally:
        await server.stop()


@pytest.mark.asyncio
async def test_start_command_advances_mission_state():
    server, port = await _running_server()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await _subscribe(ws, "/mission/state")
            await _recv_json(ws)  # idle, sent immediately on subscribe

            await _publish_command(ws, "start")
            second = await _recv_json(ws, timeout=1.0)
            assert second["msg"]["data"] in ("entering", "searching")
    finally:
        await server.stop()


@pytest.mark.asyncio
async def test_abort_command_reaches_aborted_state():
    server, port = await _running_server()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await _subscribe(ws, "/mission/state")
            await _recv_json(ws)  # idle

            await _publish_command(ws, "start")
            await _recv_json(ws, timeout=1.0)  # entering/searching

            await _publish_command(ws, "abort")
            final = await _recv_json(ws, timeout=1.0)
            assert final["msg"]["data"] == "aborted"
    finally:
        await server.stop()


@pytest.mark.asyncio
async def test_map_message_shape_over_the_wire():
    server, port = await _running_server()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await _subscribe(ws, "/map")
            msg = await _recv_json(ws)
            grid = msg["msg"]
            assert grid["info"]["width"] * grid["info"]["height"] == len(grid["data"])
            assert grid["info"]["resolution"] == 1.0
    finally:
        await server.stop()


@pytest.mark.asyncio
async def test_survivors_appear_after_start_and_late_subscriber_gets_them_too():
    server, port = await _running_server(check_interval=0.02)
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as first_ws:
            await _publish_command(first_ws, "start")
            await _subscribe(first_ws, "/vision/survivors")
            # duration_s=1.0 with 2 survivors spread across the searching
            # window -> both should have been detected well within 2s.
            first_detection = await _recv_json(first_ws, timeout=2.0)
            assert "survivor_id" in first_detection["msg"]

            # A client subscribing *after* detections started should still
            # receive the already-found survivor(s) via the latch.
            async with websockets.connect(f"ws://127.0.0.1:{port}") as late_ws:
                await asyncio.sleep(0.3)  # let more detections accumulate
                await _subscribe(late_ws, "/vision/survivors")
                replayed = await _recv_json(late_ws, timeout=1.0)
                assert "survivor_id" in replayed["msg"]
    finally:
        await server.stop()


@pytest.mark.asyncio
async def test_battery_and_pose_flow_without_explicit_start():
    """Telemetry that a real vehicle would report even before a mission
    starts (battery, pose at rest) should not require /gcs/command first."""
    server, port = await _running_server()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await _subscribe(ws, "/mavros/battery")
            await _subscribe(ws, "/mavros/local_position/pose")
            battery = await _recv_json(ws)
            pose = await _recv_json(ws)
            topics = {battery["topic"], pose["topic"]}
            assert topics == {"/mavros/battery", "/mavros/local_position/pose"}
    finally:
        await server.stop()
