"""Async rosbridge-protocol server wrapping a MissionSimulator.

This is the thing a real client (roslibjs, or the integration tests below)
connects to exactly as it would connect to the real Jetson's
`rosbridge_server` — same wire protocol, same topic names, same message
shapes. Swapping this simulator for the real drone later should require
no client-side code change, only a different WebSocket URL.

Command handling (`/gcs/command`) is the one inbound channel — this is
where "start" / "abort" from the GCS take effect (docs/DECISIONS.md D-10).
"""

from __future__ import annotations

import asyncio
import json
import time

import websockets
from websockets.asyncio.server import Server, ServerConnection

from .mission import MissionSimulator
from .protocol import (
    BATTERY_TOPIC,
    COMMAND_TOPIC,
    COVERAGE_GRID_TOPIC,
    HEARTBEAT_TOPIC,
    MAP_TOPIC,
    MISSION_STATE_TOPIC,
    PLANNED_PATH_TOPIC,
    POSE_TOPIC,
    SURVIVORS_TOPIC,
    TELEMETRY_STATE_TOPIC,
    ProtocolError,
    encode_publish,
    is_valid_command,
    parse_incoming,
)

# Topics for which a newly-subscribing client immediately gets the current
# value instead of waiting for the next periodic/on-change broadcast — the
# rosbridge equivalent of a "latched" topic. Real-world justification:
# an operator's GCS reconnecting mid-mission needs to see the current
# phase and already-found survivors immediately, not wait for the next
# change (docs/DATA_MODELS.md §1/§5 describe these as "on change" /
# "on detection", which — for a fresh subscriber — means "now").
_LATCHED_TOPICS = (MISSION_STATE_TOPIC, SURVIVORS_TOPIC)


class MissionControl:
    """Tracks whether/when a mission has been started or aborted, driven
    entirely by `/gcs/command` messages. Deliberately separate from
    MissionSimulator (which only knows "what does a running mission look
    like at time t") so that idle/aborted — properties of *whether* the
    mission is running — live in one obvious place.
    """

    def __init__(self) -> None:
        self.start_time: float | None = None
        self.abort_time: float | None = None

    def handle_command(self, command: str) -> None:
        now = time.monotonic()
        if command == "start" and self.start_time is None:
            self.start_time = now
        elif command == "abort" and self.start_time is not None and self.abort_time is None:
            self.abort_time = now

    def elapsed_s(self, now: float) -> float:
        """Seconds of *simulated mission progress* at wall-clock `now`.
        0.0 before start; frozen at the abort instant once aborted."""
        if self.start_time is None:
            return 0.0
        end_ref = self.abort_time if self.abort_time is not None else now
        return max(0.0, end_ref - self.start_time)

    def phase(self, now: float, mission: MissionSimulator) -> str:
        if self.start_time is None:
            return "idle"
        if self.abort_time is not None:
            return "aborted"
        return mission.progress_phase_at(self.elapsed_s(now))


class SimServer:
    def __init__(
        self,
        mission: MissionSimulator,
        *,
        host: str = "0.0.0.0",
        port: int = 9090,
        heartbeat_interval: float = 1.0,
        battery_interval: float = 0.5,
        pose_interval: float = 0.1,
        map_interval: float = 0.5,
        check_interval: float = 0.2,
        telemetry_state_interval: float = 0.5,
    ) -> None:
        self.mission = mission
        self.host = host
        self.port = port
        self.control = MissionControl()

        self._heartbeat_interval = heartbeat_interval
        self._battery_interval = battery_interval
        self._pose_interval = pose_interval
        self._map_interval = map_interval
        self._check_interval = check_interval
        self._telemetry_state_interval = telemetry_state_interval

        self._clients: dict[ServerConnection, set[str]] = {}
        self._latched: dict[str, list[dict]] = {topic: [] for topic in _LATCHED_TOPICS}
        self._ws_server: Server | None = None
        self._tasks: list[asyncio.Task] = []

    # -- lifecycle -----------------------------------------------------

    async def start(self) -> int:
        """Start listening and the background publisher loops. Returns
        the bound port (useful when constructed with port=0)."""
        self._ws_server = await websockets.serve(self._handle_connection, self.host, self.port)
        self._tasks = [
            asyncio.create_task(self._loop_heartbeat()),
            asyncio.create_task(self._loop_battery()),
            asyncio.create_task(self._loop_pose()),
            asyncio.create_task(self._loop_map()),
            asyncio.create_task(self._loop_survivors_and_state()),
            asyncio.create_task(self._loop_coverage_and_path()),
            asyncio.create_task(self._loop_telemetry_state()),
        ]
        sock = self._ws_server.sockets[0]
        return sock.getsockname()[1]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
        if self._ws_server is not None:
            self._ws_server.close()
            await self._ws_server.wait_closed()

    async def run_forever(self) -> None:
        await self.start()
        try:
            await asyncio.Future()
        finally:
            await self.stop()

    # -- connection handling ---------------------------------------------

    async def _handle_connection(self, ws: ServerConnection) -> None:
        self._clients[ws] = set()
        try:
            async for raw in ws:
                await self._handle_incoming(ws, raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._clients.pop(ws, None)

    async def _handle_incoming(self, ws: ServerConnection, raw: str) -> None:
        try:
            incoming = parse_incoming(raw)
        except ProtocolError:
            return  # ignore malformed input; don't crash the connection

        if incoming.op == "subscribe" and incoming.topic:
            self._clients[ws].add(incoming.topic)
            for latched_msg in self._latched.get(incoming.topic, []):
                await self._send(ws, incoming.topic, latched_msg)
        elif incoming.op == "unsubscribe" and incoming.topic:
            self._clients[ws].discard(incoming.topic)
        elif incoming.op == "publish" and incoming.topic == COMMAND_TOPIC:
            command = (incoming.msg or {}).get("data")
            if is_valid_command(command):
                self.control.handle_command(command)

    # -- outbound -----------------------------------------------------

    async def _send(self, ws: ServerConnection, topic: str, msg: dict) -> None:
        try:
            await ws.send(encode_publish(topic, msg))
        except websockets.ConnectionClosed:
            pass

    async def _broadcast(self, topic: str, msg: dict) -> None:
        for ws, subscriptions in list(self._clients.items()):
            if topic in subscriptions:
                await self._send(ws, topic, msg)

    def _latch(self, topic: str, msg: dict, *, replace: bool) -> None:
        if topic not in self._latched:
            return
        if replace:
            self._latched[topic] = [msg]
        else:
            self._latched[topic].append(msg)

    # -- publisher loops -----------------------------------------------

    async def _loop_heartbeat(self) -> None:
        seq = 0
        while True:
            now = time.monotonic()
            await self._broadcast(HEARTBEAT_TOPIC, {"seq": seq, "stamp": {"sec": int(now), "nanosec": 0}})
            seq += 1
            await asyncio.sleep(self._heartbeat_interval)

    async def _loop_battery(self) -> None:
        while True:
            elapsed = self.control.elapsed_s(time.monotonic())
            await self._broadcast(BATTERY_TOPIC, self.mission.battery_at(elapsed))
            await asyncio.sleep(self._battery_interval)

    async def _loop_pose(self) -> None:
        while True:
            elapsed = self.control.elapsed_s(time.monotonic())
            await self._broadcast(POSE_TOPIC, self.mission.pose_at(elapsed))
            await asyncio.sleep(self._pose_interval)

    async def _loop_map(self) -> None:
        while True:
            elapsed = self.control.elapsed_s(time.monotonic())
            await self._broadcast(MAP_TOPIC, self.mission.occupancy_grid_at(elapsed, stamp_sec=int(elapsed)))
            await asyncio.sleep(self._map_interval)

    async def _loop_coverage_and_path(self) -> None:
        while True:
            elapsed = self.control.elapsed_s(time.monotonic())
            await self._broadcast(COVERAGE_GRID_TOPIC, self.mission.coverage_grid_at(elapsed, stamp_sec=int(elapsed)))
            await self._broadcast(PLANNED_PATH_TOPIC, self.mission.planned_path_at(elapsed, stamp_sec=int(elapsed)))
            await asyncio.sleep(self._map_interval)

    async def _loop_telemetry_state(self) -> None:
        while True:
            now = time.monotonic()
            elapsed = self.control.elapsed_s(now)
            phase = self.control.phase(now, self.mission)
            contract = self.mission.telemetry_state_at(elapsed, phase)
            await self._broadcast(TELEMETRY_STATE_TOPIC, {"data": json.dumps(contract)})
            await asyncio.sleep(self._telemetry_state_interval)

    async def _loop_survivors_and_state(self) -> None:
        """Combined on-change / on-detection loop: cheap to poll at a
        modest rate since it only broadcasts when something actually
        changed, per docs/DATA_MODELS.md's "on change"/"on detection"
        semantics."""
        last_phase: str | None = None
        broadcast_survivor_ids: set[int] = set()

        while True:
            now = time.monotonic()
            phase = self.control.phase(now, self.mission)
            if phase != last_phase:
                msg = {"data": phase}
                await self._broadcast(MISSION_STATE_TOPIC, msg)
                self._latch(MISSION_STATE_TOPIC, msg, replace=True)
                last_phase = phase

            elapsed = self.control.elapsed_s(now)
            for detection in self.mission.survivors_detected_at(elapsed):
                if detection["survivor_id"] not in broadcast_survivor_ids:
                    await self._broadcast(SURVIVORS_TOPIC, detection)
                    self._latch(SURVIVORS_TOPIC, detection, replace=False)
                    broadcast_survivor_ids.add(detection["survivor_id"])

            await asyncio.sleep(self._check_interval)
