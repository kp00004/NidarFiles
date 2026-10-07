"""Real end-to-end test: a real sim/rosbridge_sim subprocess, a real
roslibpy connection, and real HTTP calls into the FastAPI app. Everything
else in this test suite exercises the API layer against a fake client --
this is the one test that proves the whole chain (HTTP -> FastAPI ->
RosBridgeClient -> roslibpy -> rosbridge protocol -> sim) actually works
together, the same way it will with the real Jetson swapped in for sim/.

Requires sim/.venv to exist with sim/requirements.txt installed (see
sim/README.md) -- skipped automatically if it doesn't, so this doesn't
break `pytest` for someone who has only set up the backend venv.
"""

from __future__ import annotations

import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from app.config import Settings
from app.main import create_app

REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_DIR = REPO_ROOT / "sim"
SIM_VENV_PYTHON = SIM_DIR / ".venv" / "bin" / "python"
TEST_PORT = 19191

pytestmark = pytest.mark.skipif(
    not SIM_VENV_PYTHON.exists(),
    reason=f"sim/.venv not set up ({SIM_VENV_PYTHON} missing) -- see sim/README.md",
)


def _wait_for_port(host: str, port: int, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError(f"nothing listening on {host}:{port} after {timeout}s")


@pytest.fixture
def sim_process():
    proc = subprocess.Popen(
        [
            str(SIM_VENV_PYTHON), "-m", "rosbridge_sim",
            "--port", str(TEST_PORT),
            "--duration", "2",
            "--width", "5", "--height", "5",
            "--survivors", "1",
        ],
        cwd=str(SIM_DIR),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        _wait_for_port("127.0.0.1", TEST_PORT)
        yield proc
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def test_full_stack_health_start_and_telemetry_round_trip(sim_process):
    settings = Settings(
        rosbridge_host="127.0.0.1", rosbridge_port=TEST_PORT, connect_timeout_s=5.0, telemetry_source="rosbridge"
    )
    app = create_app(settings=settings)

    with TestClient(app) as client:  # runs the real lifespan -> real roslibpy connect
        health = client.get("/health").json()
        assert health["connected"] is True

        # Telemetry should already be flowing before "start" -- battery/pose
        # are published regardless of mission phase (see sim/README.md).
        deadline = time.monotonic() + 3
        telemetry = {}
        while time.monotonic() < deadline:
            telemetry = client.get("/api/telemetry").json()
            if telemetry["mission_state"] is not None:
                break
            time.sleep(0.1)
        assert telemetry["mission_state"] == "idle"

        # START/ABORT no longer travel over rosbridge (they go over the
        # command radio -- app/radio_link.py), so the backend has no Wi-Fi
        # START at all. Drive the sim's mission the way the Jetson's radio
        # node would, by publishing /gcs/command on the sim directly; this
        # test is about telemetry flowing back through the real backend.
        assert client.post("/api/command/start").status_code in (404, 405)
        import roslibpy

        ros = roslibpy.Ros(host=settings.rosbridge_host, port=settings.rosbridge_port)
        ros.run(timeout=5)
        command_topic = roslibpy.Topic(ros, "/gcs/command", "std_msgs/String")
        command_topic.advertise()
        time.sleep(0.2)
        command_topic.publish(roslibpy.Message({"data": "start"}))

        deadline = time.monotonic() + 3
        state = telemetry.get("mission_state")
        while time.monotonic() < deadline and state in (None, "idle"):
            state = client.get("/api/telemetry").json()["mission_state"]
            time.sleep(0.1)
        assert state in ("entering", "searching")

        deadline = time.monotonic() + 3
        survivors = []
        while time.monotonic() < deadline and not survivors:
            survivors = client.get("/api/survivors").json()
            time.sleep(0.1)
        assert len(survivors) == 1
        assert survivors[0]["survivor_id"] == 1

        command_topic.publish(roslibpy.Message({"data": "abort"}))

        deadline = time.monotonic() + 3
        state = None
        while time.monotonic() < deadline and state != "aborted":
            state = client.get("/api/telemetry").json()["mission_state"]
            time.sleep(0.1)
        assert state == "aborted"
        command_topic.unadvertise()
        ros.terminate()
