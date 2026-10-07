# sim/

A standalone rosbridge-protocol simulator: it speaks the same WebSocket
wire protocol as the real Jetson Nano's `rosbridge_server` (subscribe /
unsubscribe / publish, port 9090 by default) and publishes synthetic data
matching every topic in `../docs/DATA_MODELS.md`, so the GCS frontend (and
anything else) can be developed and tested without drone hardware. See
`../docs/ARCHITECTURE.md` §6 and `../docs/DECISIONS.md` D-1/D-2.

## Run it

```sh
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m rosbridge_sim                      # ws://0.0.0.0:9090, rulebook-max arena, 300s mission
python -m rosbridge_sim --help                # see all options (duration, arena size, survivor count, seed...)
```

Then, from any rosbridge/roslibjs-compatible client: subscribe to
`/mission/state`, `/mavros/battery`, `/mavros/local_position/pose`,
`/map`, `/coverage_grid`, `/planned_path`, `/telemetry/state`,
`/vision/survivors`, or `/gcs/heartbeat`, and publish
`{"op":"publish","topic":"/gcs/command","msg":{"data":"start"}}` (or
`"abort"`) to drive the mission — that command channel is the *only*
inbound message this simulator accepts, matching the real operator
command surface (`../docs/REQUIREMENTS.md` §4).

## Test it

```sh
source .venv/bin/activate
python -m pytest -q
```

`tests/test_grid.py` and `tests/test_mission.py` cover the deterministic
simulation core with no networking involved. `tests/test_protocol.py`
covers the wire-format edge cases. `tests/test_server_integration.py`
spins up a real server on a real loopback socket and drives it with a
real WebSocket client — this is the one that actually proves the
simulator is a valid rosbridge stand-in from a client's point of view.

## Layout

```
rosbridge_sim/
  protocol.py   rosbridge wire protocol subset (subscribe/unsubscribe/publish)
  grid.py       deterministic synthetic arena + OccupancyGrid message building
  mission.py    pure simulation core: state at time t, no clock/networking
  server.py     asyncio WebSocket server tying the above together
  __main__.py   CLI entrypoint
tests/          unit tests for the pure pieces + one real end-to-end test
```
