# Progress Log (plain-language, for picking this back up later)

This file is just a running diary in normal human language, not a formal
doc. If you're back after a few days and forgot where things stood, read
this top-to-bottom (newest at the bottom) before doing anything else.

It complements, doesn't replace:
- `../NIDAR-Hardware-Bringup/CURRENT_STATE.md` and `NEXT.md` — the
  hardware/network bring-up side (Pixhawk, Jetson, MAVROS, rosbridge).
- `docs/REQUIREMENTS.md`, `docs/DECISIONS.md` — the formal, structured
  docs for this repo. This file is the "what actually happened, in
  order" version.

---

## 2026-08-25 — Hardware bring-up (see NIDAR-Hardware-Bringup repo)

Got Pixhawk 6X talking to the Jetson over Ethernet (MAVLink UDP), MAVROS
connected and pulling real IMU/battery data. rosbridge wasn't installed
yet at this point. Full details live in the hardware repo, not here.

## 2026-08-27 — Picked back up, got rosbridge running, wired up the real backend

Started the session by re-checking hardware state after 2 days idle —
turned out two things reset on their own and needed redoing (this
happens EVERY time the Jetson reboots or MAVROS restarts, it's not a
one-off):

1. The Jetson's static IP on the Ethernet port to the Pixhawk
   (192.168.144.1) disappears on reboot. Had to re-add it.
2. The Pixhawk stops actually *streaming* telemetry (SET_MESSAGE_INTERVAL
   settings are runtime-only, they don't persist). Had to re-request
   streaming for battery/position/attitude/imu.

Once that was back up:
- Confirmed `rosbridge_server` (already installed from a previous
  session, wasn't obvious at first) runs fine on port 9090 and relays
  real MAVROS data.
- Found the Jetson's WiFi IP: `10.145.63.112` (this is DHCP — it CAN
  change if the Jetson reconnects to WiFi, don't assume it's permanent).
- Wrote a small throwaway test script (`tools/laptop_rosbridge_smoke_test.py`)
  just to prove "can a laptop reach the Jetson's rosbridge over WiFi at
  all" before touching the real GCS backend. It worked.
- Then actually ran the real `gcs/backend` FastAPI service from the
  laptop, pointed at the real Jetson.

**Gotcha that cost some back-and-forth:** on Windows PowerShell, you
CANNOT set an env var and run a command on the same line like you can in
bash (`ROSBRIDGE_HOST=x uvicorn ...` — this is bash/zsh only, PowerShell
silently ignores it and falls back to defaults). The correct PowerShell
way:

```powershell
$env:ROSBRIDGE_HOST = "10.145.63.112"
$env:ROSBRIDGE_PORT = "9090"
uvicorn app.main:app --reload
```

First attempt without this quietly connected to `127.0.0.1` instead
(probably a local `sim/` instance from earlier dev work) and looked like
it was "working" — health said `connected: true` — but it wasn't talking
to the real drone at all. Worth remembering: `connected: true` alone
doesn't prove you're talking to the real Jetson, always also check what
`rosbridge_host` `/health` reports.

**Confirmed working for real** (checked the Jetson-side rosbridge logs
directly, saw the actual connection land): the backend on the laptop
really is talking to the real Pixhawk now, over WiFi, through rosbridge.
Subscribed successfully to `/mission/state`, `/mavros/battery`,
`/mavros/local_position/pose`, `/slam/map`, `/gcs/heartbeat`.

**One real gap found:** `/vision/survivors` subscription fails on the
real Jetson with `No module named 'nidar_airmouse'` — the custom ROS
message package for survivor detections only exists in the simulator
right now, nobody's built it for the real Jetson yet. This was already a
known open item (`docs/DECISIONS.md` D-13), just now confirmed for real
instead of theoretical.

**Position (x/y/z) shows 0/0/0 and doesn't move** when you physically
move the Pixhawk — this is EXPECTED, not broken. The real Pixhawk has no
indoor position source yet (no GPS allowed, no SLAM feeding it a vision
position estimate). This is D-11, still open, needs the SLAM workstream
before position will ever mean anything. Don't waste time debugging this
as a GCS/backend bug — it isn't one.

### State of things as of right now
- Hardware chain (Pixhawk ↔ Jetson ↔ MAVROS ↔ rosbridge): proven, working.
- GCS backend ↔ real Jetson over WiFi: proven, working.
- Real data flowing for real: battery, mission_state (probably empty/None
  still, nothing's publishing to it yet), pose (present but meaningless —
  see above), heartbeat.
- NOT working / not built yet: `/slam/map` has no real publisher yet (no
  SLAM node exists on the Jetson), `/vision/survivors` can't even
  subscribe (missing message package). Both are drone-side onboard
  work, not GCS backend work.
- The `/gcs/command` (Start/Abort) path exists in the backend and in
  `sim/`, but there's still no real subscriber node on the Jetson to
  receive it for real (this is D-10, was already known, still open).

### Next steps from here
1. Someone needs to build the actual onboard-autonomy side on the
   Jetson: a node that subscribes to `/gcs/command` and does something
   with start/abort, a SLAM node publishing `/slam/map` (and ideally
   feeding position back into the Pixhawk's EKF via
   `VISION_POSITION_ESTIMATE` — that's the real fix for the x/y/z=0
   problem), and a detection node publishing `/vision/survivors` (which
   first needs the `nidar_airmouse` message package actually built and
   installed on the Jetson, not just in `sim/`).
2. This explicitly does NOT belong in `custom-gcs` — see the discussion
   below. It needs its own place to live.
3. Test `/gcs/command` latency under real video load once video streaming
   exists (D-10's outstanding validation requirement, safety-critical:
   Abort must never lag behind video traffic).

---

## Important scope note (2026-08-27)

Got asked to "control drone motors" / "make it autonomous" directly from
inside `custom-gcs`. Explicitly did NOT do this — see the response in
conversation for the full reasoning, short version:

`custom-gcs`'s own `CLAUDE.md` is explicit that this repo is **only**
the GCS half — display + exactly two commands (Start/Abort) — and that
"the drone/flight-controller/onboard-autonomy side is a separate,
parallel workstream this repo interfaces with but does not implement."
Adding any navigation/motor-control code path here, even hidden or
behind a flag, is a hard rule violation (competition scoring treats any
extra operator-facing control as a penalty per instance, possible
disqualification) — and more importantly it's just architecturally the
wrong repo for it regardless of the competition rules.

The actual autonomy/motor-control work (SLAM, path planning, sending
real flight commands to the Pixhawk) needs its own codebase, running on
the Jetson, talking to MAVROS directly — not through this GCS.

**Update, same day:** that codebase now exists — `~/onboard-autonomy`,
a new separate git repo (not pushed to GitHub yet, that's a decision for
whoever's driving this next). Phase 0 only: a `nidar_airmouse` ROS 2
package with the `SurvivorDetection` message (fixes the "No module named
'nidar_airmouse'" error seen in the rosbridge log above), and a
`nidar_autonomy` package with command-handling / mission-state /
heartbeat nodes — the well-specified, low-risk parts of the interface.
Deliberately does NOT include SLAM, survivor detection, exploration/path
planning, or anything that sends real motor/flight commands — those are
either big algorithmic subsystems needing a real design session, or (for
flight commands) too safety-critical to write speculatively. See
`~/onboard-autonomy/CLAUDE.md` and `~/onboard-autonomy/README.md`.

## 2026-08-28 — Built the two onboard-autonomy packages for real on the Jetson

`colcon` wasn't installed. `sudo apt-get install python3-colcon-common-extensions`
needed a password (same wall as the rosbridge install a few days back —
no interactive terminal for sudo in this session, and the `!`-prefix
trick doesn't help either, sudo needs a real TTY for the password
prompt no matter how the command gets run). Worked around it: `pip3
install --user colcon-common-extensions` — no root needed, works fine.

Built both packages with `colcon build`. Hit one real gotcha: the
`nidar_autonomy` package's console-script executables (`command_node`,
`heartbeat_node`, `mission_state_node`) installed into `bin/` instead of
the `lib/nidar_autonomy/` path `ros2 run` actually looks in — so `ros2
run nidar_autonomy heartbeat_node` couldn't find them. Root cause: the
package was missing a `setup.cfg` (standard ROS 2 `ament_python`
boilerplate that tells `setup.py install` to route scripts into
`lib/<pkg>/`). Added it, rebuilt, fixed.

**Then actually ran all three nodes for real** and proved the loop
works end to end with `ros2 topic pub`:
- `start` on `/gcs/command` -> `/mission/state` went `idle` -> `entering`. OK
- A garbage command (`waypoint_edit`) -> rejected and logged by
  `command_node`, `/mission/state` untouched. OK (this is the
  safety-critical behavior -- see Hard Safety Rule 5 in that repo's
  CLAUDE.md)
- `abort` -> `/mission/state` went to `aborted`. OK

All three nodes were left running on the Jetson after this. Next real
test: point the actual `custom-gcs` backend at this Jetson (as before)
and hit Start/Abort from its `/docs` Swagger UI instead of raw `ros2
topic pub`, and confirm `/api/telemetry`'s `mission_state` field updates
-- that's the full real loop, GCS button all the way to onboard state
machine and back.
