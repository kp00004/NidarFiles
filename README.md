# NidarFiles — NIDAR AirMouse: radio START → real hover

> **Status (2026-10-08): radio-only (no Wi-Fi link). On the real hardware:
> radio START/ABORT, telemetry, ARM via radio START, and the Jetson-set EKF
> origin all work. Current blocker: the EKF keeps dropping the optical flow
> (MTF-01) on the ground, so ArduCopter refuses to arm in GUIDED.** No
> flight yet. See "Where we stopped" at the end of this file.

## What this is

NIDAR AirMouse is an autonomous indoor (GPS-denied) search-and-rescue drone.
This workspace implements the first real flight mission: **press START on the
GCS and the drone takes off, hovers, and lands.**

- **GCS laptop**: the operator panel. Shows telemetry and has exactly two
  commands, START (for the mission chosen in the dropdown) and ABORT.
- **MicroLR900 radios (900 MHz)**: the **only** link between the laptop and
  the drone. They carry START/ABORT from the laptop to the **Jetson**, and
  telemetry from the Jetson back. The radio link ends at the Jetson; the
  Pixhawk is not on it. **There is no Wi-Fi/hotspot link.**
- **Jetson Orin Nano**: the mission computer. Receives the radio command,
  validates it, runs the hover mission, and commands the Pixhawk via MAVROS.
- **Pixhawk 6X (ArduCopter 4.6.3)**: flies the drone (GUIDED takeoff, position
  hold, LAND), using its EKF3 indoor position estimate.
- **Mission dropdown**: lists the missions the GCS can start: **Hover** and
  **Motor Test** (props-off bench check, see "Motor Test" below).

## Architecture

```
COMMANDS (START / ABORT)
GCS laptop ──USB── MicroLR900 )))) 900 MHz )))) MicroLR900 ──USB serial── Jetson
                                                                           │
   radio_command_node ─▶ /gcs/mission_select + /gcs/command ─▶ command_node │
   ─▶ missions/hover/mission.py ─▶ ArduCopterVehicle ─▶ MAVROS ─Ethernet─▶ Pixhawk 6X ─▶ drone

TELEMETRY (same radio, other direction)
Pixhawk ─▶ MAVROS ─▶ Jetson radio_command_node ─▶ MicroLR900 )))) MicroLR900 ─▶ GCS backend ─▶ operator panel
   FCU state, battery, position, attitude, FCU messages, hover progress (~250 B/s)
```

**The telemetry-radio command terminates at the Jetson. The Jetson then
communicates with the Pixhawk through MAVROS.**

START path in detail:

1. Operator selects **Hover** and presses **START**.
2. GCS backend (`custom-gcs/gcs/backend/app/radio_link.py`) sends
   `COMMAND_LONG(MAV_CMD_USER_1)` on COM5: START, mission code 1 (Hover), a
   nonce, magic 4242, protocol version 2. It resends the same nonce until the
   Jetson answers (5 tries).
3. Jetson `radio_command_node` decodes it and checks it: known mission, hover
   mission node alive, MAVROS connected to the Pixhawk, no run in progress.
   It replies `COMMAND_ACK` ACCEPTED, or REJECTED with a reason
   (`UNKNOWN_MISSION`, `MISSION_NOT_READY`, `FCU_NOT_CONNECTED`,
   `MISSION_BUSY`, `BAD_PROTOCOL_VERSION`).
4. If accepted it publishes `/gcs/mission_select {"mission_id":"hover"}` then
   `/gcs/command "start"`; `command_node` validates it; the hover mission
   acts on it.
5. Hover mission: preflight checks → GUIDED → ARM → TAKEOFF → hold → LAND →
   ArduCopter disarms itself after touchdown.
6. GCS shows each confirmed stage: Jetson ACK → mission state (radio
   heartbeat + hover status) → vehicle state (relayed FCU telemetry), all
   over the radio. It never reports success without the Jetson's ACK.

ABORT path: same radio path, always accepted by the Jetson. On the ground
while arming → DISARM. **Airborne → LAND mode (controlled descent), never a
mid-air disarm.** If a pilot changes the flight mode on the RC transmitter,
the mission stops commanding immediately and never fights the pilot.

There is no Wi-Fi at all: no hotspot, no rosbridge. The GCS has no way to
publish `/gcs/command`; only `radio_command_node` on the Jetson does. Not
available without Wi-Fi: live video, map and perception panels (the radio
is too slow); the hover mission doesn't use them.

## Repository structure

```
NidarFiles/
├── custom-gcs/            GCS: FastAPI backend + React frontend (git, branch feature/hover-radio)
├── onboard-autonomy/      Jetson ROS 2 packages (git, branch feature/hover-radio)
├── missions/
│   ├── hover/
│   │   ├── mission.py         the hover mission (ROS 2 node, runs on the Jetson)
│   │   ├── hover_logic.py     its decision logic (pure Python, unit-tested)
│   │   └── test_hover_logic.py
│   └── motor_test/
│       ├── mission.py         motor test (ROS 2 node, Jetson) -- PROPS OFF
│       ├── motor_test_logic.py
│       └── test_motor_test_logic.py
├── scripts/
│   ├── jetson/  setup_jetson.sh, check_jetson.sh, start_jetson.sh
│   └── gcs/     start_gcs.ps1, make_jetson_bundle.ps1 (USB pen drive), deploy_to_jetson.ps1 (SSH, needs a network)
└── README.md
```

Published as one repository: https://github.com/kp00004/NidarFiles (a plain
copy -- not linked to the TeamArdra `custom-gcs` / `onboard-autonomy` repos).

### What changed in the repos (branch `feature/hover-radio`)

onboard-autonomy (`nidar_autonomy/nidar_autonomy/`):

| File | Change |
|---|---|
| `telem_command_codec.py` | Radio protocol: MAVLink2 stream parser, COMMAND_LONG/ACK/HEARTBEAT encoding, mission code, reason codes (pure Python, no pymavlink needed on the Jetson) |
| `radio_command_logic.py` (new) | Accept/reject rules, duplicate (nonce) handling, ABORT always accepted |
| `radio_command_node.py` (new) | Serial radio on the Jetson: receives START/ABORT, sends telemetry; replaces the old Pixhawk-routed `telem_command_bridge_node.py` (removed) |
| `radio_telemetry.py` (new) | What telemetry goes over the radio, how often, staleness and rate limits (pure Python) |
| `ardupilot_vehicle.py` (new) | ArduCopter control via MAVROS: set_mode, takeoff, stream rates, state snapshot; arm/disarm through the existing `FlightCommandClient` |
| `vehicle_snapshot.py` (new) | Plain vehicle-state record used by mission logic |
| `topics.py`, `setup.py`, `package.xml` | Topic constants, `radio_command_node` entry point, `python3-serial` dependency |
| `flight_command.py`, `command_node.py`, `arming_guard.py` | **Unchanged** (reused as-is) |

custom-gcs:

| File | Change |
|---|---|
| `gcs/backend/app/radio_link.py`, `radio_protocol.py` (new) | Radio on COM5: send with retries, ACK by nonce, Jetson heartbeat, link status, hands telemetry on |
| `gcs/backend/app/radio_telemetry.py` (new) | Telemetry from the radio, served to the panels in the same shape rosbridge gave (`GCS_TELEMETRY_SOURCE=radio`, default) |
| `gcs/backend/app/main.py` | START = `POST /api/mission/start {mission}`, ABORT = `POST /api/command/abort`, both over the radio; `GET /api/radio/status`; mission-less START removed |
| `gcs/backend/app/missions.py` | Exactly one mission: Hover (radio code 1) |
| `gcs/backend/app/ros_client.py` | No longer advertises `/gcs/command` on rosbridge; only used with `GCS_TELEMETRY_SOURCE=rosbridge` (development against `sim/`) |
| `gcs/frontend/src/components/ControlsPanel.tsx` | "Mission Control": Mission dropdown + START + ABORT + radio link + confirmed stages |
| `MissionSelectPanel.tsx` | Removed (replaced by the above) |
| `tools/telem_command.py` | Terminal START/ABORT tool using the backend's RadioLink |
| `docs/COMMUNICATION.md` §6, `docs/DATA_MODELS.md` §8 | Radio protocol and topics documented |

## Hardware and wiring

```
Laptop ──USB── MicroLR900  )))  MicroLR900 ──USB (CP2102)── Jetson Orin Nano ──Ethernet (eno1)── Pixhawk 6X
 COM5, 115200                                    /dev/serial/by-id/usb-Silicon_Labs_CP2102_..._0001-if00-port0
                                                 (fallback /dev/ttyUSB0), 115200
Jetson  192.168.144.1/24 on eno1   ◀──MAVLink2 UDP──▶   Pixhawk 192.168.144.14:14550 (UDP server)
No Wi-Fi: the Jetson is operated with a monitor and keyboard, and code
reaches it from GitHub with `git clone` / `git pull` (A.1).
```

Radio configuration in use: 115200 baud, DUPLEX, HIGH rate, MAX power,
address 1000, channel 0. Indoor position/altitude sensors: see section B.

## Software requirements (from the repositories)

| Machine | Requirement |
|---|---|
| Jetson | Ubuntu 22.04, ROS 2 Humble, `ros-humble-mavros`, `python3-serial`, `colcon` (`setup_jetson.sh` installs any that are missing; that needs internet) |
| Laptop | Windows, Python 3 (tested here with 3.14) + `gcs/backend/requirements.txt` (fastapi 0.141.1, uvicorn 0.52.4, roslibpy 2.1.0, pyserial 3.5, pymavlink 2.4.50); Node.js + npm for the frontend build (tested with Node 24); OpenSSH client for deployment |

## A. Setup and running

### 1. Get the code onto the Jetson (GitHub)

The code is published at **https://github.com/kp00004/NidarFiles** (public).
On the Jetson (monitor + keyboard), with internet for this step only:

```bash
mv ~/NidarFiles ~/NidarFiles.old 2>/dev/null     # keep any older copy aside
git clone https://github.com/kp00004/NidarFiles.git ~/NidarFiles
chmod +x ~/NidarFiles/scripts/jetson/*.sh ~/NidarFiles/missions/hover/mission.py
```

**Updating later:** after new code is pushed to GitHub, on the Jetson:

```bash
git -C ~/NidarFiles pull
~/NidarFiles/scripts/jetson/setup_jetson.sh      # rebuilds ~/nidar_ws
```

Without internet on the Jetson: `scripts/gcs/make_jetson_bundle.ps1
-Destination E:\` copies the same files to a USB pen drive; on the Jetson
copy `/media/$USER/<drive>/NidarFiles` to `~/`. (`deploy_to_jetson.ps1`
does it over SSH if the laptop and Jetson ever share a network.)

### 2. One-time Jetson setup

```bash
~/NidarFiles/scripts/jetson/setup_jetson.sh    # apt deps, dialout, ModemManager, build ~/nidar_ws
# log out/in if it added you to dialout
```

It builds onboard-autonomy into a separate overlay workspace `~/nidar_ws`,
leaving the existing `~/ros2_ws` untouched.

### 3. Start the Jetson stack (one terminal)

```bash
~/NidarFiles/scripts/jetson/start_jetson.sh --dry-run   # radio test: ACK + log only, nothing can arm
~/NidarFiles/scripts/jetson/start_jetson.sh             # LIVE: a radio START flies the hover
```

It restores the `eno1` address if missing, checks the Pixhawk answers, starts
MAVROS (`apm.launch fcu_url:=udp://@192.168.144.14:14550`) and waits for
`connected: true`. Then it starts `command_node`, `heartbeat_node`,
`radio_command_node` (commands + telemetry, `NIDAR_TELEMETRY_RATE_HZ`,
default 2) and the hover mission. No rosbridge. It refuses to run if
`mission_state_node` is running.
Logs go to `~/NidarFiles/logs/<time>/`. Ctrl+C stops everything.

`scripts/jetson/check_jetson.sh` prints read-only diagnostics at any time.

### 4. Start the GCS (laptop)

```powershell
cd D:\NidarFiles
.\scripts\gcs\start_gcs.ps1                 # -RadioPort COM5 by default
```

It opens `http://127.0.0.1:8000/ui/`. On the first run it creates the
backend virtualenv; it (re)builds the frontend whenever its sources changed.

Debug alternative without the UI: `python custom-gcs\tools\telem_command.py --port COM5`.

### 5. Verify before any START

In the operator panel, **Mission Control** card:

| Check | Where | Expected |
|---|---|---|
| Radio port open | Command radio | not "PORT COM5 NOT OPEN" |
| Radio link + Jetson running | Command radio | `LINK UP (Jetson heartbeat … s ago)` |
| Hover mission node running | Mission (radio) / Mission status | `idle` / `idle [real]` |
| MAVROS ↔ Pixhawk | Vehicle, Connection panel | FCU connected, `disarmed · <mode>` |
| Telemetry over the radio | other panels | live battery / attitude / position |
| Indoor position | Position panel / `check_jetson.sh` | `/mavros/local_position/pose` publishing, moves when the drone is carried |

### 6. Mission procedure

1. Start the Jetson stack, then the GCS. Wait for the checks above.
2. **Mission: Hover**.
3. Press **START**.
4. Panel shows `Jetson ACCEPTED START (hover)`, or `START not accepted: … REJECTED START: <reason>`.
5. Mission state goes `setting_guided → arming → taking_off → hovering → landing → complete`, and the Vehicle row shows ARMED / GUIDED / altitude.
6. **ABORT** at any time: on the ground it disarms; in the air it switches to LAND.

### 7. Motor Test (PROPS OFF)

Checks that every motor runs, in the right order and direction, without
needing a position estimate. Uses ArduCopter's own motor test
(`MAV_CMD_DO_MOTOR_TEST`, the same as Mission Planner's Motor Test page):
it does **not** arm the vehicle the normal way and skips the EKF/position
checks, but ArduCopter still refuses if the vehicle is armed, the safety
switch isn't pressed, or the RC isn't calibrated (the reason shows in the
FCU status text panel).

1. **Remove the props.** Start the Jetson stack **live** (no `--dry-run`) and the GCS.
2. **Mission: Motor Test**, press **START**.
3. Motors spin one at a time: **A, B, C, D** (A = front-right on a quad X,
   then clockwise -- ArduPilot's test order) at **8 %** for **5 s** each,
   1 s apart. Mission status shows `testing` and which motor.
   Compare each with ArduPilot's motor diagram for your frame: the right
   motor position, and the right direction (CW/CCW).
4. **ABORT** stops the running motor immediately. Every motor command also
   carries its own 5 s timeout, so ArduCopter stops the motor by itself if
   the Jetson or radio fails.

Settings: `CONFIG` at the top of `missions/motor_test/mission.py`
(`motor_count`, `throttle_pct`, `per_motor_s`). The Jetson refuses a START
for either mission while the other one is running (`MISSION_BUSY`).

### 8. Bench setup: Pixhawk parameters from the GCS

For bench work only -- never in a mission (the operator panel's command
surface is START and ABORT; this section does not exist unless asked for).

```bash
~/NidarFiles/scripts/jetson/start_jetson.sh --setup        # Jetson: allow parameter writes
```
```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\gcs\start_gcs.ps1 -Setup   # laptop
```

A **Pixhawk parameters** section appears under the panels: type a name (or
click one of the checklist parameters), **Read**, change the value,
**Write**. The value shown is always read back from the FCU. The Jetson
refuses writes while the vehicle is armed or a mission runs, or when it was
not started with `--setup`; the panel shows the reason. Reads always work.
Every read/write is in `radio_command_node.log`. Some parameters only take
effect after a Pixhawk reboot.

## Safety

- **This is real hardware.** In live mode a radio START (Hover) arms and flies the vehicle; START (Motor Test) spins the motors -- **props off**.
- First tests: **props OFF** (dry-run, then live START to watch GUIDED/ARM/TAKEOFF requests on the bench). First flight: props on, **vehicle tethered/netted**, people clear, low altitude (default 0.5 m, 10 s hold).
- A **safety pilot with an RC transmitter** (mode switch with LAND and a manual mode, plus motor kill) should be ready on every flight. It is the only abort independent of the GCS, radios and Jetson.
- **ABORT** = LAND in the air, DISARM on the ground before takeoff.
- **Radio link lost:** no new failsafe was added. The mission still completes its timed hover and lands. ABORT is unavailable until the link returns, so use the RC transmitter.
- **Jetson/MAVROS failure in flight:** the mission requests LAND if it still can. If the Jetson itself dies, ArduCopter keeps holding position in GUIDED, so the pilot must take over on the RC transmitter.
- **Pixhawk failsafes** (battery, EKF, RC) remain ArduCopter's. If the FCU switches to LAND itself, the mission follows it and does not override it.
- Do not run `mission_state_node` with this stack: it would arm on START by itself. `start_jetson.sh` and the hover mission both refuse.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| GCS: `PORT COM5 NOT OPEN` | Radio unplugged or different COM port (Device Manager > Ports); restart with `-RadioPort COMx`; close other programs using the port (radio config tool) |
| GCS: `RADIO LINK DOWN` | Jetson stack not running; Jetson radio missing (`check_jetson.sh`); radios not paired (address/channel/rate must match); wrong baud |
| `START not accepted … never acknowledged` (504) | Same as RADIO LINK DOWN; or `radio_command_node` not running (see its log) |
| `REJECTED START: MISSION_NOT_READY` | Hover mission node not running or crashed (`hover_mission.log`) |
| `REJECTED START: FCU_NOT_CONNECTED` | MAVROS not connected: check `eno1` address, `ping 192.168.144.14`, `mavros.log` |
| `REJECTED START: MISSION_BUSY` | A hover run is still in progress; wait or ABORT |
| `REJECTED START: UNKNOWN_MISSION` | GCS and Jetson mission tables differ (`app/missions.py` vs `telem_command_codec.MISSION_NAMES`) |
| Mission state `failed`: "no fresh local position" | EKF has no indoor position estimate — section B items 1–3 |
| Mission `failed`: "GUIDED not confirmed" / "ARM failed: …" | ArduCopter refused; the reason (prearm message) is in `hover_mission.log` and the GCS status text |
| Mission `failed`: "TAKEOFF rejected/not acknowledged" | Check `/mavros/cmd/takeoff` exists; ArduCopter must be armed in GUIDED |
| `START refused: mission_state_node is running` | Stop `mission_state_node` |
| Jetson: radio port permission denied | User not in `dialout` (run `setup_jetson.sh`, log out/in) |
| Jetson: radio garbled / busy | ModemManager probing the port: `sudo systemctl disable --now ModemManager` |
| Radio by-id path missing | Device name differs: `ls -l /dev/serial/by-id/`, set `NIDAR_RADIO_PORT=...` before `start_jetson.sh` |
| GCS: LINK UP but FCU not connected / no battery, position | MAVROS not connected (`mavros.log`), or the Pixhawk isn't streaming: check `check_jetson.sh` |
| GCS: telemetry jumps or ACKs slow | Radio saturated: lower `NIDAR_TELEMETRY_RATE_HZ` (e.g. 1) before `start_jetson.sh` |
| Local position stale / empty after MAVROS restart | Stream rates reset; the hover node re-requests them on connect — check its log for "stream … NOT confirmed" |

## Tests

| Level | Status |
|---|---|
| Mission decision logic (`missions/`: hover 33, motor test 15) | PASSED — `python -m pytest missions -q` |
| onboard-autonomy unit tests (385 passed, 4 ROS-only skipped) | PASSED on Windows without ROS — ROS node tests run only on the Jetson |
| GCS backend (170 passed, 1 skipped: needs Linux sim venv) | PASSED |
| GCS frontend (98 tests) + typecheck + production build | PASSED |
| Radio protocol vs pymavlink (byte-for-byte) | PASSED |
| GCS RadioLink ↔ Jetson codec + gate, in-memory serial | PASSED (software integration) |
| Radio telemetry: Jetson relay + codec → GCS RadioLink → `/api/telemetry`, in-memory serial | PASSED (software integration) |
| Backend live smoke test, no radio attached | PASSED: START/ABORT refused with 503, nothing claimed sent |
| RF link Jetson ↔ laptop (dry-run, 2026-10-07) | **PASSED on hardware**: START/ABORT ACKed on the 1st attempt; telemetry (FCU state, battery, attitude) live over the radio; radio unplug/replug recovered by itself |
| Jetson nodes on ROS 2 / MAVROS (radio node, hover node) | **PASSED on hardware** (start-up, MAVROS connected, stream rates confirmed) |
| Motor Test via this code | **NOT TESTED** (not run yet) |
| ARM via radio START (props off, 2026-10-07) | **PASSED on hardware**: GUIDED → ARM accepted and confirmed; TAKEOFF then refused (no EKF origin) and the hover node crashed while logging it -- both fixed |
| EKF origin set by the Jetson (2026-10-08) | **PASSED on hardware**: preflight passed with the origin set |
| GUIDED / TAKEOFF / LAND via this code | **NOT TESTED**: ARM refused (result 4) while the EKF kept dropping optical-flow aiding |
| End-to-end START → hover | **NOT TESTED** |
| Flight | **NOT TESTED** |

## B. Hardware-specific items to configure and verify later

These depend on the physical drone and could not be determined from the
repositories. Nothing here is guessed in the code. Where the code depends on
one of them, the code is marked `TODO(hardware)`, or the item fails safe if
not met.

1. **Indoor position source (required before any flight).** ArduCopter will
   not arm/take off in GUIDED, and the hover mission's preflight refuses,
   without a fresh `/mavros/local_position/pose`. Identify the installed
   optical-flow and rangefinder sensors and configure them and EKF3 in
   ArduCopter. Typical parameter groups: `FLOW_*`, `RNGFND1_*`,
   `EK3_SRC1_POSXY`/`VELXY`/`POSZ`/`YAW`, `AHRS_EKF_TYPE`, `GPS1_TYPE`,
   `ARMING_CHECK`. Use each sensor's ArduPilot documentation; values are not
   assumed here. If indoor flight needs an EKF origin, set it as that setup
   requires.
2. **Verify the position estimate on the bench:** `check_jetson.sh` shows
   local position at ≥10 Hz; carry the drone ~1 m and confirm x/y/z follow;
   the altitude reads correctly near the floor.
3. **Hover parameters** (`missions/hover/mission.py` `CONFIG`):
   `takeoff_altitude_m` (0.5) inside the rangefinder's reliable range and
   below the net; `max_horizontal_drift_m` (0.75) inside the test area;
   `min_battery_voltage_v` set for the flight battery (currently `None` =
   check disabled).
4. **Battery monitoring:** a power module must report real voltage. At
   bring-up `/mavros/battery` read 0.0 V.
5. **Failsafes in ArduCopter:** RC failsafe, battery failsafe, `FS_EKF_ACTION`,
   `FS_GCS_ENABLE` (no MAVLink GCS is connected to the Pixhawk in this design),
   `DISARM_DELAY`, `LAND_SPEED`. Decide and set them deliberately.
6. **RC transmitter / safety pilot:** bind, mode switch (LAND + manual), motor kill.
7. **Radio device name:** confirm the by-id path on the Jetson. If another
   CP2102 device (e.g. a LiDAR adapter) produces the same name, use the
   `/dev/serial/by-path/` name of the radio's USB port (`NIDAR_RADIO_PORT`).
8. **Radio round-trip time:** measure with the dry-run test; tune retry
   timings in `custom-gcs/gcs/backend/app/radio_link.py` (`START_*`, `ABORT_*`).
9. **MAVROS on the Jetson:** confirm `/mavros/set_mode`, `/mavros/cmd/takeoff`,
   `/mavros/cmd/arming`, `/mavros/set_message_interval` exist
   (`check_jetson.sh`), and that GUIDED takeoff via `/mavros/cmd/takeoff`
   climbs to the requested height on this vehicle.
10. **Telemetry over the radio:** confirm the panels update (battery,
    attitude, position, FCU messages), and that START/ABORT ACKs stay fast
    with telemetry running. Lower `NIDAR_TELEMETRY_RATE_HZ` if they don't.
11. **Persistent network:** `eno1` 192.168.144.1 is still runtime-only
    (`start_jetson.sh` restores it, using sudo).

### Hardware test sequence (each step only after the previous one passes)

1. **Radio only, props OFF:** `start_jetson.sh --dry-run`. GCS shows LINK UP.
   START → `Jetson ACCEPTED`, and `radio_command_node.log` shows the
   command, with nothing published. Unplug the Jetson radio → RADIO LINK DOWN.
2. **Telemetry:** GCS shows real FCU state, battery, attitude, local position.
3. **Rejections, props OFF, live mode:** with the Ethernet cable unplugged,
   START is rejected `FCU_NOT_CONNECTED`. With the hover node stopped,
   `MISSION_NOT_READY`.
4. **Command chain, props OFF, live mode — ask for explicit confirmation first:**
   START → GUIDED → ARM → TAKEOFF request (motors spin up; no props).
   ABORT → disarm/LAND. Check the mission state and logs at each stage.
5. **First hover — props ON, tethered/netted, safety pilot ready, explicit go/no-go first:**
   START → takeoff to 0.5 m → 10 s hold → LAND → disarm. Then repeat with an
   in-air ABORT.

## Where we stopped (2026-10-08)

**Hardware state**
- MicoAir **MTF-01** (optical flow + rangefinder) on Pixhawk **TELEM1**,
  configured through ArduPilot serial passthrough (MicoAssistant):
  `Mav_APM`, `mav_id 200`. Pixhawk: `SERIAL1_PROTOCOL=1`, `SERIAL1_BAUD=115`,
  `SERIAL1_OPTIONS=1024`, `FLOW_TYPE=5`, `RNGFND1_TYPE=10`,
  `RNGFND1_ORIENT=25`, `EK3_SRC1_POSXY=0`, `EK3_SRC1_VELXY=5`,
  `EK3_SRC1_POSZ=1`, `EK3_SRC1_YAW=1`. Rangefinder reading confirmed live.
- ModemManager disabled on the Jetson. `eno1` address still runtime-only
  (`start_jetson.sh` restores it).

**Blocker:** on a live Hover START, ArduCopter refused to ARM (result 4)
while the status text cycled `EKF3 IMU0 started relative aiding` /
`stopped aiding` / `fusing optical flow` -- the EKF keeps losing the flow
sensor, most likely because the drone sits on the floor (sensor too close,
or floor without texture/light). Local position z also read 1.43 m on the
ground (baro drift).

**Next steps (props off throughout)**
1. Get the exact refusal: `grep -E "PreArm|Arm:|EKF|flow" <latest log>/mavros.log`.
2. In Mission Planner (Jetson stack stopped), Status tab: `opt_qua` and
   `rangefinder1` on the floor vs held 30-50 cm up, on a textured surface
   in good light. Read `EK3_FLOW_USE`, `FLOW_ORIENT_YAW`, `FLOW_FXSCALER`,
   `FLOW_FYSCALER`, `RNGFND1_GNDCLEAR`, `ARMING_CHECK`.
3. Likely fixes: `RNGFND1_GNDCLEAR` (sensor height when landed), a
   textured take-off mat, mounting height; then flow calibration
   (`FLOW_FXSCALER/FYSCALER`, `FLOW_ORIENT_YAW`).
4. Run the **Motor Test** mission (props off) -- not run yet.
5. Manual hover in AltHold/Loiter on the RC transmitter to validate the flow.
6. Only then our Hover mission: tethered, low, safety pilot ready.

**Code workflow:** edit in `D:\NidarFiles`; the GitHub copy is
`D:\NidarFiles-github` (https://github.com/kp00004/NidarFiles), updated by
copying the changes there and pushing. On the Jetson: `git pull`, then
`setup_jetson.sh` when `onboard-autonomy` changed. The `custom-gcs` and
`onboard-autonomy` folders in `D:\NidarFiles` are also the TeamArdra git
repos (branch `feature/hover-radio`); none of this work is committed there.
