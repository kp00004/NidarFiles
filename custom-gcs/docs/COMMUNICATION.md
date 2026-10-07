# Communication Interfaces — NIDAR AirMouse

Status: **Phase 0 — proposed interfaces, not yet implemented or agreed
with the drone-side team.**

This document defines the data interfaces between the systems involved in
an AirMouse mission, per the task's requirement to define "the expected
communication/data interfaces between: Drone, Flight controller, Onboard
computer/autonomy system, GCS, Mapping system, Survivor detection/
localisation system, Video system."

The competition documents do not specify any of these interfaces
technically (protocol, message format, transport) — they only specify
*what information must cross them* (see
[REQUIREMENTS.md](REQUIREMENTS.md)). Every interface below is therefore an
**engineering decision or an explicitly unresolved question**, not a
competition requirement, unless a line item says otherwise.

## 1. Component Map

For AirMouse, the "drone" side is not necessarily five separate physical
boxes — flight controller, companion computer, and video encoder are often
one or two physical modules. We define the interfaces logically so the
GCS side doesn't need to know or care how the drone side partitions its
hardware.

| Component | Owns | Talks to |
|---|---|---|
| Flight Controller (FC) — **Pixhawk 6x** | Attitude control, motor output, low-level failsafes (battery, link-loss, geofence) | Onboard Autonomy System, via MAVLink/`mavros` |
| Onboard Autonomy System — **Jetson Nano**, running ROS + `rosbridge_server` | SLAM/mapping, survivor detection fusion, grid localisation, exploration/path planning, mission state machine | FC, Video System, **GCS** (via Comm Link) |
| Mapping System | Occupancy/connectivity grid construction (logically part of Onboard Autonomy) | Onboard Autonomy → GCS |
| Survivor Detection/Localisation System | Detects survivors, resolves to grid coordinate (logically part of Onboard Autonomy) | Onboard Autonomy → GCS |
| Video System | Camera capture + encode | GCS (separate stream from telemetry) |
| **GCS (this repo)** | Render + 2 commands | Onboard Autonomy System, Video System |

The FC is not expected to talk to the GCS directly — the Onboard Autonomy
System is the single source of truth that the GCS's Comm Link talks to,
aggregating whatever it needs from the FC internally. This keeps the
cross-air-gap protocol to one logical peer instead of several, which
matters because that link is also the one place the rules (no external
network, no tethers) most constrain design freedom.

## 2. The Drone ↔ GCS Interface (the one interface this repo must honor)

Everything below crosses the "local wireless link" box in
[ARCHITECTURE.md](ARCHITECTURE.md) §2. Per [DECISIONS.md](DECISIONS.md)
D-0/D-1/D-2, this link is now a concrete, working default rather than
fully open: **Jetson Nano** (companion computer, running ROS +
`rosbridge_server`) ↔ **GCS backend** (`gcs/backend/`, a FastAPI service
connecting via **`roslibpy`** over WebSocket, port 9090). The GCS
frontend does not talk to rosbridge directly — it calls the backend's
REST API, which re-exposes this data as plain JSON (D-0). The Jetson
itself talks to a **Pixhawk 6x** flight controller via **MAVLink**/
`mavros` — that hop is internal to the drone side and out of this repo's
scope (§3 below).

### 2.1 Channels

Two logically separate channels, for the reasons in ARCHITECTURE.md §5.3
(video must not block time-critical control/telemetry) — reinforced by
DECISIONS.md D-6/D-10, which flags queued video delaying the Abort command
as a safety issue, not just a UX one:

1. **Control/Telemetry channel** — bidirectional, low-bandwidth,
   latency-sensitive. This is the `rosbridge_server` WebSocket connection
   (port 9090). Carries mission status, telemetry, map, detection events,
   heartbeat, and the two operator commands.
2. **Video channel** — unidirectional (drone → GCS), high-bandwidth.
   Carries the live camera feed. **Deliberately kept off the rosbridge
   connection** (D-6) — a separate transport (leaning MJPEG-over-HTTP or
   WebRTC, per D-3) served directly from the Jetson.

### 2.2 Message Types (Control/Telemetry channel)

Schemas for each are in [DATA_MODELS.md](DATA_MODELS.md). Topics and
directions, reflecting the working ROS topic list (D-2, D-13):

| Topic | Type | Direction | Rate | Required by rules? |
|---|---|---|---|---|
| `/mission/state` | custom (`std_msgs/String` minimum) | Drone → GCS | On change | Yes — "mission progress and completion status" |
| `/mavros/battery` | `sensor_msgs/BatteryState` | Drone → GCS | 1–2 Hz | Yes — vehicle health, feeds "mission status" |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Drone → GCS | 10+ Hz | Yes — "drone position or estimated drone position" |
| `/map` | `nav_msgs/OccupancyGrid`, full grid each publish | Drone → GCS | 1–5 Hz | Yes — "2D map...continuously updated" |
| `/coverage_grid` | `nav_msgs/OccupancyGrid` (searched/unsearched, not walls) | Drone → GCS | ~4 Hz | Supports "explored/search coverage" visualization |
| `/planned_path` | `nav_msgs/Path` | Drone → GCS | On replan | Supports "planned path" visualization |
| `/telemetry/state` | custom, normalized JSON (`std_msgs/String`) | Drone → GCS | ~2 Hz | Supports autonomy/mapping/navigation state, sensor health |
| `/vision/survivors` | custom (D-13) | Drone → GCS | On detection | Yes — "grid coordinate...containing each detected survivor" |
| `/gcs/heartbeat` | custom, minimal | Drone → GCS | 1 Hz | Not explicitly required; recommended (link liveness) |
| `/gcs/command` | custom (`std_msgs/String`, `"start"`/`"abort"`) | **GCS → Drone** | On operator action | Yes — the *only* two permitted operator actions |

**Updated by the NIDAR Autonomy Migration** (folding gps_denied/raj-dev's
mapping/exploration/telemetry stack into `onboard-autonomy` as the single
canonical autonomy system — see `CHECKPOINT/CURRENT_STATE.md`): `/map`
supersedes the `/slam/map` name this table previously used (nothing had
ever published to it); `/coverage_grid`, `/planned_path`, and
`/telemetry/state` are new. Full shape and rationale for each:
`CHECKPOINT/docs/gcs_telemetry_contract.md`. `/telemetry/state` is a
read-only *summary* — it does not duplicate `/map`'s/`/coverage_grid`'s
full cell data or `/planned_path`'s full pose list, which stay on their
own native topics (see that doc's "two kinds of topic" section).

No other message type should exist on this channel. In particular: **no
waypoint, no path correction, no map-edit, no tag-correction, no
mission-replan message/topic is defined**, deliberately, per Design
Principle 1 in [ARCHITECTURE.md](ARCHITECTURE.md). If a future need
appears to add one, that is a signal to stop and re-check it against the
rules before writing it, not a routine schema change.

`/gcs/command` is currently the least-defined piece of this table — see
DECISIONS.md D-10. It needs a subscriber node on the Jetson and a latency
test under realistic (video-present) link load before it's considered
done, given it carries the operator's abort authority.

### 2.3 Video Channel

One direction, drone → GCS, continuous during flight, **not** via
`rosbridge_server`/roslibjs (D-6). Format/protocol still open (D-3) —
leaning MJPEG-over-HTTP first for simplicity and robustness to a lossy
link, WebRTC if latency proves to be a problem in testing.

**Implemented for the dev pipeline (2026-09-10):** MJPEG-over-HTTP,
served directly from the Jetson by `perception_node.py`
(`MJPEGStreamServer`, default `:8090/stream.mjpg`), consumed directly by
the frontend's `CameraPanel.tsx` `<img>` element — bypassing both
rosbridge and the FastAPI backend for the actual video bytes, exactly as
D-6 recommends. The backend's role is limited to relaying the small JSON
`GET /api/camera/status` (stream URL + connection/resolution/fps) sourced
from `/perception/status` over the normal rosbridge/telemetry channel —
see §2.5 below and `DATA_MODELS.md` §5A.

### 2.5 Perception (Development) — new read-only surface

Added alongside the perception pipeline (`DATA_MODELS.md` §5A), **not**
part of the "no other message type" rule in §2.2 above since it adds no
new *operator-facing command* — it's read-only, display-only, exactly
like `/map`/`/coverage_grid`. Two new Control/Telemetry-channel topics
(`/perception/detections`, `/perception/status`, both `std_msgs/String`
JSON, drone → GCS), and three new **GET-only** FastAPI routes:

| Route | Sourced from | Purpose |
|---|---|---|
| `GET /api/perception/detections` | `/perception/detections` | Raw, unconfirmed, image-space person detections (dev pretrained model) |
| `GET /api/perception/status` | `/perception/status` | Camera/detector health, person count, fps |
| `GET /api/camera/status` | `/perception/status` + `Settings.camera_stream_port` | Camera connection + the MJPEG `stream_url` (§2.3) |

None of these are new operator actions and none mutate anything — the
command surface stays exactly `/api/command/start`/`/api/command/abort`.
Deliberately **not** the same as `/vision/survivors`/`/api/survivors`
(§2.2) — see `DATA_MODELS.md` §5A for why the two must stay distinct.

### 2.6 Mission/Test Selection (Development/Bench-Test) — new surface

Added alongside the Mission/Test Selection and Execution System
(2026-09-15): lets the operator pick *which* mission profile a
subsequent real `"start"` applies to — the canonical Main NIDAR
Competition mission, or one of a small set of mock-only Flight Test
scenarios used for bench validation (see `nidar_autonomy/flight_test/`).
**This does not add a new operator action beyond start/abort** — same
reasoning as §2.5: `/gcs/mission_select` only ever carries *routing
metadata* consulted at the moment a `"start"` on `/gcs/command` arrives,
never a command by itself, and it can never cause a `"start"`/`"abort"`
to be acted on. The command surface stays exactly
`/api/command/start`/`/api/mission/start`/`/api/command/abort` — the
middle one is still only ever a parameterized `"start"`, not a new kind
of action (see `gcs/backend/app/main.py`'s module docstring).

One new Control/Telemetry-channel topic, GCS → Drone (`std_msgs/String`
JSON):

| Topic | Type | Direction | Rate | Purpose |
|---|---|---|---|---|
| `/gcs/mission_select` | custom JSON (`std_msgs/String`) | **GCS → Drone** | On selection change | `{"mission_id", "scenario_id", "schema_version", "timestamp"}` — which mission a subsequent "start" is for |
| `/flight_test/status` | custom JSON (`std_msgs/String`) | Drone → GCS | 1 Hz | Status of the "hover" Flight Test scenario (`nidar_autonomy/flight_test/hover_test_node.py`) |
| `/flight_test/multi_step/status` | custom JSON (`std_msgs/String`) | Drone → GCS | 1 Hz | Status of the ordered multi-step Flight Test scenarios — Test 1, Test 2, ... (`nidar_autonomy/flight_test/multi_step_test_node.py`). Deliberately a **separate** topic from `/flight_test/status`, not shared — both nodes publish unconditionally every second regardless of which scenario (if any) is active, so sharing one topic would let an idle node's status overwrite the actually-active node's in the GCS's single-latest-message cache. |

Three new FastAPI routes:

| Route | Sourced from | Purpose |
|---|---|---|
| `GET /api/missions` | `app/missions.py`'s static registry | List available missions and their scenarios (id, name, description, ordered `steps`, `implemented`) |
| `POST /api/mission/start` | validates against the registry, then publishes `/gcs/mission_select` immediately followed by the same `"start"` on `/gcs/command` that `/api/command/start` sends | Start a specific mission/scenario — 404 unknown mission, 400 unknown/not-yet-implemented scenario, 409 if a mission is already active |
| `GET /api/flight-test/status`, `GET /api/flight-test/multi-step/status` | `/flight_test/status`, `/flight_test/multi_step/status` | Read-only scenario progress for the GCS UI |

**Mock-only, by construction, and dev/bench-only in the GCS UI**:
`nidar_autonomy/flight_test/` never imports `mavros_msgs` and never
touches a real Pixhawk (see that package's `__init__.py` and each node's
own docstring) — Flight Test scenarios only ever drive
`MockFlightController`. The GCS frontend's mission/scenario selector
(`MissionSelectPanel.tsx`) is gated behind a separate build-time flag
(`VITE_ENABLE_MISSION_SELECT`), same isolation pattern as the "RUN
SIMULATION" panel (§5) — never shown in a competition-deployed build,
per `custom-gcs/CLAUDE.md` Important Constraint #1. The Main NIDAR
Competition mission (`mission_id="main_nidar"`) remains reachable through
the plain, always-available `/api/command/start` exactly as before this
addition — selecting it explicitly via `/api/mission/start` is equivalent
and additive, not a replacement.

### 2.4 Transport / Protocol

**Decided (working default), see DECISIONS.md D-0/D-1/D-2:** `mavros`/
MAVLink for the Pixhawk↔Jetson hop (standard, out of this repo's scope);
ROS topics over `rosbridge_server` + `roslibpy` for the Jetson↔GCS-backend
hop, using standard ROS message types wherever one exists and two small
custom message types (`/vision/survivors`, `/mission/state`) where it
doesn't. The frontend sits behind the GCS backend's REST API, not on this
connection directly (D-0). Video is intentionally routed around this same
connection (§2.3).

Remaining open items on this decision (tracked in DECISIONS.md, not
duplicated here): the physical RF hardware for the Jetson↔GCS local link
(D-1's residual item), and whether rosbridge holds up under combined
telemetry+map+command load once video is correctly kept off it (D-6).

This decision was made jointly with the drone-side/companion-computer
team, per the interface having two owners (§4 below) — not unilaterally
by the GCS side.

## 3. Interfaces Not Owned By This Repository (context only)

These exist on the drone side and are documented here only so the
boundary is unambiguous — this repo does not implement or specify their
internals, only what crosses into the Drone ↔ GCS interface above.

- **FC ↔ Onboard Autonomy System**: **decided** — MAVLink via `mavros`,
  between the Pixhawk 6x and the Jetson Nano (DECISIONS.md D-2). Entirely
  a drone-side concern to implement; noted here only because it's the
  source of the `/mavros/*` topics the GCS consumes.
- **Onboard Autonomy ↔ Mapping System**: internal SLAM pipeline
  (algorithm unspecified by the rules). Drone-side concern; the GCS only
  sees its *output* via `MapDelta`.
- **Onboard Autonomy ↔ Survivor Detection System**: internal detection/
  fusion pipeline. Drone-side concern; the GCS only sees its *output* via
  `SurvivorDetection`.
- **Onboard Autonomy ↔ Video System**: however the drone-side team wires
  the camera/encoder to whatever transmits the video channel.

## 4. Interface Stability Note

Because the drone-side system and this GCS will very likely be built by
different people/subteams on different timelines, the Drone ↔ GCS
interface in §2 is the single most important contract in the whole
project. Recommend treating [DATA_MODELS.md](DATA_MODELS.md) as a
versioned, reviewed schema (not something either side edits unilaterally),
and building the simulator described in
[ARCHITECTURE.md](ARCHITECTURE.md) §6 against that schema as early as
possible so both sides can develop against a stable contract instead of
against each other's in-progress code.

## 5. Simulation Channel (Not Part Of The Drone ↔ GCS Interface)

A second, entirely separate topic surface exists under the `/simulation/`
namespace (`/simulation/command`, `/simulation/mission/state`,
`/simulation/map`, etc.) — added 2026-09-10 for the GCS's dev/bench-only
"RUN SIMULATION" feature (gated off by default in the frontend build —
see `README.md`). It never overlaps with, redefines, or substitutes for
any topic in §2's table: `/simulation/command` accepts only
`"run"`/`"reset"`, never `"start"`/`"abort"`, and the real
`/gcs/command`/`/mission/state` topics are untouched by it. Full design:
`../../CHECKPOINT/docs/simulation_architecture.md`. Not documented
further here since it is not part of the real Drone ↔ GCS contract this
file describes — this section exists only so a reader of this file knows
the second channel exists and where to find it.

## 6. Command Radio (START/ABORT) -- MicroLR900, terminates at the Jetson

Added 2026-10-07. START and ABORT no longer travel over Wi-Fi/rosbridge.
They go over a dedicated 900 MHz MicroLR900 radio pair whose far end is
plugged into the **Jetson** (USB serial) -- the Pixhawk is not on this
link. Telemetry, map and video keep using the channels in §2.

```
GCS backend (app/radio_link.py, COM5 @ 115200)
  )))  MicroLR900  )))  MicroLR900
Jetson radio_command_node (/dev/serial/by-id/...CP2102..., 115200)
  -> /gcs/mission_select {"mission_id": "hover"} -> /gcs/command "start"
  -> command_node -> hover mission (NidarFiles/missions/hover) -> MAVROS -> Pixhawk
```

Wire format (private two-node MAVLink2 network; GCS 255/190, Jetson 1/191):

| Message | Direction | Fields |
|---|---|---|
| `COMMAND_LONG` (`MAV_CMD_USER_1` = 31010) | GCS → Jetson | `param1` 1 = START / 2 = ABORT, `param2` nonce (< 2^24, reused on resend), `param3` magic 4242, `param4` mission code (Hover = 1), `param7` protocol version 2 |
| `COMMAND_ACK` | Jetson → GCS | `result` MAV_RESULT, `progress` reason code (0 OK, 1 UNKNOWN_MISSION, 2 BAD_PROTOCOL_VERSION, 3 MISSION_NOT_READY, 4 FCU_NOT_CONNECTED, 5 MISSION_BUSY), `result_param2` = request nonce |
| `HEARTBEAT` (1 Hz) | both | Jetson's `custom_mode` = mission state code |

Rules: each nonce is acted on once (resends are re-ACKed with the original
answer); ABORT is always accepted; an unknown mission is rejected, never
mapped to another mission. The GCS reports success only for an ACK with
`result` = ACCEPTED -- no ACK is an error (HTTP 504), a rejection is HTTP
409 with the reason. Definitions: Jetson
`onboard-autonomy/nidar_autonomy/nidar_autonomy/telem_command_codec.py`,
GCS mirror `gcs/backend/app/radio_protocol.py` (keep in sync).

The GCS no longer publishes `/gcs/command` or `/gcs/mission_select` over
rosbridge; on the Jetson those topics are published only by
`radio_command_node`.

### 6.1 Telemetry over the radio (no Wi-Fi link)

Added 2026-10-07. The drone has **no Wi-Fi link** to the GCS: no hotspot,
no rosbridge. The MicroLR900 radio is the only Jetson ↔ GCS link, so
`radio_command_node` also relays telemetry over it
(`onboard-autonomy/.../radio_telemetry.py`), and the GCS backend turns it
back into the same ROS-shaped data the panels read
(`gcs/backend/app/radio_telemetry.py`, selected by
`GCS_TELEMETRY_SOURCE=radio`, the default; `rosbridge` remains for
development against `sim/`).

| Message | From | Content | Rate |
|---|---|---|---|
| `HEARTBEAT` | 1/191 Jetson | mission state (`custom_mode`), as above; link-up signal | 1 Hz |
| `STATUSTEXT` | 1/191 Jetson | hover mission `detail` (chunked if > 50 chars) | on change, repeated every 5 s |
| `NAMED_VALUE_FLOAT` | 1/191 Jetson | `hv_alt`, `hv_tgt`, `hv_dur`, `hv_elap` (hover altitude, target, hold time, elapsed) | 1 Hz |
| `HEARTBEAT` | 1/1 FCU relay | armed / guided (`base_mode`), ArduCopter mode number (`custom_mode`), `system_status`; sent only while MAVROS is connected | 1 Hz |
| `SYS_STATUS` | 1/1 FCU relay | battery voltage (mV), current (cA, ROS sign), remaining (%); unknown = 65535 / -1 / -1 | 1 Hz |
| `LOCAL_POSITION_NED` | 1/1 FCU relay | x y z vx vy vz | 2 Hz |
| `ATTITUDE_QUATERNION` | 1/1 FCU relay | q1..q4 = w x y z | 2 Hz |
| `STATUSTEXT` | 1/1 FCU relay | `/mavros/statustext/recv` (≤ 2 frames per tick, queue of 20) | as received |

Component 1/1 carries the **Pixhawk's state as MAVROS reports it**,
relayed by the Jetson; the Pixhawk itself is still not on the radio.
Position, velocity and attitude are in the ROS ENU/FLU frame MAVROS
publishes, **not** NED -- both ends are ours and the GCS shows the ROS
values unchanged. Stale inputs are not sent (and the GCS drops values
older than a few seconds), so a lost link or FCU shows as missing data,
never as frozen data. Load at the default 2 Hz is about 250 B/s, kept
low because START/ABORT ACKs share the radio's air time
(TODO(hardware): measure the radio's real throughput before raising it).

Not available without Wi-Fi: live video (MJPEG, §2.3), the map/coverage/
path topics and perception -- the radio is too slow for them. None of
these are used by the hover mission.
