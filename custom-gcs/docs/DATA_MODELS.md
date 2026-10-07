# Data Models — NIDAR AirMouse GCS

Status: **Phase 0 — working schemas, tied to the stack decisions in
[DECISIONS.md](DECISIONS.md) (D-0 through D-13). Not yet implemented.**
Frontend consumes these directly via **roslibjs** over
**`rosbridge_server`** (WebSocket, port 9090) — see
[COMMUNICATION.md](COMMUNICATION.md). Field names below reflect either a
standard ROS message type (used as-is) or a proposed custom `.msg`
(marked "custom", to be finalized with the drone-side/companion-computer
team, per D-13).

Grid convention (per D-7/D-7a): a **1 m × 1 m cell grid** over the
≤15 m × 15 m arena (≤225 cells), origin pinned to the designated entry/
exit point, derived from whatever `nav_msgs/OccupancyGrid.info` (resolution
+ origin) the map topic reports. Survivor coordinates and map cells share
this exact origin/resolution so they cannot drift apart from each other.

## 1. Mission Status — `/mission/state`

**Type:** custom (proposed: a small `.msg` with an enum-like string field;
`std_msgs/String` is the minimum viable version). Published **on change**,
not periodically.

```jsonc
// std_msgs/String equivalent, as seen by roslibjs
{
  "data": "searching"   // "idle" | "entering" | "searching" | "exiting" | "complete" | "aborted"
}
```

If richer state is useful later (elapsed time, survivors found so far),
that belongs in a custom message rather than overloading `std_msgs/String`
— not yet decided; start minimal, expand only if needed (see D-13).

## 2. Telemetry — Battery: `/mavros/battery`

**Type:** `sensor_msgs/BatteryState` (standard `mavros` topic — no custom
work needed). Published 1–2 Hz. GCS reads `percentage` and `voltage`.

## 3. Telemetry — Pose: `/mavros/local_position/pose`

**Type:** `geometry_msgs/PoseStamped` (standard `mavros` topic). Published
10+ Hz. Position is arena-relative (local ENU frame from Pixhawk's EKF),
**never GPS** — GPS is unavailable/forbidden indoors by rule.

**Caveat (see DECISIONS.md D-11, not a GCS-side fix):** this topic is only
meaningful if Pixhawk's EKF has a non-GPS position source indoors —
presumably SLAM pose fed back via MAVLink `VISION_POSITION_ESTIMATE`. The
GCS should not assume this pose is automatically trustworthy; if the
drone-side telemetry ever exposes a confidence/validity flag, surface it
(see `LinkHealth`, §6 below) rather than rendering position blindly.

## 4. 2D Map — `/map`

**Renamed from `/slam/map`** by the NIDAR Autonomy Migration (see
`CHECKPOINT/CURRENT_STATE.md`) — nothing had ever published to `/slam/map`
(it was a Phase-0-era placeholder), so this is a strict fill-in, not a
breaking change to a working consumer. Full contract, including the
sibling topics below:
`CHECKPOINT/docs/gcs_telemetry_contract.md`.

**Type:** `nav_msgs/OccupancyGrid` (standard — this is the right call, see
D-1/D-7a). Published 1–5 Hz, **full grid each time** (not incremental —
see D-9; at 1 m resolution over ≤15×15 m this is ≤225 `int8` cells, cheap
enough to resend in full).

```jsonc
// nav_msgs/OccupancyGrid, as seen by roslibjs
{
  "header": { "stamp": { "sec": 1745, "nanosec": 0 }, "frame_id": "map" },
  "info": {
    "resolution": 1.0,             // meters/cell — GCS-facing topic; SLAM's internal working resolution may be finer (D-7a)
    "width": 15,
    "height": 15,
    "origin": {                    // pinned to the entry/exit point (D-7)
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    }
  },
  "data": [-1, -1, 0, 0, 100, 0, "...225 int8 values, row-major, -1=unknown 0=free 100=occupied"]
}
```

`data[row * width + col]` gives that cell's occupancy (standard
`OccupancyGrid` semantics: `-1` unknown, `0` free, `100` occupied,
intermediate values = probability). The GCS derives "walls/openings
adjoining a cell" (the scoring language, Rulebook §9) from occupied cells
along a 1 m cell's boundary — this works as long as the *source* SLAM
resolution was fine enough to place occupied cells accurately relative to
the 1 m grid lines (D-7a).

### 4.1 Coverage Grid — `/coverage_grid`

**New, added by the NIDAR Autonomy Migration.** Same `nav_msgs/
OccupancyGrid` shape as `/map` above, but different cell semantics: this
grid answers "has the camera actually looked here", not "is this a wall".
`-1` unknown (not yet mapped as free), `0` free but not yet searched,
`100` searched. Distinct from `/map` because mapping where the walls are
is not the same as pointing the camera into every place a survivor could
be — see `CHECKPOINT/docs/gcs_telemetry_contract.md` and
`onboard-autonomy/nidar_autonomy/coverage_grid.py`.

### 4.2 Planned Path — `/planned_path`

**New, added by the NIDAR Autonomy Migration.** Standard `nav_msgs/Path`,
`frame_id: "map"` — the vehicle's current planned route, as produced by
`onboard-autonomy`'s own path planner (not duplicated into
`/telemetry/state`; see 4.3). A suggestion for visualization only — see
`onboard-autonomy/nidar_autonomy/frontier_explorer_node.py`'s module
docstring for why this never implies the vehicle is actually flying it
(no autonomous flight-control setpoint issuance exists yet).

### 4.3 Normalized Telemetry — `/telemetry/state`

**New, added by the NIDAR Autonomy Migration.** `std_msgs/String`
carrying one JSON-encoded object (schema versioned, `schema_version: 1`),
built by `onboard-autonomy/nidar_autonomy/telemetry_bridge_node.py` from
`telemetry_contract.py`. Lightweight *summaries* only (map
resolution/dims, coverage percent, autonomy state, sensor health, counts)
— never the full occupancy grid or path data, which stay on their own
native topics above. Full JSON shape, field-by-field:
`CHECKPOINT/docs/gcs_telemetry_contract.md`. The GCS backend flattens
this into `TelemetryResponse.{autonomy,sensors,mapping,navigation}` (see
`gcs/backend/app/schemas.py`) alongside the pre-existing mavros-sourced
fields, which are unaffected by this addition.

## 5. Survivor Tags — `/vision/survivors`

**Type:** custom (proposed — not a standard ROS message; needs a `.msg`
file on the Jetson side, see D-13). Published on detection (and again on
re-detection/confidence update for an existing survivor).

```jsonc
{
  "survivor_id": 3,             // stable int/string id — re-detections update, not duplicate; capped at 6 distinct ids
  "x": 7.2,                     // meters, same origin/frame as /map
  "y": 5.4,
  "confidence": 0.88
}
```

The GCS is responsible for quantizing `(x, y)` into the shared 1 m grid
cell (`floor((x - origin_x)/1.0)`, `floor((y - origin_y)/1.0)`) using the
*same* `origin`/`resolution` as whatever `/map` last reported — this
keeps the Jetson-side message simple (raw meters) while guaranteeing tags
and map stay aligned. A 7th distinct `survivor_id` should be treated as a
data/integration bug to surface, not silently accepted (rules cap at 6).

## 5A. Perception Detections (Development) — `/perception/detections`, `/perception/status`

**Added 2026-09-10, NIDAR perception pipeline (dev pretrained detector).
Distinct from §5 above — read that distinction carefully before touching
either.** §5's `/vision/survivors` is a **confirmed, localized** survivor:
world-frame `(x, y)` meters, capped at 6, meant to feed the map. This
section's topics are **raw, unconfirmed, image-space** person detections
from a pretrained (not NIDAR-trained) model, running purely as a
development integration layer — a bounding box in pixel coordinates is
*not* a survivor location, and nothing here ever invents a world
coordinate from one. Nothing publishes `/vision/survivors` yet; this is
not a replacement for that gap, it is a separate, additive contract that
proves out the camera→detector→ROS→FastAPI→GCS pipeline so a future real
NIDAR-trained detector *and* a future detection→localization fusion step
(which would be the thing that finally publishes `/vision/survivors`) can
slot in without rewriting any of this.

Both `std_msgs/String` carrying one JSON-encoded object each
(`schema_version: 1`), published by
`onboard-autonomy/nidar_autonomy/perception/perception_node.py` — a
read-only observer, same category as `telemetry_bridge_node.py` et al.
(never touches mavros/arming/flight-control). Full architecture,
model selection, and config: `onboard-autonomy/nidar_autonomy/perception/README.md`.

### 5A.1 `/perception/detections`

```jsonc
{
  "schema_version": 1,
  "frame_width": 640,
  "frame_height": 480,
  "timestamp": 1234567890.123,
  "detections": [
    {
      "detection_id": "a1b2c3...",
      "class_name": "person",
      "confidence": 0.94,
      "bbox": { "x_min": 120, "y_min": 40, "x_max": 260, "y_max": 400 },
      "center_x": 190, "center_y": 220,
      "track_id": null,
      "source": "huggingface",
      "model_name": "Ultralytics/YOLO11/yolo11n.pt"
    }
  ]
}
```
`bbox` is in pixel coordinates of the `frame_width x frame_height` image
— never meters, never map-frame. The GCS backend flattens this into
`PerceptionDetectionsResponse`/`DetectionResponse` (see
`gcs/backend/app/schemas.py`), exposed read-only at
`GET /api/perception/detections`.

### 5A.2 `/perception/status`

```jsonc
{
  "schema_version": 1,
  "camera_connected": true,
  "detector_enabled": true,
  "detector_ready": true,
  "detector_backend": "huggingface",
  "model_name": "Ultralytics/YOLO11/yolo11n.pt",
  "person_count": 2,
  "fps": 8.3,
  "frame_width": 640,
  "frame_height": 480,
  "video_stream_url": "http://<jetson-host>:8090/stream.mjpg",
  "last_detection_age_s": 0.3,
  "timestamp": 1234567890.123
}
```
Flattened into `PerceptionStatusResponse` at `GET /api/perception/status`,
and into `CameraStatusResponse` (camera-only fields, plus the constructed
stream URL) at `GET /api/camera/status`.

### 5A.3 Live video — MJPEG-over-HTTP, resolves D-6/D-3 for the dev pipeline

Per D-6/D-3 (§7 below), video does not go over rosbridge. The Jetson's
`perception_node.py` runs a small stdlib `http.server`-based MJPEG
server directly (`video_stream.py`, `MJPEGStreamServer`), default
`0.0.0.0:8090`, serving `/stream.mjpg` (multipart JPEG) and
`/snapshot.jpg`. `GET /api/camera/status`'s `stream_url` field gives the
frontend the full URL; the browser's `<img>`/`<canvas>` overlay
(`CameraPanel.tsx`) fetches the actual video bytes **directly from the
Jetson**, bypassing both rosbridge and the FastAPI backend for the media
itself — the backend only ever relays the small JSON status/URL. See
DECISIONS.md D-6 for this being recorded as the dev-pipeline
implementation choice, not yet a final competition-hardware decision.

## 6. System Heartbeat — `/gcs/heartbeat`

**Type:** custom, minimal (proposed: `std_msgs/Header` or an even simpler
timestamp message). Published 1 Hz. Confirms the Jetson + websocket link
is alive — this is the cheapest possible signal and should probably be
the first thing rendered/tested end-to-end, before any other topic.

Optionally paired with a richer `LinkHealth`-style message later if the
drone-side stack can report more than just "alive" (e.g. last-message age
per topic, computed client-side by the GCS instead if the Jetson doesn't
provide it directly) — not required by the rules for PS2 (see
REQUIREMENTS.md Open Question #5), but cheap value if added.

## 7. Live Video — `/camera/image_raw/compressed` — deliberately NOT via rosbridge

**Type:** would be `sensor_msgs/CompressedImage` if sent over ROS, but per
**D-6, this should not share the rosbridge/websocket connection** with
the topics above. Recommended instead: a separate transport (MJPEG-over-
HTTP or WebRTC) served directly from the Jetson and consumed by the
frontend's `<video>` element, independent of roslibjs/rosbridge. See
[DECISIONS.md](DECISIONS.md) D-6 for the reasoning (base64+JSON overhead,
and — more importantly — risk of delaying the Abort command behind queued
video frames on a shared connection).

**Implemented for the dev pipeline as MJPEG-over-HTTP** — see §5A.3 above
for the concrete implementation (`perception_node.py`'s `MJPEGStreamServer`,
`GET /api/camera/status`'s `stream_url`). No `sensor_msgs/CompressedImage`
ROS topic is published anywhere — the dev pipeline never needed the "would
be" form above, it went straight to the off-rosbridge transport D-6
recommends.

## 8. Commands — `/gcs/command` (GCS → Jetson)

**Type:** custom, minimal (proposed: `std_msgs/String`). Published by the
frontend via roslibjs. **This is the entire operator command surface —
exactly two values, nothing else:**

```jsonc
{ "data": "start" }
{ "data": "abort" }
```

See [DECISIONS.md](DECISIONS.md) D-10 — this channel is not yet defined
by the drone-side topic table and needs a subscriber node on the Jetson
side plus a latency test under realistic (video-present) link load before
it can be considered done.

**Update 2026-10-07 — transport:** the GCS no longer publishes this topic
over rosbridge. START/ABORT travel over the MicroLR900 command radio to
the Jetson, whose `radio_command_node` publishes `/gcs/command` (preceded,
for START, by `/gcs/mission_select` `{"mission_id": "hover", "source":
"radio", "nonce": n}`). The values are unchanged — still exactly
`"start"`/`"abort"`. Radio wire format: [COMMUNICATION.md](COMMUNICATION.md) §6.

Related status topics on the Jetson. **Update 2026-10-07:** there is no
Wi-Fi/rosbridge link any more -- the GCS receives the hover status and the
FCU telemetry relayed over the radio ([COMMUNICATION.md](COMMUNICATION.md)
§6.1) and rebuilds `/flight_test/status` from it; `/radio/status` stays
on the Jetson for local debugging.

- `/flight_test/status` — the real hover mission's status (`std_msgs/String`
  JSON, ~2 Hz): `scenario`, `state` (`idle`, `preflight`, `setting_guided`,
  `arming`, `taking_off`, `hovering`, `landing`, `complete`, `aborted`,
  `failed`, `pilot_override`), `detail`, `target_altitude_m`,
  `current_altitude_m`, `current_position`, `duration_s`,
  `elapsed_hover_s`, `armed`, `flight_mode`, `execution_mode` = `"real"`.
- `/radio/status` — `radio_command_node` health (`std_msgs/String` JSON,
  1 Hz): `serial_open`, `serial_port`, `last_rx_age_s`, `fcu_connected`,
  `mission_alive`, `mission_state`, `last_command`, `dry_run`.

## 8A. Mission/Test Selection (Development) — `/gcs/mission_select`, `/flight_test/status`, `/flight_test/multi_step/status`

**Added 2026-09-15, Mission/Test Selection and Execution System.**
Routing metadata for *which* mission profile a subsequent `/gcs/command`
`"start"` applies to — **not a third operator command**, see
[COMMUNICATION.md](COMMUNICATION.md) §2.6 for why this doesn't widen §8's
"exactly two values" rule. Consulted only at the moment `"start"` is
validated; publishing this topic alone never causes anything to happen.

```jsonc
// /gcs/mission_select -- std_msgs/String, JSON-encoded
{
  "schema_version": 1,
  "mission_id": "flight_test",       // "main_nidar" | "flight_test"
  "scenario_id": "forward_backward_hover",
  "timestamp": 1234567890.1
}
```

`mission_id="main_nidar"` (or no selection ever received) routes to the
existing, unchanged Main NIDAR Competition mission
(`mission_state_node.py`) — behavior is byte-for-byte identical to before
this addition. `mission_id="flight_test"` routes to one of the mock-only
scenarios in `nidar_autonomy/flight_test/`, currently:

| `scenario_id` | Node | Steps |
|---|---|---|
| `hover` | `hover_test_node.py` | Arm → takeoff to 1.0 m → hold 10 s → land → disarm |
| `forward_backward_hover` ("Test 1") | `multi_step_test_node.py` | Arm → takeoff → forward → backward → hover (3 s) → land → disarm |
| `sideways_hover_sideways_hover` ("Test 2") | `multi_step_test_node.py` | Arm → takeoff → sideways → hover → sideways → hover → land → disarm |

Adding a future scenario is a registry addition to
`nidar_autonomy/flight_test/scenarios.py` (an ordered list of named
steps) plus the matching entry in `custom-gcs/gcs/backend/app/missions.py`
(hand-synced across the two repos, same convention as `topics.py` vs.
this document generally — no shared codegen) — not a new node, unless the
new scenario needs an action `multi_step_test_node.py` doesn't already
know how to execute.

```jsonc
// /flight_test/status or /flight_test/multi_step/status -- std_msgs/String, JSON-encoded
// (shape shown is the multi-step topic; /flight_test/status is the same
// convention with hover-specific fields instead of current_step_*)
{
  "schema_version": 1,
  "mission_id": "flight_test",
  "scenario_id": "forward_backward_hover",
  "state": "executing",   // idle|arming|executing|landing|complete|aborted|failed
  "phase": "step",
  "current_step_index": 0,
  "current_step_action": "forward",
  "total_steps": 3,
  "current_position": [1.0, 0.0, 1.0],
  "armed": true,
  "execution_mode": "mock",   // always "mock" -- an honest, permanent fact, not a placeholder
  "timestamp": 1234567890.1
}
```

`state: "failed"` is distinct from `"aborted"`: `"failed"` means the
scenario's own execution went wrong (e.g. an unrecognized action);
`"aborted"` means the operator sent `/gcs/command: "abort"`. Collapsing
the two would make a real execution bug indistinguishable from an
operator action after the fact.

## 9. What Deliberately Does Not Exist

No schema is defined — anywhere, even as an unused/future placeholder —
for: waypoint upload, path correction, map edit/correction, survivor tag
correction, or mission replanning. This omission is intentional (see
[ARCHITECTURE.md](ARCHITECTURE.md) Design Principle 1 and
[COMMUNICATION.md](COMMUNICATION.md) §2.2). If a future engineering need
seems to require one of these, treat that as a signal to revisit the
architecture and the rules together, not as a routine schema addition.

## 10. Superseded

Earlier drafts of this document proposed a generic JSON `MapDelta` message
(incremental cell updates) and a placeholder `(col, row)` grid convention
with no defined origin. Both are superseded — see DECISIONS.md D-9 (full-
grid republish is cheap enough at 1 m resolution) and D-7 (origin pinned
to entry/exit point via `OccupancyGrid.info`). Kept here as a pointer, not
reproduced, so there's one source of truth.
