# Engineering Decisions Log — NIDAR AirMouse GCS

This log tracks decisions that are **ours to make** — the competition
documents don't dictate them (contrast with [REQUIREMENTS.md](REQUIREMENTS.md),
which is only things the documents *do* dictate). Each entry: the
question, the current status, the leading option and why, and what would
change it.

Format inspired by lightweight ADRs. Add new entries at the top of their
section. Don't delete superseded entries — mark them superseded and say
why, so future-us (or a teammate) can see the reasoning trail instead of
just the current answer.

---

## Decided

### D-1: Drone ↔ GCS transport & link technology
**Status:** Decided (working default). Companion computer = **Jetson
Nano**, flight controller = **Pixhawk 6x**, connected via **MAVLink**
(through `mavros`). Jetson exposes ROS topics to the GCS via
**`rosbridge_server`** (WebSocket, port 9090); the GCS frontend consumes
them directly via **roslibjs**. The physical link between Jetson and GCS
is a **local/private wireless link** (not internet, not "public Wi-Fi" —
consistent with Rulebook §8.4/Annexure 2 §7 which bans public Wi-Fi/GSM/
LTE/5G/internet specifically, not a private point-to-point link).
**Residual open item:** the actual RF hardware for that local link (a
dedicated 5 GHz WiFi bridge vs. an on-arena AP vs. something else) is not
yet chosen, and range/interference inside a netted 15×15 m arena hasn't
been tested. Video is deliberately **not** assumed to share this same
rosbridge connection — see D-6.
**Supersedes:** the original "Open" status of D-1 in earlier drafts of
this log.

### D-2: Message protocol on the control/telemetry channel
**Status:** Decided (working default). **MAVLink** between Pixhawk and
Jetson (via `mavros`, standard). **ROS topics over rosbridge** (JSON-RPC
over WebSocket, consumed by roslibjs) between Jetson and GCS, using
**standard ROS message types** wherever one exists (`nav_msgs/OccupancyGrid`,
`sensor_msgs/BatteryState`, `geometry_msgs/PoseStamped`) plus two
AirMouse-specific topics that still need explicit message-type definitions:
`/vision/survivors` and `/mission/state` (currently only described in
prose — see D-13, "Not Yet Specified" below).
**Why this beats the earlier custom-vs-MAVLink framing:** using standard
ROS/MAVLink messages where they already exist removes almost all custom
protocol work; only the two mission-specific topics need a bespoke schema.
**Supersedes:** the earlier "custom lightweight protocol vs. MAVLink"
framing — the actual answer turned out to be "both, split by hop."

### D-7: Grid coordinate convention
**Status:** Decided (working default), pending confirmation with the
drone-side team. Use `nav_msgs/OccupancyGrid`'s own `MapMetaData`
(`resolution`, `origin`) as the authoritative coordinate frame: **origin
pinned to the designated entry/exit point** (arena-relative, not GPS —
GPS is unavailable indoors by rule). Map cells and `/vision/survivors`
coordinates are both derived from this same `(origin, resolution)` pair,
so they cannot drift apart from each other. The GCS-facing/scored
representation is quantized to **1.0 m cells**: `cell = (floor((x -
origin_x) / 1.0), floor((y - origin_y) / 1.0))`, matching the Rulebook §9
scoring grid.
**Resolves:** REQUIREMENTS.md Open Question #3 (labeling convention) —
partially. The origin/resolution/quantization scheme is settled; the
*display* labeling (numeric vs. chess-style) is still cosmetic and open.
Still unresolved: how the team's independently-built grid is verified
against the organiser's reference grid at scoring time — that's an
organiser-side process question, not something we can settle unilaterally.
**Supersedes:** earlier placeholder `(col, row)` indices in DATA_MODELS.md
with no defined origin.

### D-7a: SLAM working resolution vs. GCS-facing map resolution
**Status:** Decided (working default). SLAM keeps whatever internal
working resolution it needs for good localization/mapping quality (e.g.
0.05–0.2 m) — that stays on the Jetson and is not sent to the GCS. A
**separate, coarser `OccupancyGrid`** — fine enough to resolve wall/
opening detail within a 1 m cell (recommend 0.25–0.5 m if wall thickness
needs to be visible, or straight at 1.0 m if not) — is published on
`/slam/map` specifically for the GCS.
**Why:** keeps the websocket payload small. At true 1.0 m resolution over
a ≤15×15 m arena, the full grid is **≤225 `int8` cells** — small enough to
resend in full every cycle at 1–5 Hz with no meaningful bandwidth cost.
**Revisit when:** the drone-side team has real SLAM output to test wall-
detection fidelity against — if 1.0 m native resolution loses too much
wall detail, move to 0.25–0.5 m internally and bin down to 1.0 m only for
the GCS-facing topic.

### D-9: Full-grid republish instead of incremental map deltas
**Status:** Decided — **supersedes** the original `MapDelta`
(incremental-cell-update) design in an earlier draft of
[DATA_MODELS.md](DATA_MODELS.md). Given D-7a's resolution numbers, the
whole map is cheap enough to send in full every publish (this is also how
`nav_msgs/OccupancyGrid`-producing SLAM stacks naturally behave — they
don't diff internally). Incremental deltas added complexity with no
payoff at this data size and were dropped.
**Revisit when:** if a much finer resolution is ever sent directly to the
GCS (not currently planned per D-7a), full-grid resend may become
expensive again and delta-encoding would be worth reconsidering.

### D-13: `/vision/survivors` and `/mission/state` message shapes
**Status:** Decided (working default) — see
[DATA_MODELS.md](DATA_MODELS.md) §4/§1 for the concrete field lists. Not a
standard ROS message for either; both need small custom `.msg` definitions
on the Jetson side. Minimum required fields for survivors: stable id
(so re-detections update rather than duplicate — capped at 6), grid
coordinate (or raw x/y to be quantized per D-7), confidence. Minimum for
mission state: an enum/string phase value, published on change.
**Revisit when:** the drone-side team defines the actual `.msg` files —
this entry should be updated to reference them by name once they exist.

### D-0: Overall technology stack
**Status:** Decided and **implemented** (Phase 1). **Reinstates a backend
layer** — see "D-0 history" below for why the intermediate "no backend
needed" version of this entry was wrong.

- **GCS Backend:** a FastAPI service (`gcs/backend/`) is the only thing
  that talks to rosbridge. It owns a `roslibpy` connection (real Jetson or
  `sim/rosbridge_sim`, chosen by host/port config — `app/config.py`),
  caches the latest value per topic, and exposes a plain REST API:
  `GET /health`, `GET /api/telemetry`, `GET /api/map`, `GET /api/survivors`,
  and exactly two mutating routes, `POST /api/command/start` and
  `POST /api/command/abort`. Auto-generated Swagger UI at `/docs` — every
  route, including the two commands, is testable by hand with no frontend
  written yet.
- **GCS Frontend (not yet built):** will call the backend's REST API, not
  rosbridge directly. Live data delivery is **REST polling**, not a
  WebSocket push from the backend (explicit tradeoff, chosen for now —
  simpler to build and to test via Swagger; revisit if polling lag on
  10 Hz pose proves to be a real problem once a frontend exists). Stack
  choice for the frontend itself (React/TypeScript, Tauri vs. Electron vs.
  browser) is unchanged from the original reasoning below and still
  pending — nothing in `gcs/backend/` depends on that choice.

**Why reinstating this is right, not just "back to how it was":** the
"talk to rosbridge directly" version optimized away a layer that turned
out to be pulling real weight — (1) **testability**: every route,
including the two commands, is clickable in Swagger before any frontend
exists; (2) **stronger safety encapsulation**: "the operator can only
start/abort" becomes true because *no other mutating route is defined* in
the backend, not just because the frontend promises not to call one — see
`gcs/backend/tests/test_api_with_fake_client.py::test_no_route_exists_beyond_the_documented_command_surface`;
(3) **frontend/ROS decoupling**: the frontend will only ever see
flattened, backend-owned JSON schemas (`app/schemas.py`), never
`PoseStamped`/`OccupancyGrid`/rosbridge's envelope format directly.
**Cost, stated plainly:** one more process, one more hop. Irrelevant at
this data rate (≤10 Hz telemetry, not a control loop).

**Original reasoning for the frontend stack (still holds):** web rendering
stack (Canvas/WebGL, `<video>`) is the fastest path to a live map+video+
status UI for a student team on this timeline; a native shell (Tauri,
tentative) serves the offline-only constraint and enables local file
access for mission logging.
**Alternatives considered and set aside (not ruled out):** a native
Qt/PySide desktop app for the frontend.
**Revisit when:** frontend implementation begins — the WebSocket-vs-
polling tradeoff above and the native-shell choice both deserve a
concrete look once there's a real UI to build against.

**D-0 history (why this entry changed twice):** Phase 0 originally
proposed a bridge layer. After the Jetson/rosbridge stack was locked in
(D-1/D-2), this entry was revised to remove that layer entirely, reasoning
that `roslibjs` could talk to `rosbridge_server` directly from the
frontend with no backend needed. That revision shipped as working code
(a `sim/`-only Phase 1 milestone) before the tradeoff above was raised and
reconsidered — the backend is now built and is not a hypothetical.

---

## Open / Unresolved

### D-6: Video transport — keep it off rosbridge
**Status:** Open, but with a strong recommendation: **do not** send
`/camera/image_raw/compressed` through the same rosbridge/websocket
connection as telemetry/map/command traffic.
**Why:** rosbridge serializes `sensor_msgs/CompressedImage.data` (a byte
array) as base64-in-JSON — real overhead — over a single connection shared
with everything else. At 15–30 fps this risks two things: (1) saturating
the link/serialization thread such that map/telemetry updates lag, and
(2) worse, **delaying the Abort command** behind queued video traffic,
which is a safety issue, not just a UX one, given Abort is one of exactly
two things the operator is allowed to do. This is also consistent with
[ARCHITECTURE.md](../docs/ARCHITECTURE.md) §5.3's original principle that
video must never head-of-line-block time-critical control traffic.
**Leading option:** a small, separate video path from the Jetson — e.g. an
MJPEG-over-HTTP endpoint or a WebRTC stream — consumed directly by the
frontend's `<video>` element, independent of `rosbridge_server`.
**Revisit when:** real link-bandwidth numbers exist from hardware testing;
if rosbridge genuinely keeps up under load, this recommendation could be
relaxed, but should be *proven*, not assumed.
**Implemented for the dev pipeline, 2026-09-10:** the leading option above
is now real code, not just a recommendation — `onboard-autonomy`'s
`perception_node.py` runs a stdlib-`http.server` MJPEG endpoint directly
on the Jetson (default `:8090/stream.mjpg`), and `custom-gcs`'s
`CameraPanel.tsx` consumes it directly via an `<img>` tag, with the
FastAPI backend only relaying the small JSON status/URL
(`GET /api/camera/status`) — never the video bytes themselves. This
**does not** close D-6 as a final decision: it's the dev-pipeline
implementation of the already-leading option, still subject to revisit
once real link-bandwidth/hardware numbers exist (the "Revisit when"
above still applies) and once real RF hardware (D-1) is chosen. See
`custom-gcs/docs/DATA_MODELS.md` §5A.3 for the concrete contract.

### D-10: Command channel (Start / Abort), GCS → Jetson
**Status:** Open — **not yet defined**, and this is the single most
safety-critical wire in the system. The topic table the drone-side team
provided is entirely Jetson→GCS (heartbeat, battery, pose, map, survivors,
video, mission state); there is currently no defined path for the two
things the operator is actually allowed to do.
**Leading option:** a `/gcs/command` topic (`std_msgs/String`, values
`"start"` / `"abort"`), published from the frontend via roslibjs,
subscribed by a small command-handling node on the Jetson that forwards
into the mission state machine / triggers the flight controller's abort
failsafe.
**Must be validated, not assumed:** latency of this specific topic under
realistic link load (i.e., with video and map traffic present), given D-6.
An abort command that arrives late because video was hogging the
connection is a disqualification/safety-incident risk, not an edge case.
**Revisit when:** before this is treated as "done" — needs an actual
latency test, not just a working demo in quiet conditions.

### D-11: Indoor pose source for `/mavros/local_position/pose`
**Status:** Open — needs confirmation from the drone-side/flight-control
subteam, informational for GCS design (affects how much we trust/display
`position_confidence`). The rules forbid GPS/GNSS-based navigation
indoors, so Pixhawk's EKF needs a substitute position source for
`local_position/pose` to mean anything indoors — most likely SLAM pose
(Jetson) → MAVLink `VISION_POSITION_ESTIMATE` → EKF fusion, via mavros's
vision_pose plugin. If this pipeline isn't wired, the pose topic will
drift or be meaningless indoors, and the GCS would be rendering garbage
drone-position data without any way to know it.
**GCS-side implication:** don't assume `/mavros/local_position/pose` is
trustworthy by default — surface some notion of pose validity/confidence
if the drone-side telemetry can provide it (ties to the `LinkHealth`
concept in [DATA_MODELS.md](DATA_MODELS.md) §5).
**Revisit when:** the flight-control subteam confirms the vision-pose
fusion pipeline is implemented and tested.

### D-12: Jetson Nano compute budget (informational, drone-side)
**Status:** Open, not a GCS-repo decision, but worth logging because it
directly affects what the GCS will observe (dropped/late messages look
identical to a bad link regardless of root cause). Running SLAM +
detection + video encode + `rosbridge_server` concurrently on a Jetson
Nano (a 2019-era, comparatively weak board by current detection-model
standards) is a real resource-contention risk.
**Revisit when:** the drone-side team load-tests the full stack together,
not just each component in isolation.

### D-3: Video protocol/format — merged into D-6
**Status:** Superseded by D-6 above, which now also settles *where* video
goes (off rosbridge), not just its codec. Original framing (MJPEG vs.
RTSP vs. WebRTC) is still the live sub-question once D-6's "separate
path" is agreed — leaning MJPEG-over-HTTP first for simplicity/link
robustness, revisit if latency is a problem in testing.
**Implemented for the dev pipeline, 2026-09-10:** MJPEG-over-HTTP, per
D-6's update above — not WebRTC. Revisit if/when real-link latency
testing shows it's insufficient.

### D-4: Grid coordinate labeling convention — mostly resolved by D-7
**Status:** Mostly superseded by D-7 above (origin = entry/exit point,
1.0 m quantization). What D-7 does *not* settle: the organiser-facing
verification process for aligning the team's grid to the reference grid
at scoring time — still genuinely unknown, likely needs a direct
organiser clarification.

### D-5: Mission logging / replay format
**Status:** Open, still leaning yes-build-it (see
[ARCHITECTURE.md](../docs/ARCHITECTURE.md) §3.4). Given D-0's update, this
is now a smaller, more self-contained piece of work than originally
scoped (no longer entangled with a general-purpose bridge process) — could
plausibly even be a lightweight roslibjs subscriber-that-writes-to-disk
running in the same Tauri app, rather than a separate Python service.
Format: likely a flat, append-only local log (newline-delimited JSON of
every message received/sent, with timestamps) that a `sim/` replay tool
can play back.
**Revisit when:** the topic/message schemas in
[DATA_MODELS.md](DATA_MODELS.md) are stable enough to log without
constant format churn.
