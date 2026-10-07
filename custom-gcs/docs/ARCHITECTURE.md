# Architecture — NIDAR AirMouse Ground Control Station

Status: **Phase 0 — proposed architecture, not yet implemented.** See
[CLAUDE.md](../CLAUDE.md) for current project phase and the rule against
writing application code before this is agreed.

This document proposes a system architecture for the GCS and the
technology stack to build it in, with reasoning for each major choice. It
distinguishes competition **requirements** (from
[REQUIREMENTS.md](REQUIREMENTS.md)) from **engineering decisions** we are
free to make, which are tracked with rationale in
[DECISIONS.md](DECISIONS.md).

---

## 1. Design Principles Driven Directly by the Rules

Before any stack discussion, three requirements shape the architecture more
than any technology choice does:

1. **The command surface is exactly two buttons: Start, Abort.** Any
   additional operator-facing control that could modify navigation, the
   map, or survivor tags is a scored penalty (−50/instance) and undermines
   the entire premise of the mission (autonomy). The GCS must be
   architected so that this isn't just a UI convention — there must be no
   code path from operator input to mission-affecting state. The frontend
   should not even *have* the wiring for "send waypoint" or "edit map" to
   exist as dead code that a rushed late-night change could accidentally
   expose.
2. **Everything must be live, and everything must be local.** No
   post-flight completion window, no external network, no cloud. The GCS
   consumes a real-time stream from the drone during the ≤30-minute flight
   and must render map/video/telemetry/survivor updates as they arrive —
   there is no "finalize" step. Corollary: the GCS must start from a
   correct, fully offline-capable baseline (bundled assets, no CDN
   dependencies, no phone-home behavior) rather than "works online, degrade
   gracefully offline."
3. **The GCS is a passive consumer of autonomy, not a participant in it.**
   Mapping, detection, localisation, and navigation all happen onboard.
   The GCS's job is to receive, render, and (for exactly two actions)
   command. This pushes essentially all of the "hard" autonomy problems
   (SLAM, detection, path planning) out of this repository and onto the
   drone-side system — which is out of scope here, but the interfaces to
   it are in scope (see [COMMUNICATION.md](COMMUNICATION.md)).

## 2. System Context

```
┌─────────────────────────────────────────────────────────────────────┐
│                              ARENA (offline)                         │
│                                                                        │
│   ┌────────────────────────────────────────────────────────────┐    │
│   │                         DRONE                                │    │
│   │  ┌───────────────┐   MAVLink   ┌────────────────────┐       │    │
│   │  │ Flight         │◄──/mavros─►│ Onboard Autonomy /  │       │    │
│   │  │ Controller     │            │ Companion Computer   │       │    │
│   │  │ Pixhawk 6x     │            │ Jetson Nano          │       │    │
│   │  │ (attitude,     │            │  - ROS + rosbridge   │       │    │
│   │  │  motor control,│            │  - SLAM / mapping    │       │    │
│   │  │  failsafes)    │            │  - survivor detect   │       │    │
│   │  └───────────────┘            │  - path planning     │       │    │
│   │                                │  - camera / video enc│       │    │
│   │                                └──────────┬───────────┘       │    │
│   └───────────────────────────────────────────┼──────────────────┘    │
│                                                 │ local wireless link   │
│                                                 │ rosbridge_server:9090 │
│                                                 │ (telemetry+map+cmd)   │
│                                                 │ + separate video path │
│                                                 ▼                      │
│   ┌────────────────────────────────────────────────────────────┐      │
│   │                  GROUND CONTROL STATION (GCS)                │      │
│   │   operated by exactly one operator, this repository          │      │
│   └────────────────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────┘
```

Everything inside the dotted arena box must work with **no external
network**. The Jetson↔GCS link's *protocol* layer is decided (rosbridge/
roslibjs for telemetry+map+command, a separate path for video — see
[DECISIONS.md](DECISIONS.md) D-1/D-2/D-6); the physical RF hardware
underneath it is still open (D-1's residual item).

## 3. Major Subsystems and Boundaries

This repository (`custom-gcs`) owns only the **Ground Control Station**
box above. Within it:

### 3.1 Presentation Layer ("GCS UI")
Renders: live video, live 2D grid map with survivor markers and drone
position, mission status/progress, and exposes exactly two controls
(Start, Abort). Consumes a local, well-defined API from the Link/Bridge
Layer below — it does not talk to radios/serial ports/sockets directly.
This isolation matters because it lets the UI be developed and tested
entirely offline against a simulator (see §6) without any drone hardware.

### 3.2 Link / Bridge Layer ("GCS Backend") — implemented as a FastAPI service
Owns the actual communication with the drone: the only component that
connects to rosbridge (real Jetson or `sim/rosbridge_sim`), parses
inbound telemetry/map/detection into the data model defined in
[DATA_MODELS.md](DATA_MODELS.md), and is the only component allowed to
transmit the two permitted commands upstream. Exposes a local REST API
(`gcs/backend/`, built on FastAPI) to the Presentation Layer — plain JSON,
not raw ROS message shapes. Isolating this layer means the *actual* RF
technology and drone-side stack (see [DECISIONS.md](DECISIONS.md)) can
change without touching the UI, and the UI can be built and tested against
this backend pointed at `sim/` long before real hardware exists. See
[DECISIONS.md](DECISIONS.md) D-0 for why this layer was reinstated after
an intermediate design briefly removed it.

### 3.3 Communication Link (protocol + transport)
The abstract contract for what bytes/messages cross the air gap between
drone and GCS. Defined in [COMMUNICATION.md](COMMUNICATION.md) and
[DATA_MODELS.md](DATA_MODELS.md). Treated as a subsystem in its own right
because it is co-designed with the drone-side team and is the seam where
most integration risk lives.

### 3.4 Mission Recording / Local Logging (proposed, non-mandated)
Not required by the rules, but recommended: persist everything the GCS
receives and sends (video, map deltas, detections, telemetry, commands,
timestamps) to local disk during the mission. This is cheap to add if the
Link/Bridge Layer already models these as discrete messages, and it is the
only way to reconstruct/debug a run afterward (e.g., for scoring disputes
or post-mortems) given the "no post-flight processing" rule leaves no
other record. See [DECISIONS.md](DECISIONS.md).

### 3.5 Out of Scope for This Repository (drone-side)
Explicitly *not* built here, but interfaced with:
- Flight controller / autopilot
- Onboard autonomy (SLAM/mapping, survivor detection, grid localisation,
  path planning, exploration strategy)
- Onboard camera + video encoding
- Onboard radio/link driver (the drone-side half of §3.3)

These live in a separate repository/workstream. This repo's job regarding
them is to pin down clear, versioned interface contracts so the two sides
can be developed in parallel — see [COMMUNICATION.md](COMMUNICATION.md).

## 4. Subsystem Interaction (data flow during a mission)

```
Drone-side autonomy                  Link            GCS Backend         GCS UI
──────────────────────               ────            ───────────         ──────
telemetry (pos, batt, state)  ──────►  │  ──────────►  parse/validate ──► status panel
map deltas (1m×1m cells)      ──────►  │  ──────────►  parse/merge    ──► map renderer
survivor detections (grid ref)──────►  │  ──────────►  parse/dedupe   ──► survivor markers
video frames                  ──────►  │  ──────────►  relay/decode   ──► video panel
                                        │
operator "Start"  ◄────────────────────│◄────────────  command send   ◄── Start button
operator "Abort"  ◄────────────────────│◄────────────  command send   ◄── Abort button
```

Only two arrows point from GCS UI back toward the drone. That is
deliberate and load-bearing — see Design Principle 1.

## 5. Proposed Technology Stack

**We are proposing, not implementing.** Nothing below has been built. Each
choice is an engineering decision (not a competition requirement) and is
also logged in [DECISIONS.md](DECISIONS.md) with status.

### 5.1 Presentation Layer: Desktop web app (React + TypeScript) in a native shell (Tauri, tentative) or plain browser window

**Reasoning:**
- The UI's hard problems — a real-time 2D grid-map renderer with live
  survivor markers and drone position, plus a live video panel, updating
  continuously during a timed mission — are exactly what the web
  rendering stack (Canvas/WebGL, `<video>`, CSS layout) is good at, and
  where a student team gets the most velocity per hour invested.
- React + TypeScript gives strong typing across the message boundary
  (pairs naturally with the schemas in [DATA_MODELS.md](DATA_MODELS.md)),
  a huge ecosystem, and is a skill set most student teams already have or
  can pick up fast — this matters given the competition timeline (Design
  Review in Jan 2027, per the Rulebook's competition timeline) leaves
  limited runway.
  Given the timeline runs 2026-08-24 → January 2027, "fast to build,
  fast to debug under time pressure" outweighs "most resource-efficient."
- A native shell (Tauri is the current lean toward — see
  [DECISIONS.md](DECISIONS.md)) rather than a plain browser tab gets us:
  file-system access for local mission logging, no dependency on a
  separately-launched browser, a controllable offline environment, and no
  reliance on internet-fetched resources at runtime — directly serving the
  "no external network" constraint. Electron is the fallback if Tauri's
  Rust toolchain proves to be more friction than it's worth for this team;
  a plain local browser window is the zero-setup fallback if neither native
  shell is worth the packaging effort before finals.
- **Alternative considered:** a native Qt/PySide desktop app. Rejected as
  the default because it's slower to iterate UI in, though it remains a
  reasonable alternative if the team has strong Qt experience already —
  flagged as an open alternative, not ruled out.

### 5.2 Link / Bridge Layer: a FastAPI backend, using `roslibpy` as its rosbridge client

**Implemented** (`gcs/backend/`, Phase 1) — see DECISIONS.md D-0 for the
reasoning and the short history of why this layer was briefly removed and
then reinstated. The backend is the only component that speaks rosbridge
protocol: it uses `roslibpy` (the standard Python rosbridge client) to
connect to `rosbridge_server` (real Jetson or `sim/rosbridge_sim`,
selected by `app/config.py`), subscribes to every topic in
[DATA_MODELS.md](DATA_MODELS.md), and caches the latest value per topic in
memory. It exposes that cache to the Presentation Layer as plain REST
JSON (`app/schemas.py`) — the frontend never sees a `PoseStamped` or an
`OccupancyGrid` directly. The two permitted commands are the only
mutating routes in the whole API (`POST /api/command/start`,
`POST /api/command/abort`); no other route exists, which is what makes
"the operator can only start/abort" a structural property of the backend
rather than a frontend convention.

**Video** stays a separate concern by design (§5.3, and DECISIONS.md
D-6) — it is not sent through rosbridge and is not proxied by this
backend either; whatever serves it (an MJPEG-over-HTTP endpoint or
WebRTC, per D-3) is consumed by the frontend directly from the Jetson.

Mission logging (DECISIONS.md D-5) is a natural fit for this layer if
added later — the backend already sees every message that crosses the
link.

### 5.3 Local Transport (Frontend ↔ Backend ↔ Jetson)

- **Frontend ↔ Backend:** REST over HTTP (`gcs/backend/app/main.py`),
  polled by the frontend rather than pushed — an explicit, revisitable
  tradeoff (DECISIONS.md D-0) made for simplicity and Swagger-based
  testability before a frontend exists. Swagger UI is served at `/docs`.
- **Backend ↔ Jetson:** `rosbridge_server` over WebSocket (port 9090),
  via `roslibpy` — see [COMMUNICATION.md](COMMUNICATION.md) §2.2 for the
  topic list.
- Video: a **separate** path that bypasses both of the above (leaning
  MJPEG-over-HTTP first, per DECISIONS.md D-3), consumed by the UI's
  `<video>` element directly from the Jetson, so that heavy video traffic
  never head-of-line-blocks time-critical telemetry or — critically — the
  abort command (DECISIONS.md D-6/D-10).

### 5.4 Drone ↔ GCS Link

**Protocol layer decided** (DECISIONS.md D-1/D-2): `rosbridge_server` +
`roslibpy` (from the `gcs/backend/` FastAPI service, not the frontend
directly — D-0) for control/telemetry/map/commands; a separate path for
video.
**Physical RF hardware still open** — depends on drone-side hardware
decisions made in parallel, and on resolving Open Question #1 in
[REQUIREMENTS.md](REQUIREMENTS.md) (whether a private/local Wi-Fi link is
acceptable — the current working answer, per D-1, is yes: the rules ban
"public Wi-Fi," not a private point-to-point link). Candidates and
tradeoffs for the actual radio hardware are recorded in
[DECISIONS.md](DECISIONS.md) D-1 rather than decided here.

## 6. Offline Development Strategy

Because the real drone/arena won't be available for most of the
development timeline, and because the rules forbid any runtime dependency
on external services anyway, the architecture treats a **local simulator/
replay tool** as a first-class, permanent part of the system rather than a
throwaway test harness:

- A simulator can emit synthetic telemetry, map-delta, and detection
  messages matching the exact schemas the real drone-side system will
  produce, letting the entire GCS (UI + bridge) be built and demoed long
  before flight hardware is ready.
- A replay tool can play back logged mission data (§3.4) for debugging and
  demos.
- Because the Presentation Layer only ever talks to the Bridge's local
  API (§4), it cannot tell the difference between a simulator and a real
  drone — this is a direct consequence of Design Principle 3, not extra
  work.

This is reflected in the repository's project structure (see
[README.md](../README.md)) as a `sim/` directory, planned but not yet
implemented.

## 7. Non-Goals

Explicitly out of scope for this repository, to keep boundaries clear:

- Flight control, autopilot, or any control-loop logic — that is the
  drone's onboard system.
- SLAM/mapping algorithms, survivor detection models, path planning —
  drone-side.
- Any UI affordance for manual navigation, map editing, or survivor
  tagging — forbidden by the rules (§6 of
  [REQUIREMENTS.md](REQUIREMENTS.md)), and therefore also forbidden by
  this architecture.
- Any dependency, at mission runtime, on internet/cloud services.
