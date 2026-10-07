# CLAUDE.md

Guidance for Claude Code (and any other agent or human) working in this
repository.

## Project Purpose

This repository builds a **custom Ground Control Station (GCS)** for an
autonomous indoor search-and-rescue drone competing in **NIDAR AirMouse**
(Track 1, Problem Statement 2, of the National Innovation Challenge for
Drone Application and Research, 2026-27 edition). The GCS is the sole
authorized interface between the human operator and the drone during the
mission: it displays live video, a live-generated 2D map, tagged survivor
locations, and mission status, and it can send exactly two commands —
start the mission, and abort it.

This is **not** a general-purpose ground control station. It is built for
one specific mission profile, with a deliberately minimal command surface,
because the competition scores the system down for any operator action
beyond start/abort.

## Competition Context

- Competition: NIDAR (National Innovation Challenge for Drone Application
  and Research), 2026-27 edition, organised by Drone Federation India
  under the MeitY SwaYaan initiative.
- Track: Track 1 – Drone Innovation, Problem Statement 2 – **NIDAR
  AirMouse**: "Autonomous GPS-Denied Indoor Search, Mapping & Survivor
  Localisation Challenge."
- Scenario: an earthquake-damaged building, GPS-denied, up to 6 survivors
  in a ≤15 m × 15 m maze-like arena. The drone must autonomously enter,
  explore, map, detect/localise survivors, and exit — all in ≤30 minutes,
  with no post-flight processing time.
- Final Mission (flight) is tentatively scheduled January 2027; Design
  Review and Business Strategy presentations happen the same month. See
  `docs/REQUIREMENTS.md` §12 for the timing constraints that bind the GCS
  specifically.
- This repository (`custom-gcs`) is only the GCS half of the system. The
  drone/flight-controller/onboard-autonomy side is a separate, parallel
  workstream this repo interfaces with but does not implement — see
  `docs/COMMUNICATION.md`.

## Authoritative References

Two documents are the **only** sources of competition requirements. They
live one level above this repository (not committed to git — see
"Rules for Modifying This Repository" below):

- `../Mission Brief - NIDAR AirMouse.pdf`
- `../NIDAR-26-27-Rulebook-Ver_2_1.pdf` (specifically: §8 "Rules for PS2 -
  NIDAR AirMouse", §9 "Problem Statement 2 - NIDAR AirMouse" scoring, and
  Annexure 2 "Problem Statement Brief - NIDAR AirMouse")

Every requirement claim made anywhere in this repository's docs should be
traceable to one of these two files. `docs/REQUIREMENTS.md` is the
extracted, organized version of what they say about the GCS specifically —
treat it as the working reference; go back to the PDFs directly if
`docs/REQUIREMENTS.md` seems ambiguous or you need something it doesn't
cover.

## Architecture

Full writeup: `docs/ARCHITECTURE.md`. Summary:

- **Presentation Layer** (GCS UI): renders video, map, survivor tags,
  mission status; exposes exactly two controls (Start, Abort).
- **Link/Bridge Layer** (GCS Backend): the only component that talks to
  the drone's radio link; parses inbound telemetry/map/detection/video,
  sends the two permitted commands, exposes a local API to the UI.
- **Communication Link**: drone side is Jetson Nano (companion computer,
  ROS + `rosbridge_server`) talking MAVLink to a Pixhawk 6x flight
  controller. The **GCS backend** (`gcs/backend/`, FastAPI) is the only
  thing that talks to rosbridge — via **roslibpy** (WebSocket, port 9090)
  — for telemetry/map/survivors/commands. The **GCS frontend** (not yet
  built) will call the backend's REST API, not rosbridge directly.
  **Video is kept on a separate transport entirely**, bypassing both the
  backend and rosbridge, for safety-latency reasons (an abort command must
  never queue behind video frames). See `docs/COMMUNICATION.md` and
  `docs/DATA_MODELS.md`. Protocol layer decided; physical RF hardware
  still open — see `docs/DECISIONS.md` D-0/D-1/D-2/D-6.
- Stack: **backend built** — FastAPI (`gcs/backend/`), `roslibpy` for the
  rosbridge connection, plain REST JSON to the frontend (polled, not
  pushed — see D-0), Swagger UI at `/docs`. **Frontend not yet built** —
  planned as React/TypeScript in a native shell (Tauri, tentative),
  Tailwind CSS, Zustand for state; talks to the backend's REST API only.
  Reasoning and alternatives in `docs/ARCHITECTURE.md` §5 and
  `docs/DECISIONS.md` D-0.

## Integration Checkpoints (read before touching the command/telemetry path)

The path from the current software-only integration to a full autonomous
mission is broken into 10 gated checkpoints, defined in
`../CHECKPOINT/INTEGRATION_CHECKPOINTS.md` (sibling `CHECKPOINT/`
directory at the workspace root, alongside this repo and
`onboard-autonomy`). **Checkpoint 1 (GCS ↔ Autonomy integration via the
real FastAPI API) is this repo's checkpoint — PASSED (2026-09-02)** —
see `../CHECKPOINT/CURRENT_STATE.md` §0 and §15 for the closing
verification record; re-check that file before assuming this or any
later checkpoint's status, since it changes far more often than this
paragraph. Checkpoints 3 and 4 (GCS `start`→ARM, GCS `abort`→DISARM) are
implemented and demonstrated on real hardware but **not yet PASSED** —
see `INTEGRATION_CHECKPOINTS.md` Checkpoints 3/4 for the specific
criterion still outstanding on each. Checkpoints 2 onward are primarily
`onboard-autonomy`'s responsibility (Jetson↔Pixhawk arming, flight
control) but this repo's `start`/`abort` commands are the trigger for
several of them (3, 4, 5, 10) — the architectural principle that the GCS
never talks to the Pixhawk directly, only ever through the Jetson, is
detailed there and must not be violated by any change here.

## Important Constraints

These come directly from the rules (`docs/REQUIREMENTS.md`) and are
non-negotiable:

1. **The operator command surface is exactly Start and Abort/Emergency-
   Stop.** No waypoint editing, no path correction, no manual survivor
   tagging, no map editing/correction, no mission replanning may ever be
   exposed to the operator or exist as a code path the UI can trigger.
   Any additional manual input during mission execution is a scored
   penalty (−50/instance) and can trigger disqualification.
2. **No post-flight processing.** Everything the GCS displays (map,
   survivor tags, mission data) must be complete and correct by the
   moment the flight ends. There is no "finalize" step.
3. **No external network at runtime, ever.** No internet, no cloud APIs,
   no GSM/LTE/5G, no public Wi-Fi. Only a locally-deployed link to the
   drone. The GCS app itself must not depend on any externally-fetched
   resource (CDN scripts, remote fonts/tiles, telemetry pings) to
   function.
4. **No wired/tethered link to the drone during flight** (no optical
   fibre, no wired comms, no physical tether).
5. **Video is the only authorized live-viewing channel** — the operator
   supervises through the GCS only; there is no FPV-goggle or secondary
   monitoring path to design for or accommodate.
6. **The 2D map and survivor tagging use a shared 1 m × 1 m grid** over
   the ≤15 m × 15 m arena — this is the resolution scoring is judged
   against (see `docs/REQUIREMENTS.md` §8–9), and should be treated as the
   canonical internal representation once implementation starts.

## Development Principles

- **Docs and interfaces before code.** The Drone ↔ GCS interface
  (`docs/COMMUNICATION.md`, `docs/DATA_MODELS.md`) is a two-team contract;
  changes to it should be deliberate, not incidental to a UI change.
- **Requirements vs. decisions vs. assumptions vs. open questions are
  always kept distinct.** `docs/REQUIREMENTS.md` only contains things the
  competition documents actually say. Everything else — technology
  choices, protocol choices, conventions the rules don't specify — belongs
  in `docs/DECISIONS.md`, and unresolved gaps belong in the "Open
  Questions" section of `docs/REQUIREMENTS.md`. Do not blend these
  categories, and do not present a decision or assumption as if it were a
  competition requirement.
- **Build for offline-only, from the start**, not "online with an offline
  fallback." Every dependency the running app needs must be locally
  available.
- **Prefer building the interface's simulator/replay tooling early**
  (`sim/`, planned) so UI and bridge work can proceed without waiting on
  drone hardware.
- **Minimal command surface is a structural property, not a UI
  convention** — see Constraint 1 above. When in doubt about whether a
  feature request adds a forbidden control, check `docs/REQUIREMENTS.md`
  §6 before adding it.

## Coding Conventions

Not yet applicable — no application code exists yet (see "Current Project
Phase" below). Once implementation begins, this section should be updated
with the actual conventions in use (formatting, linting, test layout,
commit style, etc.) rather than prescribed speculatively here.

## Current Project Phase

**Phase 1: Implementation, started.** Phase 0 (requirements, architecture,
repository bootstrap) is closed. Architecture and requirements docs are
considered settled — don't rewrite them just because implementation is
underway; only touch them if implementation surfaces a genuine missing
architectural decision (and if so, stop and explain before making a major
structural change, don't just silently edit the docs).

What exists:

- Requirements (`docs/REQUIREMENTS.md`), architecture (`docs/ARCHITECTURE.md`),
  decisions (`docs/DECISIONS.md`), and communication interfaces
  (`docs/COMMUNICATION.md`, `docs/DATA_MODELS.md`) — all from Phase 0,
  reflecting the decided stack (Jetson Nano + Pixhawk 6x + rosbridge_server,
  React/TypeScript frontend via roslibjs).
- `sim/`: a working rosbridge-protocol simulator (Python) — speaks the
  same WebSocket wire protocol the real Jetson's `rosbridge_server` will,
  publishing synthetic data matching every topic in `docs/DATA_MODELS.md`.
  See `sim/README.md` for how to run/test it. Covered by unit tests (pure
  simulation core, protocol encode/decode) and one real end-to-end test
  (a live WebSocket client against a live server).
- `gcs/backend/`: a working FastAPI backend — the only component that
  connects to rosbridge (real Jetson or `sim/`, via `roslibpy`), and
  re-exposes it to the frontend as plain REST JSON. The entire operator
  command surface (`POST /api/command/start`, `POST /api/command/abort`)
  is enforced structurally — those are the only two mutating routes that
  exist. Swagger UI at `/docs` for manual testing with no frontend needed.
  See `gcs/backend/README.md`. Covered by fast fake-client API tests plus
  one real end-to-end test against a live `sim/` subprocess.
- **Hardware-isolation seam, currently:** `gcs/backend/` ↔ `sim/` today,
  `gcs/backend/` ↔ real Jetson later — a host/port config change
  (`app/config.py`), not a code change.
- `gcs/frontend/` and `protocol/`: still not started — the Presentation
  Layer (UI) hasn't been built yet; it will call the backend's REST API.

## Known Unknowns

Tracked in full, with reasoning, in `docs/REQUIREMENTS.md` under "Open
Questions" and in `docs/DECISIONS.md` under "Open / Unresolved." Headline
items (updated now that the drone-side stack — Jetson Nano + Pixhawk 6x +
ROS/rosbridge — has been decided, per `docs/DECISIONS.md` D-1/D-2):

- **The `/gcs/command` (Start/Abort) topic is implemented** end-to-end
  now (`gcs/backend/app/ros_client.py` → `sim/`), including automated
  tests — but only against `sim/`. Still needs a real subscriber node on
  the actual Jetson, and a latency test under realistic (video-present)
  link load, before it's done for real — see `docs/DECISIONS.md` D-10.
- **Video must not share the rosbridge/WebSocket connection** with
  telemetry/map/commands (base64+JSON overhead, and risk of delaying the
  abort command) — transport TBD (MJPEG vs. WebRTC), see D-6/D-3.
- The physical RF hardware for the Jetson↔GCS local link (WiFi bridge
  type, range/interference in a netted arena) — D-1's residual item.
- Whether Pixhawk's EKF has a valid non-GPS position source indoors
  (presumably SLAM pose → `VISION_POSITION_ESTIMATE` → EKF fusion) — needs
  confirmation from the flight-control subteam, see D-11.
- `/vision/survivors` and `/mission/state` need concrete custom `.msg`
  definitions from the drone-side team (currently only specified as field
  lists in `docs/DATA_MODELS.md`) — see D-13.
- Jetson Nano compute budget running SLAM + detection + video encode +
  rosbridge concurrently — informational risk, drone-side, see D-12.
- The grid coordinate labeling convention's *display* form (numeric vs.
  chess-style) and how the team's grid is reconciled with the organiser's
  reference grid at scoring time — origin/resolution convention itself is
  now decided (D-7), the rest isn't.
- The physical form factor of the GCS hardware (laptop/tablet/custom
  panel) — not mandated by the rules.

## Rules Claude Must Follow When Modifying This Repository

1. **Phase 0 has ended; implementation is underway.** Keep new code
   modular and testable, keep hardware-specific interfaces isolated
   (per the `sim/` pattern — a component a real client can't tell apart
   from the real drone-side system), and write tests alongside meaningful
   functionality rather than after the fact. Prefer a working vertical
   slice over scaffolding many empty modules.
2. **Never invent a competition requirement.** If it isn't in the Mission
   Brief or Rulebook, it does not go in `docs/REQUIREMENTS.md`. It goes in
   `docs/DECISIONS.md` (if it's a choice we're making) or the "Open
   Questions" section (if it's a gap we can't yet resolve). When
   summarizing or citing competition rules, be able to point to the
   specific section of the source PDF.
3. **Do not modify, move, or commit anything outside this repository.**
   `../Mission Brief - NIDAR AirMouse.pdf` and
   `../NIDAR-26-27-Rulebook-Ver_2_1.pdf` are read-only local reference
   material and must never be copied into this repo, edited, or deleted.
   Only `custom-gcs/` is version-controlled and pushed to GitHub.
4. **If the Rulebook or Mission Brief is later updated** (the Rulebook is
   already at "Version 2.1" with a revision history — check its "Record of
   Revision" table for the latest version before treating any requirement
   as current), re-verify `docs/REQUIREMENTS.md` against the new version
   before relying on it, and note what changed.
5. **Keep `docs/COMMUNICATION.md` and `docs/DATA_MODELS.md` in sync.**
   They describe the same interface from two angles (interface boundaries
   vs. message shapes); a change to one that isn't reflected in the other
   is a documentation bug.
6. **Never add a UI control or message type that lets the operator modify
   navigation, the map, or survivor tags during a mission**, even behind a
   flag or in a "debug" mode — this is a hard rule constraint (see
   Important Constraints §1), not a stylistic preference.
7. **When genuinely blocked by a gap in the competition documents**, add
   it to the Open Questions / Known Unknowns rather than guessing and
   presenting the guess as settled fact.
