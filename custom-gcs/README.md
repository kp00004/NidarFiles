# NIDAR AirMouse — Custom Ground Control Station

A custom-built Ground Control Station (GCS) for an autonomous, GPS-denied
indoor search-and-rescue drone, developed for **NIDAR AirMouse** — Track 1
(Drone Innovation), Problem Statement 2, of the National Innovation
Challenge for Drone Application and Research (NIDAR), 2026-27 edition,
organised by Drone Federation India under the MeitY SwaYaan initiative.

## What This Is

The competition scenario: an earthquake-damaged, GPS-denied indoor arena
(≤15 m × 15 m), up to 6 survivors hidden in different rooms. An autonomous
drone must enter through a designated point, explore the maze, detect and
localise survivors, build a live 2D map, tag survivor locations on it, and
exit — entirely autonomously, in ≤30 minutes, with no manual navigation
assistance permitted.

This repository builds the **Ground Control Station**: the single
authorized interface between the human operator and the mission. It must
display live video, a live-updating 2D map, tagged survivor locations, and
mission status — and it can send exactly two commands: **start the
mission** and **abort it**. Everything else about the mission (mapping,
detection, navigation) happens autonomously onboard the drone; the GCS is
a real-time viewer, not a controller.

This repository contains **only** the GCS. The drone, flight controller,
and onboard autonomy system are a separate, parallel workstream.

**Read `../CHECKPOINT/CURRENT_STATE.md` and `../CHECKPOINT/NEXT.md`
before doing any work** — they are the authoritative, dated record of
what has actually been verified against real hardware versus merely
implemented, and are updated far more often than this file.

## Reference Documentation

Phase 0 (requirements, architecture, and interface definition, closed
2026-08-27) produced the documents below; they remain the working
reference for *why* the system is built the way it is. See "Current
Status" further down for what's actually implemented today.

| Document | Contents |
|---|---|
| [`docs/REQUIREMENTS.md`](docs/REQUIREMENTS.md) | Everything the competition documents say about the GCS, organized by topic, with every claim traceable to a specific rule. Also lists open questions the documents don't resolve. |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Proposed system architecture, subsystem boundaries, and technology stack with reasoning for each choice. |
| [`docs/COMMUNICATION.md`](docs/COMMUNICATION.md) | Proposed data interfaces between the drone, flight controller, onboard autonomy, mapping, survivor detection, video system, and the GCS. |
| [`docs/DATA_MODELS.md`](docs/DATA_MODELS.md) | Draft message schemas for the Drone ↔ GCS interface. |
| [`docs/DECISIONS.md`](docs/DECISIONS.md) | Log of engineering decisions (and open ones) that the competition rules don't dictate — technology and protocol choices, with reasoning. |
| [`CLAUDE.md`](CLAUDE.md) | Project context and rules for AI-assisted development in this repo. |

## Source of Truth

All competition requirements trace back to two documents, kept outside
this repository (they are local reference material, not part of the
GitHub repo):

- `Mission Brief - NIDAR AirMouse.pdf`
- `NIDAR-26-27-Rulebook-Ver_2_1.pdf`

`docs/REQUIREMENTS.md` is the working extraction of what those documents
say about the GCS. If anything here seems to conflict with them, the PDFs
win.

## Repository Layout

```
custom-gcs/
├── CLAUDE.md          Project context & rules for AI-assisted development
├── README.md           This file
├── docs/                Engineering documentation (see table above)
├── gcs/
│   ├── backend/         FastAPI backend (Python) — the only component that talks to rosbridge.
│   │                    Exposes a plain REST API + Swagger docs at /docs, and serves the
│   │                    operator frontend as static files at /ui. See gcs/backend/README.md.
│   └── frontend/        Minimal static operator UI (plain HTML/JS, no build step) — served
│                        by the FastAPI backend at /ui, not a standalone app. Exactly two
│                        controls (START, STOP/ABORT) plus a read-only telemetry dashboard.
├── protocol/            (planned) Shared, versioned Drone↔GCS interface definitions. Empty — not started yet.
├── sim/                 Rosbridge-protocol simulator (Python) — a working stand-in for the real
│                        Jetson's rosbridge_server, for developing/testing without drone hardware.
│                        See sim/README.md to run and test it.
└── tools/               (planned) Dev scripts, log inspection, etc. Empty — not started yet.
```

## Current Status: Phase 1 — Implementation

Phase 0 (requirements/architecture) is closed; see the docs above for
what was decided and why. Implementation, real hardware verification
included, is well underway — see `../CHECKPOINT/CURRENT_STATE.md` and
`../CHECKPOINT/INTEGRATION_CHECKPOINTS.md` for the authoritative,
checkpoint-gated status:

- **Built and hardware-verified:** `sim/` — a rosbridge-protocol-
  compatible simulator publishing synthetic telemetry/map/survivor data
  matching `docs/DATA_MODELS.md`, used for development without drone
  hardware. `gcs/backend/` — a FastAPI service that connects to rosbridge
  (real Jetson or `sim/`, a config change not a code change) and
  re-exposes it as a plain REST API, now carrying real FCU telemetry
  (armed/mode/system_status, battery, pose, velocity, attitude, GPS,
  STATUSTEXT). The two-command operator surface (start/abort) is enforced
  structurally: those are the only mutating routes that exist.
  `gcs/frontend/` — a minimal static operator dashboard (no build step),
  served by the backend at `/ui`, with exactly the two permitted controls
  (START, STOP/ABORT) — see `gcs/frontend/index.html`. **Checkpoint 1**
  (this repo's checkpoint: the full GCS↔onboard-autonomy loop through the
  real API) is **PASSED**; this repo's `start`/`abort` have also driven
  real Pixhawk ARM/DISARM through `onboard-autonomy` (Checkpoints 3/4,
  implemented and demonstrated, not yet formally closed — see
  `INTEGRATION_CHECKPOINTS.md`).
- **Not yet started:** the shared `protocol/` package and video handling.

## Simulation ("RUN SIMULATION")

The GCS frontend has a second, clearly-separate "Simulation" section (a
dashed-violet-bordered panel, badged "SIMULATION — NOT REAL FLIGHT")
below the real operator panels. Its **RUN SIMULATION**/**RESET** buttons
call `POST /api/simulation/{run,reset}` — routes that publish only to
`/simulation/command` on the Jetson (never the real `/gcs/command`) and
drive a deterministic simulated exploration mission running entirely
inside `onboard-autonomy` (`mission_simulator.py`/`simulation_node.py`).
See `../CHECKPOINT/docs/simulation_architecture.md` for the full
architecture and `../CHECKPOINT/CURRENT_STATE.md` for real end-to-end
test evidence (a full run driven purely through this repo's real FastAPI
backend over the real rosbridge connection).

**Gated OFF by default in the frontend build** — per this repo's
Important Constraint #1 (operator command surface is exactly Start and
Abort), a competition-deployed build must not expose any additional
clickable control. `gcs/frontend/src/App.tsx` only renders the
simulation section when `VITE_ENABLE_SIMULATION=true` is set at build/
dev-server time; the default (unset) production build tree-shakes
`SimulationPanel` out of the bundle entirely (verified: ~4kB smaller,
zero occurrences of its UI strings in the built JS). To see/test it
locally: `VITE_ENABLE_SIMULATION=true npm run dev` (or `... npm run
build`). **Never set this flag for a competition build.**

## Mission / Test Select

The GCS frontend also has a "Mission / Test Select" panel (dashed-accent-
bordered, badged "DEV / BENCH ONLY — ALTERNATE START PATH") that lets an
operator pick a complete named test procedure — e.g. "Test 1: Forward /
Backward / Hover" — from a dropdown backed by the backend's mission/
scenario registry (`gcs/backend/app/missions.py`), see its ordered steps,
and START it. It sends no new kind of command: `POST /api/mission/start`
still only ever publishes the same real `"start"` the plain START button
(`ControlsPanel.tsx`) sends, parameterized with which mission/scenario
profile it applies to. Selection locks while that mission/scenario is
active and unlocks again on completion/abort; ABORT is not duplicated
here, `ControlsPanel.tsx` stays the single abort control.

**Gated OFF by default in the frontend build**, same reasoning and same
pattern as "RUN SIMULATION" above — a competition-deployed build must not
expose an additional clickable control. `gcs/frontend/src/App.tsx` only
renders this panel when `VITE_ENABLE_MISSION_SELECT=true` is set at
build/dev-server time. **To see/test it locally** (this is the command
that actually shows the dropdown — the plain `npm run dev` does not):

```sh
cd gcs/frontend
VITE_ENABLE_MISSION_SELECT=true npm run dev
```

(Windows/PowerShell: `$env:VITE_ENABLE_MISSION_SELECT = "true"; npm run dev`.)
Works fully with the backend in ROS-disabled local-development mode
(`GCS_ROS_ENABLED=false` — see `gcs/backend/README.md`): the mission
registry and step previews are plain application data with no ROS
dependency; only pressing START degrades, honestly 503ing rather than
pretending a mission started. **Never set this flag for a competition
build.**

## Next Step

See `../CHECKPOINT/NEXT.md` for the live, dated next-actions list. In
short: close out Checkpoints 3/4 by demonstrating `abort` against a
genuinely-armed vehicle (both mid-`start`-sequence and post-ARM), then
reconcile that result into `INTEGRATION_CHECKPOINTS.md`, before any
Checkpoint 5+ (autonomous takeoff) work begins.
