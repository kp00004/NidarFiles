# Requirements — NIDAR AirMouse Ground Control Station

This document extracts every requirement that bears on the Ground Control
Station (GCS) from the two authoritative competition references:

- `../../Mission Brief - NIDAR AirMouse.pdf` ("Mission Brief")
- `../../NIDAR-26-27-Rulebook-Ver_2_1.pdf` ("Rulebook") — specifically:
  - §8 General Technical and Operational Requirements → "Rules for PS2 - NIDAR AirMouse"
  - §9 Scoring Structure → "Problem Statement 2 - NIDAR AirMouse"
  - Annexure 2: Problem Statement Brief - NIDAR AirMouse

Every line below is traceable to one of those documents. Nothing here is
invented. Where the documents are silent, that is called out explicitly in
[Open Questions](#open-questions) or deferred to
[DECISIONS.md](DECISIONS.md) as an engineering decision.

We are Track 1, Problem Statement 2 (PS2) — **NIDAR AirMouse**. (PS1
RescueSwarm and PS3 VEGAPilot are different problem statements with their
own GCS requirements; they are out of scope for this repository but are
occasionally referenced below for contrast, clearly marked as such.)

---

## 1. Mission Scenario (context)

A building has suffered earthquake/structural collapse. Interior spaces are
unsafe for human entry. GPS is unavailable indoors. Up to 6 survivors are
placed in different rooms at unknown locations. An autonomous indoor drone
must enter a maze-like arena from a designated entry point, explore it,
detect survivors, build a 2D map, tag survivor locations on that map, and
exit from a designated exit point (same as entry point) — all autonomously,
in ≤30 minutes, with the GCS as the *only* authorized window into the
mission for the human operator.

## 2. What the GCS Must Display

Stated twice, consistently, in the Mission Brief (§5) and the Rulebook
(Annexure 2 §5, and Rulebook §8.27):

- Live mission status of the drone
- Live camera feed from the drone
- A 2D map, generated and **continuously updated while the drone is
  flying** (not after)
- Identified corridors, rooms, or sections — "wherever technically
  feasible" (Mission Brief phrasing; not an absolute requirement)
- The grid coordinate/grid box containing each detected survivor
- Tagged survivor locations (a marker on the map per survivor)
- Drone position, or estimated drone position, within the mapped area
- Mission progress and completion status

Scoring (Rulebook §9, PS2, Phase 4D) makes two of these pass/fail-gated
with explicit point value:

- **GCS/Mission Planner Map Display — 50 pts (Yes/No).** Full marks only if
  the GCS displays *all* of: the generated 2D map, estimated drone
  position, survivor tags, and mission progress, simultaneously.
- **Safe Mission Completion — 15 pts (Yes/No).** Zero if the mission ends
  in a crash, unsafe contact, uncontrolled landing, or organiser
  intervention.

## 3. What the GCS Must Receive (data inbound)

Derived from the display requirements above, since the GCS can only show
what it receives:

- Telemetry: mission/vehicle status, drone position or position estimate,
  mission progress/completion state
- Live video stream from the drone's camera
- Incremental 2D map data (occupancy/classification of explored cells,
  and — where feasible — labeled corridors/rooms) as it is built in flight
- Survivor detection events: which grid coordinate/box each detected
  survivor is in
- System/communication health, to the extent the team's link protocol
  reports it (not separately mandated by the rules for PS2, unlike PS1
  which explicitly requires "communication and system health status" —
  see §9 below)

All of the above must arrive **during flight**. Rulebook Annexure 2 §4 and
§5 are explicit: *"No additional time shall be provided after flying for
generating or completing the map, processing mission data, transferring
data, or updating the Ground Control Station"* and *"No post-flight time
shall be provided for completing the map, transferring data or making any
corrections."*

## 4. What the GCS Must Be Capable of Sending (commands outbound)

This is deliberately minimal. Rulebook Annexure 2 §4 states:

> "The operator may only start the mission and trigger the safety
> abort/emergency stop if required."

So the GCS's command surface is exactly:

1. **Start mission**
2. **Safety abort / emergency stop** (may coincide with "emergency recall"
   — the rulebook uses both terms; see [Open Questions](#open-questions))

No waypoint upload, no path correction, no manual tagging, no map editing,
no mission replanning capability may exist in the GCS command surface —
see §6 below, this is a hard constraint, not a soft one.

## 5. What the Operator Is Allowed To Do

(Rulebook Annexure 2 §8, §4; Rulebook §10 Safety)

- Exactly **one operator** supervises the mission through the GCS.
- The operator may start the mission.
- The operator may trigger the safety abort/emergency stop if required.
- The operator supervises/views the mission **only** through the GCS
  display.
- Up to **two team members** may set up the drone, GCS, and comms
  equipment during the 5-minute setup window (this is a *setup-phase*
  allowance, not a mission-execution allowance).

## 6. What the Operator Is NOT Allowed To Do

(Rulebook Annexure 2 §4, §5, §8; Rulebook §9 Penalties)

Explicitly defined as **manual intervention** if performed during mission
execution:

- Any manual control input
- Path correction
- Waypoint adjustment
- Survivor tagging input
- Operator-assisted navigation
- Manual navigation assistance
- Map editing
- Localisation correction
- Path-planning input
- Any manual drawing of the map
- Manual tagging of survivors
- Operator-assisted correction of survivor locations
- System or mission reset after mission start (other than the permitted
  safety abort/emergency recall)

Also explicitly prohibited:

- **FPV goggles or any separate piloting, navigation, monitoring, or
  video-viewing device.** All authorized mission supervision and live
  video viewing must occur *only* through the GCS. (Rulebook Annexure 2 §4)
- Assistance of any kind from any other team member during mission
  execution — other members may only observe. (Rulebook Annexure 2 §8)
- More than one operator supervising via the GCS.

**Scoring consequence:** "Manual Input or Reset" is a named penalty
condition worth **−50 points per instance**, uncapped exception for
safety-critical violations (organisers may terminate the mission or
disqualify the team). This is why the GCS architecture treats "no control
surface beyond start/abort" as a structural constraint, not just an
operating procedure — see [ARCHITECTURE.md](ARCHITECTURE.md).

## 7. Communication Constraints

(Rulebook §8 General Rules; Rulebook Annexure 2 §7)

- The mission is designed and executed assuming a **no-external-network
  environment**.
- Teams shall not rely on GSM, LTE, 5G, **public** Wi-Fi, internet
  connectivity, cloud-based communication, or any external network for
  drone operations, mission execution, data transfer, or coordination.
- All communication between drones, onboard systems, and the GCS must
  operate through **locally deployed communication systems** (the team's
  own link).
- Optical-fibre cables, wired communication links, physical tethers, or
  any other cable connected to a drone during flight are **not permitted**
  (Rulebook §8.5, general rule; also explicit for PS1 in Annexure 1 §5 —
  applied here as a general-rule constraint on all tracks).
- The system must operate on the assumption that there is no mobile
  network or internet connectivity at the venue.
- Use of any external network-based communication interface during the
  mission is treated as a rules violation / manual-external intervention.

Note: the rules say "public Wi-Fi," not "Wi-Fi" — this leaves room for a
private, local, point-to-point or ad-hoc wireless link. See
[Open Questions](#open-questions).

## 8. Mapping Requirements

(Rulebook Annexure 2 §3, §5; Rulebook §9 Scoring)

- Arena: covered, maze-like indoor arena, ≤15 m × 15 m, modular grid
  structure, netted top (simulates GPS-denial).
- Corridors: uniform clear width ≥1 m, vertical clearance ≥8 ft.
- Standard room size: 2 m × 2 m.
- Height clearance in corridors and rooms: 8 ft, at all times.
- Same designated entry point and exit point.
- Exact internal layout is not disclosed to teams before the mission.
- The system must generate and **continuously display** a real-time 2D map
  of the explored area during flight — not a static, post-processed map.
- The map should be usable by rescue teams to understand the approximate
  layout and proceed to tagged survivor locations — i.e., it is a
  human-legible operational map, not a raw point cloud or internal SLAM
  debug view.

**Scoring reveals the ground-truth grid resolution the map is judged
against:** Rulebook §9, PS2, Phase 4D, "2D Grid Mapping Accuracy" (220
pts) is scored per correctly mapped **1 m × 1 m grid cell** within the
15 m × 15 m zone, verified against the organiser's reference grid layout.
For each cell, the team must correctly map the presence of walls,
openings, corridors, or accessible directions adjoining that cell. This is
the most detailed technical spec available for what "the map" must
represent internally — **a 1 m × 1 m occupancy/connectivity grid**, 225
cells maximum for the full arena.

## 9. Survivor Localisation Requirements

(Rulebook Annexure 2 §6; Rulebook §9 Scoring)

- Up to 6 survivors (real humans or dummies), placed in different
  rooms/room-like sections.
- Detection must be autonomous, via onboard sensing and processing.
- Each detected survivor's location must be tagged with a grid
  coordinate/grid box on the generated 2D map.
- Scoring: **Survivor Detection & Localisation on 2D Map — 240 pts** (40
  pts × up to 6 survivors). "Location tagging shall mean marking the
  survivor location with the correct grid reference **on the 1 m × 1 m
  reference grid**." This confirms survivor tagging uses the same 1 m × 1
  m grid as the mapping accuracy scoring (§8 above) — one shared
  coordinate system, not a separate coarser "room-level" grid.

## 10. Live Video Requirements

(Mission Brief §1, §5; Rulebook Annexure 2 §1, §4)

- The GCS **must display a live camera feed** from the drone throughout
  the mission.
- This is the *sole* authorized channel for the operator to view video —
  FPV goggles and any other viewing/monitoring device are explicitly
  prohibited.
- No resolution, frame rate, latency, or codec is specified anywhere in
  either document. This is an engineering decision — see
  [DECISIONS.md](DECISIONS.md).

## 11. Safety / Failsafe Requirements

(Rulebook Annexure 2 §10; Rulebook §8.31)

- The drone must include emergency stop / mission-abort capability.
- The drone must include failsafes for: low battery; loss of command-and-
  control link; geofence breach; mission abort; emergency recall.
- The system (i.e., GCS + drone together) must let the operator safely
  abort the mission if required.
- The drone must be designed for safe operation in confined indoor
  spaces, must avoid contact with walls/ceiling/arena structures/
  obstacles, and must carry propeller guards covering the full propeller
  operating area.

Scoring penalizes unsafe outcomes directly: Crash (−50/instance), Geofence
Breach (−20/instance, +additional −20 for a repeat breach by the same
drone), Landing outside the designated zone (−10/drone). Total penalties
are capped at 150 marks **except** for safety-critical violations, for
which organisers may terminate the mission or disqualify the team outright
— i.e., the cap does not protect against a true safety failure.

## 12. Timing Constraints

(Rulebook Annexure 2 §4; Rulebook §4.36, §9)

- **Setup time: max 5 minutes** to position/prepare the drone, GCS,
  communication systems, and associated equipment before the mission
  begins.
- **Flight time: max 30 minutes**, measured from take-off until the drone
  exits the maze or the mission is terminated.
- **No post-flight processing time** — see §3 above; everything the GCS
  shows must be complete and correct by the moment the flight ends.
- Scoring rewards speed directly: "Fast Completion Bonus" (25 pts,
  Yes/No) is awarded if the mission completes within **half** of the
  maximum permitted mission time (i.e., ≤15 minutes).

## 13. Offline / Network Constraints

Covered in §7 above. Restated for emphasis because it is the single
constraint with the widest architectural blast radius: **the entire GCS
must function with zero connectivity to any network outside the team's own
local link to the drone.** No cloud APIs, no external map tile servers, no
NTP servers, no package registries reachable at runtime, no telemetry
services. Everything the GCS needs — base map rendering, fonts, assets,
logging, and any local storage — must be bundled or generated locally.

## 14. Team / Operational Constraints (context, not GCS-internal)

(Rulebook Annexure 2 §8; Rulebook §4.34–4.36)

- Max 2 team members during the 5-minute setup.
- Exactly 1 operator supervises the mission via the GCS.
- No assistance from other team members during setup, launch, mission
  execution, mapping, survivor detection, exit, landing, troubleshooting,
  or recovery — other members may only observe.
- At Final Mission time, the "Command and Control Station" — explicitly
  including antennas, displays, computers, and remote-control devices —
  is positioned within a designated area, and a maximum of two team
  members may operate/supervise it (Rulebook §4.34; note this widens the
  operator count momentarily at setup/positioning but the *mission
  execution* rule above still caps active supervision at one operator).

## 15. Drone-Side Constraints Relevant to GCS Design (context)

Not GCS requirements per se, but they bound what the GCS can assume about
its counterpart:

- No commercially available market-ready/RTF drone airframes — the whole
  system, including the flight stack, is custom-built or built from
  components. Open-source software (e.g. ROS, PX4, ArduPilot, SLAM
  libraries) and commercially available *components* are explicitly
  permitted (Rulebook §8.2).
- Combined all-up weight ≤10 kg.
- No GPS/GNSS-based navigation (indoor, GPS-denied by design).
- Launch area: fixed 2 ft × 2 ft.

---

## Open Questions

Things the competition documents do not resolve. These are not
requirements — they are gaps that need either an organiser clarification,
a documented assumption, or an engineering decision (tracked in
[DECISIONS.md](DECISIONS.md)).

1. **Is a private/local Wi-Fi link (ad-hoc or AP, not internet-connected)
   permitted?** The rule bans "public Wi-Fi" specifically, and separately
   requires "locally deployed communication systems" — this reads as
   permissive toward a private local WiFi link, but it is not stated
   outright. Needs confirmation before committing to a link technology.
2. **"Emergency recall" vs. "safety abort/emergency stop"** — are these
   one operator action or two? Rulebook Annexure 2 §4 names only "start"
   and "safety abort/emergency stop" as permitted operator actions, but
   §10 lists "emergency recall" as a separate drone failsafe alongside
   "mission abort." Not clarified whether recall is operator-triggered or
   purely an autonomous drone-side failsafe response.
3. **Grid coordinate labeling convention** — the rules require tagging
   survivors and mapped cells against a 1 m × 1 m grid, and that this must
   match the organiser's reference grid layout at scoring time, but the
   labeling scheme (alphanumeric like chess notation, numeric (row, col),
   or something else) and the grid's origin/alignment relative to the
   arena are not specified. The team's own grid, established from onboard
   SLAM, will need to line up with the organiser's reference at scoring
   time — how that alignment is verified/communicated is not described.
4. **"Wherever technically feasible" for corridor/room identification** —
   this is explicitly soft language in the Mission Brief; not a hard pass/
   fail scoring item the way map-cell accuracy is. Treat as best-effort.
5. **Communication/system health display** — required explicitly for PS1
   RescueSwarm's GCS (Rulebook §8.14) but *not* listed among PS2 AirMouse's
   required GCS display items (Rulebook §8.27, Annexure 2 §5). Given PS2
   is single-drone rather than a multi-drone swarm, this may simply be
   less critical, but nothing prohibits including it, and it is good
   engineering practice for an operator who can only start/abort — see
   [DECISIONS.md](DECISIONS.md).
6. **Video/telemetry protocol, resolution, latency, format** — entirely
   unspecified; an engineering decision.
7. **Physical form factor of the GCS** — laptop, tablet, custom panel?
   Rulebook §4.34 mentions "displays, computers and remote-control
   devices" as part of the Command and Control Station, which suggests a
   conventional computer + display, but doesn't mandate one.
8. **Onboard sensing/compute stack** — not specified (no mandated SLAM
   algorithm, detection model, or onboard compute platform for PS2, unlike
   PS3 which mandates the ARIES v3/VEGA board). This is a parallel,
   drone-side workstream, but it determines the real content and update
   rate of the map/telemetry/detection data the GCS will receive.
