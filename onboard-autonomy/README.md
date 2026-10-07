# onboard-autonomy

Drone-side onboard autonomy for NIDAR AirMouse — SLAM, exploration/path
planning, survivor detection, mission state, and flight command issuance
to the Pixhawk. Runs on the Jetson. Counterpart to the `custom-gcs` repo
(sibling directory), which is the operator-facing GCS this repo talks to
over `rosbridge_server`.

**Read `CLAUDE.md` first** — it defines scope, the binding Drone↔GCS
interface contract (owned jointly with `custom-gcs`, not redefined here),
and hard safety rules for anything that touches real flight.

**Read `../CHECKPOINT/CURRENT_STATE.md` and `../CHECKPOINT/NEXT.md`
before doing any work** — they are the authoritative, dated record of
what has actually been verified against real hardware versus merely
implemented, and are updated far more often than this file.

## What exists right now

Checkpoints 1–2 (GCS↔autonomy software integration; Jetson↔Pixhawk
ARM/DISARM) are **PASSED**. Checkpoints 3–4 (GCS `start`→ARM,
GCS `abort`→DISARM) are implemented and demonstrated on real hardware but
**not yet formally PASSED** — see
`../CHECKPOINT/INTEGRATION_CHECKPOINTS.md` for the specific outstanding
criterion on each. No SLAM, no detection, no autonomous flight control.

- `nidar_airmouse/` — one ROS 2 message (`SurvivorDetection.msg`),
  needed because the real GCS backend expects it. Builds and installs on
  the Jetson.
- `nidar_autonomy/` — ROS 2 nodes for command handling
  (`command_node.py`), mission state (`mission_state_node.py` /
  `state_machine.py`), heartbeat (`heartbeat_node.py`), and the real
  Jetson→Pixhawk ARM/DISARM path (`flight_command.py`, `arming_guard.py`,
  `arm_trigger.py`). Real bench ARM/DISARM (props off/restrained) has
  been demonstrated against the live Pixhawk both standalone and driven
  by real GCS `start`/`abort` commands. `checkpoint2_arm_test.py` is a
  standalone bench-test harness, not part of the normal runtime node set.
  Unit tests: `cd nidar_autonomy && PYTHONPATH=. python3 -m pytest test/ -q`
  (after sourcing a ROS 2 Humble environment — see `../source_ros.sh`).

## Simulation ("RUN SIMULATION")

`nidar_autonomy/mission_simulator.py` + `simulation_node.py` — a
deterministic, self-contained simulated mission (maze world, simulated
LiDAR, real frontier detection/planning/mission-state code, simulated
vehicle motion) that the GCS's **RUN SIMULATION** button drives, entirely
isolated from the real Pixhawk/mavros path (`/simulation/`-namespaced
topics only, no mavros_msgs/flight_command.py import anywhere in this
feature). See `../CHECKPOINT/docs/simulation_architecture.md` for the
full design, and `../CHECKPOINT/CURRENT_STATE.md` for the session record
and real end-to-end test evidence. Run it directly:
`ros2 run nidar_autonomy simulation_node`.

## What does NOT exist yet

SLAM, survivor detection, exploration/path planning, and — the biggest
one — actually sending flight/motor setpoints to make the vehicle fly
autonomously. See `CLAUDE.md`'s Architecture section for the honest
status of each. None of these were started here because they're either
large algorithmic subsystems needing a real design pass, or (for flight
commands specifically) too safety-critical to scaffold speculatively —
see `CLAUDE.md`'s Hard Safety Rules.

## Related repos and reference material

- `../custom-gcs/` — the GCS this repo serves telemetry to and takes
  exactly two commands from.
- `../CHECKPOINT/` — the canonical, dated checkpoint/state record for
  the whole `~/NIDAR/` workspace (this repo, `custom-gcs`, and the
  overall integration roadmap).
- `~/NIDAR-Hardware-Bringup/` — older Pixhawk↔Jetson hardware/network
  bring-up notes (MAVROS, rosbridge_server setup) from before this repo
  moved under `~/NIDAR/`. Superseded by `../CHECKPOINT/` for anything
  it overlaps with, but the original bring-up notes are still there if
  needed. Not a subdirectory of this repo — it lives at `~/NIDAR-Hardware-Bringup`, not `../NIDAR-Hardware-Bringup`.

## Radio START/ABORT and the real hover (ArduCopter)

The MicroLR900 command radio is plugged into the **Jetson** (USB serial);
the Pixhawk is not on the radio link.

- `radio_command_node` -- opens the radio (`serial_port`, default the
  CP2102 `/dev/serial/by-id/...` path, fallback `/dev/ttyUSB0`, 115200),
  decodes `COMMAND_LONG(MAV_CMD_USER_1)` (`telem_command_codec.py`),
  accepts/rejects it (`radio_command_logic.py`), publishes
  `/gcs/mission_select` + `/gcs/command`, replies `COMMAND_ACK` with a
  reason code, and sends a 1 Hz radio heartbeat. `-p dry_run:=true` ACKs
  and logs without publishing anything (radio bench test).
- `ardupilot_vehicle.py` -- ArduCopter control through MAVROS for GUIDED
  missions: mode changes, takeoff, stream-rate requests, state snapshot;
  arm/disarm delegated to the existing `FlightCommandClient`.
- The hover mission itself lives outside this repo, in
  `NidarFiles/missions/hover/` (`mission.py`, run with `python3`).
  Do not run `mission_state_node` alongside it -- it would arm on the same
  START; the hover mission refuses START while it is running.

Workspace README (architecture, deployment, hardware checklist):
`NidarFiles/README.md`.
