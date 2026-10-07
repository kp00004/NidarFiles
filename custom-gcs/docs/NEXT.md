> **SUPERSEDED (2026-09-04).** This checklist is fully complete and
> predates `CHECKPOINT/NEXT.md` (workspace root), the actively-maintained
> forward-plan doc since 2026-08-28. Read `CHECKPOINT/NEXT.md` instead for
> the current plan. Kept here unmodified as a historical record.

# NIDAR Hardware Bring-Up — Next Steps

1. ~~Install ros-humble-rosbridge-suite on Jetson.~~ DONE.
2. ~~Launch rosbridge_server on port 9090.~~ DONE.
3. ~~Verify rosbridge is listening on the Jetson.~~ DONE.
4. ~~Verify rosbridge can actually relay a real MAVROS topic.~~ DONE — /mavros/imu/data relayed live on 2026-08-27.
5. ~~Determine Jetson Wi-Fi IP and confirm laptop can reach Jetson over the local network.~~ Jetson Wi-Fi IP found: 10.145.63.112 (wlP1p1s0, DHCP). Laptop-side reachability NOT yet confirmed.
6. From laptop, perform raw WebSocket/roslibpy smoke test against Jetson rosbridge.
   - Script ready: scripts/laptop_rosbridge_smoke_test.py
   - Run on the LAPTOP (not the Jetson): `pip install roslibpy` then
     `python3 laptop_rosbridge_smoke_test.py 10.145.63.112`
   - Confirms /mavros/imu/data, /mavros/battery, /mavros/state all relay real data over Wi-Fi.
7. Point the existing FastAPI GCS backend at the real Jetson using ROSBRIDGE_HOST and ROSBRIDGE_PORT.
   - GCS backend location not yet identified in this repo — user to provide.
8. Verify:
   - /health
   - /api/telemetry
9. Verify real battery/state/IMU data reaches the GCS.
10. Keep /mavros/local_position/pose explicitly marked as blocked on indoor position-source integration, not a communication failure.
11. Later integrate SLAM → vision position → Pixhawk EKF → MAVROS local_position/pose.
12. Later define and implement /gcs/command subscriber on Jetson.
13. Test ONLY "start" and "abort".
14. Measure command round-trip latency.
15. Test abort latency while video traffic is running.
16. Later integrate survivor detection and /vision/survivors.
17. Later integrate /slam/map.
18. Eventually make the Jetson Ethernet configuration persistent instead of relying on the runtime 192.168.144.1 address.
    - Confirmed again on 2026-08-27 that this address does NOT survive reboot; had to be restored manually.
19. Decide how MAVLink telemetry stream intervals should be made persistent.
    - Confirmed again on 2026-08-27 that SRx_* rates reset on MAVROS restart; had to reissue
      MAV_CMD_SET_MESSAGE_INTERVAL (511) for message IDs 1, 147, 32, 30, 27. See CURRENT_STATE.md
      "RUNTIME STATE THAT DOES NOT SURVIVE A REBOOT / RESTART" for the exact restore procedure.

For every future stage, preserve the rule:
ONE LAYER AT A TIME.
Do not jump to GCS/frontend work before the underlying communication layer is proven.

==================================================
RESUME INSTRUCTION
==================================================

RESUME HERE:

Run scripts/laptop_rosbridge_smoke_test.py from the actual laptop against
10.145.63.112:9090 (step 6). Then identify/wire up the FastAPI GCS backend
(step 7 onward).

The Pixhawk ↔ Jetson Ethernet/MAVLink/MAVROS chain and the rosbridge layer
are both proven as of 2026-08-27.

Do NOT redo the Ethernet/MAVROS/rosbridge bring-up unless a later test
proves it has regressed. BUT NOTE: two pieces of runtime state do NOT
survive a reboot or a MAVROS restart and must be checked/redone each
session — see CURRENT_STATE.md "RUNTIME STATE THAT DOES NOT SURVIVE A
REBOOT / RESTART":
  1. eno1 runtime IP 192.168.144.1/24
  2. MAVLink SRx_* stream intervals (reissue SET_MESSAGE_INTERVAL)

Do NOT assume PX4 — this is ArduCopter 4.6.3.

Do NOT use 10.41.10.2.

The correct Pixhawk Ethernet address is 192.168.144.14.

The next unfinished layer is the laptop-side rosbridge smoke test, then GCS integration.
