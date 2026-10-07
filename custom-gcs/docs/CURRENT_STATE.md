> **SUPERSEDED (2026-09-04).** This file was the pre-cursor to
> `CHECKPOINT/CURRENT_STATE.md` (workspace root, sibling of this repo),
> which has been the canonical, actively-maintained cross-repo status
> record since 2026-08-28 — the day after this file's last update.
> Neither `custom-gcs/CLAUDE.md` nor `onboard-autonomy/CLAUDE.md`
> reference this file anymore. Kept here, unmodified below this notice,
> as a historical record of the initial 2026-08-25–27 hardware bring-up
> (may contain detail not repeated in `CHECKPOINT/CURRENT_STATE.md`) —
> do not treat anything below as current. Read `CHECKPOINT/CURRENT_STATE.md`
> instead for anything about present-day project state.

# NIDAR Hardware Bring-Up — Current State

Checkpoint created: 2026-08-25
Updated: 2026-08-27

==================================================
HARDWARE
==================================================

**Pixhawk:**
- Pixhawk 6X
- ArduCopter 4.6.3
- Connected to Jetson Orin Nano via Pixhawk ETH0
- Props OFF / vehicle DISARMED during bring-up

**Jetson:**
- NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
- Ubuntu 22.04.5 LTS
- JetPack 6
- ARM64
- Ethernet interface: eno1
- Wi-Fi interface: wlP1p1s0
- Wi-Fi is used for laptop connectivity
- Ethernet is dedicated to Pixhawk communication

==================================================
PIXHAWK NETWORK
==================================================

NET_ENABLE = 1
NET_DHCP was changed from 1 → 0

Pixhawk static Ethernet configuration:
- IP: 192.168.144.14
- Netmask: /24
- Gateway: 192.168.144.1

Jetson Ethernet:
- Runtime address: 192.168.144.1/24
- This address is currently runtime-only and is NOT yet persistent in NetworkManager.
- Jetson also has its normal 192.168.1.100/24 Ethernet configuration.
- Be careful: reboot/link flap can remove the runtime 192.168.144.1 address.
- A narrow NOPASSWD sudoers rule was granted for `ip addr add/del/show` on `eno1` only
  (installed at /etc/sudoers.d/010-claude-ip-eno1), so this address can be restored
  without asking for a password each time.

==================================================
PHYSICAL ETHERNET
==================================================

Verified:
- Ethernet carrier/link is UP
- 100 Mb/s
- Full duplex
- Pixhawk Ethernet PHY successfully negotiates with Jetson
- ARP eventually resolved:
  192.168.144.14 → c2:af:51:47:8b:68
- Ping to Pixhawk succeeded after fixing the DHCP/static-IP configuration and restoring the Jetson runtime address.

==================================================
MAVLINK
==================================================

Pixhawk Ethernet MAVLink endpoint was configured as:

NET_P1_TYPE = 2
- UDP Server

NET_P1_PROTOCOL = 2
- MAVLink2

NET_P1_PORT = 14550

NET_P1_IP remains 0.0.0.0 because server mode does not require a destination IP.

Verified with pymavlink:
- Real MAVLink heartbeats received from Pixhawk
- 5/5 heartbeats received
- sysid = 1
- compid = 1
- autopilot = ArduPilot
- vehicle type = quadrotor
- Vehicle confirmed DISARMED
- No arm/motor/flight commands were sent

==================================================
MAVROS
==================================================

ROS 2 Humble is installed.

Installed:
- ros-humble-ros-base
- ros-humble-mavros
- ros-humble-mavros-msgs
- GeographicLib datasets

MAVROS launch file:
- /opt/ros/humble/share/mavros/launch/apm.launch

MAVROS is running with:

fcu_url:=udp://@192.168.144.14:14550

Verified:
- MAVROS connects to FCU
- FCU reported ArduPilot
- Firmware reported 4.6.3
- /mavros/state:
  connected = true
  armed = false
  mode = STABILIZE

==================================================
REAL TELEMETRY VERIFIED
==================================================

**/mavros/imu/data**
- steady ~10 Hz
- real accelerometer/gyro/orientation data
- Z acceleration approximately 9.78 m/s²
- confirms genuine live sensor telemetry

**/mavros/battery**
- publishing, steady ~10 Hz
- voltage currently 0.0
- this is expected because the bench setup currently has no battery/power module wired in
- NOT considered a communication failure

**/mavros/local_position/pose**
- currently empty
- this is EXPECTED because there is no valid indoor position source / EKF position source yet
- GPS is unavailable/forbidden
- future SLAM/vision-position integration will address this
- do NOT treat this as a MAVROS Ethernet failure

**MAVROS stream-rate investigation:**
- MAVROS initially connected and performed parameter pull
- periodic telemetry initially appeared absent
- investigation found SR0_* had normal non-zero rates while other SRx channels were zero
- runtime SET_MESSAGE_INTERVAL requests (via /mavros/cmd/command, MAV_CMD_SET_MESSAGE_INTERVAL = 511)
  were used to force streaming of SYS_STATUS(1), BATTERY_STATUS(147), LOCAL_POSITION_NED(32),
  ATTITUDE(30), RAW_IMU(27)
- telemetry then became sustained and real
- These message-interval requests are runtime-only and reset after restart (MAVROS restart or Pixhawk reboot)
- A future decision is required on whether to:
  1. persist appropriate SRx_* parameters, OR
  2. have Jetson/MAVROS reissue message-interval requests on connection
- Prefer minimal Pixhawk parameter footprint unless there is a strong reason otherwise.

==================================================
SAFETY
==================================================

- Vehicle remains disarmed.
- No motor commands were issued.
- No arm command was issued.
- No flight command was issued.
- Current communication work is bench-only.

==================================================
ROSBRIDGE
==================================================

INSTALLED and VERIFIED WORKING (as of 2026-08-27).

Installed packages:
- ros-humble-rosbridge-library
- ros-humble-rosbridge-msgs
- ros-humble-rosbridge-server
- ros-humble-rosbridge-suite

Launched with:
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090

Verified:
- rosbridge WebSocket server listening on 0.0.0.0:9090 (both IPv4 and IPv6)
- Confirmed with a raw TCP connect test and with a Python `websockets`
  client that sent a rosbridge `subscribe` op for /mavros/imu/data
- Real MAVROS data was relayed through rosbridge:
  linear_acceleration.z ≈ 9.80 m/s² (matches expected gravity reading)

Jetson Wi-Fi IP (for laptop connectivity): 10.145.63.112
Interface: wlP1p1s0
(This is a DHCP-assigned address and may change on reconnect — re-check
with `ip addr show wlP1p1s0` if the laptop can't reach it.)

A laptop-side smoke test script was added at
scripts/laptop_rosbridge_smoke_test.py (uses roslibpy). It has NOT yet
been run from the actual laptop — that is the next unverified step.

==================================================
RUNTIME STATE THAT DOES NOT SURVIVE A REBOOT / RESTART
==================================================

Two things reset and must be redone any time the Jetson reboots, the
eno1 link flaps, or MAVROS is restarted:

1. Jetson runtime IP on eno1 (192.168.144.1/24) — restore with:
   sudo ip addr add 192.168.144.1/24 dev eno1
   (covered by the narrow NOPASSWD sudoers rule, no password needed)

2. Pixhawk MAVLink stream intervals (SRx_* effectively zero except SR0) —
   restore by reissuing MAV_CMD_SET_MESSAGE_INTERVAL (command 511) via
   /mavros/cmd/command for each of:
     SYS_STATUS        (1)   @ 10 Hz  (100000 us)
     BATTERY_STATUS   (147)  @ 1 Hz   (1000000 us)
     LOCAL_POSITION_NED (32) @ 10 Hz  (100000 us)
     ATTITUDE          (30)  @ 10 Hz  (100000 us)
     RAW_IMU           (27)  @ 10 Hz  (100000 us)
   Call each service individually with a timeout — calling them in a fast
   loop without per-call timeout has been observed to hang on one call.
   This was re-verified working on 2026-08-27 after a MAVROS restart.

Both of these were redone and reverified on 2026-08-27. This is exactly
the "future decision" flagged below — still not made persistent yet.

==================================================
CURRENT ARCHITECTURE
==================================================

Current proven chain:

Pixhawk 6X
    ↓ Ethernet / MAVLink2 UDP
192.168.144.14:14550
    ↓
Jetson Orin Nano
    ↓
MAVROS
    ↓
ROS 2 telemetry

Target chain:

Pixhawk
    ↓
MAVLink
    ↓
MAVROS
    ↓
ROS 2
    ↓
rosbridge_server :9090
    ↓
Wi-Fi
    ↓
Laptop
    ↓
FastAPI / roslibpy
    ↓
GCS frontend later
