# NIDAR AirMouse — Daily Startup SOP
## Purpose
This is the standard manual bring-up procedure for the NIDAR AirMouse development stack.

Use this procedure after a Jetson reboot, shutdown, or whenever the runtime ROS/network processes are not already running.

> **Important:** This SOP is for the current development setup. Do not treat it as the final competition deployment procedure.

# So your daily startup becomes 🔥

Instead of all that typing:

### Terminal 1

```
source ~/NIDAR/scripts/ros_env.sh
```

### Terminal 2

```
~/NIDAR/scripts/setup_network.sh
```

### Terminal 3

```
~/NIDAR/scripts/start_mavros.sh
```

### Terminal 4

```
~/NIDAR/scripts/start_rosbridge.sh
```

### Terminal 5

```
~/NIDAR/scripts/start_autonomy.sh
```

That's already **way nicer**.

---
# Phase 0 — Physical Safety
Before starting anything:

- 🔴 **Props OFF**
- Pixhawk connected to Jetson via Ethernet
- Jetson powered
- Pixhawk powered
- Laptop connected to the **same phone hotspot/network as the Jetson**
- Physical emergency/kill mechanism accessible
- **Do not ARM during startup**

---
# Phase 1 — Connect Jetson to Hotspot

On the phone:

1. Turn hotspot ON.
2. Connect the Jetson to the hotspot.
3. Connect the Windows laptop to the same hotspot.

Find the Jetson's current IP from:

> Phone → Hotspot → Connected devices → Ubuntu

Example:

```text
10.90.220.112
```

⚠️ **The Jetson Wi-Fi IP can change.**

Always use the current IP shown by the phone.

---

# Phase 2 — Start ROS Environment

Open a Jetson terminal.

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Verify:

```bash
ros2 --help
```

If ROS responds normally:

> ✅ ROS environment ready.

Every new Jetson terminal used for ROS commands must source these files.

---

# Phase 3 — Restore Pixhawk Ethernet IP

In the Jetson terminal:

```bash
sudo ip addr add 192.168.144.1/24 dev eno1
```

Verify:

```bash
ip addr show eno1
```

You should see:

```text
192.168.144.1/24
```

Test the Pixhawk:

```bash
ping -c 3 192.168.144.14
```

Expected:

```text
64 bytes from 192.168.144.14
```

### 🛑 If ping fails

Stop here.

Do not start MAVROS or attempt ARM/DISARM until the Jetson can reach the Pixhawk.

---

# Phase 4 — Start MAVROS

Open a **new Jetson terminal**.

Source ROS:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Start MAVROS:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://@192.168.144.14:14550
```

Leave this terminal running.

## Verify MAVROS

Open another Jetson terminal and source ROS:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Run:

```bash
ros2 topic echo /mavros/state --once
```

Expected:

```text
connected: true
armed: false
```

### 🛑 If `connected: false`

Stop here.

Do not proceed to GCS startup.

---

# Phase 5 — Restore MAVROS Telemetry Streams

After MAVROS starts, the runtime MAVLink message intervals may need to be reissued.

Previously used message IDs:

| Message ID | Purpose |
|---:|---|
| 147 | Battery |
| 32 | Local position |
| 30 | Attitude |
| 27 | IMU |
| 33 | Global position |
| 24 | GPS raw |

⚠️ These are runtime stream-rate requests and may reset when MAVROS/Pixhawk restarts.

Follow the project's currently documented stream-interval procedure.

---

# Phase 6 — Start Rosbridge

Open another Jetson terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Start rosbridge:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

Leave this terminal running.

Verify:

```bash
sudo ss -lntp | grep 9090
```

You want to see something listening on:

```text
0.0.0.0:9090
```

or:

```text
[::]:9090
```

---

# Phase 7 — Start Onboard Autonomy

Open a Jetson terminal and source ROS:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Start `command_node`:

```bash
ros2 run nidar_autonomy command_node
```

Leave it running.

Open another terminal and start `mission_state_node`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nidar_autonomy mission_state_node
```

Leave it running.

Open another terminal and start `heartbeat_node`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nidar_autonomy heartbeat_node
```

Leave it running.

---

# Phase 8 — Verify the Jetson ROS Stack

From a fresh Jetson terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Check nodes:

```bash
ros2 node list
```

Then check FCU:

```bash
ros2 topic echo /mavros/state --once
```

Confirm:

```text
connected: true
armed: false
```

Check mission state:

```bash
ros2 topic echo /mission/state --once
```

The mission state should be available.

---

# Phase 9 — Test Windows → Jetson Connection

On Windows PowerShell, use the **current Jetson hotspot IP**.

Example:

```powershell
Test-NetConnection 10.90.220.112 -Port 9090
```

Replace `10.90.220.112` with today's actual Jetson IP.

You want:

```text
TcpTestSucceeded : True
```

### 🛑 If `TcpTestSucceeded : False`

Do not modify the GCS code.

The Jetson/network side is not ready.

---

# Phase 10 — Start GCS Backend

On Windows:

```powershell
cd D:\Ardra\custom-gcs\gcsackend
```

Set the current Jetson IP:

```powershell
$e0..
nv:ROSBRIDGE_HOST="10.90.220.112"
$env:ROSBRIDGE_PORT="9090"
```

Replace the IP with the current Jetson hotspot IP.

Start FastAPI:

```powershell
uvicorn app.main:app --reload
```

Expected:

```text
Uvicorn running on http://127.0.0.1:8000
```

And importantly:

```text
Application startup complete
```

You should **not** see:

```text
RosTimeoutError: Failed to connect to ROS
```

---

# Phase 11 — Open GCS

Open:

```text
http://127.0.0.1:8000/ui/
```

The NIDAR AirMouse Operator Panel should load.

Check telemetry before doing anything else.

Expected baseline:

```text
Connected: YES
Armed: FALSE
```

---

# Phase 12 — Daily Bench Test

## Before pressing START

Confirm:

```text
☑ Props OFF
☑ Pixhawk connected
☑ MAVROS connected
☑ ROS nodes running
☑ rosbridge running
☑ GCS connected
☑ FCU says armed=false
☑ Physical safety/kill path available
```

## START

Press **START**.

Expected command chain:

```text
START
  ↓
FastAPI
  ↓
rosbridge
  ↓
command_node
  ↓
mission_state_node
  ↓
FlightCommandClient
  ↓
MAVROS
  ↓
Pixhawk
  ↓
ARM
```

Confirm through telemetry:

```text
armed: true
```

## ABORT

For a deliberate bench test, press **ABORT/STOP** while the vehicle is genuinely showing:

```text
armed: true
```

Expected chain:

```text
ABORT
  ↓
FastAPI
  ↓
rosbridge
  ↓
command_node
  ↓
mission_state_node
  ↓
FlightCommandClient
  ↓
MAVROS
  ↓
Pixhawk
  ↓
DISARM
```

Confirm:

```text
armed: false
```

---

# 🧠 Quick Daily Checklist

## Jetson

```text
1. Phone hotspot ON
2. Find Jetson IP
3. source /opt/ros/humble/setup.bash
4. source ~/ros2_ws/install/setup.bash
5. Restore eno1 → 192.168.144.1/24
6. ping 192.168.144.14
7. Start MAVROS
8. Verify /mavros/state → connected=true
9. Restore telemetry stream intervals if required
10. Start rosbridge :9090
11. Start command_node
12. Start mission_state_node
13. Start heartbeat_node
```

## Windows

```text
14. Test-NetConnection <JETSON_IP> -Port 9090
15. cd D:\Ardra\custom-gcs\gcs\backend
16. Set ROSBRIDGE_HOST to current Jetson IP
17. Set ROSBRIDGE_PORT=9090
18. Start uvicorn
19. Open http://127.0.0.1:8000/ui/
20. Verify telemetry
```

## Bench

```text
21. Props OFF
22. Confirm armed=false
23. START → verify armed=true
24. ABORT → verify armed=false
```

---

# 🔧 Current Temporary Limitations

The following are known and intentionally manual for now:

### Jetson Ethernet IP

```bash
sudo ip addr add 192.168.144.1/24 dev eno1
```

does not currently survive reboot.

### ROS processes

MAVROS, rosbridge, and the onboard-autonomy nodes currently need to be launched manually after reboot.

### Hotspot IP

The Jetson's Wi-Fi IP can change between sessions, so the Windows `ROSBRIDGE_HOST` value must be updated accordingly.

---

# 🚀 Future Automation

Once the underlying stack is stable, the manual startup process should eventually be replaced with:

```text
Jetson
└── NIDAR bringup
    ├── Network setup
    ├── MAVROS
    ├── rosbridge
    ├── command_node
    ├── mission_state_node
    └── heartbeat_node

Windows
└── GCS startup script
    ├── Set Jetson IP
    ├── Start FastAPI
    └── Open GCS
```

The goal is eventually:

```bash
./bringup.sh
```

on Jetson and:

```powershell
.\start-gcs.ps1
```

on Windows.

**Do not automate this until the current architecture and startup sequence are stable. Automating the bring-up should come after the underlying system is trusted.**

---

# 🚨 Hard Safety Rules

- **Never fly with props installed during software/bench bring-up unless an explicitly approved flight-test procedure is being followed.**
- Never add arbitrary throttle/motor control to the competition GCS.
- The competition GCS command surface remains **START + ABORT**.
- Do not modify Pixhawk parameters as part of normal daily startup.
- Do not force-arm.
- Do not bypass safety checks merely to make startup succeed.
- If a verification step fails, stop at that step and diagnose it.
- Git commits or code changes are not part of the normal daily bring-up procedure.

---

## Current Architecture

```text
WINDOWS LAPTOP
└── custom-gcs
    └── FastAPI :8000
            │
            │ Wi-Fi / local network
            ▼
JETSON
├── rosbridge :9090
├── onboard-autonomy
│   ├── command_node
│   ├── mission_state_node
│   └── heartbeat_node
│
└── MAVROS
        │
        │ Ethernet
        ▼
PIXHAWK 6X
└── ArduCopter 4.6.3
```

This SOP is the **manual daily baseline**. Use it without Claude for normal startup and bring-up.
