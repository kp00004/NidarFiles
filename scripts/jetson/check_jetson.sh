#!/usr/bin/env bash
# Read-only Jetson checks for the NIDAR hover stack. Changes nothing,
# commands nothing. Run before a test session and paste the output when
# asking for help. The MAVROS checks need start_jetson.sh (or MAVROS)
# already running.
set -uo pipefail

RADIO_PORT="${NIDAR_RADIO_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
PIXHAWK_IP="${NIDAR_PIXHAWK_IP:-192.168.144.14}"
JETSON_ETH_IP="${NIDAR_JETSON_ETH_IP:-192.168.144.1}"
ETH_IF="${NIDAR_ETH_IF:-eno1}"

source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/nidar_ws/install/setup.bash" ] && source "$HOME/nidar_ws/install/setup.bash"

section() { echo; echo "=== $*"; }

section "System"
lsb_release -ds 2>/dev/null; echo "ROS_DISTRO=${ROS_DISTRO:-unset}"; python3 --version
python3 -c "import serial; print('pyserial', serial.__version__)" 2>&1

section "Radio serial device"
ls -l /dev/serial/by-id/ 2>&1
ls -l /dev/ttyUSB* 2>&1
if [ -e "$RADIO_PORT" ]; then echo "OK: radio port exists: $RADIO_PORT"; else echo "MISSING: $RADIO_PORT"; fi
echo "user groups: $(id -nG)"
id -nG | grep -qw dialout && echo "OK: in dialout" || echo "PROBLEM: not in dialout (run setup_jetson.sh, re-login)"
echo "ModemManager: $(systemctl is-active ModemManager 2>/dev/null)"
command -v fuser >/dev/null && fuser -v "$RADIO_PORT" 2>&1 | sed 's/^/  in use by: /'

section "Pixhawk Ethernet"
ip -4 addr show "$ETH_IF" 2>&1
ip -4 addr show "$ETH_IF" | grep -q "$JETSON_ETH_IP" && echo "OK: $ETH_IF has $JETSON_ETH_IP" || echo "PROBLEM: $ETH_IF lacks $JETSON_ETH_IP"
ping -c 2 -W 1 "$PIXHAWK_IP" >/dev/null 2>&1 && echo "OK: Pixhawk $PIXHAWK_IP answers ping" || echo "PROBLEM: Pixhawk $PIXHAWK_IP does not answer"

section "ROS graph"
ros2 node list 2>&1
ros2 node list 2>/dev/null | grep -q mission_state_node && echo "PROBLEM: mission_state_node is running -- it would arm on START by itself; stop it"

section "MAVROS (needs MAVROS running)"
timeout 5 ros2 topic echo --once /mavros/state 2>&1 | head -12
echo "--- local position (the hover needs this; empty = no indoor position estimate)"
timeout 5 ros2 topic echo --once /mavros/local_position/pose 2>&1 | head -12
timeout 6 ros2 topic hz /mavros/local_position/pose 2>&1 | tail -2
echo "--- battery"
timeout 5 ros2 topic echo --once /mavros/battery 2>&1 | grep -E "voltage|percentage" | head -2
echo "--- related topics"
ros2 topic list 2>/dev/null | grep -E "rangefinder|optical_flow|vision_pose|extended_state" || echo "(none)"
echo "--- services the hover uses"
ros2 service list 2>/dev/null | grep -E "/mavros/(set_mode|cmd/takeoff|cmd/arming|set_message_interval)$" || echo "PROBLEM: MAVROS services not found"

section "Radio / hover nodes (needs start_jetson.sh running)"
timeout 3 ros2 topic echo --once /radio/status 2>&1 | head -3
timeout 3 ros2 topic echo --once /flight_test/status 2>&1 | head -3
