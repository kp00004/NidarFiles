#!/usr/bin/env bash
# Starts the complete Jetson side of the NIDAR hover in one terminal:
#
#   MAVROS (Ethernet to the Pixhawk)
#   command_node + heartbeat_node    (existing onboard-autonomy nodes)
#   radio_command_node               (MicroLR900 radio, USB serial: START/ABORT
#                                     in, telemetry out -- the GCS's only link)
#   missions/hover/mission.py        (the hover mission -- REAL FLIGHT)
#   missions/motor_test/mission.py   (motor test -- spins motors, PROPS OFF)
#
# There is no Wi-Fi link to the GCS and no rosbridge: everything the GCS
# shows comes over the radio.
#
# Does NOT start mission_state_node: it would arm on the same START by
# itself. Refuses to run if it is already running.
#
# Usage:
#   start_jetson.sh            full stack (a radio START will fly the hover)
#   start_jetson.sh --lidar    also run the RPLIDAR A2 and send its scan to the
#                              GCS over the radio (LiDAR panel)
#   start_jetson.sh --setup    also let the GCS Setup page write Pixhawk
#                              parameters (bench only; refused while armed)
#   start_jetson.sh --dry-run  radio node ACKs and logs but publishes nothing;
#                              nothing can arm. Use for the first radio test.
#
# Ctrl+C stops everything this script started. Logs: ~/NidarFiles/logs/<time>/
#
# Values below are the documented setup (custom-gcs Daily Startup SOP);
# override with environment variables if the hardware differs.
set -uo pipefail
[ "$(id -u)" -eq 0 ] && { echo "ERROR: run as your normal user, not with sudo." >&2; exit 1; }

NIDAR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PIXHAWK_IP="${NIDAR_PIXHAWK_IP:-192.168.144.14}"
PIXHAWK_PORT="${NIDAR_PIXHAWK_PORT:-14550}"
JETSON_ETH_IP="${NIDAR_JETSON_ETH_IP:-192.168.144.1}"
ETH_IF="${NIDAR_ETH_IF:-eno1}"
RADIO_PORT="${NIDAR_RADIO_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
RADIO_FALLBACK_PORT="${NIDAR_RADIO_FALLBACK_PORT:-/dev/ttyUSB0}"
RADIO_BAUD="${NIDAR_RADIO_BAUD:-115200}"
# Position/attitude rate over the radio (FCU state, battery: 1 Hz). Keep it
# low: START/ABORT ACKs share the radio's air time.
# TODO(hardware): raise only after measuring the radio's real throughput.
TELEMETRY_RATE_HZ="${NIDAR_TELEMETRY_RATE_HZ:-2.0}"
# LiDAR (--lidar): scans sent over the radio per second (~180 B each), and the
# direction of the LiDAR's 0 deg mark relative to the drone's nose (clockwise).
LIDAR_RATE_HZ="${NIDAR_LIDAR_RATE_HZ:-1.0}"
LIDAR_YAW_DEG="${NIDAR_LIDAR_YAW_DEG:-0}"

DRY_RUN=false
SETUP=false
LIDAR=false
for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=true ;;
    --setup) SETUP=true ;;
    --lidar) LIDAR=true ;;
    *) echo "unknown option: $arg (use --dry-run, --setup, --lidar)"; exit 1 ;;
  esac
done

LOG_DIR="$NIDAR_DIR/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
PIDS=()

say() { echo "[$(date +%H:%M:%S)] $*"; }
fail() { say "STOP: $*"; exit 1; }

# ros2 reads "-p name:=270" as an INTEGER, and a node that declares the
# parameter as a double then refuses it (InvalidParameterTypeException) and
# exits. Every float-typed parameter passed below goes through ros_double,
# which turns a whole number into a decimal ("270" -> "270.0") and rejects
# anything that isn't a plain number.
ros_double() {
  local v="$1"
  [[ "$v" =~ ^[+-]?[0-9]+(\.[0-9]+)?$ ]] || return 1
  [[ "$v" == *.* ]] && echo "$v" || echo "$v.0"
}
TELEMETRY_RATE_HZ="$(ros_double "$TELEMETRY_RATE_HZ")" || fail "NIDAR_TELEMETRY_RATE_HZ must be a number (e.g. 2 or 2.0)"
LIDAR_RATE_HZ="$(ros_double "$LIDAR_RATE_HZ")" || fail "NIDAR_LIDAR_RATE_HZ must be a number (e.g. 1 or 0.5)"
LIDAR_YAW_DEG="$(ros_double "$LIDAR_YAW_DEG")" || fail "NIDAR_LIDAR_YAW_DEG must be a number of degrees (e.g. 0, 90, 180, 270)"

cleanup() {
  say "stopping ${#PIDS[@]} processes"
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null; done
}
trap cleanup EXIT

run() {  # run <name> <command...>  -- background, logged
  local name=$1; shift
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=($!)
  say "started $name (pid $!, log $LOG_DIR/$name.log)"
}

# -- environment ----------------------------------------------------------------
set +u  # ROS setup files read unset variables
source /opt/ros/humble/setup.bash || fail "ROS 2 Humble not found"
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/nidar_ws/install/setup.bash" ] || fail "~/nidar_ws not built -- run scripts/jetson/setup_jetson.sh"
source "$HOME/nidar_ws/install/setup.bash"
python3 -c "import nidar_autonomy.radio_command_node" 2>/dev/null || fail "nidar_autonomy (with radio_command_node) not importable -- rebuild with setup_jetson.sh"
set -u

# -- USB serial ports: radio vs LiDAR (before anything opens the radio) ---------
# The RPLIDAR's USB adapter and the radio are both CP2102 chips. On our units
# their /dev/serial/by-id names differ (the LiDAR's has a long unique serial,
# the radio's ends in "Controller_0001"), but CP2102 adapters MAY share a
# name, so the port choice does not rely on names: whenever more than one USB
# serial port is present (or --lidar is given), ask each port which one
# answers like an RPLIDAR (GET_INFO) and give the radio another port -- so a
# plugged-in LiDAR can never take the radio's place, with or without --lidar.
LIDAR_PORT=""
USB_SERIAL_COUNT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | wc -l)
if $LIDAR || [ "$USB_SERIAL_COUNT" -gt 1 ]; then
  say "checking $USB_SERIAL_COUNT USB serial port(s) for the RPLIDAR..."
  eval "$(python3 -m nidar_autonomy.rplidar_probe)"
  if [ -z "$LIDAR_PORT" ]; then
    $LIDAR && say "WARNING: no RPLIDAR answered (ports: ${OTHER_PORTS:-none}) -- continuing WITHOUT LiDAR"
    LIDAR=false
  else
    say "RPLIDAR on $LIDAR_PORT @ $LIDAR_BAUD ($LIDAR_INFO)$($LIDAR || echo ' -- not used (no --lidar)')"
    if [ -n "${NIDAR_RADIO_PORT:-}" ]; then
      [ "$(readlink -f "$NIDAR_RADIO_PORT")" = "$LIDAR_PORT" ] && fail "NIDAR_RADIO_PORT=$NIDAR_RADIO_PORT is the LiDAR, not the radio"
    else
      read -r FIRST_OTHER _ <<<"${OTHER_PORTS:-}"
      if [ -n "${FIRST_OTHER:-}" ]; then
        RADIO_PORT="$FIRST_OTHER"
        RADIO_FALLBACK_PORT="$FIRST_OTHER"
        say "radio -> $RADIO_PORT (the USB serial port that is not the LiDAR)"
      else
        say "WARNING: only the LiDAR is plugged in -- no USB serial port left for the radio"
      fi
    fi
  fi
fi

# -- safety: one authoritative execution path ----------------------------------
if pgrep -f "nidar_autonomy/mission_state_node|lib/nidar_autonomy/mission_state_node" >/dev/null; then
  fail "mission_state_node is running -- it would arm on START by itself. Stop it first."
fi

# -- Pixhawk link ---------------------------------------------------------------
if ! ip -4 addr show "$ETH_IF" | grep -q "$JETSON_ETH_IP"; then
  say "$ETH_IF lacks $JETSON_ETH_IP -- restoring (runtime only, see SOP Phase 3)"
  sudo ip addr add "$JETSON_ETH_IP/24" dev "$ETH_IF" || fail "could not set $JETSON_ETH_IP on $ETH_IF"
fi
ping -c 2 -W 1 "$PIXHAWK_IP" >/dev/null 2>&1 || fail "Pixhawk $PIXHAWK_IP does not answer on $ETH_IF"
say "Pixhawk reachable at $PIXHAWK_IP"

[ -e "$RADIO_PORT" ] || [ -e "$RADIO_FALLBACK_PORT" ] || say "WARNING: radio not found ($RADIO_PORT / $RADIO_FALLBACK_PORT) -- radio node will keep retrying"

# -- MAVROS ---------------------------------------------------------------------
if pgrep -f "mavros_node" >/dev/null; then
  say "MAVROS already running -- reusing it"
else
  run mavros ros2 launch mavros apm.launch "fcu_url:=udp://@$PIXHAWK_IP:$PIXHAWK_PORT"
fi
say "waiting for /mavros/state connected=true (30 s)"
connected=false
for _ in $(seq 1 15); do
  if timeout 3 ros2 topic echo --once /mavros/state 2>/dev/null | grep -q "connected: true"; then
    connected=true; break
  fi
  sleep 1
done
$connected || fail "MAVROS not connected to the Pixhawk (see $LOG_DIR/mavros.log)"
say "MAVROS connected"

# -- onboard-autonomy nodes -------------------------------------------------------
run command_node ros2 run nidar_autonomy command_node
run heartbeat_node ros2 run nidar_autonomy heartbeat_node
run radio_command_node ros2 run nidar_autonomy radio_command_node --ros-args \
  -p "serial_port:=$RADIO_PORT" -p "fallback_serial_port:=$RADIO_FALLBACK_PORT" \
  -p "baud:=$RADIO_BAUD" -p "dry_run:=$DRY_RUN" -p "telemetry_rate_hz:=$TELEMETRY_RATE_HZ"   -p "allow_param_write:=$SETUP" -p "lidar_rate_hz:=$LIDAR_RATE_HZ"
run hover_mission python3 "$NIDAR_DIR/missions/hover/mission.py"
run motor_test_mission python3 "$NIDAR_DIR/missions/motor_test/mission.py"
if $LIDAR; then
  run lidar_node ros2 run nidar_autonomy lidar_node --ros-args \
    -p "serial_port:=$LIDAR_PORT" -p "baud:=$LIDAR_BAUD" -p "yaw_offset_deg:=$LIDAR_YAW_DEG"
fi

say "-------------------------------------------------------------------"
if $SETUP; then
  say "SETUP: the GCS Setup page may WRITE Pixhawk parameters (refused while armed)."
fi
if $DRY_RUN; then
  say "DRY RUN: radio commands are ACKed and logged, nothing is published, nothing can arm."
else
  say "LIVE: a radio START (Hover) from the GCS WILL arm and fly the vehicle;"
  say "      a radio START (Motor Test) WILL spin the motors -- PROPS OFF."
fi
say "Ctrl+C stops everything. Following radio + mission logs:"
say "-------------------------------------------------------------------"
LOGS=("$LOG_DIR/radio_command_node.log" "$LOG_DIR/hover_mission.log" "$LOG_DIR/motor_test_mission.log")
$LIDAR && LOGS+=("$LOG_DIR/lidar_node.log")
tail -n +1 -F "${LOGS[@]}" &
PIDS+=($!)
wait
