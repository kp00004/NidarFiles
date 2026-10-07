#!/usr/bin/env bash
# Starts the complete Jetson side of the NIDAR hover in one terminal:
#
#   MAVROS (Ethernet to the Pixhawk)
#   command_node + heartbeat_node    (existing onboard-autonomy nodes)
#   radio_command_node               (MicroLR900 radio, USB serial: START/ABORT
#                                     in, telemetry out -- the GCS's only link)
#   missions/hover/mission.py        (the hover mission -- REAL FLIGHT)
#
# There is no Wi-Fi link to the GCS and no rosbridge: everything the GCS
# shows comes over the radio.
#
# Does NOT start mission_state_node: it would arm on the same START by
# itself. Refuses to run if it is already running.
#
# Usage:
#   start_jetson.sh            full stack (a radio START will fly the hover)
#   start_jetson.sh --dry-run  radio node ACKs and logs but publishes nothing;
#                              nothing can arm. Use for the first radio test.
#
# Ctrl+C stops everything this script started. Logs: ~/NidarFiles/logs/<time>/
#
# Values below are the documented setup (custom-gcs Daily Startup SOP);
# override with environment variables if the hardware differs.
set -uo pipefail

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

DRY_RUN=false
[ "${1:-}" = "--dry-run" ] && DRY_RUN=true

LOG_DIR="$NIDAR_DIR/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
PIDS=()

say() { echo "[$(date +%H:%M:%S)] $*"; }
fail() { say "STOP: $*"; exit 1; }

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
source /opt/ros/humble/setup.bash || fail "ROS 2 Humble not found"
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/nidar_ws/install/setup.bash" ] || fail "~/nidar_ws not built -- run scripts/jetson/setup_jetson.sh"
source "$HOME/nidar_ws/install/setup.bash"
python3 -c "import nidar_autonomy.radio_command_node" 2>/dev/null || fail "nidar_autonomy (with radio_command_node) not importable -- rebuild with setup_jetson.sh"

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
  -p "baud:=$RADIO_BAUD" -p "dry_run:=$DRY_RUN" -p "telemetry_rate_hz:=$TELEMETRY_RATE_HZ"
run hover_mission python3 "$NIDAR_DIR/missions/hover/mission.py"

say "-------------------------------------------------------------------"
if $DRY_RUN; then
  say "DRY RUN: radio commands are ACKed and logged, nothing is published, nothing can arm."
else
  say "LIVE: a radio START (Hover) from the GCS WILL arm and fly the vehicle."
fi
say "Ctrl+C stops everything. Following radio + hover logs:"
say "-------------------------------------------------------------------"
tail -n +1 -F "$LOG_DIR/radio_command_node.log" "$LOG_DIR/hover_mission.log" &
PIDS+=($!)
wait
