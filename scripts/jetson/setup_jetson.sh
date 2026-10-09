#!/usr/bin/env bash
# One-time Jetson setup for the NIDAR hover stack. Safe to re-run.
#
# Expects this NidarFiles folder copied to the Jetson (default ~/NidarFiles,
# see README.md "Deploy to the Jetson"). Does NOT touch Pixhawk parameters
# and never arms anything.
#
# Builds onboard-autonomy into its own overlay workspace (~/nidar_ws) so the
# existing ~/ros2_ws is left untouched; start_jetson.sh sources ~/nidar_ws
# after ~/ros2_ws, so this build of nidar_autonomy takes precedence.
set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
  echo "ERROR: run this as your normal user, NOT with sudo (it calls sudo itself where needed;
  as root it would build into /root/nidar_ws)." >&2
  exit 1
fi

NIDAR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OVERLAY_WS="${NIDAR_OVERLAY_WS:-$HOME/nidar_ws}"

echo "== NidarFiles: $NIDAR_DIR"
echo "== overlay workspace: $OVERLAY_WS"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: ROS 2 Humble not found at /opt/ros/humble" >&2
  exit 1
fi
# shellcheck disable=SC1091
# ROS setup files read unset variables -- not compatible with set -u.
set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
set -u

echo "== apt packages (pyserial, MAVROS, colcon)"
# Only installs what is missing, so a Jetson with no internet (no Wi-Fi
# link in this setup) passes this step when everything is already there.
PACKAGES=(python3-serial python3-colcon-common-extensions ros-humble-mavros ros-humble-mavros-msgs)
MISSING=()
for pkg in "${PACKAGES[@]}"; do
  dpkg -s "$pkg" >/dev/null 2>&1 || MISSING+=("$pkg")
done
if [ ${#MISSING[@]} -eq 0 ]; then
  echo "   all installed"
else
  echo "   missing: ${MISSING[*]} (needs internet)"
  # One broken third-party apt source makes `apt-get update` fail as a whole;
  # the install below still works from the sources that did update.
  sudo apt-get update || echo "   WARNING: apt-get update reported errors (see above) -- trying the install anyway"
  sudo apt-get install -y "${MISSING[@]}"
fi

echo "== optional: Cartographer (2D SLAM map for start_jetson.sh --map)"
if dpkg -s ros-humble-cartographer-ros >/dev/null 2>&1; then
  echo "   installed"
elif sudo apt-get install -y ros-humble-cartographer-ros; then
  echo "   installed now"
else
  echo "   WARNING: could not install ros-humble-cartographer-ros (no internet?) -- --map won't work until it is"
fi

echo "== serial permissions"
if id -nG "$USER" | grep -qw dialout; then
  echo "   $USER is already in dialout"
else
  sudo usermod -aG dialout "$USER"
  echo "   added $USER to dialout -- LOG OUT AND BACK IN (or reboot) before starting the stack"
fi

echo "== ModemManager (it probes new USB serial devices and can grab the radio)"
if systemctl is-active --quiet ModemManager 2>/dev/null; then
  read -r -p "   ModemManager is running. Disable it now? [y/N] " answer
  if [[ "$answer" =~ ^[Yy]$ ]]; then
    sudo systemctl disable --now ModemManager
    echo "   ModemManager disabled"
  else
    echo "   left running -- if the radio port is busy/garbled, disable it"
  fi
else
  echo "   not running"
fi

echo "== build onboard-autonomy (nidar_airmouse, nidar_autonomy) into $OVERLAY_WS"
mkdir -p "$OVERLAY_WS"
cd "$OVERLAY_WS"
colcon build --symlink-install \
  --base-paths "$NIDAR_DIR/onboard-autonomy" \
  --packages-select nidar_airmouse nidar_autonomy

chmod +x "$NIDAR_DIR"/scripts/jetson/*.sh "$NIDAR_DIR"/missions/*/mission.py

echo
echo "Done. Next: $NIDAR_DIR/scripts/jetson/check_jetson.sh (read-only checks),"
echo "then $NIDAR_DIR/scripts/jetson/start_jetson.sh --dry-run for the first radio test."
