#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"
RVIZ_CONFIG="/opt/forcewalker/forcewalker/config/forcewalker_sensors.rviz"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--rviz]

Default starts Foxglove bridge on port ${FOXGLOVE_PORT}.
Use --rviz to start RViz2 with the Force Walker sensor config.
EOF
}

MODE="foxglove"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rviz)
      MODE="rviz"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Missing ROS setup: $ROS_SETUP" >&2
  exit 1
fi

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

source_setup "$ROS_SETUP"

if [[ -f "$FW_SETUP" ]]; then
  source_setup "$FW_SETUP"
else
  echo "Warning: workspace setup missing: $FW_SETUP" >&2
fi

echo "Current Force Walker sensor topics:"
ros2 topic list 2>/dev/null | grep -E 'zed_main|rs_upward|rs_downward' || echo "No Force Walker sensor topics currently visible."
echo

if [[ "$MODE" == "rviz" ]]; then
  if ! command -v rviz2 >/dev/null 2>&1; then
    echo "rviz2 command not found." >&2
    exit 1
  fi
  if [[ -f "$RVIZ_CONFIG" ]]; then
    echo "Starting RViz2 with $RVIZ_CONFIG"
    exec rviz2 -d "$RVIZ_CONFIG"
  fi
  echo "RViz config missing: $RVIZ_CONFIG; starting plain RViz2." >&2
  exec rviz2
fi

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "foxglove_bridge ROS package is not available. Try RViz instead:" >&2
  echo "  $0 --rviz" >&2
  exit 1
fi

host_name="$(hostname 2>/dev/null || printf soar-jeston)"
host_ips="$(hostname -I 2>/dev/null | xargs || true)"

echo "Starting Foxglove bridge."
echo "Connect from another computer/browser with one of:"
if [[ -n "$host_ips" ]]; then
  for ip in $host_ips; do
    echo "  ws://${ip}:${FOXGLOVE_PORT}"
  done
fi
echo "  ws://${host_name}:${FOXGLOVE_PORT}"
echo "  ws://${host_name}.local:${FOXGLOVE_PORT}"
echo

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:="$FOXGLOVE_PORT" \
  address:=0.0.0.0
