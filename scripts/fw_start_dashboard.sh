#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"
DASHBOARD="/opt/forcewalker/forcewalker/scripts/fw_simple_dashboard.py"

HOST="${FORCEWALKER_DASHBOARD_HOST:-127.0.0.1}"
PORT="${FORCEWALKER_DASHBOARD_PORT:-8088}"

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Missing ROS setup: $ROS_SETUP" >&2
  exit 1
fi

source_setup "$ROS_SETUP"
if [[ -f "$FW_SETUP" ]]; then
  source_setup "$FW_SETUP"
fi

exec /usr/bin/python3 "$DASHBOARD" --host "$HOST" --port "$PORT"
