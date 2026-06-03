#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"
SESSION="forcewalker_smoke_$(date +%Y%m%d_%H%M%S)"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forcewalker_ros_logs}"
DURATION=8
SETTLE=12
MODE="full"
LAUNCH=1
KEEP_RUNNING=0
STARTED_SESSION=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [--duration SEC] [--settle SEC] [--force-only] [--no-launch] [--keep-running] [--session NAME]

Runs the Force Walker hardware check, optionally launches the sensor stack in a
temporary tmux session, verifies expected ROS topics, and samples topic rates.

Defaults to the full camera + force/IMU stack.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      DURATION="$2"
      shift 2
      ;;
    --settle)
      SETTLE="$2"
      shift 2
      ;;
    --force-only)
      MODE="force"
      shift
      ;;
    --no-launch)
      LAUNCH=0
      shift
      ;;
    --keep-running)
      KEEP_RUNNING=1
      shift
      ;;
    --session)
      SESSION="$2"
      shift 2
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

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

cleanup() {
  if [[ "$STARTED_SESSION" -eq 1 && "$KEEP_RUNNING" -eq 0 ]]; then
    tmux kill-session -t "$SESSION" 2>/dev/null || true
  fi
}
trap cleanup EXIT

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "Missing required file: $1" >&2
    exit 1
  fi
}

sample_hz() {
  local topic="$1"
  local min_rate="$2"
  local output
  local rate

  echo
  echo "=== $topic ==="
  output="$(timeout "$((DURATION + 3))s" ros2 topic hz "$topic" 2>&1 || true)"
  printf '%s\n' "$output"
  rate="$(printf '%s\n' "$output" | awk '/average rate:/ {rate=$3} END {print rate}')"

  if [[ -z "$rate" ]]; then
    echo "FAIL no average rate observed for $topic" >&2
    return 1
  fi
  if ! awk -v rate="$rate" -v min="$min_rate" 'BEGIN { exit(rate >= min ? 0 : 1) }'; then
    echo "FAIL $topic rate $rate Hz is below required $min_rate Hz" >&2
    return 1
  fi
  echo "PASS $topic rate $rate Hz >= $min_rate Hz"
}

require_topic() {
  local topic="$1"
  if ! grep -qx "$topic" /tmp/forcewalker_smoke_topics.$$; then
    echo "FAIL missing topic: $topic" >&2
    return 1
  fi
  echo "PASS topic visible: $topic"
}

require_file "$ROS_SETUP"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

"$SCRIPT_DIR/fw_check_sensors.sh"

if [[ "$LAUNCH" -eq 1 ]]; then
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session already exists: $SESSION" >&2
    exit 1
  fi

  if [[ "$MODE" == "force" ]]; then
    FW_TMUX_SESSION="$SESSION" "$SCRIPT_DIR/fw_start_sensors.sh" --force-only --no-attach
  else
    FW_TMUX_SESSION="$SESSION" "$SCRIPT_DIR/fw_start_sensors.sh" --with-force --no-attach
  fi
  STARTED_SESSION=1
  echo "Waiting ${SETTLE}s for ROS discovery and sensor startup..."
  sleep "$SETTLE"
fi

source_setup "$ROS_SETUP"
if [[ -f "$FW_SETUP" ]]; then
  source_setup "$FW_SETUP"
else
  echo "Warning: workspace setup missing: $FW_SETUP" >&2
fi

ros2 topic list > /tmp/forcewalker_smoke_topics.$$
trap 'rm -f /tmp/forcewalker_smoke_topics.$$; cleanup' EXIT

echo
echo "Visible Force Walker topics:"
grep -E 'zed_main|rs_upward|rs_downward|forcewalker' /tmp/forcewalker_smoke_topics.$$ | sed 's/^/  /' || true

if [[ "$MODE" == "force" ]]; then
  require_topic "/forcewalker/force_channels"
  require_topic "/forcewalker/imu"
  require_topic "/forcewalker/sensor_diag"
  sample_hz "/forcewalker/force_channels" 40
  sample_hz "/forcewalker/sensor_diag" 0.5
else
  require_topic "/zed_main/zed_node/rgb/color/rect/image"
  require_topic "/zed_main/zed_node/depth/depth_registered"
  require_topic "/rs_upward/rs_upward/depth/image_rect_raw"
  require_topic "/rs_downward/rs_downward/depth/image_rect_raw"
  require_topic "/forcewalker/force_channels"
  require_topic "/forcewalker/sensor_diag"
  sample_hz "/zed_main/zed_node/rgb/color/rect/image" 15
  sample_hz "/zed_main/zed_node/depth/depth_registered" 15
  sample_hz "/rs_upward/rs_upward/depth/image_rect_raw" 10
  sample_hz "/rs_downward/rs_downward/depth/image_rect_raw" 10
  sample_hz "/forcewalker/force_channels" 40
  sample_hz "/forcewalker/sensor_diag" 0.5
fi

echo
echo "Smoke test passed."
if [[ "$STARTED_SESSION" -eq 1 && "$KEEP_RUNNING" -eq 1 ]]; then
  echo "Sensor session left running: $SESSION"
  echo "Stop with: tmux kill-session -t $SESSION"
fi
