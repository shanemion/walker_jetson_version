#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"
BAG_ROOT="/data/forcewalker/rosbags"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forcewalker_ros_logs}"
SESSION="forcewalker_record_$(date +%Y%m%d_%H%M%S)"
LABEL="trial"
DURATION=60
MODE="full"
KEEP_RUNNING=0
ROSBAG_STOP_TIMEOUT=30

usage() {
  cat <<EOF
Usage: $(basename "$0") --label LABEL [--duration SEC] [--force-only] [--cameras-only] [--keep-running]

Starts a labeled MCAP recording session, waits for the requested duration, then
stops the tmux session and prints rosbag info for the newest bag.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --label)
      LABEL="$2"
      shift 2
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    --force-only)
      MODE="force"
      shift
      ;;
    --cameras-only)
      MODE="cameras"
      shift
      ;;
    --keep-running)
      KEEP_RUNNING=1
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

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

cleanup() {
  if [[ "$KEEP_RUNNING" -eq 0 ]]; then
    tmux kill-session -t "$SESSION" 2>/dev/null || true
  fi
}
trap cleanup EXIT

latest_bag_dir() {
  local label_pattern="$1"
  local latest_mcap

  latest_mcap="$(
    find "$BAG_ROOT" -mindepth 2 -maxdepth 2 -type f -name "*.mcap" -path "*${label_pattern}*" -printf '%T@ %p\n' 2>/dev/null \
      | sort -n \
      | tail -1 \
      | cut -d' ' -f2-
  )"

  if [[ -n "$latest_mcap" ]]; then
    dirname "$latest_mcap"
    return 0
  fi
  return 1
}

wait_for_rosbag_metadata() {
  local label_pattern="$1"
  local bag_dir=""

  for _ in $(seq 1 "$ROSBAG_STOP_TIMEOUT"); do
    bag_dir="$(latest_bag_dir "$label_pattern" || true)"
    if [[ -n "$bag_dir" && -f "$bag_dir/metadata.yaml" ]]; then
      printf '%s\n' "$bag_dir"
      return 0
    fi
    sleep 1
  done

  if [[ -n "$bag_dir" ]]; then
    printf '%s\n' "$bag_dir"
    return 1
  fi
  return 1
}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Missing ROS setup: $ROS_SETUP" >&2
  exit 1
fi
if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required." >&2
  exit 1
fi

mkdir -p "$ROS_LOG_DIR" "$BAG_ROOT"
export ROS_LOG_DIR
safe_label="$(printf '%s' "$LABEL" | tr -cs 'A-Za-z0-9_.-' '_' | sed 's/^_*//; s/_*$//')"

case "$MODE" in
  force)
    FW_TMUX_SESSION="$SESSION" "$SCRIPT_DIR/fw_start_sensors.sh" --force-only --record --no-attach --bag-label "$LABEL"
    ;;
  cameras)
    FW_TMUX_SESSION="$SESSION" "$SCRIPT_DIR/fw_start_sensors.sh" --record --no-attach --bag-label "$LABEL"
    ;;
  full)
    FW_TMUX_SESSION="$SESSION" "$SCRIPT_DIR/fw_start_sensors.sh" --with-force --record --no-attach --bag-label "$LABEL"
    ;;
  *)
    echo "Invalid mode: $MODE" >&2
    exit 2
    ;;
esac

echo "Recording label: $LABEL"
echo "Recording mode: $MODE"
echo "Recording duration: ${DURATION}s"
echo "tmux session: $SESSION"

if [[ "$KEEP_RUNNING" -eq 1 ]]; then
  echo "Recording left running. Stop with: tmux kill-session -t $SESSION"
  exit 0
fi

sleep "$DURATION"

if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx rosbag; then
  echo "Stopping rosbag recorder gracefully..."
  tmux send-keys -t "$SESSION:rosbag" C-c 2>/dev/null || true
  bag_dir="$(wait_for_rosbag_metadata "$safe_label" || true)"
else
  bag_dir="$(latest_bag_dir "$safe_label" || true)"
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
trap - EXIT

source_setup "$ROS_SETUP"
if [[ -f "$FW_SETUP" ]]; then
  source_setup "$FW_SETUP"
fi

if [[ -z "${bag_dir:-}" ]]; then
  echo "No MCAP bag directory found under $BAG_ROOT for label '$LABEL'" >&2
  exit 1
fi

metadata_file="$bag_dir/forcewalker_trial_metadata.txt"
{
  echo "label: $LABEL"
  echo "mode: $MODE"
  echo "duration_s: $DURATION"
  echo "session: $SESSION"
  echo "recorded_at_local: $(date --iso-8601=seconds)"
} > "$metadata_file"

echo
echo "Recorded bag: $bag_dir"
echo "Metadata: $metadata_file"
ros2 bag info "$bag_dir"
