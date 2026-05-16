#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"
BAG_ROOT="/data/forcewalker/rosbags"

usage() {
  printf 'Usage: %s [--yes]\n' "$(basename "$0")"
}

YES=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --yes|-y)
      YES=1
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
fi

if [[ ! -d "$BAG_ROOT" ]]; then
  echo "Bag root does not exist: $BAG_ROOT" >&2
  exit 1
fi

latest_mcap="$(
  find "$BAG_ROOT" -mindepth 2 -maxdepth 2 -type f -name '*.mcap' -printf '%T@ %p\n' 2>/dev/null \
    | sort -n \
    | tail -1 \
    | cut -d' ' -f2-
)"

if [[ -z "$latest_mcap" ]]; then
  echo "No MCAP files found under $BAG_ROOT" >&2
  exit 1
fi

bag_dir="$(dirname "$latest_mcap")"

echo "Latest bag directory: $bag_dir"
echo
ros2 bag info "$bag_dir"
echo

if [[ "$YES" -ne 1 ]]; then
  read -r -p "Replay this bag? [y/N] " answer
  case "$answer" in
    y|Y|yes|YES)
      ;;
    *)
      echo "Replay cancelled."
      exit 0
      ;;
  esac
fi

exec ros2 bag play "$bag_dir"
