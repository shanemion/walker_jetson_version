#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"

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

echo "Force Walker sensor topics:"
topics="$(ros2 topic list 2>/dev/null | grep -E 'zed_main|rs_upward|rs_downward' || true)"
if [[ -z "$topics" ]]; then
  echo "  No matching topics are currently visible."
  exit 0
fi

printf '%s\n' "$topics" | sed 's/^/  /'
echo

hz_candidates="$(
  printf '%s\n' "$topics" \
    | grep -E '/(image_rect_raw|depth_registered|rgb/color/rect/image)$' \
    | grep -v -E '/(compressed|compressedDepth|theora|zstd)' \
    || true
)"

if [[ -z "$hz_candidates" ]]; then
  echo "No image/depth topics selected for hz sampling."
  exit 0
fi

echo "Short topic hz samples; each sample is bounded and may report no data if publishers are idle."
while IFS= read -r topic; do
  [[ -z "$topic" ]] && continue
  echo
  echo "=== $topic ==="
  timeout 6s ros2 topic hz "$topic" 2>&1 || true
done <<<"$hz_candidates"
