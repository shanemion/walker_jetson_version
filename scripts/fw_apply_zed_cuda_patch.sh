#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCH_FILE="$REPO_ROOT/patches/zed-ros2-wrapper-cuda13-thor.patch"
ZED_WRAPPER="${ZED_WRAPPER:-/opt/forcewalker/ros2_ws/src/zed-ros2-wrapper}"

if [[ ! -f "$PATCH_FILE" ]]; then
  echo "Patch file missing: $PATCH_FILE" >&2
  exit 1
fi

if [[ ! -d "$ZED_WRAPPER/.git" ]]; then
  echo "ZED wrapper git checkout missing: $ZED_WRAPPER" >&2
  echo "Clone https://github.com/stereolabs/zed-ros2-wrapper.git into /opt/forcewalker/ros2_ws/src first." >&2
  exit 1
fi

if git -C "$ZED_WRAPPER" apply --check "$PATCH_FILE" >/dev/null 2>&1; then
  git -C "$ZED_WRAPPER" apply "$PATCH_FILE"
  echo "Applied Jetson Thor CUDA/ZED patch to $ZED_WRAPPER"
  exit 0
fi

if git -C "$ZED_WRAPPER" apply --reverse --check "$PATCH_FILE" >/dev/null 2>&1; then
  echo "Jetson Thor CUDA/ZED patch is already applied in $ZED_WRAPPER"
  exit 0
fi

echo "Patch could not be applied cleanly and does not appear to be already applied." >&2
echo "Inspect local changes in: $ZED_WRAPPER" >&2
git -C "$ZED_WRAPPER" status --short >&2 || true
exit 1
