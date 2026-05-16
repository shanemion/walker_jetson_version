#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS="${FORCEWALKER_ROS_WS:-/opt/forcewalker/ros2_ws}"
CUDA_ROOT="${CUDA_ROOT:-/usr/local/cuda-13.0}"

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

if [[ ! -d "$WS/src" ]]; then
  echo "Missing ROS workspace source directory: $WS/src" >&2
  exit 1
fi

if [[ ! -d "$CUDA_ROOT" ]]; then
  echo "Missing CUDA root: $CUDA_ROOT" >&2
  exit 1
fi

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "WARN: CONDA_PREFIX is set to $CONDA_PREFIX; this build script will avoid using conda activation."
fi
unset PYTHONHOME
unset PYTHONPATH

if declare -F deactivate >/dev/null 2>&1; then
  deactivate >/dev/null 2>&1 || true
fi
if command -v conda >/dev/null 2>&1; then
  conda deactivate >/dev/null 2>&1 || true
fi

source_setup "$ROS_SETUP"

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is missing. Run ./scripts/fw_install_host_deps.sh first." >&2
  exit 1
fi

if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths "$WS/src" --ignore-src -r -y --rosdistro jazzy
else
  echo "WARN: rosdep is not available; skipping dependency resolution."
fi

cd "$WS"
colcon build --symlink-install \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_DEFAULT_CMP0074=NEW \
    -DCUDAToolkit_ROOT="$CUDA_ROOT"

echo
echo "Build complete."
echo "Source with:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source $WS/install/setup.bash"
