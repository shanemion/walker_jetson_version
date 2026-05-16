#!/usr/bin/env bash
set -euo pipefail

if ! command -v apt-get >/dev/null 2>&1; then
  echo "apt-get is required. This script is intended for Ubuntu 24.04 / JetPack 7.1." >&2
  exit 1
fi

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

echo "Installing Force Walker host dependencies."
echo "This script will not run apt autoremove."

base_packages=(
  ca-certificates
  curl
  git
  gh
  gnupg
  lsb-release
  tmux
  usbutils
  v4l-utils
  python3-pip
  python3-venv
  python3-colcon-common-extensions
  python3-rosdep
)

ros_packages=(
  ros-jazzy-rviz2
  ros-jazzy-tf2-tools
  ros-jazzy-image-transport
  ros-jazzy-image-pipeline
  ros-jazzy-cv-bridge
  ros-jazzy-vision-msgs
  ros-jazzy-pcl-ros
  ros-jazzy-pcl-conversions
  ros-jazzy-foxglove-bridge
  ros-jazzy-rosbag2-storage-mcap
  ros-jazzy-camera-calibration
  ros-jazzy-image-view
  ros-jazzy-zed-msgs
  ros-jazzy-zed-description
  ros-jazzy-realsense2-camera
  ros-jazzy-realsense2-camera-msgs
  ros-jazzy-realsense2-description
  ros-jazzy-librealsense2
)

available=()
missing=()
for pkg in "${base_packages[@]}" "${ros_packages[@]}"; do
  if apt-cache show "$pkg" >/dev/null 2>&1; then
    available+=("$pkg")
  else
    missing+=("$pkg")
  fi
done

"${SUDO[@]}" apt-get update

if [[ "${#available[@]}" -gt 0 ]]; then
  "${SUDO[@]}" apt-get install -y "${available[@]}"
fi

if [[ "${#missing[@]}" -gt 0 ]]; then
  echo
  echo "WARN: These packages were not found in the currently configured apt sources:"
  printf '  %s\n' "${missing[@]}"
  echo "If ROS packages are missing, configure the ROS 2 Jazzy apt source."
  echo "If RealSense packages are missing, configure the RealSense apt source."
fi

if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    echo "Initializing rosdep."
    "${SUDO[@]}" rosdep init
  fi
  rosdep update
else
  echo "WARN: rosdep is not available after install attempt."
fi

echo
echo "Host dependency install step complete."
echo "Next: ./scripts/fw_apply_zed_cuda_patch.sh"
