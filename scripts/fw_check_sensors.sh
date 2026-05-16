#!/usr/bin/env bash
set -uo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"

pass() { printf 'PASS %s\n' "$*"; }
warn() { printf 'WARN %s\n' "$*"; }
fail() { printf 'FAIL %s\n' "$*"; }
info() { printf 'INFO %s\n' "$*"; }

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

check_pkg() {
  local pkg="$1"
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    pass "ROS package available: $pkg"
  else
    fail "ROS package missing: $pkg"
  fi
}

info "host=$(hostname 2>/dev/null || printf unknown) user=$(whoami 2>/dev/null || printf unknown)"

if have_cmd dpkg-query; then
  jetpack_version="$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null || true)"
  if [[ -n "$jetpack_version" ]]; then
    pass "JetPack nvidia-jetpack=$jetpack_version"
  else
    warn "JetPack package nvidia-jetpack not found by dpkg-query"
  fi
else
  warn "dpkg-query not available; cannot check JetPack package"
fi

if have_cmd docker; then
  if docker info >/tmp/fw_docker_info.$$ 2>/tmp/fw_docker_err.$$; then
    pass "Docker daemon reachable"
    runtimes="$(grep -i '^ Runtimes:' /tmp/fw_docker_info.$$ | sed 's/^[[:space:]]*//')"
    default_runtime="$(grep -i '^ Default Runtime:' /tmp/fw_docker_info.$$ | sed 's/^[[:space:]]*//')"
    if grep -q 'nvidia' /tmp/fw_docker_info.$$; then
      pass "Docker NVIDIA runtime listed (${runtimes:-runtime line unavailable})"
    else
      warn "Docker NVIDIA runtime not found in docker info"
    fi
    if grep -qi '^ Default Runtime: nvidia' /tmp/fw_docker_info.$$; then
      pass "Docker default runtime is nvidia"
    else
      warn "Docker default runtime is not nvidia (${default_runtime:-unknown})"
    fi
  else
    fail "Docker command exists but daemon is not reachable: $(tr '\n' ' ' </tmp/fw_docker_err.$$)"
  fi
  rm -f /tmp/fw_docker_info.$$ /tmp/fw_docker_err.$$
else
  fail "docker command not found"
fi

if [[ "${ROS_DISTRO:-}" == "jazzy" ]]; then
  pass "ROS_DISTRO=$ROS_DISTRO"
elif [[ -n "${ROS_DISTRO:-}" ]]; then
  warn "ROS_DISTRO=$ROS_DISTRO before sourcing; expected jazzy"
else
  info "ROS_DISTRO is unset before sourcing"
fi

if [[ -f "$ROS_SETUP" ]]; then
  pass "ROS setup exists: $ROS_SETUP"
else
  fail "ROS setup missing: $ROS_SETUP"
fi

if [[ -f "$FW_SETUP" ]]; then
  pass "Force Walker workspace setup exists: $FW_SETUP"
else
  fail "Force Walker workspace setup missing: $FW_SETUP"
fi

if [[ -f "$ROS_SETUP" ]]; then
  source_setup "$ROS_SETUP"
fi
if [[ -f "$FW_SETUP" ]]; then
  source_setup "$FW_SETUP"
fi

if [[ "${ROS_DISTRO:-}" == "jazzy" ]]; then
  pass "ROS distro after sourcing is jazzy"
else
  fail "ROS distro after sourcing is '${ROS_DISTRO:-unset}', expected jazzy"
fi

if have_cmd rs-enumerate-devices; then
  rs_output="$(rs-enumerate-devices -s 2>&1 || true)"
  if grep -q 'HID Motion Sensor Failure' <<<"$rs_output"; then
    warn "RealSense HID/IMU warning observed from rs-enumerate-devices; IMU is disabled in the Force Walker baseline"
  fi
  printf '%s\n' "$rs_output" \
    | grep -v -E 'HID Motion Sensor Failure|d400-motion.cpp' \
    | sed '/^[[:space:]]*$/d; s/^/INFO rs-enumerate-devices: /'
  for serial in 052622072229 034422071087; do
    if grep -q "$serial" <<<"$rs_output"; then
      pass "RealSense serial found: $serial"
    else
      fail "RealSense serial missing: $serial"
    fi
  done
else
  fail "rs-enumerate-devices command not found"
fi

if have_cmd lsusb; then
  lsusb_output="$(lsusb 2>/dev/null || true)"
  if grep -qi '2b03:f580' <<<"$lsusb_output"; then
    pass "ZED USB camera present: 2b03:f580"
  else
    fail "ZED USB camera not found by lsusb (expected 2b03:f580)"
  fi
  info "USB camera devices:"
  printf '%s\n' "$lsusb_output" | grep -Ei '2b03|stereolabs|8086|realsense|intel' | sed 's/^/INFO   /' || true
  info "USB tree:"
  lsusb -t 2>/dev/null | sed 's/^/INFO   /' || warn "lsusb -t failed"
else
  fail "lsusb command not found"
fi

if have_cmd ros2; then
  check_pkg zed_wrapper
  check_pkg zed_components
  check_pkg realsense2_camera
else
  fail "ros2 command not found after sourcing"
fi
