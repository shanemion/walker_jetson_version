#!/usr/bin/env bash
set -euo pipefail

SESSION="${FW_TMUX_SESSION:-forcewalker_sensors}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"
ROSBAG_DIR="/data/forcewalker/rosbags"

usage() {
  printf 'Usage: %s [--record] [--with-force] [--force-only] [--with-rs-color] [--no-attach] [--bag-label LABEL]\n' "$(basename "$0")"
  printf '\n'
  printf 'Bench force/IMU only:\n'
  printf '  %s --force-only\n' "$(basename "$0")"
  printf '\n'
  printf 'Camera stack plus Teensy force/IMU bridge:\n'
  printf '  %s --with-force\n' "$(basename "$0")"
  printf '\n'
  printf 'Camera stack plus RealSense RGB/color streams:\n'
  printf '  %s --with-force --with-rs-color\n' "$(basename "$0")"
}

RECORD=0
WITH_FORCE=0
FORCE_ONLY=0
WITH_RS_COLOR=0
NO_ATTACH=0
BAG_LABEL=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --record)
      RECORD=1
      shift
      ;;
    --with-force)
      WITH_FORCE=1
      shift
      ;;
    --force-only)
      WITH_FORCE=1
      FORCE_ONLY=1
      shift
      ;;
    --with-rs-color)
      WITH_RS_COLOR=1
      shift
      ;;
    --no-attach)
      NO_ATTACH=1
      shift
      ;;
    --bag-label)
      if [[ $# -lt 2 ]]; then
        echo "--bag-label requires a value" >&2
        exit 2
      fi
      BAG_LABEL="$2"
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

if [[ "$FORCE_ONLY" -eq 1 && "$WITH_FORCE" -ne 1 ]]; then
  echo "--force-only implies --with-force" >&2
  exit 2
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required for this script. Install tmux or start each ROS launch command manually." >&2
  exit 1
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Missing ROS setup: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -f "$FW_SETUP" ]]; then
  echo "Missing Force Walker ROS setup: $FW_SETUP. Run colcon build first." >&2
  exit 1
fi

mkdir -p "$ROSBAG_DIR"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  if [[ "$NO_ATTACH" -eq 1 ]]; then
    echo "tmux session '$SESSION' already exists."
    echo "Attach with: tmux attach -t $SESSION"
    exit 0
  fi
  echo "tmux session '$SESSION' already exists; attaching."
  exec tmux attach -t "$SESSION"
fi

common_prefix="source $ROS_SETUP && source $FW_SETUP"
teensy_port="${FORCEWALKER_TEENSY_PORT:-/dev/serial/by-id/forcewalker_teensy}"
force_calib="${FORCEWALKER_FORCE_CALIB:-/opt/forcewalker/forcewalker/config/force_calibration.yaml}"

safe_bag_label=""
if [[ -n "$BAG_LABEL" ]]; then
  safe_bag_label="$(printf '%s' "$BAG_LABEL" | tr -cs 'A-Za-z0-9_.-' '_' | sed 's/^_*//; s/_*$//')"
  if [[ -n "$safe_bag_label" ]]; then
    safe_bag_label="_$safe_bag_label"
  fi
fi

zed_cmd="$common_prefix && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed camera_name:=zed_main serial_number:=13262 param_overrides:='general.grab_resolution:=VGA;general.grab_frame_rate:=60;general.pub_resolution:=NATIVE;general.pub_frame_rate:=20.0;video.enable_24bit_output:=true;depth.depth_mode:=PERFORMANCE;depth.depth_stabilization:=0;depth.openni_depth_mode:=true;depth.publish_point_cloud:=false;pos_tracking.pos_tracking_enabled:=false;pos_tracking.publish_tf:=false;pos_tracking.publish_map_tf:=false'"

rs_color_args="enable_color:=false"
if [[ "$WITH_RS_COLOR" -eq 1 ]]; then
  rs_color_args="enable_color:=true rgb_camera.color_profile:=640,480,15"
fi

rs_upward_cmd="$common_prefix && ros2 launch realsense2_camera rs_launch.py camera_name:=rs_upward camera_namespace:=rs_upward serial_no:=\"'_052622072229'\" enable_depth:=true depth_module.depth_profile:=640,480,15 $rs_color_args enable_infra:=false enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false enable_motion:=false pointcloud.enable:=false align_depth.enable:=false"

rs_downward_cmd="$common_prefix && ros2 launch realsense2_camera rs_launch.py camera_name:=rs_downward camera_namespace:=rs_downward serial_no:=\"'_034422071087'\" enable_depth:=true depth_module.depth_profile:=640,480,15 $rs_color_args enable_infra:=false enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false enable_motion:=false pointcloud.enable:=false align_depth.enable:=false"

force_cmd="$common_prefix && ros2 launch forcewalker_sensors teensy_force_bridge.launch.py port:=$teensy_port calib_yaml:=$force_calib"

# Bench force/IMU only:
#   /opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --force-only
# Camera stack plus Teensy force/IMU bridge:
#   /opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force
# Camera stack plus Teensy force/IMU bridge and RealSense RGB:
#   /opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force --with-rs-color

if [[ "$FORCE_ONLY" -eq 1 ]]; then
  tmux new-session -d -s "$SESSION" -n force_imu "$force_cmd"
else
  tmux new-session -d -s "$SESSION" -n zed_main "$zed_cmd"
  tmux new-window -t "$SESSION" -n rs_upward "$rs_upward_cmd"
  tmux new-window -t "$SESSION" -n rs_downward "$rs_downward_cmd"
  if [[ "$WITH_FORCE" -eq 1 ]]; then
    tmux new-window -t "$SESSION" -n force_imu "$force_cmd"
  fi
fi

if [[ "$RECORD" -eq 1 ]]; then
  stamp="$(date +%Y%m%d_%H%M%S)"
  if [[ "$FORCE_ONLY" -eq 1 ]]; then
    record_cmd="$common_prefix && ros2 bag record -s mcap -o $ROSBAG_DIR/fw_force_imu_$stamp$safe_bag_label /forcewalker/force_channels /forcewalker/imu /forcewalker/sensor_diag"
  elif [[ "$WITH_FORCE" -eq 1 ]]; then
    record_topics="/zed_main/zed_node/rgb/color/rect/image /zed_main/zed_node/rgb/color/rect/camera_info /zed_main/zed_node/depth/depth_registered /zed_main/zed_node/depth/depth_registered/camera_info /rs_upward/rs_upward/depth/image_rect_raw /rs_upward/rs_upward/depth/camera_info /rs_downward/rs_downward/depth/image_rect_raw /rs_downward/rs_downward/depth/camera_info /forcewalker/force_channels /forcewalker/imu /forcewalker/sensor_diag"
    if [[ "$WITH_RS_COLOR" -eq 1 ]]; then
      record_topics="$record_topics /rs_upward/rs_upward/color/image_raw /rs_upward/rs_upward/color/camera_info /rs_downward/rs_downward/color/image_raw /rs_downward/rs_downward/color/camera_info"
    fi
    record_cmd="$common_prefix && ros2 bag record -s mcap -o $ROSBAG_DIR/fw_sensors_$stamp$safe_bag_label $record_topics"
  else
    record_topics="/zed_main/zed_node/rgb/color/rect/image /zed_main/zed_node/rgb/color/rect/camera_info /zed_main/zed_node/depth/depth_registered /zed_main/zed_node/depth/depth_registered/camera_info /rs_upward/rs_upward/depth/image_rect_raw /rs_upward/rs_upward/depth/camera_info /rs_downward/rs_downward/depth/image_rect_raw /rs_downward/rs_downward/depth/camera_info"
    if [[ "$WITH_RS_COLOR" -eq 1 ]]; then
      record_topics="$record_topics /rs_upward/rs_upward/color/image_raw /rs_upward/rs_upward/color/camera_info /rs_downward/rs_downward/color/image_raw /rs_downward/rs_downward/color/camera_info"
    fi
    record_cmd="$common_prefix && ros2 bag record -s mcap -o $ROSBAG_DIR/fw_sensors_$stamp$safe_bag_label $record_topics"
  fi
  tmux new-window -t "$SESSION" -n rosbag "$record_cmd"
fi

if [[ "$FORCE_ONLY" -eq 1 ]]; then
  tmux select-window -t "$SESSION:force_imu"
else
  tmux select-window -t "$SESSION:zed_main"
fi

if [[ "$NO_ATTACH" -eq 1 ]]; then
  echo "Started tmux session '$SESSION'."
  echo "Attach with: tmux attach -t $SESSION"
  echo "Stop with: tmux kill-session -t $SESSION"
  exit 0
fi

exec tmux attach -t "$SESSION"
