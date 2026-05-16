#!/usr/bin/env bash
set -euo pipefail

SESSION="${FW_TMUX_SESSION:-forcewalker_sensors}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
FW_SETUP="/opt/forcewalker/ros2_ws/install/setup.bash"
ROSBAG_DIR="/data/forcewalker/rosbags"

usage() {
  printf 'Usage: %s [--record]\n' "$(basename "$0")"
}

RECORD=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --record)
      RECORD=1
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
  echo "tmux session '$SESSION' already exists; attaching."
  exec tmux attach -t "$SESSION"
fi

common_prefix="source $ROS_SETUP && source $FW_SETUP"

zed_cmd="$common_prefix && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed camera_name:=zed_main serial_number:=13262 param_overrides:='general.grab_resolution:=HD720;general.grab_frame_rate:=30'"

rs_upward_cmd="$common_prefix && ros2 launch realsense2_camera rs_launch.py camera_name:=rs_upward camera_namespace:=rs_upward serial_no:=\"'_052622072229'\" enable_depth:=true depth_module.depth_profile:=640,480,30 enable_color:=false enable_infra:=false enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false enable_motion:=false pointcloud.enable:=false align_depth.enable:=false"

rs_downward_cmd="$common_prefix && ros2 launch realsense2_camera rs_launch.py camera_name:=rs_downward camera_namespace:=rs_downward serial_no:=\"'_034422071087'\" enable_depth:=true depth_module.depth_profile:=640,480,30 enable_color:=false enable_infra:=false enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false enable_motion:=false pointcloud.enable:=false align_depth.enable:=false"

tmux new-session -d -s "$SESSION" -n zed_main "$zed_cmd"
tmux new-window -t "$SESSION" -n rs_upward "$rs_upward_cmd"
tmux new-window -t "$SESSION" -n rs_downward "$rs_downward_cmd"

if [[ "$RECORD" -eq 1 ]]; then
  stamp="$(date +%Y%m%d_%H%M%S)"
  record_cmd="$common_prefix && ros2 bag record -s mcap -o $ROSBAG_DIR/fw_sensors_$stamp /zed_main/zed_node/rgb/color/rect/image /zed_main/zed_node/rgb/color/rect/camera_info /zed_main/zed_node/depth/depth_registered /zed_main/zed_node/depth/depth_registered/camera_info /rs_upward/rs_upward/depth/image_rect_raw /rs_upward/rs_upward/depth/camera_info /rs_downward/rs_downward/depth/image_rect_raw /rs_downward/rs_downward/depth/camera_info"
  tmux new-window -t "$SESSION" -n rosbag "$record_cmd"
fi

tmux select-window -t "$SESSION:zed_main"
exec tmux attach -t "$SESSION"
