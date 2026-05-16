# Force Walker Sensor Stack

This machine is configured as the local ROS 2 sensor stack for Force Walker environment modeling and human gait work.

## System Overview

- Host/user: `soar@soar-jeston`
- JetPack: `7.1-b112`
- ROS 2 distro: Jazzy
- ROS workspace: `/opt/forcewalker/ros2_ws`
- Force Walker config/scripts: `/opt/forcewalker/forcewalker`
- Data and logs root: `/data/forcewalker`
- Rosbag output: `/data/forcewalker/rosbags`
- ROS runtime logs: `~/.ros/log`

The stack publishes one Stereolabs ZED USB camera and two Intel RealSense D435i cameras into ROS.

## Cameras

| ROS name | Device | Serial | Baseline |
| --- | --- | --- | --- |
| `zed_main` | Stereolabs ZED USB camera | `13262` | HD720 @ 30 FPS |
| `rs_upward` | Intel RealSense D435i | `052622072229` | Depth only, 640x480 @ 30 FPS |
| `rs_downward` | Intel RealSense D435i | `034422071087` | Depth only, 640x480 @ 30 FPS |

RealSense RGB and IMU streams are disabled by default to keep the baseline conservative.

## Start And Stop

Start all sensors:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh
```

Start all sensors and record an MCAP rosbag:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --record
```

Stop the sensor session:

```bash
tmux kill-session -t forcewalker_sensors
```

The start script uses `tmux` windows for `zed_main`, `rs_upward`, `rs_downward`, and optionally `rosbag`.

## Check, View, Replay

Check connected hardware and ROS package visibility:

```bash
/opt/forcewalker/forcewalker/scripts/fw_check_sensors.sh
```

List active Force Walker topics:

```bash
/opt/forcewalker/forcewalker/scripts/fw_topics.sh
```

Start Foxglove bridge:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_viewer.sh
```

Start RViz:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_viewer.sh --rviz
```

Replay the latest MCAP bag:

```bash
/opt/forcewalker/forcewalker/scripts/fw_replay_latest_bag.sh
```

Replay without confirmation:

```bash
/opt/forcewalker/forcewalker/scripts/fw_replay_latest_bag.sh --yes
```

## Recreate This Setup

For a new Jetson Thor or a rebuilt machine, start with:

```bash
/opt/forcewalker/forcewalker/docs/SETUP_FROM_SCRATCH.md
```

The helper scripts are:

```bash
/opt/forcewalker/forcewalker/scripts/fw_install_host_deps.sh
/opt/forcewalker/forcewalker/scripts/fw_apply_zed_cuda_patch.sh
/opt/forcewalker/forcewalker/scripts/fw_build_ros_ws.sh
```

## CUDA/ZED Build Note

The ZED ROS wrapper is patched locally for Jetson Thor / JetPack 7.1 / CUDA 13.

On this system, CMake found CUDA 13 but did not reliably create the `CUDA::cudart` imported target because CUDA runtime discovery pointed at the Thor CUDA target layout under:

```text
/usr/local/cuda-13.0/targets/sbsa-linux
```

The unversioned CUDA runtime symlink can point at a missing runtime version, while the installed runtime is available through `libcudart.so.13`. The local wrapper patch falls back to that runtime and creates `CUDA::cudart` if CMake does not.

Patched packages:

- `/opt/forcewalker/ros2_ws/src/zed-ros2-wrapper/zed_components/CMakeLists.txt`
- `/opt/forcewalker/ros2_ws/src/zed-ros2-wrapper/zed_debug/CMakeLists.txt`

Do not remove this patch unless the ZED wrapper build is verified on Jetson Thor with the installed CUDA layout.

## Known ZED Warning

`ZED_Explorer` and `ZED_Depth_Viewer` are the functional pass/fail tools for the ZED camera on this machine.

`ZED_Diagnostic` can report low USB bandwidth or frame drops even when Explorer, Depth Viewer, and the ROS node work.

During all-camera ROS runs, watch for ZED log lines such as:

```text
CORRUPTED FRAME
CAMERA REBOOTING
```

These warnings indicate USB/ZED stability margin under combined load. The ROS stack has been verified to launch and publish topics, but long recording sessions should monitor these warnings.
