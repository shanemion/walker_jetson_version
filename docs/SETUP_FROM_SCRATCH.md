# Force Walker Jetson Thor Setup From Scratch

This document captures the reproducible host setup layer for a Jetson Thor running Force Walker sensors.

It assumes the machine has already been flashed with JetPack 7.1 / Ubuntu 24.04 and that the user is a normal operator account with `sudo`.

## What This Repo Recreates

The repo recreates the team-facing operational layer:

- camera inventory and naming
- sensor launch scripts
- health/view/replay/topic scripts
- RViz config
- the local ZED ROS wrapper CUDA 13 patch
- repeatable ROS workspace build command

It does not flash JetPack, install the Stereolabs ZED SDK, or configure Docker/NVIDIA Container Toolkit from scratch.

## Expected Host Layout

```text
/opt/forcewalker/forcewalker              this repo
/opt/forcewalker/ros2_ws                  ROS 2 workspace
/opt/forcewalker/ros2_ws/src              ROS 2 package source
/opt/forcewalker/ros2_ws/src/zed-ros2-wrapper
/data/forcewalker                         data root
/data/forcewalker/rosbags                 MCAP rosbag output
/usr/local/zed                            ZED SDK
/usr/local/cuda-13.0                      CUDA 13 on Jetson Thor
/usr/local/cuda                           should point to /usr/local/cuda-13.0
```

## 1. Install Host Dependencies

Run:

```bash
cd /opt/forcewalker/forcewalker
./scripts/fw_install_host_deps.sh
```

The script installs common tools, ROS build tools, and Force Walker ROS package dependencies that are available from configured apt repositories. It does not run `apt autoremove`.

If a package is unavailable, the script prints it as missing. Missing ROS packages usually mean the ROS 2 apt source is not configured. Missing RealSense packages usually mean the RealSense apt source is not configured.

## 2. Prepare ROS Workspace Source

Create the workspace if needed:

```bash
mkdir -p /opt/forcewalker/ros2_ws/src
```

Clone the ZED ROS wrapper if it is not already present:

```bash
cd /opt/forcewalker/ros2_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
```

The verified local checkout on `soar-jeston` was:

```text
repo:   https://github.com/stereolabs/zed-ros2-wrapper.git
branch: master
commit: c3cb812
```

## 3. Apply The Jetson Thor CUDA/ZED Patch

Run:

```bash
cd /opt/forcewalker/forcewalker
./scripts/fw_apply_zed_cuda_patch.sh
```

Why this exists:

- Jetson Thor / JetPack 7.1 uses CUDA 13.
- CUDA libraries are under `/usr/local/cuda-13.0/targets/sbsa-linux`.
- CMake can find CUDA but fail to create `CUDA::cudart`.
- The local patch falls back to `libcudart.so.13` and creates `CUDA::cudart` if needed.

The patch is idempotent: if it is already applied, the script exits successfully.

## 4. Build The ROS Workspace

Run:

```bash
cd /opt/forcewalker/forcewalker
./scripts/fw_build_ros_ws.sh
```

The build script:

- avoids conda activation
- sources `/opt/ros/jazzy/setup.bash`
- optionally runs `rosdep install`
- builds `/opt/forcewalker/ros2_ws`
- passes the CUDA 13 root to CMake

## 5. Verify Sensors

Run:

```bash
/opt/forcewalker/forcewalker/scripts/fw_check_sensors.sh
```

Then start sensors:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh
```

Start and record:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --record
```

Stop:

```bash
tmux kill-session -t forcewalker_sensors
```

## Known Manual Pieces

These are intentionally not automated here:

- Flashing JetPack 7.1.
- Installing the ZED SDK into `/usr/local/zed`.
- Docker/NVIDIA Container Toolkit setup.
- Force sensor hardware integration.
- Long-run USB stability testing.

## Current Camera Names

| ROS name | Device | Serial | Baseline |
| --- | --- | --- | --- |
| `zed_main` | Stereolabs ZED USB | `13262` | HD720 @ 30 FPS |
| `rs_upward` | Intel RealSense D435i | `052622072229` | depth only, 640x480 @ 30 FPS |
| `rs_downward` | Intel RealSense D435i | `034422071087` | depth only, 640x480 @ 30 FPS |

Do not change names or serials unless the physical rig inventory changes.
