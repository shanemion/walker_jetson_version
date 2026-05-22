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


## Teensy Force/IMU Bench Subsystem

The force/IMU bridge is a separate subsystem from the camera stack. The default camera launch remains unchanged; add the Teensy bridge only when bench testing or recording force/IMU data.

### Bench Wiring

Use 3.3V logic only.

| Teensy 4.1 | Qwiic Mux TCA9548A MAIN |
| --- | --- |
| Pin 18 SDA | SDA |
| Pin 19 SCL | SCL |
| 3.3V | 3.3V |
| GND | GND |

Mux port map:

| Mux port | Device | Channel |
| --- | --- | --- |
| 0 | SparkFun Qwiic Scale NAU7802 | `left_handle_force`, ATO 100 kg load cell |
| 1 | SparkFun Qwiic Scale NAU7802 | `right_handle_force`, ATO 100 kg load cell |
| 2 | SparkFun Qwiic Scale NAU7802 | `left_lower_frame_force`, 200 kg button load cell |
| 3 | SparkFun Qwiic Scale NAU7802 | `right_lower_frame_force`, 200 kg button load cell |
| 4-6 | Empty | Reserved |
| 7 | SparkFun BNO086 Qwiic VR IMU | `/forcewalker/imu` |

Typical load-cell terminal wiring: red to `E+`, black to `E-`, green or blue to `A+`, and white to `A-`.

### Teensy Firmware

Firmware source lives at:

```bash
/opt/forcewalker/forcewalker/firmware/teensy_force_imu_bridge/teensy_force_imu_bridge.ino
```

Build it with Arduino IDE or Teensyduino for Teensy 4.1. Install these Arduino libraries before compiling:

- SparkFun Qwiic Scale NAU7802 Arduino Library
- SparkFun BNO08x Arduino Library

The Teensy prints a short non-JSON startup scan, then streams newline-delimited JSON at `115200` baud. Expected sample shape:

```json
{"seq":12,"t_us":240000,"force_raw":[123,456,789,1011],"force_valid":[true,true,true,true],"imu":{"valid":true,"q":[1.0,0.0,0.0,0.0],"accel_mps2":[0.0,0.0,9.8],"gyro_radps":[0.0,0.0,0.0]},"status":{"mux_addr":112,"force_present":[true,true,true,true]}}
```

Test the Teensy alone:

```bash
python3 -m serial.tools.miniterm /dev/serial/by-id/forcewalker_teensy 115200
```

If the by-id alias has not been added yet, list available serial devices:

```bash
ls -l /dev/serial/by-id/
```

### ROS Bridge

The ROS package is `forcewalker_sensors`. It publishes:

| Topic | Type | Notes |
| --- | --- | --- |
| `/forcewalker/force_channels` | `sensor_msgs/JointState` | `position` is raw counts, `effort` is calibrated Newtons |
| `/forcewalker/imu` | `sensor_msgs/Imu` | Quaternion, acceleration, gyro from BNO086 |
| `/forcewalker/sensor_diag` | `diagnostic_msgs/DiagnosticArray` | Serial health, packet counters, Teensy `t_us`, channel validity |

Calibration defaults live in:

```bash
/opt/forcewalker/forcewalker/config/force_calibration.yaml
```

Build after adding or changing the ROS package:

```bash
/opt/forcewalker/forcewalker/scripts/fw_build_ros_ws.sh
```

Run only the Teensy bridge for bench testing:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/forcewalker/ros2_ws/install/setup.bash
ros2 launch forcewalker_sensors teensy_force_bridge.launch.py \
  port:=/dev/serial/by-id/forcewalker_teensy \
  calib_yaml:=/opt/forcewalker/forcewalker/config/force_calibration.yaml
```

Or use the tmux helper:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --force-only
```

Start cameras plus force/IMU:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force
```

Record a short force/IMU bag:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/forcewalker/ros2_ws/install/setup.bash
ros2 bag record -s mcap -o /data/forcewalker/rosbags/fw_force_imu_test \
  /forcewalker/force_channels \
  /forcewalker/imu \
  /forcewalker/sensor_diag
```

Record cameras plus force/IMU with the helper:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force --record
```

### Bench Dashboard

Use Foxglove Bridge, already included in the host dependency script:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_viewer.sh
```

Import the starter layout from:

```bash
/opt/forcewalker/forcewalker/config/foxglove/force_imu_bench_layout.json
```

The layout plots calibrated force, raw counts, IMU quaternion, and diagnostics. Foxglove can also show topic health and rates from the active ROS connection.
