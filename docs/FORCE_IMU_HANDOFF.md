# Force Walker Force/IMU Handoff

## Summary

The Force Walker camera stack was left intact and still defaults to the existing ZED plus two RealSense D435i cameras. A new Teensy 4.1 force/IMU subsystem was added as a separate bench-testable path.

The new subsystem reads four SparkFun NAU7802 Qwiic Scale boards through a SparkFun TCA9548A Qwiic mux and one SparkFun BNO086 IMU. The Teensy streams newline-delimited JSON over USB serial, and a ROS 2 Python bridge publishes force, IMU, and diagnostic topics.

Current status:

- Teensy USB serial alias works: `/dev/serial/by-id/forcewalker_teensy`.
- Teensy firmware is installed and streaming JSON at `115200`.
- TCA9548A mux is detected at `0x70`.
- NAU7802 boards are detected on mux ports `0`, `1`, `2`, and `3` at `0x2A`.
- BNO086 is detected on mux port `7` at `0x4B`.
- ROS bridge connects and publishes at about `50 Hz`.
- Zero offsets have been written for unloaded bench state.
- Force scale factors are placeholders and still need known-weight calibration.
- One attempted 20.4 lb capture was not clean and should be discarded.

## Files Added Or Changed

New ROS package:

```text
/opt/forcewalker/ros2_ws/src/forcewalker_sensors
```

Important files:

```text
/opt/forcewalker/ros2_ws/src/forcewalker_sensors/forcewalker_sensors/teensy_force_bridge.py
/opt/forcewalker/ros2_ws/src/forcewalker_sensors/launch/teensy_force_bridge.launch.py
/opt/forcewalker/forcewalker/firmware/teensy_force_imu_bridge/teensy_force_imu_bridge.ino
/opt/forcewalker/forcewalker/config/force_calibration.yaml
/opt/forcewalker/forcewalker/config/foxglove/force_imu_bench_layout.json
```

Updated scripts:

```text
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh
/opt/forcewalker/forcewalker/scripts/fw_check_sensors.sh
/opt/forcewalker/forcewalker/scripts/fw_topics.sh
/opt/forcewalker/forcewalker/scripts/fw_install_host_deps.sh
```

Updated docs:

```text
/opt/forcewalker/forcewalker/README_SENSOR_STACK.md
```

## Hardware Wiring

Use 3.3 V logic only.

Teensy to Qwiic mux MAIN port:

| Teensy 4.1 | TCA9548A Qwiic mux MAIN |
| --- | --- |
| Pin 18 | SDA |
| Pin 19 | SCL |
| 3.3V | 3.3V |
| GND | GND |

Mux port map:

| Mux port | Device | Channel |
| --- | --- | --- |
| 0 | NAU7802 | `left_handle_force` |
| 1 | NAU7802 | `right_handle_force` |
| 2 | NAU7802 | `left_lower_frame_force` |
| 3 | NAU7802 | `right_lower_frame_force` |
| 4-6 | Empty | Reserved |
| 7 | BNO086 | `/forcewalker/imu` |

Typical load-cell wiring:

| Load cell wire | NAU7802 terminal |
| --- | --- |
| Red | `E+` |
| Black | `E-` |
| Green or blue | `A+` |
| White | `A-` |

## Current Capabilities

Firmware:

- Scans the main I2C bus and mux ports at startup.
- Reads four force channels through the mux.
- Reads BNO086 quaternion, acceleration, and gyro fields.
- Streams newline-delimited JSON.
- Continues streaming if one sensor is missing and marks that channel invalid.
- Reports `mux_present`, force channel validity, and IMU validity.

ROS bridge:

- Reads JSON from `/dev/serial/by-id/forcewalker_teensy`.
- Publishes raw counts and calibrated effort values.
- Publishes IMU messages.
- Publishes diagnostics with serial health, packet counts, malformed JSON count, dropped packet count, last Teensy timestamp, and channel validity.

Topics:

```text
/forcewalker/force_channels  sensor_msgs/JointState
/forcewalker/imu             sensor_msgs/Imu
/forcewalker/sensor_diag     diagnostic_msgs/DiagnosticArray
```

In `/forcewalker/force_channels`:

- `name` contains the four force channel names.
- `position` contains raw NAU7802 counts.
- `effort` contains calibrated values.
- At the moment, `effort` is zeroed count delta, not Newtons, because `scale_N_per_count` is still `1.0`.

## Commands

Avoid running ROS commands from an active conda environment. If the prompt starts with `(base)`, run:

```bash
conda deactivate
```

Source ROS and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/forcewalker/ros2_ws/install/setup.bash
```

Check Teensy serial alias:

```bash
ls -l /dev/serial/by-id/forcewalker_teensy
```

Check raw Teensy JSON:

```bash
timeout 8s head -n 40 /dev/serial/by-id/forcewalker_teensy
```

Expected healthy startup includes:

```text
main i2c: 0x70
mux_present true
mux port 0: 0x2A 0x70
mux port 1: 0x2A 0x70
mux port 2: 0x2A 0x70
mux port 3: 0x2A 0x70
mux port 7: 0x4B 0x70
```

Start force/IMU only:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --force-only
```

Start cameras plus force/IMU:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force
```

Start cameras plus force/IMU and record:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force --record
```

Stop the tmux sensor session:

```bash
tmux kill-session -t forcewalker_sensors
```

View live force topic:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/forcewalker/ros2_ws/install/setup.bash
ros2 topic echo /forcewalker/force_channels
```

Measure force topic rate:

```bash
ros2 topic hz /forcewalker/force_channels
```

Record a force/IMU bench bag:

```bash
ros2 bag record -s mcap -o /data/forcewalker/rosbags/fw_force_imu_bench \
  /forcewalker/force_channels \
  /forcewalker/imu \
  /forcewalker/sensor_diag
```

Run the sensor check:

```bash
/opt/forcewalker/forcewalker/scripts/fw_check_sensors.sh
```

## Current Calibration State

Current zero-offset file:

```text
/opt/forcewalker/forcewalker/config/force_calibration.yaml
```

Unloaded offsets were averaged from a live bench capture:

| Channel | Offset |
| --- | ---: |
| `left_handle_force` | `-1701.85` |
| `right_handle_force` | `19358.98` |
| `left_lower_frame_force` | `19756.27` |
| `right_lower_frame_force` | `31083.97` |

Current scale values are placeholders:

```yaml
scale_N_per_count: 1.0
```

This means `effort` currently means:

```text
raw_count - raw_offset
```

It does not yet mean Newtons.

## Next Force Calibration Steps

Known weight calibration can use different weights per channel. The same weight is not required for all four load cells.

Formula:

```text
known_force_N = mass_kg * 9.80665
known_force_N = weight_lb * 4.4482216
scale_N_per_count = known_force_N / loaded_effort_counts
```

The available 20.4 lb object is:

```text
20.4 lb = 90.74 N
```

The available 221 g object is:

```text
0.221 kg = 2.17 N
```

Use whichever object applies a clean, repeatable load to the specific load cell. A smaller clean load is better than a heavier object that is physically awkward or contacts the sensor inconsistently.

Recommended calibration sequence for each force channel:

1. Start force-only ROS bridge.
2. Verify the channel is unloaded.
3. Capture 2-3 seconds unloaded.
4. Apply known weight in the intended positive load direction.
5. Hold still for 5-8 seconds.
6. Remove the weight.
7. Average the stable loaded section.
8. Compute `scale_N_per_count`.
9. If applying load produces negative count delta, keep scale positive and set `sign: -1.0`.
10. Restart the bridge after editing YAML.
11. Confirm unloaded effort is near `0` and known load reports approximately the known Newton value.

Do not use the interrupted 20.4 lb capture from May 21. It contained mixed loading and likely loaded the wrong channel; discard it.

## Mechanical/CAD Next Steps For Force Sensors

The force readings are electrically valid, but mechanical fixturing determines whether they are meaningful.

CAD/mechanical requirements:

- Each load cell needs a controlled load path so force passes through the intended axis.
- The handles need an adapter that fits a smaller calibration mass and later transfers user hand force without side-loading the cell.
- The lower-frame button cells need flat, centered contact pads.
- Avoid off-axis torque, cable strain, rubbing, or hard stops that bypass the load cell.
- Add repeatable calibration fixtures or temporary calibration shelves/hooks for known weights.
- Ensure all load cells can be unloaded mechanically for zeroing.
- Label physical mux port/channel on the harness and in CAD.

For the two handle load cells, the 20.4 lb object is too physically large. Use a smaller known object or design a temporary fixture that applies force cleanly without side-load.

For the two lower-frame load cells, the 20.4 lb object may be appropriate if it sits centered and stable.

## Camera Stack Status

The existing camera stack was not renamed or refactored.

Current camera setup:

| ROS name | Device | Serial |
| --- | --- | --- |
| `zed_main` | Stereolabs ZED USB camera | `13262` |
| `rs_upward` | Intel RealSense D435i | `052622072229` |
| `rs_downward` | Intel RealSense D435i | `034422071087` |

Camera-only launch remains:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh
```

Camera plus force/IMU launch:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force
```

## Camera Calibration Next Steps

The current camera stack publishes data, but geometric calibration/extrinsics still need a separate plan before mounting is final.

Recommended camera calibration work:

1. Preserve existing intrinsic calibration from the vendor drivers unless image quality or depth alignment indicates a problem.
2. Create CAD mount geometry for:
   - `zed_main`
   - `rs_upward`
   - `rs_downward`
   - force/IMU Teensy enclosure and cable routing
3. Define the robot/walker coordinate frames:
   - walker base frame
   - ZED frame
   - RealSense upward frame
   - RealSense downward frame
   - IMU frame
   - force sensor frames or named channels
4. Measure or derive initial extrinsics from CAD.
5. Publish static transforms for camera and IMU frames.
6. Validate transforms in RViz/Foxglove using visible landmarks, ground plane, and known fixture dimensions.
7. Record synchronized rosbag data while moving known calibration targets through overlapping fields of view.
8. Refine extrinsics if CAD-only transforms are not accurate enough.

Likely CAD outputs needed:

- Rigid camera brackets with known datum points.
- Dimensioned coordinate frame origin and axis directions for each camera.
- A cable/strain-relief plan that does not change sensor alignment.
- Optional calibration target mount or reference fixture on the walker frame.

Useful camera commands:

```bash
/opt/forcewalker/forcewalker/scripts/fw_check_sensors.sh
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force
/opt/forcewalker/forcewalker/scripts/fw_topics.sh
/opt/forcewalker/forcewalker/scripts/fw_start_viewer.sh --rviz
```

## Known Issues And Follow-Ups

- Force scale values are not calibrated to Newtons yet.
- Handle load-cell calibration needs a smaller clean known load or a calibration fixture.
- The BNO086 currently publishes valid quaternion, but accel/gyro fields should be rechecked during motion.
- The ROS bridge reports occasional initial malformed/dropped counts when it connects mid-stream; this is expected if it starts reading in the middle of a JSON line.
- Conda can shadow system Python. Deactivate conda before ROS bring-up.
- Firmware source exists in both the repo and the Arduino sketch folder:
  - repo source: `/opt/forcewalker/forcewalker/firmware/teensy_force_imu_bridge`
  - Arduino working copy: `/home/soar/Arduino/teensy_force_imu_bridge`
  Keep these synchronized if editing firmware.

