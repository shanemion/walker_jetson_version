# ForceWalker Team Handoff - 2026-06-03

This handoff summarizes the current ForceWalker Jetson sensor-stack state, what
was completed, what was validated, and how it maps back to the original task
plan.

## Executive Summary

The ROS 2 data-capture stack is now usable for first-pass dataset collection.
The system can launch the ZED, both RealSense terrain cameras, and the Teensy
force/IMU bridge; run a repeatable smoke test; record labeled MCAP bags; and
replay/inspect recorded data.

Current accepted raw ROS topic baseline:

- ZED RGB: approximately 15-18 Hz
- ZED registered depth: approximately 19-20 Hz
- RealSense terrain depth: approximately 10-15 Hz
- Force channels: approximately 50 Hz
- Sensor diagnostics: approximately 1 Hz

Known-weight force calibration is intentionally deferred. Force messages are
still useful for relative push/pull/braking analysis, but calibrated Newton
values should not be treated as final.

## Repository State

GitHub remote:

```text
https://github.com/shanemion/forcewalker-jetson-sensor-stack.git
```

Recent pushed commits:

```text
db831c9 Stop trial recordings gracefully
a050f9c Accept current raw sensor rate baseline
5147e19 Tune camera profile for higher ZED publish rate
6f59e96 Add ForceWalker data capture workflow tooling
8dba338 Add Teensy force IMU subsystem
```

Important files:

```text
README_SENSOR_STACK.md
docs/DATA_CAPTURE_RUNBOOK.md
docs/FORCE_IMU_HANDOFF.md
scripts/fw_check_sensors.sh
scripts/fw_smoke_test.sh
scripts/fw_start_sensors.sh
scripts/fw_record_trial.sh
scripts/fw_replay_latest_bag.sh
scripts/fw_force_calibrate.py
config/cameras.yaml
config/force_calibration.yaml
```

## Completed Work

### Sensor Launch And Topic Stack

The launch helper now supports:

- camera-only capture
- force/IMU-only capture
- full camera plus force/IMU capture
- non-attached tmux sessions for automation
- labeled bag names

Current camera profile:

```text
zed_main:
  VGA grab @ 60 FPS
  RGBD publish target @ 20 FPS
  PERFORMANCE depth
  depth stabilization off
  16-bit depth output
  point cloud off
  positional tracking off

rs_upward:
  depth only, 640x480 @ 15 FPS

rs_downward:
  depth only, 640x480 @ 15 FPS
```

The ZED raw RGB topic did not reach 20 Hz on this host even when probed
RGB-only with depth disabled. We accepted the current raw ROS topic baseline
instead of continuing to chase this in software.

### Smoke Testing

Run:

```bash
/opt/forcewalker/forcewalker/scripts/fw_smoke_test.sh
```

Current pass criteria:

- ZED RGB and depth topics publish at 15 Hz or better
- each RealSense terrain depth topic publishes at 10 Hz or better
- `/forcewalker/force_channels` publishes at 40 Hz or better
- `/forcewalker/sensor_diag` publishes at 0.5 Hz or better

Last validated full smoke test:

```text
ZED RGB:                       17.553 Hz
ZED depth_registered:          19.907 Hz
RealSense upward depth:        12.300 Hz
RealSense downward depth:      10.949 Hz
Force channels:                49.661 Hz
Diagnostics:                   1.000 Hz
```

### Recording Workflow

Run a full trial:

```bash
/opt/forcewalker/forcewalker/scripts/fw_record_trial.sh \
  --label walking_pass_01 \
  --duration 60
```

The recorder now stops `ros2 bag record` gracefully by sending `Ctrl-C` to the
rosbag tmux window and waiting for `metadata.yaml`. Manual `ros2 bag reindex`
should no longer be needed for new recordings.

Verified graceful-stop test:

```text
/data/forcewalker/rosbags/fw_force_imu_20260603_143335_graceful_stop_test_01
metadata.yaml written automatically
ros2 bag info worked immediately
```

### Dataset Captured

The first dataset set was recorded under:

```text
/data/forcewalker/rosbags
```

Recorded and validated bags:

| Label | Bag directory | Duration | Size | Messages |
| --- | --- | ---: | ---: | ---: |
| static_smoke_01 | `fw_sensors_20260603_142104_static_smoke_01` | 28.69s | 1.1 GiB | 6,178 |
| walking_pass_01 | `fw_sensors_20260603_142521_walking_pass_01` | 58.74s | 2.3 GiB | 12,813 |
| turning_pass_01 | `fw_sensors_20260603_142717_turning_pass_01` | 58.66s | 2.3 GiB | 12,780 |
| handle_push_pull_01 | `fw_sensors_20260603_142840_handle_push_pull_01` | 28.68s | 1.1 GiB | 6,168 |
| braking_load_01 | `fw_sensors_20260603_142931_braking_load_01` | 28.43s | 1.1 GiB | 6,128 |
| terrain_pass_01 | `fw_sensors_20260603_143042_terrain_pass_01` | 56.40s | 2.2 GiB | 12,274 |

All dataset bags contain:

- `/zed_main/zed_node/rgb/color/rect/image`
- `/zed_main/zed_node/depth/depth_registered`
- `/zed_main/zed_node/rgb/color/rect/camera_info`
- `/rs_upward/rs_upward/depth/image_rect_raw`
- `/rs_upward/rs_upward/depth/camera_info`
- `/rs_downward/rs_downward/depth/image_rect_raw`
- `/rs_downward/rs_downward/depth/camera_info`
- `/forcewalker/force_channels`
- `/forcewalker/imu`
- `/forcewalker/sensor_diag`

## Mapping To The Original Plan

### RGBD Cameras

Status: usable for first-pass capture.

Completed:

- ZED main RGBD stream publishes into ROS.
- RealSense upward/downward terrain depth streams publish into ROS.
- Full-stack smoke test validates topic visibility and rates.
- First dataset bags have been recorded and validated.

Accepted compromise:

- The original target was at least 20 Hz image publishing. ZED raw RGB is
  accepted at 15-18 Hz for now because software-only tuning did not make the raw
  ROS image path reach 20 Hz on this host.

### Force Sensors

Status: usable for relative force/IMU data.

Completed:

- Teensy force/IMU bridge publishes into ROS.
- Four force channels and IMU are recorded in every full bag.
- Diagnostics publish serial health, packet counts, channel validity, and timing
  state.

Deferred:

- Known-weight calibration to final Newtons.
- Current `effort` values should be treated as relative calibrated-from-offset
  readings, not final physical force.

### Monitoring

Status: starter workflow exists.

Completed:

- Foxglove bridge helper exists.
- Starter force/IMU Foxglove layout exists.
- Smoke test provides command-line rate and topic health checks.

Next:

- Build/import a full capture dashboard showing ZED, RealSense depth, force,
  IMU, diagnostics, and topic rates.

### Data Storage

Status: completed for v1.

Completed:

- MCAP rosbag workflow is in place.
- Labeled trial recording is implemented.
- Graceful recorder stop is implemented.
- First dataset set is captured and validated.

### Calibration

Status: deferred.

Completed:

- Calibration helper exists for sample/zero/scale workflow.
- Force calibration YAML exists.

Deferred:

- Known-weight calibration of all four force channels.
- Camera extrinsic calibration between ZED, RealSense cameras, walker frame, and
  world/terrain frame.

### Algorithms

Status: ready to begin offline prototypes from bags.

Next offline order:

1. Extract synchronized RGB, depth, force, IMU, and diagnostics from selected
   bags.
2. Run 2D human pose/keypoint detection on ZED RGB frames.
3. Generate first terrain/depth representation from RealSense and ZED depth.
4. Correlate push/pull/braking events with force channels and body/walker
   observations.
5. Evaluate Sapiens/SAM-3D-body only after 2D data quality is confirmed.

### Architecture Document And 3D Model

Status: not completed in this software pass.

Next:

- Write a system architecture document using this stack as the source of truth:
  sensor names, topics, rates, storage workflow, known limitations.
- Create or update the CAD/3D assembly model using the current physical sensor
  placements.

## Known Issues And Limitations

- ZED raw RGB did not reach 20 Hz through ROS on this host.
- ZED `PERFORMANCE` depth mode logs a deprecation warning under the installed
  SDK, but it is accepted and works.
- RealSense IMU and RGB are disabled intentionally.
- Force known-weight calibration is deferred.
- Existing captured bags before commit `db831c9` needed manual `ros2 bag
  reindex`; the listed dataset bags have already been reindexed and validated.
- MCAP files are large. The first six full bags occupy roughly 10 GiB total.

## Quick Commands For Teammate

Check hardware and packages:

```bash
/opt/forcewalker/forcewalker/scripts/fw_check_sensors.sh
```

Run full smoke test:

```bash
/opt/forcewalker/forcewalker/scripts/fw_smoke_test.sh
```

Record a full trial:

```bash
/opt/forcewalker/forcewalker/scripts/fw_record_trial.sh \
  --label new_trial_01 \
  --duration 60
```

Replay latest bag:

```bash
/opt/forcewalker/forcewalker/scripts/fw_replay_latest_bag.sh
```

Start Foxglove bridge:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_viewer.sh
```

Inspect a bag:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/forcewalker/ros2_ws/install/setup.bash
ros2 bag info /data/forcewalker/rosbags/<bag_dir>
```

## Recommended Next Work

1. Review the recorded bags visually in Foxglove or RViz.
2. Confirm that the physical actions in each labeled trial are actually present
   in the footage.
3. Build a full Foxglove capture dashboard.
4. Start offline bag extraction and 2D pose/terrain prototypes.
5. Later, complete known-weight force calibration and camera extrinsics.
