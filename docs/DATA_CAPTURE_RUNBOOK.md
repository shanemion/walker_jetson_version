# Force Walker Data Capture Runbook

This runbook is the software operating order for Force Walker capture sessions.
It assumes the cameras and force sensors are already mounted and wired.

## 1. Preflight

Run the hardware and package check:

```bash
/opt/forcewalker/forcewalker/scripts/fw_check_sensors.sh
```

Expected baseline:

- both RealSense D435i serials are present
- ZED USB camera is present
- `/dev/serial/by-id/forcewalker_teensy` exists
- ROS packages `zed_wrapper`, `realsense2_camera`, and `forcewalker_sensors` are available

## 2. Smoke Test

Run the full camera plus force/IMU smoke test:

```bash
/opt/forcewalker/forcewalker/scripts/fw_smoke_test.sh
```

For force/IMU only:

```bash
/opt/forcewalker/forcewalker/scripts/fw_smoke_test.sh --force-only
```

Pass criteria:

- ZED RGB and depth topics publish at 20 Hz or better
- each RealSense terrain depth topic publishes at 12 Hz or better
- `/forcewalker/force_channels` publishes at 40 Hz or better
- `/forcewalker/sensor_diag` publishes at 0.5 Hz or better

If you are only checking an already-running stack, use:

```bash
/opt/forcewalker/forcewalker/scripts/fw_smoke_test.sh --no-launch
```

## 3. Force Calibration

Check raw force stability:

```bash
/opt/forcewalker/forcewalker/scripts/fw_force_calibrate.py sample --duration 5
```

Zero the unloaded channels. The first command is a dry run:

```bash
/opt/forcewalker/forcewalker/scripts/fw_force_calibrate.py zero --duration 5
/opt/forcewalker/forcewalker/scripts/fw_force_calibrate.py zero --duration 5 --write
```

Scale one channel with a known load:

```bash
/opt/forcewalker/forcewalker/scripts/fw_force_calibrate.py scale \
  --channel left_handle_force \
  --weight-lb 20.4 \
  --duration 5
```

If the dry-run values look right, write the scale:

```bash
/opt/forcewalker/forcewalker/scripts/fw_force_calibrate.py scale \
  --channel left_handle_force \
  --weight-lb 20.4 \
  --duration 5 \
  --write
```

Repeat scale calibration for all four force channels. After this, the
`/forcewalker/force_channels` `effort` field is in Newtons.

## 4. Record Trials

Record a full camera plus force/IMU trial:

```bash
/opt/forcewalker/forcewalker/scripts/fw_record_trial.sh \
  --label walking_pass_01 \
  --duration 60
```

Record force/IMU only:

```bash
/opt/forcewalker/forcewalker/scripts/fw_record_trial.sh \
  --label handle_push_01 \
  --duration 30 \
  --force-only
```

Each trial writes an MCAP bag under `/data/forcewalker/rosbags` and adds a
small `forcewalker_trial_metadata.txt` file beside the bag.

Minimum first dataset:

- static scene, 30 seconds
- walking pass, 60 seconds
- turn-in-place or turn-around pass, 60 seconds
- handle push/pull pass, 30 seconds
- braking/handle-load pass, 30 seconds
- terrain pass, 60 seconds

## 5. Monitor And Replay

Start Foxglove bridge:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_viewer.sh
```

Replay the latest bag:

```bash
/opt/forcewalker/forcewalker/scripts/fw_replay_latest_bag.sh
```

During capture, monitor:

- camera views and depth coverage
- force plots and channel validity
- topic rates
- diagnostics for drops, invalid channels, and stale samples
- ZED logs for corrupted frames or camera reboot warnings

## 6. Offline Prototype Order

Use clean replayable bags before building live inference nodes:

1. Extract camera frames, depth, force, and diagnostics from selected bags.
2. Run 2D pose/keypoint detection on body-view frames.
3. Generate first terrain/depth representations from RGBD data.
4. Correlate force-channel events with body/walker/terrain observations.
5. Evaluate Sapiens/SAM-3D-body on selected frames after 2D data quality is proven.
