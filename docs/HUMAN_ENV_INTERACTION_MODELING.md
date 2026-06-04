# Human-Environment Interaction Modeling

## Goal

Build a software path that uses the existing temporary camera placement plus force/IMU topics to understand human gait and human-walker-environment interaction. Alignment and final calibration are intentionally deferred; this plan focuses on what can be done now with the mounted temporary sensors.

## Feasibility

This is feasible as an offline-first pipeline.

Do not start with live 3D body reconstruction on the Jetson. Start by collecting synchronized bags, extracting camera/depth/force streams, and running model inference offline. Once the data path and labels are stable, decide which parts should become live ROS nodes.

Recommended model roles:

- YOLO11 pose: fast 2D person keypoints and tracking baseline.
- Sapiens: higher-quality offline human pose, body-part segmentation, depth, and normal prediction.
- SAM 3D Body: offline single-image 3D human mesh recovery for selected frames or short clips, not first-pass real-time inference.

## What We Can Model Before Final Calibration

Even without final camera extrinsics or force scale calibration, we can extract useful signals:

- gait timing: step cadence, stance/swing timing proxies, stride regularity
- upper-body motion: trunk lean, shoulder/hip motion, arm/handle relation
- walker interaction: force-channel events, left/right handle loading, load asymmetry
- terrain interaction: depth discontinuities, obstacles, floor plane roughness, proximity to walker feet/wheels
- event correlation: force spikes around gait phase, turns, braking, terrain changes

Until force scale calibration is complete, force values should be treated as relative count-delta signals, not Newtons.

Until camera extrinsics are complete, spatial reasoning should be per-camera or qualitative, not fused metric 3D.

## Data Pipeline

1. Record ROS bags.
   - cameras: ZED RGB/depth, RealSense depth
   - force/IMU: `/forcewalker/force_channels`, `/forcewalker/imu`, `/forcewalker/sensor_diag`
   - diagnostics: topic rates and health

2. Extract synchronized samples.
   - RGB frames from ZED or selected body-facing camera
   - depth frames from ZED/RealSense
   - nearest force/IMU sample by timestamp
   - trial metadata: scenario label, camera placement, subject notes

3. Run 2D human pose first.
   - Use YOLO11 pose for 17-keypoint COCO-style skeleton.
   - Track keypoints over time.
   - Compute simple gait metrics from ankles/knees/hips/shoulders.

4. Add human-environment signals.
   - Detect foot/leg proximity to depth discontinuities.
   - Track body pose relative to walker handles in image space.
   - Correlate force-channel deltas with 2D pose events.
   - Detect turns, braking, load shifts, asymmetry, and terrain approach events.

5. Add heavier offline models on selected frames.
   - Sapiens pose for better keypoint/body-part quality.
   - Sapiens body-part segmentation to isolate legs, arms, torso, hands.
   - Sapiens depth/normal as auxiliary human-centric geometry.
   - SAM 3D Body mesh recovery for selected high-value frames.

6. Review and score.
   - Save model outputs beside the bag.
   - Generate plots and overlay videos.
   - Compare force events, keypoints, depth context, and trial labels.

## First Software Milestone

Create an offline extractor/analyzer that can process one recorded bag and produce:

- sampled RGB frames
- sampled depth images or depth statistics
- force/IMU CSV
- YOLO pose JSON
- overlay video with 2D skeleton and force plots
- summary CSV with basic gait/interaction features

Suggested output folder:

```text
/data/forcewalker/analysis/<bag_name>/
```

Suggested outputs:

```text
frames/
depth/
force_imu.csv
yolo_pose.jsonl
interaction_features.csv
overlay.mp4
summary.json
```

## Candidate Features

2D gait features:

- ankle horizontal trajectory
- knee angle proxy from hip/knee/ankle
- hip center motion
- shoulder center motion
- left/right step timing
- stance/swing timing proxy
- cadence
- gait asymmetry

Force features:

- per-channel mean, peak, RMS, slope
- left/right handle load asymmetry
- lower-frame load events
- force impulse over short windows
- force phase relative to detected steps

Environment/depth features:

- nearest depth discontinuity in forward region
- ground-plane roughness proxy
- obstacle proximity
- person-to-walker image-space distance
- foot-to-depth-edge distance

Interaction event labels:

- handle push
- handle pull
- braking/load transfer
- turn initiation
- terrain approach
- stumble/irregular step candidate
- force asymmetry event

## Model Notes

YOLO11:

- Use for the first working 2D pose baseline.
- It is fast enough to test on many frames and can be exported later.
- Start offline with Python inference, then consider TensorRT/export only if live performance is needed.

Sapiens:

- Use for offline high-quality human-centric understanding.
- Useful tasks include pose estimation, body-part segmentation, human depth, and normals.
- The lite inference path is the likely first setup target.

SAM 3D Body:

- Use for selected-frame 3D mesh recovery and qualitative body pose/shape analysis.
- Treat it as offline enrichment unless a faster derivative proves practical.
- It can use prompts such as 2D keypoints and masks, so YOLO/Sapiens outputs can help guide it.

## What Not To Do Yet

- Do not build a live ROS 3D mesh node first.
- Do not assume uncalibrated force values are Newtons.
- Do not fuse all cameras into one metric 3D frame until extrinsics are defined.
- Do not spend effort on perfect gait metrics before the data extraction and overlay review loop exists.

## Natural Next Steps

1. Run a full smoke test.

```bash
/opt/forcewalker/forcewalker/scripts/fw_smoke_test.sh
```

2. Record a small first dataset.

```bash
/opt/forcewalker/forcewalker/scripts/fw_record_trial.sh --label static_scene_01 --duration 30
/opt/forcewalker/forcewalker/scripts/fw_record_trial.sh --label walking_pass_01 --duration 60
/opt/forcewalker/forcewalker/scripts/fw_record_trial.sh --label handle_load_01 --duration 30
```

3. Build the offline bag extractor.

Initial implementation exists:

```bash
/opt/forcewalker/forcewalker/scripts/fw_analyze_bag.py \
  --bag /data/forcewalker/rosbags/fw_sensors_20260603_142521_walking_pass_01 \
  --output /data/forcewalker/analysis/fw_sensors_20260603_142521_walking_pass_01_analysis \
  --frame-rate 1
```

For a quick smoke run that writes only a few images/depth arrays:

```bash
/opt/forcewalker/forcewalker/scripts/fw_analyze_bag.py \
  --bag /data/forcewalker/rosbags/fw_sensors_20260603_142521_walking_pass_01 \
  --output /data/forcewalker/analysis/fw_sensors_20260603_142521_walking_pass_01_smoke \
  --frame-rate 1 \
  --max-frames 12
```

Current analyzer outputs:

- `force_channels.csv`
- `imu.csv`
- `diagnostics.jsonl`
- `camera_info.json`
- `frames_index.csv`
- sampled RGB frames under `frames/`
- sampled depth arrays under `depth/`
- `force_plot.png`
- `overlay.mp4`
- `index.html`
- `summary.json`

Open the generated visual report in a browser:

```bash
xdg-open /data/forcewalker/analysis/fw_sensors_20260603_142521_walking_pass_01_smoke/index.html
```

The report is a static HTML interface. It embeds the force plot and overlay video, so it does not need a ROS node or web server. The overlay video currently shows camera frames plus force bars. If YOLO pose is available and the analyzer is run with `--yolo`, the same overlay path will also draw 2D skeletons.

RealSense RGB/color streams are optional because they add USB bandwidth. Start the full sensor stack with ZED RGB/depth, RealSense depth, RealSense RGB, and force/IMU:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_sensors.sh --with-force --with-rs-color
```

Expected extra RealSense color topics:

```text
/rs_upward/rs_upward/color/image_raw
/rs_downward/rs_downward/color/image_raw
```

For live local viewing on Jetson browsers where WebGL is disabled, use the plain HTTP dashboard instead of Foxglove:

```bash
/opt/forcewalker/forcewalker/scripts/fw_start_dashboard.sh
```

Then open:

```text
http://127.0.0.1:8088
```

If port `8088` is already in use:

```bash
FORCEWALKER_DASHBOARD_PORT=8089 /opt/forcewalker/forcewalker/scripts/fw_start_dashboard.sh
```

Then open:

```text
http://127.0.0.1:8089
```

Latest visual smoke-test output:

```text
/data/forcewalker/analysis/fw_sensors_20260603_142521_walking_pass_01_visual_smoke/index.html
/data/forcewalker/analysis/fw_sensors_20260603_142521_walking_pass_01_visual_smoke/overlay.mp4
/data/forcewalker/analysis/fw_sensors_20260603_142521_walking_pass_01_visual_smoke/force_plot.png
```

Next implementation:

- install and validate YOLO pose dependencies on the Jetson or a workstation
- run the analyzer with `--yolo` to write `yolo_pose.jsonl`
- confirm the generated `overlay.mp4` draws 2D skeletons over the camera frames
- compute first-pass gait/interaction features from keypoints plus force streams

4. Add YOLO11 pose inference to extracted frames.

Minimum implementation:

- run pose model on RGB frames
- write keypoints as JSONL
- render skeleton overlay video

5. Add interaction plots.

Minimum implementation:

- plot force channels over time
- overlay step/keypoint events
- produce a `summary.json`

6. Evaluate Sapiens and SAM 3D Body on selected frames after the 2D pipeline is useful.

## References

- Ultralytics YOLO11 documentation: https://docs.ultralytics.com/models/yolo11/
- Ultralytics pose task documentation: https://docs.ultralytics.com/tasks/pose/
- Sapiens repository: https://github.com/facebookresearch/sapiens
- SAM 3D Body repository: https://github.com/facebookresearch/sam-3d-body
- Sapiens project page: https://www.meta.com/emerging-tech/codec-avatars/sapiens/
