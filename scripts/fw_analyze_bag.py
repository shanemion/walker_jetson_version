#!/usr/bin/python3
"""Extract Force Walker bag data for offline human-environment modeling."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState


DEFAULT_BAG_ROOT = Path("/data/forcewalker/rosbags")
DEFAULT_ANALYSIS_ROOT = Path("/data/forcewalker/analysis")
DEFAULT_IMAGE_TOPICS = [
    "/zed_main/zed_node/rgb/color/rect/image",
]
DEFAULT_DEPTH_TOPICS = [
    "/zed_main/zed_node/depth/depth_registered",
    "/rs_upward/rs_upward/depth/image_rect_raw",
    "/rs_downward/rs_downward/depth/image_rect_raw",
]
FORCE_TOPIC = "/forcewalker/force_channels"
IMU_TOPIC = "/forcewalker/imu"
DIAG_TOPIC = "/forcewalker/sensor_diag"
COCO_KEYPOINTS = [
    "nose",
    "left_eye",
    "right_eye",
    "left_ear",
    "right_ear",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
]
COCO_SKELETON = [
    (5, 7),
    (7, 9),
    (6, 8),
    (8, 10),
    (5, 6),
    (5, 11),
    (6, 12),
    (11, 12),
    (11, 13),
    (13, 15),
    (12, 14),
    (14, 16),
]


@dataclass
class TopicInfo:
    name: str
    msg_type: str


def latest_bag(root: Path) -> Path:
    candidates = sorted(root.glob("*/metadata.yaml"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise RuntimeError(f"no bags with metadata.yaml found under {root}")
    return candidates[-1].parent


def safe_topic_name(topic: str) -> str:
    return topic.strip("/").replace("/", "__") or "root"


def stamp_to_ns(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


def msg_stamp_ns(msg: Any, fallback_ns: int) -> int:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_ns
    return stamp_to_ns(stamp.sec, stamp.nanosec)


def read_metadata_topics(bag: Path) -> dict[str, TopicInfo]:
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.exists():
        raise RuntimeError(f"missing metadata.yaml: {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    topics: dict[str, TopicInfo] = {}
    for item in metadata.get("rosbag2_bagfile_information", {}).get("topics_with_message_count", []):
        topic_metadata = item.get("topic_metadata", {})
        name = topic_metadata.get("name")
        msg_type = topic_metadata.get("type")
        if name and msg_type:
            topics[name] = TopicInfo(name=name, msg_type=msg_type)
    return topics


def open_reader(bag: Path) -> SequentialReader:
    reader = SequentialReader()
    storage_options = StorageOptions(uri=str(bag), storage_id="mcap")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    return reader


def image_to_cv(bridge: CvBridge, msg: Image) -> np.ndarray:
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    if msg.encoding.lower() == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif msg.encoding.lower() == "rgba8":
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)
    return image


def write_color_image(path: Path, bridge: CvBridge, msg: Image) -> None:
    image = image_to_cv(bridge, msg)
    if image.ndim == 2:
        cv2.imwrite(str(path), image)
    else:
        cv2.imwrite(str(path), image)


def write_depth(path: Path, bridge: CvBridge, msg: Image) -> None:
    depth = image_to_cv(bridge, msg)
    np.save(path, depth)


def flatten_diag(msg: Any, bag_ns: int) -> list[dict[str, Any]]:
    rows = []
    header_ns = msg_stamp_ns(msg, bag_ns)
    for status in msg.status:
        row = {
            "bag_time_ns": bag_ns,
            "stamp_ns": header_ns,
            "name": status.name,
            "hardware_id": status.hardware_id,
            "level": int.from_bytes(status.level, byteorder="little") if isinstance(status.level, bytes) else int(status.level),
            "message": status.message,
            "values": {kv.key: kv.value for kv in status.values},
        }
        rows.append(row)
    return rows


def force_row(msg: JointState, bag_ns: int) -> dict[str, Any]:
    row: dict[str, Any] = {
        "bag_time_ns": bag_ns,
        "stamp_ns": msg_stamp_ns(msg, bag_ns),
        "frame_id": msg.header.frame_id,
    }
    for index, name in enumerate(msg.name):
        if index < len(msg.position):
            row[f"{name}_raw"] = msg.position[index]
        if index < len(msg.effort):
            row[f"{name}_effort"] = msg.effort[index]
    return row


def imu_row(msg: Imu, bag_ns: int) -> dict[str, Any]:
    return {
        "bag_time_ns": bag_ns,
        "stamp_ns": msg_stamp_ns(msg, bag_ns),
        "frame_id": msg.header.frame_id,
        "orientation_x": msg.orientation.x,
        "orientation_y": msg.orientation.y,
        "orientation_z": msg.orientation.z,
        "orientation_w": msg.orientation.w,
        "angular_velocity_x": msg.angular_velocity.x,
        "angular_velocity_y": msg.angular_velocity.y,
        "angular_velocity_z": msg.angular_velocity.z,
        "linear_acceleration_x": msg.linear_acceleration.x,
        "linear_acceleration_y": msg.linear_acceleration.y,
        "linear_acceleration_z": msg.linear_acceleration.z,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    keys: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def summarize_force(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {}
    keys = [key for key in rows[0].keys() if key.endswith("_effort") or key.endswith("_raw")]
    summary = {}
    for key in keys:
        values = [float(row[key]) for row in rows if key in row and row[key] is not None and not math.isnan(float(row[key]))]
        if not values:
            continue
        summary[key] = {
            "count": len(values),
            "mean": statistics.fmean(values),
            "stdev": statistics.pstdev(values) if len(values) > 1 else 0.0,
            "min": min(values),
            "max": max(values),
        }
    return summary


def maybe_run_yolo(output_dir: Path, enabled: bool) -> dict[str, Any]:
    if not enabled:
        return {"enabled": False}
    try:
        from ultralytics import YOLO  # type: ignore
    except Exception as exc:
        return {"enabled": True, "status": "unavailable", "error": str(exc)}

    frames = sorted((output_dir / "frames").glob("*.png"))
    if not frames:
        return {"enabled": True, "status": "no_frames"}

    model = YOLO("yolo11n-pose.pt")
    output_path = output_dir / "yolo_pose.jsonl"
    count = 0
    with output_path.open("w", encoding="utf-8") as stream:
        for frame in frames:
            results = model(str(frame), verbose=False)
            for result in results:
                item = {
                    "frame": str(frame.relative_to(output_dir)),
                    "boxes_xyxy": result.boxes.xyxy.cpu().tolist() if result.boxes is not None else [],
                    "keypoints_xy": result.keypoints.xy.cpu().tolist() if result.keypoints is not None else [],
                    "keypoints_conf": result.keypoints.conf.cpu().tolist() if result.keypoints is not None and result.keypoints.conf is not None else [],
                }
                stream.write(json.dumps(item) + "\n")
                count += 1
    return {"enabled": True, "status": "ok", "frames": len(frames), "results": count, "output": str(output_path)}


def row_float(row: dict[str, Any], key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def make_force_plot(output_dir: Path, force_rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not force_rows:
        return {"status": "no_force_rows"}

    effort_keys = [key for key in force_rows[0] if key.endswith("_effort")]
    if not effort_keys:
        return {"status": "no_effort_keys"}

    start_ns = row_float(force_rows[0], "stamp_ns", row_float(force_rows[0], "bag_time_ns", 0.0))
    times = [
        (row_float(row, "stamp_ns", row_float(row, "bag_time_ns", start_ns)) - start_ns) / 1_000_000_000.0
        for row in force_rows
    ]

    fig, axis = plt.subplots(figsize=(12, 5), constrained_layout=True)
    for key in effort_keys:
        axis.plot(times, [row_float(row, key) for row in force_rows], linewidth=1.0, label=key.replace("_effort", ""))
    axis.set_title("Force channel effort, current calibration")
    axis.set_xlabel("Time from bag start (s)")
    axis.set_ylabel("Effort (N, or relative counts before calibration)")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize="small")
    path = output_dir / "force_plot.png"
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return {"status": "ok", "path": str(path)}


def load_yolo_by_frame(output_dir: Path) -> dict[str, dict[str, Any]]:
    path = output_dir / "yolo_pose.jsonl"
    if not path.exists():
        return {}
    by_frame: dict[str, dict[str, Any]] = {}
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            frame = item.get("frame")
            if isinstance(frame, str):
                by_frame[frame] = item
    return by_frame


def draw_pose(image: np.ndarray, pose: dict[str, Any] | None) -> None:
    if not pose:
        return
    keypoints = pose.get("keypoints_xy") or []
    confidences = pose.get("keypoints_conf") or []
    if not keypoints:
        return

    for person_index, person_points in enumerate(keypoints):
        person_conf = confidences[person_index] if person_index < len(confidences) else []
        points: list[tuple[int, int] | None] = []
        for index, point in enumerate(person_points):
            conf = float(person_conf[index]) if index < len(person_conf) else 1.0
            if len(point) < 2 or conf < 0.25:
                points.append(None)
                continue
            x, y = int(point[0]), int(point[1])
            points.append((x, y))
            cv2.circle(image, (x, y), 4, (0, 220, 255), -1)
        for a, b in COCO_SKELETON:
            if a < len(points) and b < len(points) and points[a] is not None and points[b] is not None:
                cv2.line(image, points[a], points[b], (0, 255, 0), 2)


def nearest_force_row(force_rows: list[dict[str, Any]], stamp_ns: int, start_index: int) -> tuple[dict[str, Any] | None, int]:
    if not force_rows:
        return None, start_index
    index = max(0, min(start_index, len(force_rows) - 1))
    while index + 1 < len(force_rows):
        current_delta = abs(int(row_float(force_rows[index], "stamp_ns", row_float(force_rows[index], "bag_time_ns"))) - stamp_ns)
        next_delta = abs(int(row_float(force_rows[index + 1], "stamp_ns", row_float(force_rows[index + 1], "bag_time_ns"))) - stamp_ns)
        if next_delta > current_delta:
            break
        index += 1
    return force_rows[index], index


def draw_force_panel(image: np.ndarray, force_row: dict[str, Any] | None) -> None:
    if not force_row:
        return
    effort_keys = [key for key in force_row if key.endswith("_effort")]
    if not effort_keys:
        return
    panel_w = 360
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (panel_w, 26 + 42 * len(effort_keys)), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, image, 0.45, 0, image)
    cv2.putText(image, "Force effort", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    for i, key in enumerate(effort_keys):
        value = row_float(force_row, key, 0.0)
        label = key.replace("_effort", "")
        y = 52 + 40 * i
        bar_center = 260
        scale = 0.04
        bar = int(max(-95, min(95, value * scale)))
        cv2.putText(image, f"{label}: {value:8.1f}", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(image, (bar_center - 100, y + 8), (bar_center + 100, y + 8), (120, 120, 120), 1)
        cv2.line(image, (bar_center, y + 2), (bar_center, y + 14), (180, 180, 180), 1)
        color = (0, 180, 255) if value >= 0 else (255, 160, 0)
        x0 = min(bar_center, bar_center + bar)
        x1 = max(bar_center, bar_center + bar)
        cv2.rectangle(image, (x0, y + 4), (x1, y + 13), color, -1)


def make_overlay_video(output_dir: Path, frame_rows: list[dict[str, Any]], force_rows: list[dict[str, Any]]) -> dict[str, Any]:
    rgb_rows = [row for row in frame_rows if str(row.get("path", "")).startswith("frames/")]
    if not rgb_rows:
        return {"status": "no_rgb_frames"}

    yolo_by_frame = load_yolo_by_frame(output_dir)
    first_image = cv2.imread(str(output_dir / rgb_rows[0]["path"]))
    if first_image is None:
        return {"status": "first_frame_unreadable"}
    height, width = first_image.shape[:2]
    path = output_dir / "overlay.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 8.0, (width, height))
    if not writer.isOpened():
        return {"status": "video_writer_failed"}

    force_index = 0
    written = 0
    start_ns = int(row_float(rgb_rows[0], "stamp_ns", row_float(rgb_rows[0], "bag_time_ns", 0)))
    for row in rgb_rows:
        image = cv2.imread(str(output_dir / row["path"]))
        if image is None:
            continue
        stamp_ns = int(row_float(row, "stamp_ns", row_float(row, "bag_time_ns", start_ns)))
        force_row, force_index = nearest_force_row(force_rows, stamp_ns, force_index)
        draw_force_panel(image, force_row)
        draw_pose(image, yolo_by_frame.get(str(row["path"])))
        t_s = (stamp_ns - start_ns) / 1_000_000_000.0
        cv2.putText(image, f"t={t_s:.2f}s", (width - 140, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        writer.write(image)
        written += 1
    writer.release()
    return {"status": "ok", "path": str(path), "frames": written}


def write_html_report(output_dir: Path, summary: dict[str, Any], visualizations: dict[str, Any]) -> dict[str, Any]:
    force_plot = Path(visualizations.get("force_plot", {}).get("path", "")).name if visualizations.get("force_plot", {}).get("path") else ""
    overlay = Path(visualizations.get("overlay_video", {}).get("path", "")).name if visualizations.get("overlay_video", {}).get("path") else ""
    yolo = summary.get("yolo", {})
    rows = summary.get("rows", {})
    topics = summary.get("topics", {})
    topic_items = "\n".join(
        f"<tr><td>{html.escape(topic)}</td><td>{html.escape(info.get('type', ''))}</td><td>{info.get('count_read', 0)}</td></tr>"
        for topic, info in topics.items()
    )
    force_items = "\n".join(
        f"<tr><td>{html.escape(key)}</td><td>{stats['mean']:.2f}</td><td>{stats['stdev']:.2f}</td><td>{stats['min']:.2f}</td><td>{stats['max']:.2f}</td></tr>"
        for key, stats in summary.get("force_summary", {}).items()
    )

    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Force Walker Analysis</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 24px; color: #1f2933; }}
    table {{ border-collapse: collapse; margin: 12px 0 24px; width: 100%; }}
    th, td {{ border: 1px solid #d5d9df; padding: 6px 8px; font-size: 13px; }}
    th {{ background: #eef2f6; text-align: left; }}
    code {{ background: #eef2f6; padding: 2px 4px; }}
    .grid {{ display: grid; grid-template-columns: minmax(0, 1fr); gap: 18px; max-width: 1200px; }}
    img, video {{ max-width: 100%; border: 1px solid #d5d9df; }}
  </style>
</head>
<body>
  <h1>Force Walker Analysis</h1>
  <p><strong>Bag:</strong> <code>{html.escape(summary.get('bag', ''))}</code></p>
  <p><strong>Output:</strong> <code>{html.escape(summary.get('output_dir', ''))}</code></p>
  <p><strong>Rows:</strong> force={rows.get('force_channels', 0)}, imu={rows.get('imu', 0)}, diagnostics={rows.get('diagnostics', 0)}, frames={rows.get('frames', 0)}</p>
  <p><strong>YOLO:</strong> {html.escape(json.dumps(yolo))}</p>

  <div class="grid">
    <section>
      <h2>Overlay Video</h2>
      {f'<video controls src="{html.escape(overlay)}"></video>' if overlay else '<p>No overlay video generated.</p>'}
    </section>
    <section>
      <h2>Force Plot</h2>
      {f'<img src="{html.escape(force_plot)}" alt="Force plot">' if force_plot else '<p>No force plot generated.</p>'}
    </section>
  </div>

  <h2>Force Summary</h2>
  <table>
    <tr><th>Signal</th><th>Mean</th><th>Std</th><th>Min</th><th>Max</th></tr>
    {force_items}
  </table>

  <h2>Topics</h2>
  <table>
    <tr><th>Topic</th><th>Type</th><th>Messages read</th></tr>
    {topic_items}
  </table>
</body>
</html>
"""
    path = output_dir / "index.html"
    path.write_text(html_text, encoding="utf-8")
    return {"status": "ok", "path": str(path)}


def analyze(args: argparse.Namespace) -> Path:
    bag = Path(args.bag) if args.bag else latest_bag(Path(args.bag_root))
    if not bag.exists():
        raise RuntimeError(f"bag path does not exist: {bag}")
    if bag.is_file():
        bag = bag.parent

    output_dir = Path(args.output) if args.output else Path(args.analysis_root) / bag.name
    output_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = output_dir / "frames"
    depth_dir = output_dir / "depth"
    frames_dir.mkdir(exist_ok=True)
    depth_dir.mkdir(exist_ok=True)

    topics = read_metadata_topics(bag)
    message_types = {name: get_message(info.msg_type) for name, info in topics.items()}
    reader = open_reader(bag)
    bridge = CvBridge()

    image_topics = set(args.image_topic)
    depth_topics = set(args.depth_topic)
    topic_counts: dict[str, int] = {}
    force_rows: list[dict[str, Any]] = []
    imu_rows: list[dict[str, Any]] = []
    diag_rows: list[dict[str, Any]] = []
    frame_rows: list[dict[str, Any]] = []
    camera_info: dict[str, Any] = {}
    last_saved_ns: dict[str, int] = {}
    saved_frame_count = 0

    min_interval_ns = int(1_000_000_000 / args.frame_rate) if args.frame_rate > 0 else 0

    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        if topic not in message_types:
            continue
        msg = deserialize_message(serialized, message_types[topic])

        if topic == FORCE_TOPIC and isinstance(msg, JointState):
            force_rows.append(force_row(msg, bag_ns))
        elif topic == IMU_TOPIC and isinstance(msg, Imu):
            imu_rows.append(imu_row(msg, bag_ns))
        elif topic == DIAG_TOPIC:
            diag_rows.extend(flatten_diag(msg, bag_ns))
        elif isinstance(msg, CameraInfo):
            camera_info[topic] = {
                "bag_time_ns": bag_ns,
                "stamp_ns": msg_stamp_ns(msg, bag_ns),
                "width": msg.width,
                "height": msg.height,
                "k": list(msg.k),
                "d": list(msg.d),
                "r": list(msg.r),
                "p": list(msg.p),
                "distortion_model": msg.distortion_model,
            }
        elif isinstance(msg, Image) and (topic in image_topics or topic in depth_topics):
            previous = last_saved_ns.get(topic)
            if previous is not None and min_interval_ns > 0 and bag_ns - previous < min_interval_ns:
                continue
            if args.max_frames is not None and saved_frame_count >= args.max_frames:
                continue

            basename = f"{bag_ns}_{safe_topic_name(topic)}"
            if topic in image_topics:
                rel_path = Path("frames") / f"{basename}.png"
                write_color_image(output_dir / rel_path, bridge, msg)
            else:
                rel_path = Path("depth") / f"{basename}.npy"
                write_depth(output_dir / rel_path, bridge, msg)
            last_saved_ns[topic] = bag_ns
            saved_frame_count += 1
            frame_rows.append(
                {
                    "bag_time_ns": bag_ns,
                    "stamp_ns": msg_stamp_ns(msg, bag_ns),
                    "topic": topic,
                    "path": str(rel_path),
                    "encoding": msg.encoding,
                    "width": msg.width,
                    "height": msg.height,
                }
            )

    write_csv(output_dir / "force_channels.csv", force_rows)
    write_csv(output_dir / "imu.csv", imu_rows)
    write_csv(output_dir / "frames_index.csv", frame_rows)
    with (output_dir / "diagnostics.jsonl").open("w", encoding="utf-8") as stream:
        for row in diag_rows:
            stream.write(json.dumps(row) + "\n")
    with (output_dir / "camera_info.json").open("w", encoding="utf-8") as stream:
        json.dump(camera_info, stream, indent=2)

    yolo_summary = maybe_run_yolo(output_dir, args.yolo)
    summary = {
        "bag": str(bag),
        "output_dir": str(output_dir),
        "topics": {name: {"type": info.msg_type, "count_read": topic_counts.get(name, 0)} for name, info in topics.items()},
        "rows": {
            "force_channels": len(force_rows),
            "imu": len(imu_rows),
            "diagnostics": len(diag_rows),
            "frames": len(frame_rows),
        },
        "force_summary": summarize_force(force_rows),
        "yolo": yolo_summary,
    }

    visualizations = {
        "force_plot": make_force_plot(output_dir, force_rows),
        "overlay_video": make_overlay_video(output_dir, frame_rows, force_rows),
    }
    summary["visualizations"] = visualizations
    summary["visualizations"]["html_report"] = write_html_report(output_dir, summary, visualizations)

    with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)

    print(json.dumps(summary, indent=2))
    return output_dir


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", help="Bag directory or MCAP file. Defaults to latest bag under --bag-root.")
    parser.add_argument("--bag-root", default=str(DEFAULT_BAG_ROOT))
    parser.add_argument("--output", help="Analysis output directory. Defaults to /data/forcewalker/analysis/<bag_name>.")
    parser.add_argument("--analysis-root", default=str(DEFAULT_ANALYSIS_ROOT))
    parser.add_argument("--frame-rate", default=1.0, type=float, help="Maximum sample rate per image/depth topic.")
    parser.add_argument("--max-frames", type=int, help="Maximum total image/depth frames to write.")
    parser.add_argument("--image-topic", action="append", default=list(DEFAULT_IMAGE_TOPICS))
    parser.add_argument("--depth-topic", action="append", default=list(DEFAULT_DEPTH_TOPICS))
    parser.add_argument("--yolo", action="store_true", help="Run YOLO pose on extracted RGB frames if ultralytics is installed.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        analyze(args)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
