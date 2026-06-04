#!/usr/bin/python3
"""Plain HTTP Force Walker dashboard that does not require WebGL."""

from __future__ import annotations

import argparse
import errno
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, JointState


IMAGE_TOPICS = {
    "zed_rgb": {
        "label": "ZED RGB",
        "topic": "/zed_main/zed_node/rgb/color/rect/image",
        "depth": False,
    },
    "zed_depth": {
        "label": "ZED Depth",
        "topic": "/zed_main/zed_node/depth/depth_registered",
        "depth": True,
    },
    "rs_upward": {
        "label": "RealSense Upward Depth",
        "topic": "/rs_upward/rs_upward/depth/image_rect_raw",
        "depth": True,
    },
    "rs_upward_rgb": {
        "label": "RealSense Upward RGB",
        "topic": "/rs_upward/rs_upward/color/image_raw",
        "depth": False,
    },
    "rs_downward": {
        "label": "RealSense Downward Depth",
        "topic": "/rs_downward/rs_downward/depth/image_rect_raw",
        "depth": True,
    },
    "rs_downward_rgb": {
        "label": "RealSense Downward RGB",
        "topic": "/rs_downward/rs_downward/color/image_raw",
        "depth": False,
    },
}
FORCE_TOPIC = "/forcewalker/force_channels"
IMU_TOPIC = "/forcewalker/imu"


class DashboardState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.images: dict[str, dict[str, Any]] = {}
        self.force: dict[str, Any] = {"count": 0, "channels": []}
        self.imu: dict[str, Any] = {"count": 0, "valid": False}
        self.start_time = time.time()

    def update_image(self, key: str, jpeg: bytes) -> None:
        with self.lock:
            item = self.images.setdefault(key, {"count": 0})
            item["jpeg"] = jpeg
            item["count"] = int(item.get("count", 0)) + 1
            item["stamp"] = time.time()

    def update_force(self, msg: JointState) -> None:
        channels = []
        for index, name in enumerate(msg.name):
            channels.append(
                {
                    "name": name,
                    "raw": float(msg.position[index]) if index < len(msg.position) else None,
                    "effort": float(msg.effort[index]) if index < len(msg.effort) else None,
                }
            )
        with self.lock:
            self.force = {
                "count": int(self.force.get("count", 0)) + 1,
                "stamp": time.time(),
                "frame_id": msg.header.frame_id,
                "channels": channels,
            }

    def update_imu(self, msg: Imu) -> None:
        with self.lock:
            self.imu = {
                "count": int(self.imu.get("count", 0)) + 1,
                "stamp": time.time(),
                "frame_id": msg.header.frame_id,
                "valid": True,
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w,
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z,
                },
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z,
                },
            }

    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self.lock:
            images = {}
            for key, cfg in IMAGE_TOPICS.items():
                item = self.images.get(key, {})
                stamp = item.get("stamp")
                images[key] = {
                    "label": cfg["label"],
                    "topic": cfg["topic"],
                    "count": int(item.get("count", 0)),
                    "age_s": None if stamp is None else now - float(stamp),
                    "ok": stamp is not None and now - float(stamp) < 2.0,
                }
            force = dict(self.force)
            imu = dict(self.imu)
        for item in (force, imu):
            stamp = item.get("stamp")
            item["age_s"] = None if stamp is None else now - float(stamp)
            item["ok"] = stamp is not None and now - float(stamp) < 2.0
        return {
            "uptime_s": now - self.start_time,
            "images": images,
            "force": force,
            "imu": imu,
        }

    def image(self, key: str) -> bytes | None:
        with self.lock:
            item = self.images.get(key)
            if not item:
                return None
            jpeg = item.get("jpeg")
        return jpeg if isinstance(jpeg, bytes) else None


def resize_for_dashboard(image: np.ndarray, max_width: int = 720) -> np.ndarray:
    height, width = image.shape[:2]
    if width <= max_width:
        return image
    scale = max_width / float(width)
    return cv2.resize(image, (max_width, max(1, int(height * scale))), interpolation=cv2.INTER_AREA)


def colorize_depth(depth: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth)
    depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    if depth.dtype == np.uint16:
        meters = depth.astype(np.float32) / 1000.0
    else:
        meters = depth.astype(np.float32)
    valid = meters[np.isfinite(meters) & (meters > 0.0)]
    upper = 4.0
    if valid.size:
        upper = float(np.percentile(valid, 95))
        upper = max(0.5, min(upper, 8.0))
    normalized = np.clip(meters / upper, 0.0, 1.0)
    gray = (255.0 * (1.0 - normalized)).astype(np.uint8)
    color = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    color[meters <= 0.0] = (20, 20, 20)
    return color


def image_to_jpeg(bridge: CvBridge, msg: Image, label: str, is_depth: bool) -> bytes | None:
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if is_depth:
            bgr = colorize_depth(image)
        elif image.ndim == 2:
            bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            encoding = msg.encoding.lower()
            if encoding == "rgb8":
                bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif encoding == "rgba8":
                bgr = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            elif encoding == "bgra8":
                bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            else:
                bgr = image
        bgr = resize_for_dashboard(bgr)
        cv2.putText(bgr, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 76])
        if not ok:
            return None
        return encoded.tobytes()
    except Exception:
        return None


class DashboardNode(Node):
    def __init__(self, state: DashboardState) -> None:
        super().__init__("forcewalker_simple_dashboard")
        self.state = state
        self.bridge = CvBridge()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(depth=10)
        for key, cfg in IMAGE_TOPICS.items():
            self.create_subscription(
                Image,
                cfg["topic"],
                self._make_image_callback(key, cfg["label"], bool(cfg["depth"])),
                qos,
            )
        self.create_subscription(JointState, FORCE_TOPIC, self.state.update_force, state_qos)
        self.create_subscription(Imu, IMU_TOPIC, self.state.update_imu, state_qos)

    def _make_image_callback(self, key: str, label: str, is_depth: bool):
        def callback(msg: Image) -> None:
            jpeg = image_to_jpeg(self.bridge, msg, label, is_depth)
            if jpeg is not None:
                self.state.update_image(key, jpeg)

        return callback


def html_page(port: int) -> str:
    cards = "\n".join(
        f"""
        <section class="card">
          <div class="card-title">{cfg['label']}</div>
          <img id="img-{key}" src="/image/{key}.jpg" alt="{cfg['label']}">
          <div class="topic">{cfg['topic']}</div>
          <div id="status-{key}" class="status">waiting</div>
        </section>
        """
        for key, cfg in IMAGE_TOPICS.items()
    )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Force Walker Dashboard</title>
  <style>
    body {{ margin: 0; font-family: system-ui, sans-serif; background: #101418; color: #edf2f7; }}
    header {{ padding: 14px 18px; border-bottom: 1px solid #2d3748; display: flex; gap: 18px; align-items: baseline; }}
    h1 {{ font-size: 20px; margin: 0; }}
    main {{ display: grid; grid-template-columns: minmax(0, 2fr) minmax(320px, 1fr); gap: 14px; padding: 14px; }}
    .grid {{ display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 14px; }}
    .card, .side {{ background: #171c22; border: 1px solid #2d3748; border-radius: 6px; padding: 10px; }}
    .card-title {{ font-weight: 700; margin-bottom: 8px; }}
    img {{ width: 100%; background: #050608; min-height: 180px; object-fit: contain; display: block; }}
    .topic, .status {{ color: #a0aec0; font-size: 12px; margin-top: 6px; overflow-wrap: anywhere; }}
    .ok {{ color: #68d391; }}
    .bad {{ color: #fc8181; }}
    .force-row {{ display: grid; grid-template-columns: 1fr 88px; gap: 8px; align-items: center; margin: 9px 0; }}
    .bar-wrap {{ height: 12px; background: #2d3748; border-radius: 3px; overflow: hidden; }}
    .bar {{ height: 100%; background: #63b3ed; width: 0%; }}
    pre {{ white-space: pre-wrap; background: #0b0f13; padding: 10px; border-radius: 4px; overflow: auto; }}
    @media (max-width: 1100px) {{ main, .grid {{ grid-template-columns: 1fr; }} }}
  </style>
</head>
<body>
  <header>
    <h1>Force Walker Dashboard</h1>
    <div id="bridge">localhost:{port}</div>
  </header>
  <main>
    <div class="grid">{cards}</div>
    <aside class="side">
      <h2>Force Channels</h2>
      <div id="force"></div>
      <h2>IMU</h2>
      <pre id="imu">waiting</pre>
    </aside>
  </main>
  <script>
    const imageKeys = {json.dumps(list(IMAGE_TOPICS.keys()))};
    function fmtAge(age) {{
      return age === null || age === undefined ? "never" : age.toFixed(2) + "s";
    }}
    function barWidth(value) {{
      if (value === null || value === undefined || Number.isNaN(value)) return 0;
      return Math.min(100, Math.abs(value) / 40);
    }}
    async function update() {{
      const stamp = Date.now();
      for (const key of imageKeys) {{
        document.getElementById("img-" + key).src = "/image/" + key + ".jpg?t=" + stamp;
      }}
      const response = await fetch("/api/status?t=" + stamp);
      const data = await response.json();
      for (const key of imageKeys) {{
        const item = data.images[key];
        const elem = document.getElementById("status-" + key);
        elem.className = "status " + (item.ok ? "ok" : "bad");
        elem.textContent = "frames=" + item.count + " age=" + fmtAge(item.age_s);
      }}
      const force = data.force.channels || [];
      document.getElementById("force").innerHTML = force.map(ch => `
        <div class="force-row">
          <div>
            <div>${{ch.name}}</div>
            <div class="bar-wrap"><div class="bar" style="width:${{barWidth(ch.effort)}}%"></div></div>
          </div>
          <div>${{ch.effort === null ? "n/a" : ch.effort.toFixed(1)}}</div>
        </div>
      `).join("") || "<div class='bad'>waiting for force topic</div>";
      document.getElementById("imu").textContent = JSON.stringify(data.imu, null, 2);
    }}
    update();
    setInterval(update, 500);
  </script>
</body>
</html>
"""


def make_handler(state: DashboardState, port: int):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _send(self, code: int, content_type: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self._send(200, "text/html; charset=utf-8", html_page(port).encode("utf-8"))
                return
            if parsed.path == "/api/status":
                self._send(200, "application/json", json.dumps(state.snapshot()).encode("utf-8"))
                return
            if parsed.path.startswith("/image/") and parsed.path.endswith(".jpg"):
                key = parsed.path.removeprefix("/image/").removesuffix(".jpg")
                jpeg = state.image(key)
                if jpeg is None:
                    self._send(404, "text/plain; charset=utf-8", b"waiting for image\n")
                    return
                self._send(200, "image/jpeg", jpeg)
                return
            self._send(404, "text/plain; charset=utf-8", b"not found\n")

    return Handler


def spin_ros(node: DashboardNode) -> None:
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8088, type=int)
    parser.add_argument("--port-tries", default=6, type=int, help="Try this many consecutive ports if the first is busy.")
    args = parser.parse_args()

    state = DashboardState()
    rclpy.init()
    node = DashboardNode(state)
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    server = None
    selected_port = args.port
    for port in range(args.port, args.port + max(1, args.port_tries)):
        try:
            server = ThreadingHTTPServer((args.host, port), make_handler(state, port))
            selected_port = port
            break
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise
    if server is None:
        print(f"ERROR: ports {args.port}-{args.port + max(1, args.port_tries) - 1} are already in use.", flush=True)
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    print(f"Force Walker dashboard: http://{args.host}:{selected_port}", flush=True)
    print("Press Ctrl-C to stop.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
