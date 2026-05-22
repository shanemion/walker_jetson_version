import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

try:
    import serial
except ImportError:  # pragma: no cover - exercised on hosts missing pyserial
    serial = None

try:
    import yaml
except ImportError:  # pragma: no cover - exercised on hosts missing PyYAML
    yaml = None


DEFAULT_CHANNELS = [
    {"name": "left_handle_force", "mux_port": 0, "raw_offset": 0.0, "scale_N_per_count": 1.0, "sign": 1.0, "valid": True},
    {"name": "right_handle_force", "mux_port": 1, "raw_offset": 0.0, "scale_N_per_count": 1.0, "sign": 1.0, "valid": True},
    {"name": "left_lower_frame_force", "mux_port": 2, "raw_offset": 0.0, "scale_N_per_count": 1.0, "sign": 1.0, "valid": True},
    {"name": "right_lower_frame_force", "mux_port": 3, "raw_offset": 0.0, "scale_N_per_count": 1.0, "sign": 1.0, "valid": True},
]


@dataclass
class ChannelCalibration:
    name: str
    mux_port: int
    raw_offset: float
    scale_n_per_count: float
    sign: float
    valid: bool


class TeensyForceBridge(Node):
    def __init__(self) -> None:
        super().__init__("teensy_force_bridge")
        self.declare_parameter("port", "/dev/serial/by-id/forcewalker_teensy")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("calib_yaml", "/opt/forcewalker/forcewalker/config/force_calibration.yaml")
        self.declare_parameter("frame_id", "forcewalker_force")
        self.declare_parameter("imu_frame_id", "forcewalker_imu")

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.calib_yaml = str(self.get_parameter("calib_yaml").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)

        self.channels = self._load_calibration(self.calib_yaml)
        self.force_pub = self.create_publisher(JointState, "/forcewalker/force_channels", 10)
        self.imu_pub = self.create_publisher(Imu, "/forcewalker/imu", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/forcewalker/sensor_diag", 10)

        self._samples: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=200)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._serial_connected = False
        self._last_error = ""
        self._last_t_us: Optional[int] = None
        self._last_seq: Optional[int] = None
        self._last_sample_wall = 0.0
        self._packet_count = 0
        self._malformed_count = 0
        self._dropped_count = 0
        self._queue_drop_count = 0
        self._reconnect_count = 0
        self._last_force_valid = [False] * len(self.channels)
        self._last_imu_valid = False

        if serial is None:
            self._last_error = "python3-serial is not installed"
            self.get_logger().error(self._last_error)
        elif yaml is None:
            self._last_error = "python3-yaml is not installed"
            self.get_logger().error(self._last_error)
        else:
            self._thread = threading.Thread(target=self._serial_worker, daemon=True)
            self._thread.start()

        self.create_timer(0.02, self._publish_available_samples)
        self.create_timer(1.0, self._publish_diagnostics)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        return super().destroy_node()

    def _load_calibration(self, path: str) -> List[ChannelCalibration]:
        raw_channels = DEFAULT_CHANNELS
        if yaml is not None:
            try:
                with open(path, "r", encoding="utf-8") as stream:
                    data = yaml.safe_load(stream) or {}
                raw_channels = data.get("channels", raw_channels)
            except FileNotFoundError:
                self.get_logger().warning(f"Calibration YAML not found, using defaults: {path}")
            except Exception as exc:
                self.get_logger().error(f"Failed to read calibration YAML {path}: {exc}; using defaults")

        channels: List[ChannelCalibration] = []
        for index, item in enumerate(raw_channels):
            channels.append(
                ChannelCalibration(
                    name=str(item.get("name", DEFAULT_CHANNELS[index]["name"])),
                    mux_port=int(item.get("mux_port", index)),
                    raw_offset=float(item.get("raw_offset", 0.0)),
                    scale_n_per_count=float(item.get("scale_N_per_count", 1.0)),
                    sign=float(item.get("sign", 1.0)),
                    valid=bool(item.get("valid", True)),
                )
            )
        return channels

    def _serial_worker(self) -> None:
        assert serial is not None
        while not self._stop_event.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
                    with self._lock:
                        self._serial_connected = True
                        self._last_error = ""
                        self._reconnect_count += 1
                    self.get_logger().info(f"Connected to Teensy force/IMU bridge on {self.port} at {self.baud}")
                    while not self._stop_event.is_set():
                        raw_line = ser.readline()
                        if not raw_line:
                            continue
                        line = raw_line.decode("utf-8", errors="replace").strip()
                        if not line:
                            continue
                        if not line.startswith("{"):
                            self.get_logger().info(f"Teensy startup: {line}")
                            continue
                        self._handle_json_line(line)
            except Exception as exc:
                with self._lock:
                    self._serial_connected = False
                    self._last_error = str(exc)
                time.sleep(1.0)

    def _handle_json_line(self, line: str) -> None:
        try:
            sample = json.loads(line)
        except json.JSONDecodeError:
            with self._lock:
                self._malformed_count += 1
            return

        try:
            self._samples.put_nowait(sample)
        except queue.Full:
            with self._lock:
                self._queue_drop_count += 1

    def _publish_available_samples(self) -> None:
        while True:
            try:
                sample = self._samples.get_nowait()
            except queue.Empty:
                return
            self._publish_sample(sample)

    def _publish_sample(self, sample: Dict[str, Any]) -> None:
        stamp = self.get_clock().now().to_msg()
        t_us = self._as_int(sample.get("t_us"))
        seq = self._as_int(sample.get("seq"))
        force_raw = self._list_from(sample.get("force_raw"), len(self.channels))
        force_valid = self._bool_list_from(sample.get("force_valid"), len(self.channels))
        imu_data = sample.get("imu", {}) if isinstance(sample.get("imu", {}), dict) else {}
        imu_valid = bool(imu_data.get("valid", sample.get("imu_valid", False)))

        joint = JointState()
        joint.header.stamp = stamp
        joint.header.frame_id = self.frame_id
        joint.name = [channel.name for channel in self.channels]
        joint.position = []
        joint.effort = []
        for index, channel in enumerate(self.channels):
            raw = self._as_float(force_raw[index])
            channel_valid = channel.valid and force_valid[index] and not math.isnan(raw)
            joint.position.append(raw)
            if channel_valid:
                joint.effort.append((raw - channel.raw_offset) * channel.scale_n_per_count * channel.sign)
            else:
                joint.effort.append(math.nan)
        self.force_pub.publish(joint)

        if imu_valid:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.imu_frame_id
            q = self._list_from(imu_data.get("q", sample.get("q")), 4)
            accel = self._list_from(imu_data.get("accel_mps2", sample.get("accel_mps2")), 3)
            gyro = self._list_from(imu_data.get("gyro_radps", sample.get("gyro_radps")), 3)
            # Firmware emits quaternion as [w, x, y, z].
            imu.orientation.w = self._as_float(q[0])
            imu.orientation.x = self._as_float(q[1])
            imu.orientation.y = self._as_float(q[2])
            imu.orientation.z = self._as_float(q[3])
            imu.linear_acceleration.x = self._as_float(accel[0])
            imu.linear_acceleration.y = self._as_float(accel[1])
            imu.linear_acceleration.z = self._as_float(accel[2])
            imu.angular_velocity.x = self._as_float(gyro[0])
            imu.angular_velocity.y = self._as_float(gyro[1])
            imu.angular_velocity.z = self._as_float(gyro[2])
            self.imu_pub.publish(imu)

        with self._lock:
            if self._last_seq is not None and seq is not None and seq > self._last_seq + 1:
                self._dropped_count += seq - self._last_seq - 1
            self._last_seq = seq if seq is not None else self._last_seq
            self._last_t_us = t_us if t_us is not None else self._last_t_us
            self._last_sample_wall = time.monotonic()
            self._last_force_valid = force_valid
            self._last_imu_valid = imu_valid
            self._packet_count += 1

    def _publish_diagnostics(self) -> None:
        with self._lock:
            age = time.monotonic() - self._last_sample_wall if self._last_sample_wall else math.inf
            serial_connected = self._serial_connected
            last_error = self._last_error
            packet_count = self._packet_count
            malformed_count = self._malformed_count
            dropped_count = self._dropped_count
            queue_drop_count = self._queue_drop_count
            reconnect_count = self._reconnect_count
            last_t_us = self._last_t_us
            last_seq = self._last_seq
            force_valid = list(self._last_force_valid)
            imu_valid = self._last_imu_valid

        status = DiagnosticStatus()
        status.name = "forcewalker_teensy_bridge"
        status.hardware_id = self.port
        if not serial_connected:
            status.level = DiagnosticStatus.ERROR
            status.message = "serial disconnected"
        elif age > 2.0:
            status.level = DiagnosticStatus.WARN
            status.message = "serial connected, no recent samples"
        elif not all(force_valid) or not imu_valid:
            status.level = DiagnosticStatus.WARN
            status.message = "one or more sensor channels invalid"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"

        status.values = [
            KeyValue(key="port", value=self.port),
            KeyValue(key="baud", value=str(self.baud)),
            KeyValue(key="serial_connected", value=str(serial_connected).lower()),
            KeyValue(key="last_error", value=last_error),
            KeyValue(key="last_sample_age_s", value="inf" if math.isinf(age) else f"{age:.3f}"),
            KeyValue(key="packet_count", value=str(packet_count)),
            KeyValue(key="malformed_json_count", value=str(malformed_count)),
            KeyValue(key="dropped_packet_count", value=str(dropped_count)),
            KeyValue(key="queue_drop_count", value=str(queue_drop_count)),
            KeyValue(key="reconnect_count", value=str(reconnect_count)),
            KeyValue(key="last_teensy_t_us", value=str(last_t_us if last_t_us is not None else "")),
            KeyValue(key="last_seq", value=str(last_seq if last_seq is not None else "")),
            KeyValue(key="force_valid", value=json.dumps(force_valid)),
            KeyValue(key="imu_valid", value=str(imu_valid).lower()),
            KeyValue(key="channel_names", value=json.dumps([channel.name for channel in self.channels])),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)

    @staticmethod
    def _as_int(value: Any) -> Optional[int]:
        try:
            if value is None:
                return None
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _as_float(value: Any) -> float:
        try:
            if value is None:
                return math.nan
            return float(value)
        except (TypeError, ValueError):
            return math.nan

    @staticmethod
    def _list_from(value: Any, length: int) -> List[Any]:
        if not isinstance(value, list):
            return [None] * length
        return (value + [None] * length)[:length]

    @staticmethod
    def _bool_list_from(value: Any, length: int) -> List[bool]:
        if not isinstance(value, list):
            return [False] * length
        result = [bool(item) for item in value]
        return (result + [False] * length)[:length]


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TeensyForceBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
