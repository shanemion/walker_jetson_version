#!/usr/bin/python3
"""ForceWalker force channel calibration helper."""

import argparse
import glob
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Any

try:
    import serial
except ImportError:
    serial = None

try:
    import yaml
except ImportError:
    yaml = None


DEFAULT_PORT = "/dev/serial/by-id/forcewalker_teensy"
DEFAULT_CALIB = "/opt/forcewalker/forcewalker/config/force_calibration.yaml"
DEFAULT_CHANNELS = [
    {"name": "left_handle_force", "mux_port": 0, "raw_offset": 0.0, "scale_N_per_count": 1.0, "sign": 1.0, "valid": True},
    {"name": "right_handle_force", "mux_port": 1, "raw_offset": 0.0, "scale_N_per_count": 1.0, "sign": 1.0, "valid": True},
    {"name": "left_lower_frame_force", "mux_port": 2, "raw_offset": 0.0, "scale_N_per_count": 1.0, "sign": 1.0, "valid": True},
    {"name": "right_lower_frame_force", "mux_port": 3, "raw_offset": 0.0, "scale_N_per_count": 1.0, "sign": 1.0, "valid": True},
]


def load_calibration(path: Path) -> dict[str, Any]:
    if yaml is None:
        if path.exists():
            raise RuntimeError("python3-yaml is required")
        return {"channels": [dict(channel) for channel in DEFAULT_CHANNELS]}
    if not path.exists():
        return {"channels": [dict(channel) for channel in DEFAULT_CHANNELS]}
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if "channels" not in data:
        data["channels"] = [dict(channel) for channel in DEFAULT_CHANNELS]
    return data


def save_calibration(path: Path, data: dict[str, Any]) -> None:
    if yaml is None:
        raise RuntimeError("python3-yaml is required")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)


def parse_sample(line: bytes) -> list[float] | None:
    text = line.decode("utf-8", errors="replace").strip()
    start = text.find("{")
    if start < 0:
        return None
    try:
        sample = json.loads(text[start:])
    except json.JSONDecodeError:
        return None
    force_raw = sample.get("force_raw")
    force_valid = sample.get("force_valid")
    if not isinstance(force_raw, list) or not isinstance(force_valid, list):
        return None
    values: list[float] = []
    for raw, valid in zip(force_raw, force_valid):
        values.append(float(raw) if valid else math.nan)
    return values


def port_candidates(port: str) -> list[str]:
    candidates = [port]
    real_path = os.path.realpath(port)
    if real_path != port:
        candidates.append(real_path)
    if port.startswith("/dev/serial/"):
        candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
        candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
    result: list[str] = []
    for candidate in candidates:
        if candidate not in result:
            result.append(candidate)
    return result


def open_serial_with_retry(port: str, baud: int, timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        for candidate in port_candidates(port):
            if Path(candidate).exists():
                try:
                    return open(candidate, "rb", buffering=0)
                except OSError as file_exc:
                    last_error = file_exc
            if serial is None:
                continue
            try:
                return serial.Serial(candidate, baud, timeout=1.0)
            except OSError as exc:
                last_error = exc
        time.sleep(0.25)
    if last_error is not None:
        raise last_error
    return serial.Serial(port, baud, timeout=1.0)


def collect_samples(port: str, baud: int, duration: float, warmup: float, port_timeout: float) -> list[list[float]]:
    rows: list[list[float]] = []
    start = time.monotonic()
    end = start + warmup + duration
    with open_serial_with_retry(port, baud, port_timeout) as stream:
        while time.monotonic() < end:
            values = parse_sample(stream.readline())
            if values is None or time.monotonic() < start + warmup:
                continue
            rows.append(values)
    if not rows:
        raise RuntimeError("no valid force samples collected")
    return rows


def summarize(rows: list[list[float]], channels: list[dict[str, Any]]) -> list[dict[str, Any]]:
    width = min(len(channels), max(len(row) for row in rows))
    result = []
    for index in range(width):
        values = [row[index] for row in rows if index < len(row) and not math.isnan(row[index])]
        if not values:
            result.append({"index": index, "name": channels[index]["name"], "count": 0})
            continue
        result.append(
            {
                "index": index,
                "name": channels[index]["name"],
                "count": len(values),
                "mean": statistics.fmean(values),
                "stdev": statistics.pstdev(values) if len(values) > 1 else 0.0,
                "min": min(values),
                "max": max(values),
            }
        )
    return result


def print_summary(summary: list[dict[str, Any]]) -> None:
    print("channel,count,mean,stdev,min,max")
    for item in summary:
        if item.get("count", 0) == 0:
            print(f"{item['name']},0,,,,")
            continue
        print(
            f"{item['name']},{item['count']},{item['mean']:.2f},"
            f"{item['stdev']:.2f},{item['min']:.2f},{item['max']:.2f}"
        )


def known_force_n(args: argparse.Namespace) -> float:
    provided = [args.force_n is not None, args.mass_kg is not None, args.weight_lb is not None]
    if sum(provided) != 1:
        raise RuntimeError("provide exactly one of --force-n, --mass-kg, or --weight-lb")
    if args.force_n is not None:
        return float(args.force_n)
    if args.mass_kg is not None:
        return float(args.mass_kg) * 9.80665
    return float(args.weight_lb) * 4.4482216152605


def channel_index(channels: list[dict[str, Any]], name: str) -> int:
    for index, channel in enumerate(channels):
        if channel.get("name") == name:
            return index
    raise RuntimeError(f"unknown channel: {name}")


def maybe_write(path: Path, data: dict[str, Any], write: bool) -> None:
    if not write:
        print()
        print("Dry run only. Re-run with --write to update calibration YAML.")
        return
    save_calibration(path, data)
    print()
    print(f"Wrote calibration: {path}")


def command_sample(args: argparse.Namespace) -> None:
    try:
        data = load_calibration(Path(args.calib))
    except RuntimeError:
        data = {"channels": [dict(channel) for channel in DEFAULT_CHANNELS]}
    rows = collect_samples(args.port, args.baud, args.duration, args.warmup, args.port_timeout)
    print_summary(summarize(rows, data["channels"]))


def command_zero(args: argparse.Namespace) -> None:
    path = Path(args.calib)
    data = load_calibration(path)
    summary = summarize(collect_samples(args.port, args.baud, args.duration, args.warmup, args.port_timeout), data["channels"])
    print_summary(summary)
    for item in summary:
        if item.get("count", 0) == 0:
            continue
        data["channels"][item["index"]]["raw_offset"] = round(float(item["mean"]), 2)
    maybe_write(path, data, args.write)


def command_scale(args: argparse.Namespace) -> None:
    path = Path(args.calib)
    data = load_calibration(path)
    channels = data["channels"]
    index = channel_index(channels, args.channel)
    force_n = known_force_n(args)
    summary = summarize(collect_samples(args.port, args.baud, args.duration, args.warmup, args.port_timeout), channels)
    print_summary(summary)

    loaded_mean = float(summary[index]["mean"])
    raw_offset = float(channels[index].get("raw_offset", 0.0))
    delta = loaded_mean - raw_offset
    if abs(delta) < 1.0:
        raise RuntimeError(f"loaded delta too small for {args.channel}: {delta:.2f} counts")

    channels[index]["sign"] = 1.0 if delta > 0 else -1.0
    channels[index]["scale_N_per_count"] = round(force_n / abs(delta), 8)
    channels[index]["valid"] = True

    print()
    print(f"channel: {args.channel}")
    print(f"known_force_N: {force_n:.3f}")
    print(f"loaded_delta_counts: {delta:.2f}")
    print(f"sign: {channels[index]['sign']}")
    print(f"scale_N_per_count: {channels[index]['scale_N_per_count']}")
    maybe_write(path, data, args.write)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", default=115200, type=int)
    parser.add_argument("--calib", default=DEFAULT_CALIB)
    parser.add_argument("--port-timeout", default=30.0, type=float)
    parser.add_argument("--duration", default=5.0, type=float)
    parser.add_argument("--warmup", default=1.0, type=float)
    subparsers = parser.add_subparsers(dest="command", required=True)

    sample = subparsers.add_parser("sample", help="print raw force channel statistics")
    sample.add_argument("--port-timeout", default=30.0, type=float)
    sample.add_argument("--duration", default=5.0, type=float)
    sample.add_argument("--warmup", default=1.0, type=float)
    sample.set_defaults(func=command_sample)

    zero = subparsers.add_parser("zero", help="update raw_offset from unloaded samples")
    zero.add_argument("--port-timeout", default=30.0, type=float)
    zero.add_argument("--duration", default=5.0, type=float)
    zero.add_argument("--warmup", default=1.0, type=float)
    zero.add_argument("--write", action="store_true", help="write calibration YAML")
    zero.set_defaults(func=command_zero)

    scale = subparsers.add_parser("scale", help="update one channel scale from a known load")
    scale.add_argument("--port-timeout", default=30.0, type=float)
    scale.add_argument("--duration", default=5.0, type=float)
    scale.add_argument("--warmup", default=1.0, type=float)
    scale.add_argument("--channel", required=True, help="channel name from calibration YAML")
    scale.add_argument("--force-n", type=float)
    scale.add_argument("--mass-kg", type=float)
    scale.add_argument("--weight-lb", type=float)
    scale.add_argument("--write", action="store_true", help="write calibration YAML")
    scale.set_defaults(func=command_scale)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.func(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
