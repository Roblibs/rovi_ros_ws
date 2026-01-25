#!/usr/bin/env python3

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def _sanitize_topic(topic: str) -> str:
    return topic.strip("/").replace("/", "_") or "image"


def _extract_packed_rows(data: bytes, step: int, row_bytes: int, height: int) -> bytes:
    if step == row_bytes:
        return data[: row_bytes * height]
    out = bytearray(row_bytes * height)
    for row in range(height):
        src_start = row * step
        dst_start = row * row_bytes
        out[dst_start : dst_start + row_bytes] = data[src_start : src_start + row_bytes]
    return bytes(out)


def _swap_u16_endianness(data: bytes) -> bytes:
    out = bytearray(len(data))
    out[0::2] = data[1::2]
    out[1::2] = data[0::2]
    return bytes(out)


def _bgr_to_rgb(data: bytes) -> bytes:
    out = bytearray(len(data))
    out[0::3] = data[2::3]
    out[1::3] = data[1::3]
    out[2::3] = data[0::3]
    return bytes(out)


def _write_pgm8(path: str, width: int, height: int, data: bytes) -> None:
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(data)


def _write_pgm16_be(path: str, width: int, height: int, data: bytes) -> None:
    header = f"P5\n{width} {height}\n65535\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(data)


def _u16_stats_le(data: bytes) -> tuple[int, int, int, int]:
    if len(data) % 2 != 0:
        raise ValueError("u16 buffer has odd length")
    total = len(data) // 2
    zeros = 0
    min_nonzero = None
    max_val = 0
    for i in range(0, len(data), 2):
        v = data[i] | (data[i + 1] << 8)
        if v == 0:
            zeros += 1
            continue
        if min_nonzero is None or v < min_nonzero:
            min_nonzero = v
        if v > max_val:
            max_val = v
    return total, zeros, (min_nonzero or 0), max_val


def _depth_u16_to_u8_viz_le(data: bytes, clip_max_mm: int = 5000) -> bytes:
    out = bytearray(len(data) // 2)
    for idx in range(0, len(data), 2):
        v = data[idx] | (data[idx + 1] << 8)
        if v <= 0:
            out[idx // 2] = 0
            continue
        if v > clip_max_mm:
            v = clip_max_mm
        out[idx // 2] = int(v * 255 / clip_max_mm)
    return bytes(out)


def _write_ppm8_rgb(path: str, width: int, height: int, data: bytes) -> None:
    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(data)


def _write_pfm_f32(path: str, width: int, height: int, data: bytes, is_little_endian: bool) -> None:
    scale = -1.0 if is_little_endian else 1.0
    header = f"Pf\n{width} {height}\n{scale}\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(data)


def _save_image_msg(out_dir: str, topic: str, msg: Image) -> str:
    encoding = (msg.encoding or "").lower()
    width = int(msg.width)
    height = int(msg.height)
    stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
    base = f"{_sanitize_topic(topic)}_{stamp_ns}"

    if encoding in {"mono8", "8uc1"}:
        row_bytes = width
        packed = _extract_packed_rows(bytes(msg.data), int(msg.step), row_bytes, height)
        out_path = os.path.join(out_dir, f"{base}.pgm")
        _write_pgm8(out_path, width, height, packed)
        return out_path

    if encoding in {"mono16", "16uc1"}:
        row_bytes = width * 2
        packed = _extract_packed_rows(bytes(msg.data), int(msg.step), row_bytes, height)
        is_little_endian = int(msg.is_bigendian) == 0
        packed_le = packed if is_little_endian else _swap_u16_endianness(packed)
        total, zeros, min_nz, max_val = _u16_stats_le(packed_le)

        if zeros == total:
            print(f"{topic}: mono16 stats: ALL ZERO ({total} px)", file=sys.stderr)
        else:
            print(
                f"{topic}: mono16 stats: total={total} zeros={zeros} min_nonzero={min_nz} max={max_val}",
                file=sys.stderr,
            )

        packed_be = _swap_u16_endianness(packed_le)
        out_path = os.path.join(out_dir, f"{base}.pgm")
        _write_pgm16_be(out_path, width, height, packed_be)

        viz8 = _depth_u16_to_u8_viz_le(packed_le, clip_max_mm=5000)
        viz_path = os.path.join(out_dir, f"{base}_viz.pgm")
        _write_pgm8(viz_path, width, height, viz8)
        return out_path

    if encoding == "rgb8":
        row_bytes = width * 3
        packed = _extract_packed_rows(bytes(msg.data), int(msg.step), row_bytes, height)
        out_path = os.path.join(out_dir, f"{base}.ppm")
        _write_ppm8_rgb(out_path, width, height, packed)
        return out_path

    if encoding == "bgr8":
        row_bytes = width * 3
        packed = _extract_packed_rows(bytes(msg.data), int(msg.step), row_bytes, height)
        packed = _bgr_to_rgb(packed)
        out_path = os.path.join(out_dir, f"{base}.ppm")
        _write_ppm8_rgb(out_path, width, height, packed)
        return out_path

    if encoding == "32fc1":
        row_bytes = width * 4
        packed = _extract_packed_rows(bytes(msg.data), int(msg.step), row_bytes, height)
        out_path = os.path.join(out_dir, f"{base}.pfm")
        _write_pfm_f32(out_path, width, height, packed, is_little_endian=(int(msg.is_bigendian) == 0))
        return out_path

    raise RuntimeError(f"Unsupported encoding '{msg.encoding}' on topic '{topic}'")


@dataclass
class _TopicState:
    msg: Optional[Image] = None


class SnapshotNode(Node):
    def __init__(self, topics: list[str]) -> None:
        super().__init__("rovi_ros_snapshot")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._states: Dict[str, _TopicState] = {t: _TopicState() for t in topics}
        self._subs = []
        for topic in topics:
            self._subs.append(self.create_subscription(Image, topic, self._make_cb(topic), qos))

    def _make_cb(self, topic: str):
        def cb(msg: Image) -> None:
            if self._states[topic].msg is None:
                self._states[topic].msg = msg
        return cb

    def all_received(self) -> bool:
        return all(state.msg is not None for state in self._states.values())

    def received(self) -> Dict[str, Image]:
        return {topic: state.msg for topic, state in self._states.items() if state.msg is not None}


def main() -> int:
    parser = argparse.ArgumentParser(description="Save one snapshot from ROS image topics.")
    parser.add_argument(
        "--topic",
        action="append",
        default=["/depth_raw/image", "/ir/image_raw"],
        help="Image topic to capture (repeatable). Default: /depth_raw/image and /ir/image_raw",
    )
    parser.add_argument("--out-dir", default="output/cam_snapshot_ros", help="Output directory (gitignored).")
    parser.add_argument("--timeout-s", type=float, default=5.0, help="Timeout in seconds.")
    args = parser.parse_args()

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init(args=None)
    node = SnapshotNode(args.topic)
    start = time.time()
    try:
        while rclpy.ok() and not node.all_received():
            if time.time() - start > args.timeout_s:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    received = node.received()
    if not received:
        print(f"No images received within {args.timeout_s}s. Topics tried: {', '.join(args.topic)}", file=sys.stderr)
        return 2

    for topic, msg in received.items():
        try:
            saved = _save_image_msg(out_dir, topic, msg)
            print(f"{topic} -> {saved}")
        except Exception as e:
            print(f"{topic} -> ERROR: {e}", file=sys.stderr)

    missing = [t for t in args.topic if t not in received]
    if missing:
        print(f"Missing topics (no messages in time): {', '.join(missing)}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
