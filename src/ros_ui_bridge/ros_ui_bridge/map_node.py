"""Map throttle node: subscribes to /map (OccupancyGrid), encodes to PNG, notifies gRPC."""

from __future__ import annotations

import binascii
import struct
import time
import zlib
from dataclasses import dataclass

from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from .throttled_forwarder import AsyncStreamBroadcaster, ThrottledForwarder


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id).lstrip('/')


@dataclass(frozen=True)
class MapPoseData:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class MapData:
    """Immutable map snapshot for gRPC streaming."""

    timestamp_ms: int
    frame_id: str
    resolution_m_per_px: float
    width: int
    height: int
    origin: MapPoseData
    png: bytes


def _normalize_quat(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 0.0 or norm != norm or norm == float('inf'):
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return x * inv, y * inv, z * inv, w * inv


def _png_chunk(chunk_type: bytes, chunk_data: bytes) -> bytes:
    length = struct.pack('>I', len(chunk_data))
    crc = binascii.crc32(chunk_type)
    crc = binascii.crc32(chunk_data, crc)
    crc_bytes = struct.pack('>I', crc & 0xFFFFFFFF)
    return length + chunk_type + chunk_data + crc_bytes


def _occupancy_value_to_gray(v: int) -> int:
    # ROS OccupancyGrid: -1 unknown, 0 free, 100 occupied
    if v < 0:
        return 127
    if v > 100:
        v = 100
    # Map to grayscale: 0=occupied (black), 255=free (white)
    return int(round(255.0 - (float(v) * 255.0 / 100.0)))


def encode_occupancy_grid_to_png(msg: OccupancyGrid) -> bytes:
    """Encode an OccupancyGrid into an 8-bit grayscale PNG.

    PNG pixel convention:
    - 0   = occupied
    - 255 = free
    - 127 = unknown

    Row order matches the ROS message data ordering (row-major, y increasing).
    """

    width = int(msg.info.width)
    height = int(msg.info.height)
    if width <= 0 or height <= 0:
        return b''

    data = msg.data
    expected = width * height
    if len(data) < expected:
        # Corrupt/incomplete message; avoid throwing inside ROS callback.
        return b''

    raw = bytearray()
    # Each scanline begins with filter type 0 (None).
    for y in range(height):
        raw.append(0)
        row_start = y * width
        for x in range(width):
            raw.append(_occupancy_value_to_gray(int(data[row_start + x])))

    compressed = zlib.compress(bytes(raw), level=6)

    # PNG signature
    sig = b'\x89PNG\r\n\x1a\n'

    # IHDR: width, height, bit_depth=8, color_type=0 (grayscale), compression=0, filter=0, interlace=0
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)

    return sig + _png_chunk(b'IHDR', ihdr) + _png_chunk(b'IDAT', compressed) + _png_chunk(b'IEND', b'')


class UiBridgeMapNode(Node):
    """ROS node that subscribes to OccupancyGrid, throttles, encodes, and notifies gRPC."""

    def __init__(
        self,
        *,
        topic: str,
        period_s: float,
        grpc_broadcaster: AsyncStreamBroadcaster[MapData],
    ) -> None:
        super().__init__('ui_bridge_map')

        self._topic = self.resolve_topic_name(str(topic))
        self._grpc_broadcaster = grpc_broadcaster

        self._forwarder: ThrottledForwarder[OccupancyGrid] = ThrottledForwarder(
            period_s=period_s,
            on_forward=self._on_forward,
        )

        qos_map = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._sub = self.create_subscription(OccupancyGrid, self._topic, self._on_map, qos_map)
        self._timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(f"Map stream: {self._topic} @ {1.0/period_s:.2f} Hz cap")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._forwarder.on_input(msg)

    def _on_timer(self) -> None:
        self._forwarder.on_timer()

    def _on_forward(self, msg: OccupancyGrid) -> None:
        frame_id = _normalize_frame(msg.header.frame_id) if msg.header.frame_id else ''
        if not frame_id:
            frame_id = 'map'

        origin = msg.info.origin
        q = origin.orientation
        qx, qy, qz, qw = _normalize_quat(float(q.x), float(q.y), float(q.z), float(q.w))

        png = encode_occupancy_grid_to_png(msg)
        if not png:
            return

        map_data = MapData(
            timestamp_ms=int(time.time() * 1000),
            frame_id=frame_id,
            resolution_m_per_px=float(msg.info.resolution),
            width=int(msg.info.width),
            height=int(msg.info.height),
            origin=MapPoseData(
                x=float(origin.position.x),
                y=float(origin.position.y),
                z=float(origin.position.z),
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
            ),
            png=png,
        )

        self._grpc_broadcaster.publish_sync(map_data)
