"""Floor topology node: subscribes to /floor/topology (MarkerArray), extracts polylines, notifies gRPC.

This stream is snapshot-oriented: each gRPC publish represents the full current set of polylines
after applying MarkerArray actions (ADD/DELETE/DELETEALL).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

from .throttled_forwarder import AsyncStreamBroadcaster, ThrottledForwarder


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id).lstrip('/')


@dataclass(frozen=True)
class Point3Data:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class FloorPolylineData:
    ns: str
    id: int
    frame_id: str
    points: tuple[Point3Data, ...]
    closed: bool


@dataclass(frozen=True)
class FloorTopologyData:
    timestamp_ms: int
    polylines: tuple[FloorPolylineData, ...]


def _is_closed(points: tuple[Point3Data, ...], eps: float = 1e-6) -> bool:
    if len(points) < 2:
        return False
    first = points[0]
    last = points[-1]
    return (
        abs(first.x - last.x) <= eps
        and abs(first.y - last.y) <= eps
        and abs(first.z - last.z) <= eps
    )


def _signature(points: tuple[Point3Data, ...]) -> tuple[tuple[float, float, float], ...]:
    if not points:
        return ()
    n = 5

    def r(p: Point3Data) -> tuple[float, float, float]:
        return (round(float(p.x), 4), round(float(p.y), 4), round(float(p.z), 4))

    head = tuple(r(p) for p in points[:n])
    tail = tuple(r(p) for p in points[-n:])
    return head + tail


def _polylines_signature(polylines: tuple[FloorPolylineData, ...]) -> tuple:
    out = []
    for pl in polylines:
        out.append(
            (
                str(pl.ns),
                int(pl.id),
                str(pl.frame_id),
                bool(pl.closed),
                len(pl.points),
                _signature(pl.points),
            )
        )
    return tuple(out)


class UiBridgeFloorTopologyNode(Node):
    """ROS node that subscribes to floor topology markers and forwards to gRPC subscribers."""

    def __init__(
        self,
        *,
        topic: str,
        downsampling_period_s: float | None,
        grpc_broadcaster: AsyncStreamBroadcaster[FloorTopologyData],
        frame_id_fallback: str = 'base_footprint',
    ) -> None:
        super().__init__('ui_bridge_floor_topology')

        self._topic = self.resolve_topic_name(str(topic))
        self._grpc_broadcaster = grpc_broadcaster
        self._frame_id_fallback = _normalize_frame(str(frame_id_fallback))

        self._forwarder: ThrottledForwarder[MarkerArray] | None
        if downsampling_period_s is not None and float(downsampling_period_s) > 0:
            self._forwarder = ThrottledForwarder(
                period_s=float(downsampling_period_s),
                on_forward=self._on_forward,
            )
        else:
            self._forwarder = None

        self._state_by_key: dict[tuple[str, int], FloorPolylineData] = {}
        # NOTE: Do not dedupe publishes here.
        # UIs often drop "stale" overlays; if the topology is stable, we still want a steady stream
        # so consumers can treat lack of updates as "frozen".

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._sub = self.create_subscription(MarkerArray, self._topic, self._on_msg, qos_best_effort)

        self._timer = None
        if self._forwarder is not None:
            period_s = self._forwarder.period_s
            self._timer = self.create_timer(period_s, self._on_timer)
            self.get_logger().info(f"Floor topology downsampling: {self._topic} @ {1.0/period_s:.1f} Hz cap")
        else:
            self.get_logger().info(f"Floor topology downsampling: {self._topic} disabled (forward all updates)")

    def _on_msg(self, msg: MarkerArray) -> None:
        if self._forwarder is None:
            self._on_forward(msg)
        else:
            self._forwarder.on_input(msg)

    def _on_timer(self) -> None:
        if self._forwarder is not None:
            self._forwarder.on_timer()

    def _on_forward(self, msg: MarkerArray) -> None:
        stamp_ns = 0

        if any(int(m.action) == int(Marker.DELETEALL) for m in msg.markers):
            self._state_by_key.clear()

        for m in msg.markers:
            if int(m.type) != int(Marker.LINE_STRIP):
                continue

            ns = str(m.ns or '')
            marker_id = int(m.id)
            key = (ns, marker_id)

            if int(m.action) == int(Marker.DELETE):
                self._state_by_key.pop(key, None)
                continue
            if int(m.action) != int(Marker.ADD):
                continue

            frame_id = _normalize_frame(m.header.frame_id) or self._frame_id_fallback
            points = tuple(Point3Data(x=float(p.x), y=float(p.y), z=float(p.z)) for p in m.points)
            if not points:
                self._state_by_key.pop(key, None)
                continue

            closed = _is_closed(points)
            self._state_by_key[key] = FloorPolylineData(
                ns=ns,
                id=marker_id,
                frame_id=frame_id,
                points=points,
                closed=closed,
            )

            m_stamp_ns = Time.from_msg(m.header.stamp).nanoseconds
            if m_stamp_ns > stamp_ns:
                stamp_ns = int(m_stamp_ns)

        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        timestamp_ms = int(stamp_ns // 1_000_000)

        polylines = tuple(self._state_by_key[k] for k in sorted(self._state_by_key.keys()))
        self._grpc_broadcaster.publish_sync(FloorTopologyData(timestamp_ms=timestamp_ms, polylines=polylines))

    def _fallback_frame_id(self, msg: MarkerArray) -> str:
        for m in msg.markers:
            frame_id = _normalize_frame(m.header.frame_id)
            if frame_id:
                return frame_id
        return ''

    def _fallback_stamp_ns(self, msg: MarkerArray) -> int:
        for m in msg.markers:
            stamp_ns = Time.from_msg(m.header.stamp).nanoseconds
            if stamp_ns > 0:
                return int(stamp_ns)
        return 0

    def _select_line_strip_marker(self, msg: MarkerArray) -> Marker | None:
        selected: Marker | None = None
        for m in msg.markers:
            if int(m.type) != int(Marker.LINE_STRIP):
                continue
            if int(m.action) != int(Marker.ADD):
                continue
            if not m.points:
                continue
            if str(m.ns) == 'floor_topology':
                return m
            if selected is None:
                selected = m
        return selected
