"""Lidar throttle node: subscribes to /scan, republishes throttled to /viz/scan, notifies gRPC."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .throttled_forwarder import AsyncStreamBroadcaster, ThrottledForwarder


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id).lstrip('/')


@dataclass(frozen=True)
class LidarScanData:
    """Immutable lidar scan snapshot for gRPC streaming."""
    timestamp_ms: int
    frame_id: str
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple[float, ...]  # Immutable for safe sharing


class UiBridgeLidarNode(Node):
    """ROS node that subscribes to LaserScan, throttles, and republishes + notifies gRPC.
    
    This is the single source of truth for lidar data in the UI bridge:
    - Subscribes to input topic (e.g., /scan)
    - Republishes throttled scans to output topic (e.g., /viz/scan) for RViz
    - Notifies gRPC subscribers via AsyncStreamBroadcaster
    """

    def __init__(
        self,
        *,
        input_topic: str,
        output_topic: str,
        frame_id: str,
        period_s: float,
        grpc_broadcaster: AsyncStreamBroadcaster[LidarScanData],
    ) -> None:
        super().__init__('ui_bridge_lidar')

        self._input_topic = self.resolve_topic_name(str(input_topic))
        self._output_topic = self.resolve_topic_name(str(output_topic))
        self._frame_id = _normalize_frame(str(frame_id))
        self._grpc_broadcaster = grpc_broadcaster

        # Throttled forwarder: on forward, republish to ROS + notify gRPC
        self._forwarder: ThrottledForwarder[LaserScan] = ThrottledForwarder(
            period_s=period_s,
            on_forward=self._on_forward,
        )

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscribe to input
        self._scan_sub = self.create_subscription(
            LaserScan, self._input_topic, self._on_scan, qos_best_effort
        )

        # Publisher for throttled output
        self._scan_pub = self.create_publisher(LaserScan, self._output_topic, qos_best_effort)

        # Timer for throttle (fires at period to check for pending)
        timer_period_s = period_s
        self._timer = self.create_timer(timer_period_s, self._on_timer)

        self.get_logger().info(
            f"Lidar throttle: {self._input_topic} -> {self._output_topic} @ {1.0/period_s:.1f} Hz cap"
        )

    @property
    def frame_id(self) -> str:
        return self._frame_id

    def _on_scan(self, msg: LaserScan) -> None:
        """Called on every incoming scan. Passes to throttler."""
        self._forwarder.on_input(msg)

    def _on_timer(self) -> None:
        """Called periodically. Lets throttler send pending if any."""
        self._forwarder.on_timer()

    def _on_forward(self, msg: LaserScan) -> None:
        """Called by throttler when it's time to forward a scan."""
        # Republish to ROS output topic
        self._scan_pub.publish(msg)

        # Notify gRPC subscribers
        frame_id = self._frame_id
        if not frame_id:
            frame_id = _normalize_frame(msg.header.frame_id)

        scan_data = LidarScanData(
            timestamp_ms=int(time.time() * 1000),
            frame_id=frame_id,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            ranges=tuple(msg.ranges),
        )
        self._grpc_broadcaster.publish_sync(scan_data)
