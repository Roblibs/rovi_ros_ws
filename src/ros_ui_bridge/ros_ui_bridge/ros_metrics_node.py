from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

from builtin_interfaces.msg import Time as RosTime
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

from .config import StatusFieldConfig, StatusFieldSource, TfDemuxConfig
from .rate_tracker import RateTracker


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id).lstrip('/')


def _sanitize_topic_segment(value: str) -> str:
    """Sanitize a ROS topic segment.

    ROS 2 topic segments should be composed of alphanumerics and underscores.
    Frame ids are not guaranteed to follow that, so we map other characters to '_'.
    """
    s = str(value).strip()
    if not s:
        return 'unknown'
    out_chars: list[str] = []
    for ch in s:
        if ch.isalnum() or ch == '_':
            out_chars.append(ch)
        else:
            out_chars.append('_')
    out = ''.join(out_chars).strip('_')
    if not out:
        return 'unknown'
    while '__' in out:
        out = out.replace('__', '_')
    return out


@dataclass
class _ValueState:
    last_value: Optional[float] = None
    last_stamp: Optional[Time] = None
    next_accept_after: Optional[Time] = None


class UiBridgeRosNode(Node):
    """ROS-facing collector for status fields (rates + values + TF demux)."""

    def __init__(
        self,
        *,
        status_fields: list[StatusFieldConfig],
        tf_demux: TfDemuxConfig,
    ) -> None:
        super().__init__('ui_bridge_metrics')

        self._qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # TF demux for visualization tools that can't filter individual transforms in a TFMessage.
        self._tf_demux_enabled = bool(tf_demux.enabled)
        self._tf_demux_prefix = str(tf_demux.prefix).strip() or '/viz/tf'
        if not self._tf_demux_prefix.startswith('/'):
            self._tf_demux_prefix = '/' + self._tf_demux_prefix
        self._tf_demux_pub_by_pair: dict[tuple[str, str], Any] = {}

        self._fields_by_id: dict[str, StatusFieldConfig] = {f.id: f for f in status_fields}
        self._lock = threading.Lock()

        # ROS topics to subscribe for values and rates.
        self._value_fields_by_topic: dict[str, list[StatusFieldConfig]] = {}
        self._value_state_by_id: dict[str, _ValueState] = {}

        self._rate_fields_by_topic: dict[str, list[StatusFieldConfig]] = {}
        self._rate_tracker_by_id: dict[str, RateTracker] = {}
        self._rate_last_ros_stamp: dict[str, Optional[Time]] = {}

        # TF rate trackers keyed by (parent, child).
        self._tf_rate_field_by_pair: dict[tuple[str, str], StatusFieldConfig] = {}
        self._tf_rate_tracker_by_id: dict[str, RateTracker] = {}
        self._tf_rate_last_ros_stamp: dict[str, Optional[Time]] = {}
        self._tf_sub = None

        self._topic_type_by_topic: dict[str, Optional[str]] = {}
        self._topic_subscriptions: list[Any] = []
        self._pending_topics: set[str] = set()

        self._configure_fields(status_fields)

        # Subscribe to TF if needed for rates or demux.
        if self._tf_rate_field_by_pair or self._tf_demux_enabled:
            self._tf_sub = self.create_subscription(TFMessage, '/tf', self._on_tf, self._qos_best_effort)

        if self._pending_topics:
            self._resolve_timer = self.create_timer(2.0, self._try_resolve_pending_topics)
        else:
            self._resolve_timer = None

    # Public API -----------------------------------------------------

    def collect_ros_values(self, *, now_ros: Time) -> list[tuple[str, float, RosTime]]:
        """Return non-stale ROS-derived status values."""
        out: list[tuple[str, float, RosTime]] = []

        with self._lock:
            # Topic value fields
            for field_id, state in self._value_state_by_id.items():
                stamp = state.last_stamp
                field = self._fields_by_id[field_id]
                if state.last_value is None or stamp is None:
                    continue
                if self._is_stale(now_ros, stamp, field.source.stale_after_s):
                    continue
                out.append((field_id, float(state.last_value), stamp.to_msg()))

            # Topic rate fields
            for field_id, tracker in self._rate_tracker_by_id.items():
                hz = tracker.sample_hz(now_monotonic=time.monotonic())
                stamp = self._rate_last_ros_stamp.get(field_id)
                field = self._fields_by_id[field_id]
                if hz is None or stamp is None:
                    continue
                if self._is_stale(now_ros, stamp, field.source.stale_after_s):
                    continue
                out.append((field_id, float(hz), stamp.to_msg()))

            # TF rate fields
            for field_id, tracker in self._tf_rate_tracker_by_id.items():
                hz = tracker.sample_hz(now_monotonic=time.monotonic())
                stamp = self._tf_rate_last_ros_stamp.get(field_id)
                field = self._fields_by_id[field_id]
                if hz is None or stamp is None:
                    continue
                if self._is_stale(now_ros, stamp, field.source.stale_after_s):
                    continue
                out.append((field_id, float(hz), stamp.to_msg()))

        return out

    # Internal wiring -----------------------------------------------

    def _configure_fields(self, fields: list[StatusFieldConfig]) -> None:
        for field in fields:
            source = field.source
            if source.provider != 'ros':
                continue

            if source.type == 'topic_value':
                topic = self.resolve_topic_name(str(source.topic))
                self._value_fields_by_topic.setdefault(topic, []).append(field)
                self._value_state_by_id[field.id] = _ValueState()
                self._topic_type_by_topic.setdefault(topic, source.msg_type)
            elif source.type == 'topic_rate':
                topic = self.resolve_topic_name(str(source.topic))
                self._rate_fields_by_topic.setdefault(topic, []).append(field)
                self._rate_tracker_by_id[field.id] = RateTracker(target_hz=field.target, emit_zero_when_unseen=False)
                self._rate_last_ros_stamp[field.id] = None
                self._topic_type_by_topic.setdefault(topic, source.msg_type)
            elif source.type == 'tf_rate':
                parent = _normalize_frame(str(source.parent))
                child = _normalize_frame(str(source.child))
                key = (parent, child)
                if key in self._tf_rate_field_by_pair:
                    raise RuntimeError(f"Duplicate tf_rate for {parent}->{child}")
                self._tf_rate_field_by_pair[key] = field
                self._tf_rate_tracker_by_id[field.id] = RateTracker(target_hz=field.target, emit_zero_when_unseen=False)
                self._tf_rate_last_ros_stamp[field.id] = None
            else:
                raise RuntimeError(f"Unsupported ROS source type: {source.type}")

        # Create subscriptions for topics we know the type for; others go pending.
        try:
            from rosidl_runtime_py.utilities import get_message  # type: ignore
        except ModuleNotFoundError:
            get_message = None

        for topic, type_hint in self._topic_type_by_topic.items():
            if type_hint and get_message is not None:
                try:
                    msg_cls = get_message(type_hint)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"Failed to import message type {type_hint} for {topic}: {exc}")
                    self._pending_topics.add(topic)
                    continue

                sub = self.create_subscription(msg_cls, topic, lambda msg, t=topic: self._on_topic_message(t, msg), self._qos_best_effort)
                self._topic_subscriptions.append(sub)
                self.get_logger().info(f"Subscribed for topic={topic} type={type_hint} (configured)")
            else:
                self._pending_topics.add(topic)

    def _on_topic_message(self, topic: str, msg: Any) -> None:
        now_ros = self.get_clock().now()
        msg_stamp = _resolve_msg_stamp(msg, now_ros)

        with self._lock:
            for field in self._rate_fields_by_topic.get(topic, []):
                tracker = self._rate_tracker_by_id[field.id]
                tracker.on_event(time.monotonic())
                self._rate_last_ros_stamp[field.id] = msg_stamp

            for field in self._value_fields_by_topic.get(topic, []):
                state = self._value_state_by_id[field.id]
                if field.source.downsample_period_s is not None and state.next_accept_after is not None:
                    if msg_stamp < state.next_accept_after:
                        continue

                value = _extract_value(msg, field.source.value_key or 'data')
                if value is None:
                    self.get_logger().warn(f"Could not extract value for field {field.id} from topic {topic} using key '{field.source.value_key or 'data'}'")
                    continue

                state.last_value = value
                state.last_stamp = msg_stamp
                if field.source.downsample_period_s:
                    state.next_accept_after = msg_stamp + Duration(seconds=field.source.downsample_period_s)
                else:
                    state.next_accept_after = None

    def _on_tf(self, msg: TFMessage) -> None:
        now_ros = self.get_clock().now()
        now_monotonic = time.monotonic()
        for transform in msg.transforms:
            parent = _normalize_frame(transform.header.frame_id)
            child = _normalize_frame(transform.child_frame_id)
            field = self._tf_rate_field_by_pair.get((parent, child))
            if field is not None:
                with self._lock:
                    tracker = self._tf_rate_tracker_by_id[field.id]
                    tracker.on_event(now_monotonic)
                    self._tf_rate_last_ros_stamp[field.id] = Time.from_msg(transform.header.stamp) if transform.header.stamp else now_ros

            if not self._tf_demux_enabled:
                continue

            key = (parent, child)
            pub = self._tf_demux_pub_by_pair.get(key)
            if pub is None:
                parent_seg = _sanitize_topic_segment(parent)
                child_seg = _sanitize_topic_segment(child)
                topic = f"{self._tf_demux_prefix}/{parent_seg}_{child_seg}"
                pub = self.create_publisher(TFMessage, topic, self._qos_best_effort)
                self._tf_demux_pub_by_pair[key] = pub

            out = TFMessage()
            out.transforms.append(transform)
            pub.publish(out)

    def _try_resolve_pending_topics(self) -> None:
        if not self._pending_topics:
            return

        try:
            from rosidl_runtime_py.utilities import get_message  # type: ignore
        except ModuleNotFoundError as exc:
            self.get_logger().error(f"Missing rosidl_runtime_py (cannot resolve topic types): {exc}")
            return

        name_and_types = dict(self.get_topic_names_and_types())
        resolved: list[str] = []

        for topic in sorted(self._pending_topics):
            types = name_and_types.get(topic)
            if not types:
                continue

            # Take the first advertised type.
            type_str = types[0]
            try:
                msg_cls = get_message(type_str)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Failed to import message type {type_str} for {topic}: {exc}")
                continue

            sub = self.create_subscription(msg_cls, topic, lambda msg, t=topic: self._on_topic_message(t, msg), self._qos_best_effort)
            self._topic_subscriptions.append(sub)
            self._topic_type_by_topic[topic] = type_str
            resolved.append(topic)
            self.get_logger().info(f"Subscribed for topic={topic} type={type_str} (resolved)")

        for topic in resolved:
            self._pending_topics.discard(topic)

        if not self._pending_topics and self._resolve_timer is not None:
            self._resolve_timer.cancel()
            self._resolve_timer = None

    @staticmethod
    def _is_stale(now_ros: Time, stamp: Time, stale_after_s: float) -> bool:
        if stale_after_s <= 0:
            return False
        return (now_ros - stamp) > Duration(seconds=stale_after_s)


def _extract_value(msg: Any, key: str) -> Optional[float]:
    """Extract a numeric value from a message using a dotted key path."""
    parts = key.split('.')
    cur: Any = msg
    try:
        for part in parts:
            cur = getattr(cur, part)
    except AttributeError:
        return None
    try:
        return float(cur)
    except (TypeError, ValueError):
        return None


def _resolve_msg_stamp(msg: Any, fallback: Time) -> Time:
    try:
        stamp = msg.header.stamp  # type: ignore[attr-defined]
    except Exception:
        stamp = None
    if stamp:
        try:
            return Time.from_msg(stamp)
        except Exception:
            pass
    return fallback
