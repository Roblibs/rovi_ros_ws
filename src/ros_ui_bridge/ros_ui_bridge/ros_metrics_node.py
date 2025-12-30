from __future__ import annotations

import time
from typing import Any

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

from .config import TfRateMetricConfig, TopicRateMetricConfig
from .rate_tracker import RateTracker
from .voltage_state import VoltageState


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id).lstrip('/')


class UiBridgeRosNode(Node):
    def __init__(
        self,
        *,
        voltage_state: VoltageState,
        voltage_topic: str,
        topic_rates: list[TopicRateMetricConfig],
        tf_rates: list[TfRateMetricConfig],
    ) -> None:
        super().__init__('ros_ui_bridge')

        self._voltage_state = voltage_state
        self._qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self._topic_metrics_by_topic: dict[str, list[str]] = {}
        self._topic_type_by_topic: dict[str, str] = {}
        self._topic_trackers: dict[str, RateTracker] = {}
        self._tf_trackers: dict[str, RateTracker] = {}
        self._tf_metric_by_pair: dict[tuple[str, str], str] = {}
        self._metric_order: list[str] = []

        self._pending_topics: set[str] = set()
        self._topic_subscriptions: list[Any] = []

        self._voltage_topic = self.resolve_topic_name(str(voltage_topic))
        self._voltage_sub = self.create_subscription(Float32, self._voltage_topic, self._on_voltage, self._qos_best_effort)

        self._configure_topic_rates(topic_rates)
        self._configure_tf_rates(tf_rates)

        if self._pending_topics:
            self._resolve_timer = self.create_timer(2.0, self._try_resolve_pending_topics)
        else:
            self._resolve_timer = None

    def _configure_topic_rates(self, configs: list[TopicRateMetricConfig]) -> None:
        # Treat voltage as a normal topic-rate metric too; the voltage subscription updates all trackers for that topic.
        for cfg in configs:
            metric_id = cfg.id
            if metric_id in self._topic_trackers:
                raise RuntimeError(f"Duplicate topic-rate metric id: {metric_id}")

            tracker = RateTracker(target_hz=cfg.target_hz, emit_zero_when_unseen=cfg.emit_zero_when_unseen)
            self._topic_trackers[metric_id] = tracker
            self._metric_order.append(metric_id)

            topic = self.resolve_topic_name(str(cfg.topic))
            self._topic_metrics_by_topic.setdefault(topic, []).append(metric_id)
            if cfg.type:
                existing = self._topic_type_by_topic.get(topic)
                if existing and existing != cfg.type:
                    raise RuntimeError(f"Topic {topic} has conflicting message types in config: {existing} vs {cfg.type}")
                self._topic_type_by_topic[topic] = cfg.type

        # Create subscriptions for non-voltage topics (voltage is already subscribed for value + rate).
        if not self._topic_metrics_by_topic:
            return

        try:
            from rosidl_runtime_py.utilities import get_message  # type: ignore
        except ModuleNotFoundError:
            get_message = None

        for topic in sorted(set(self._topic_metrics_by_topic.keys())):
            if topic == self._voltage_topic:
                continue

            type_str = self._topic_type_by_topic.get(topic)
            if type_str and get_message is not None:
                try:
                    msg_cls = get_message(type_str)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn("Failed to import message type %s for %s: %s", type_str, topic, exc)
                    self._pending_topics.add(topic)
                    continue

                sub = self.create_subscription(
                    msg_cls,
                    topic,
                    lambda _msg, t=topic: self._on_generic_topic(t),
                    self._qos_best_effort,
                )
                self._topic_subscriptions.append(sub)
                self.get_logger().info("Subscribed for hz metric topic=%s type=%s (configured)", topic, type_str)
            else:
                self._pending_topics.add(topic)

    def _configure_tf_rates(self, configs: list[TfRateMetricConfig]) -> None:
        self._tf_sub = None
        if not configs:
            return

        for cfg in configs:
            metric_id = cfg.id
            if metric_id in self._tf_trackers:
                raise RuntimeError(f"Duplicate tf-rate metric id: {metric_id}")

            parent = _normalize_frame(cfg.parent)
            child = _normalize_frame(cfg.child)
            key = (parent, child)
            if key in self._tf_metric_by_pair:
                raise RuntimeError(f"Duplicate tf-rate (parent,child): {parent}->{child}")

            tracker = RateTracker(target_hz=cfg.target_hz, emit_zero_when_unseen=cfg.emit_zero_when_unseen)
            self._tf_trackers[metric_id] = tracker
            self._tf_metric_by_pair[key] = metric_id
            self._metric_order.append(metric_id)

        self._tf_sub = self.create_subscription(TFMessage, '/tf', self._on_tf, self._qos_best_effort)

    def _on_voltage(self, msg: Float32) -> None:
        now = time.monotonic()
        self._voltage_state.update(float(msg.data))
        for metric_id in self._topic_metrics_by_topic.get(self._voltage_topic, []):
            self._topic_trackers[metric_id].on_event(now)

    def _on_tf(self, msg: TFMessage) -> None:
        now = time.monotonic()
        for transform in msg.transforms:
            parent = _normalize_frame(transform.header.frame_id)
            child = _normalize_frame(transform.child_frame_id)
            metric_id = self._tf_metric_by_pair.get((parent, child))
            if metric_id is not None:
                self._tf_trackers[metric_id].on_event(now)

    def _on_generic_topic(self, topic: str) -> None:
        now = time.monotonic()
        for metric_id in self._topic_metrics_by_topic.get(topic, []):
            self._topic_trackers[metric_id].on_event(now)

    def _try_resolve_pending_topics(self) -> None:
        if not self._pending_topics:
            return

        try:
            from rosidl_runtime_py.utilities import get_message  # type: ignore
        except ModuleNotFoundError as exc:
            self.get_logger().error("Missing rosidl_runtime_py (cannot resolve topic types): %s", exc)
            return

        name_and_types = dict(self.get_topic_names_and_types())

        resolved: list[str] = []
        for topic in sorted(self._pending_topics):
            types = name_and_types.get(topic)
            if not types:
                continue
            if len(types) != 1:
                self.get_logger().warn("Topic %s has multiple types %s; cannot subscribe for hz metrics.", topic, types)
                continue

            type_str = types[0]
            try:
                msg_cls = get_message(type_str)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn("Failed to import message type %s for %s: %s", type_str, topic, exc)
                continue

            sub = self.create_subscription(
                msg_cls,
                topic,
                lambda _msg, t=topic: self._on_generic_topic(t),
                self._qos_best_effort,
            )
            self._topic_subscriptions.append(sub)
            resolved.append(topic)
            self.get_logger().info("Subscribed for hz metric topic=%s type=%s", topic, type_str)

        for topic in resolved:
            self._pending_topics.discard(topic)

    def sample_rate_metrics(self) -> list[tuple[str, float, float | None]]:
        """Returns (id, hz, target_hz) in config order, resetting counters each call."""
        now = time.monotonic()
        out: list[tuple[str, float, float | None]] = []

        for metric_id in self._metric_order:
            tracker = self._topic_trackers.get(metric_id) or self._tf_trackers.get(metric_id)
            if tracker is None:
                continue
            hz = tracker.sample_hz(now_monotonic=now)
            if hz is None:
                continue
            out.append((metric_id, hz, tracker.target_hz))

        return out
