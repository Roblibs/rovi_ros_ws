#!/usr/bin/env python3
from __future__ import annotations

import argparse
import dataclasses
import time
from typing import Any, Callable, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def _load_yaml(path: str) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing dependency 'PyYAML'. Install workspace Python deps (e.g. `uv sync`) or ensure the venv is on PYTHONPATH."
        ) from exc

    try:
        with open(path, encoding='utf-8') as fh:
            return yaml.safe_load(fh) or {}
    except OSError as exc:
        raise RuntimeError(f"Failed to read config file: {path}") from exc
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"Failed to parse YAML config: {path}") from exc


@dataclasses.dataclass(frozen=True)
class TopicConfig:
    source: str
    dest: str
    type_str: str
    period_s: float


def _parse_topics(config: dict[str, Any]) -> list[TopicConfig]:
    out: list[TopicConfig] = []
    topics = config.get('topics', [])
    if not topics:
        return out
    if not isinstance(topics, list):
        raise RuntimeError("Invalid 'topics' section (expected list)")

    for entry in topics:
        if not isinstance(entry, dict):
            continue
        source = str(entry.get('from', '')).strip()
        dest = str(entry.get('to', '')).strip()
        type_str = str(entry.get('type', '')).strip()
        rate_hz = float(entry.get('rate_hz', 0.0))
        if not source or not dest or not type_str or rate_hz <= 0:
            continue
        out.append(TopicConfig(source=source, dest=dest, type_str=type_str, period_s=1.0 / rate_hz))
    return out


class _Throttle(Node):
    def __init__(self, config_path: str, *, qos: QoSProfile) -> None:
        super().__init__('viz_downsample')

        raw_config = _load_yaml(config_path)
        topics = _parse_topics(raw_config)
        if not topics:
            self.get_logger().warn("No topics configured for viz downsampling (config: %s)", config_path)

        try:
            from rosidl_runtime_py.utilities import get_message  # type: ignore
        except ModuleNotFoundError as exc:
            raise RuntimeError("Missing rosidl_runtime_py (needed for dynamic message imports).") from exc

        self._last_pub: Dict[str, float] = {}
        self._pub_by_dest: Dict[str, Any] = {}

        for topic_cfg in topics:
            try:
                msg_cls = get_message(topic_cfg.type_str)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn("Skipping topic %s (type import failed: %s)", topic_cfg.source, exc)
                continue

            pub = self.create_publisher(msg_cls, topic_cfg.dest, qos)
            self._pub_by_dest[topic_cfg.dest] = pub
            self._last_pub[topic_cfg.dest] = 0.0

            self.create_subscription(
                msg_cls,
                topic_cfg.source,
                self._make_cb(dest=topic_cfg.dest, period_s=topic_cfg.period_s),
                qos,
            )
            self.get_logger().info(
                f"Downsampling {topic_cfg.source} -> {topic_cfg.dest} @ {1.0 / topic_cfg.period_s:.2f} Hz"
            )

    def _make_cb(self, *, dest: str, period_s: float) -> Callable[[Any], None]:
        def _cb(msg: Any) -> None:
            now = time.monotonic()
            last = self._last_pub.get(dest, 0.0)
            if now - last < period_s:
                return
            pub = self._pub_by_dest.get(dest)
            if pub is None:
                return
            pub.publish(msg)
            self._last_pub[dest] = now

        return _cb


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description='Downsample topics for RViz visualization (publishes to /viz/*).')
    parser.add_argument('--config', default=None, help='Path to YAML config (defaults to package config/viz_downsample.yaml)')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    config_path = args.config
    if not config_path:
        from ament_index_python.packages import get_package_share_directory

        config_path = (
            f"{get_package_share_directory('rovi_bringup')}/config/viz_downsample.yaml"
        )

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    node = _Throttle(config_path=config_path, qos=qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
