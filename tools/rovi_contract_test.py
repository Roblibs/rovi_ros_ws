#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


SESSION_CURRENT_LAUNCH_REF_TOPIC = "/rovi/session/current_launch_ref"


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except Exception as e:  # pragma: no cover
        raise RuntimeError(
            "Missing PyYAML dependency (import yaml failed). "
            "Install python deps for this workspace (uv sync) or system python3-yaml."
        ) from e

    if not path.is_file():
        raise FileNotFoundError(str(path))
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping at root of {path}")
    return data


def _norm_frame(value: str) -> str:
    return str(value).strip().lstrip("/")


def _norm_topic(value: str) -> str:
    value = str(value).strip()
    if not value.startswith("/"):
        value = "/" + value
    return value


def _stack_from_launch_ref(launch_ref: str | None) -> str | None:
    if not launch_ref:
        return None

    value = str(launch_ref).strip()
    if not value:
        return None

    if "/" in value:
        _package, launch_file = value.split("/", 1)
        base = Path(launch_file).name
        key = base.removesuffix(".launch.py").removesuffix(".py")
        return key or None

    return value


@dataclass(frozen=True)
class TopicReq:
    name: str
    type: str


@dataclass(frozen=True)
class TfReq:
    parent: str
    child: str


@dataclass(frozen=True)
class StackContract:
    required_topics: list[TopicReq]
    required_topics_by_mode: dict[str, list[TopicReq]]
    required_tf: list[TfReq]


def _parse_topic_reqs(items: Iterable[dict[str, Any]] | None) -> list[TopicReq]:
    out: list[TopicReq] = []
    for item in items or []:
        if not isinstance(item, dict):
            raise ValueError(f"Expected topic mapping, got: {type(item)}")
        name = _norm_topic(item.get("name", ""))
        ty = str(item.get("type", "")).strip()
        if not name or name == "/":
            raise ValueError("Topic req missing name")
        if not ty:
            raise ValueError(f"Topic req missing type for {name}")
        out.append(TopicReq(name=name, type=ty))
    return out


def _parse_tf_reqs(items: Iterable[dict[str, Any]] | None) -> list[TfReq]:
    out: list[TfReq] = []
    for item in items or []:
        if not isinstance(item, dict):
            raise ValueError(f"Expected tf mapping, got: {type(item)}")
        parent = _norm_frame(item.get("parent", ""))
        child = _norm_frame(item.get("child", ""))
        if not parent or not child:
            raise ValueError("TF req missing parent/child")
        out.append(TfReq(parent=parent, child=child))
    return out


def _parse_contract(doc: dict[str, Any]) -> dict[str, StackContract]:
    stacks_raw = doc.get("stacks", {})
    if not isinstance(stacks_raw, dict):
        raise ValueError("contract.yaml: expected 'stacks' mapping")

    stacks: dict[str, StackContract] = {}
    for stack_name, stack_node in stacks_raw.items():
        if not isinstance(stack_node, dict):
            raise ValueError(f"contract.yaml: stack '{stack_name}' must be a mapping")
        req_topics = _parse_topic_reqs(stack_node.get("required_topics", []))
        by_mode: dict[str, list[TopicReq]] = {}
        req_by_mode = stack_node.get("required_topics_by_mode", {}) or {}
        if not isinstance(req_by_mode, dict):
            raise ValueError(f"contract.yaml: stack '{stack_name}' required_topics_by_mode must be a mapping")
        for mode, items in req_by_mode.items():
            if not isinstance(items, list):
                raise ValueError(f"contract.yaml: stack '{stack_name}' mode '{mode}' must be a list")
            by_mode[str(mode)] = _parse_topic_reqs(items)
        req_tf = _parse_tf_reqs(stack_node.get("required_tf", []))
        stacks[str(stack_name)] = StackContract(required_topics=req_topics, required_topics_by_mode=by_mode, required_tf=req_tf)

    return stacks


class GraphProbe(Node):
    def __init__(self) -> None:
        super().__init__("rovi_contract_probe")

        self._current_launch_ref: str | None = None
        self._tf_edges: set[tuple[str, str]] = set()

        session_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub_session = self.create_subscription(String, SESSION_CURRENT_LAUNCH_REF_TOPIC, self._on_session, session_qos)

        self._sub_tf = self.create_subscription(TFMessage, "/tf", self._on_tf, qos_profile_sensor_data)

        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub_tf_static = self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, tf_static_qos)

    def _on_session(self, msg: String) -> None:
        value = str(msg.data or "").strip()
        if value:
            self._current_launch_ref = value

    def _on_tf(self, msg: TFMessage) -> None:
        self._add_tf_edges(msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._add_tf_edges(msg)

    def _add_tf_edges(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            parent = _norm_frame(t.header.frame_id)
            child = _norm_frame(t.child_frame_id)
            if parent and child:
                self._tf_edges.add((parent, child))

    def wait_current_launch_ref(self, *, timeout_s: float) -> str:
        timeout_s = float(timeout_s)
        if timeout_s < 0:
            raise ValueError("timeout_s must be >= 0")
        deadline = None if timeout_s == 0 else (time.monotonic() + timeout_s)
        while rclpy.ok() and self._current_launch_ref is None and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._current_launch_ref is None:
            raise TimeoutError(f"Timed out waiting for {SESSION_CURRENT_LAUNCH_REF_TOPIC}")
        return self._current_launch_ref

    def topic_types(self) -> dict[str, set[str]]:
        out: dict[str, set[str]] = {}
        for name, types in self.get_topic_names_and_types():
            out[_norm_topic(name)] = set(types)
        return out

    def wait_topics(self, reqs: list[TopicReq], *, timeout_s: float) -> list[str]:
        deadline = time.monotonic() + float(timeout_s)
        req_by_name = {r.name: r.type for r in reqs}
        missing: set[str] = set(req_by_name.keys())
        while time.monotonic() < deadline and rclpy.ok():
            types_by_topic = self.topic_types()
            for r in list(missing):
                got = types_by_topic.get(r, set())
                expected = req_by_name.get(r, "")
                if expected and expected in got:
                    missing.discard(r)
            if not missing:
                return []
            rclpy.spin_once(self, timeout_sec=0.1)
        return sorted(missing)

    def wait_tf(self, reqs: list[TfReq], *, timeout_s: float) -> list[str]:
        deadline = time.monotonic() + float(timeout_s)
        missing: set[tuple[str, str]] = {(r.parent, r.child) for r in reqs}
        while time.monotonic() < deadline and rclpy.ok():
            for parent, child in list(missing):
                if (parent, child) in self._tf_edges:
                    missing.discard((parent, child))
            if not missing:
                return []
            rclpy.spin_once(self, timeout_sec=0.1)
        return [f"{p}->{c}" for (p, c) in sorted(missing)]


def _infer_robot_mode(types_by_topic: dict[str, set[str]]) -> str:
    # Minimal heuristic for observer mode:
    # - If /clock exists, assume sim (use_sim_time graphs should have it).
    # - Otherwise assume real.
    got = types_by_topic.get("/clock", set())
    if "rosgraph_msgs/msg/Clock" in got:
        return "sim"
    return "real"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ROVI contract check (observer-only; does not launch anything).")
    parser.add_argument("--contract", default="docs/contract.yaml", help="Path to contract YAML.")
    parser.add_argument("--timeout-session-s", type=float, default=30.0, help="Wait for session topic (0 = forever).")
    parser.add_argument("--timeout-topics-s", type=float, default=25.0, help="Timeout for topic presence/type checks.")
    parser.add_argument("--timeout-tf-s", type=float, default=30.0, help="Timeout for TF edge checks.")
    args = parser.parse_args(argv)

    contract_path = Path(args.contract).expanduser()
    doc = _load_yaml(contract_path)
    stacks = _parse_contract(doc)

    rclpy.init(args=None)
    probe: GraphProbe | None = None
    try:
        probe = GraphProbe()

        print(f"[contract] Waiting for session: {SESSION_CURRENT_LAUNCH_REF_TOPIC}…", file=sys.stderr)
        launch_ref = probe.wait_current_launch_ref(timeout_s=float(args.timeout_session_s))
        stack = _stack_from_launch_ref(launch_ref)
        if not stack:
            print(f"[contract] FAIL: could not derive stack from launch_ref='{launch_ref}'", file=sys.stderr)
            return 2
        if stack not in stacks:
            print(f"[contract] FAIL: unknown stack '{stack}' (from '{launch_ref}')", file=sys.stderr)
            print(f"[contract] Known stacks: {', '.join(sorted(stacks.keys()))}", file=sys.stderr)
            return 2

        print(f"[contract] Observing stack='{stack}' launch_ref='{launch_ref}'…", file=sys.stderr)

        stack_contract = stacks[stack]
        types_by_topic = probe.topic_types()
        robot_mode = _infer_robot_mode(types_by_topic)

        topic_reqs = list(stack_contract.required_topics)
        topic_reqs.extend(stack_contract.required_topics_by_mode.get(robot_mode, []))

        failures: list[str] = []

        missing_topics = probe.wait_topics(topic_reqs, timeout_s=float(args.timeout_topics_s))
        if missing_topics:
            failures.append(f"missing topics: {', '.join(missing_topics)}")

        missing_tf = probe.wait_tf(stack_contract.required_tf, timeout_s=float(args.timeout_tf_s))
        if missing_tf:
            failures.append(f"missing TF: {', '.join(missing_tf)}")

        if failures:
            print("[contract] FAIL", file=sys.stderr)
            for line in failures:
                print(f"  - {line}", file=sys.stderr)
            return 1

        print(f"[contract] OK (robot_mode={robot_mode})", file=sys.stderr)
        return 0

    except TimeoutError as e:
        print(f"[contract] FAIL: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            if probe is not None:
                probe.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

