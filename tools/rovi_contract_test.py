#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from tf2_msgs.msg import TFMessage


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


def _parse_contract(doc: dict[str, Any]) -> tuple[dict[str, StackContract], StackContract | None]:
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

    offline_node = doc.get("offline_sanity")
    offline: StackContract | None = None
    if offline_node is not None:
        if not isinstance(offline_node, dict):
            raise ValueError("contract.yaml: offline_sanity must be a mapping")
        offline = StackContract(
            required_topics=_parse_topic_reqs(offline_node.get("required_topics", [])),
            required_topics_by_mode={},
            required_tf=_parse_tf_reqs(offline_node.get("required_tf", [])),
        )

    return stacks, offline


class GraphProbe(Node):
    def __init__(self) -> None:
        super().__init__("rovi_contract_probe")
        self._tf_edges: set[tuple[str, str]] = set()

        self._sub_tf = self.create_subscription(TFMessage, "/tf", self._on_tf, qos_profile_sensor_data)

        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub_tf_static = self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, tf_static_qos)

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

    def topic_types(self) -> dict[str, set[str]]:
        out: dict[str, set[str]] = {}
        for name, types in self.get_topic_names_and_types():
            out[_norm_topic(name)] = set(types)
        return out

    def wait_topics(self, reqs: list[TopicReq], *, timeout_s: float) -> list[str]:
        deadline = time.monotonic() + timeout_s
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
        deadline = time.monotonic() + timeout_s
        missing: set[tuple[str, str]] = {(r.parent, r.child) for r in reqs}
        while time.monotonic() < deadline and rclpy.ok():
            for parent, child in list(missing):
                if (parent, child) in self._tf_edges:
                    missing.discard((parent, child))
            if not missing:
                return []
            rclpy.spin_once(self, timeout_sec=0.1)
        return [f"{p}->{c}" for (p, c) in sorted(missing)]


def _launch_cmd(*, robot_mode: str, stack: str) -> list[str]:
    # Always keep launches headless for tests.
    cmd = [
        "ros2",
        "launch",
        "rovi_bringup",
        "rovi.launch.py",
        f"robot_mode:={robot_mode}",
        f"stack:={stack}",
        "rviz:=false",
        "gazebo_gui:=false",
        "joy_enabled:=false",
    ]
    return cmd


def _offline_cmd() -> list[str]:
    return [
        "ros2",
        "launch",
        "rovi_bringup",
        "robot_bringup.launch.py",
        "robot_mode:=offline",
        "use_sim_time:=false",
    ]


def _terminate(proc: subprocess.Popen, *, timeout_s: float = 10.0) -> None:
    if proc.poll() is not None:
        return
    try:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except Exception:
            proc.send_signal(signal.SIGINT)
        proc.wait(timeout=timeout_s)
        return
    except Exception:
        pass
    try:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except Exception:
            proc.terminate()
        proc.wait(timeout=timeout_s)
        return
    except Exception:
        pass
    try:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            proc.kill()
    except Exception:
        pass


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ROVI backend/stack contract tests (real ↔ sim parity).")
    parser.add_argument("--contract", default="docs/contract.yaml", help="Path to contract YAML.")
    parser.add_argument("--robot-mode", action="append", default=None, help="Robot mode(s) to test (repeatable).")
    parser.add_argument(
        "--profile",
        choices=["quick", "full"],
        default="quick",
        help="Test preset (quick: teleop+camera, full: teleop+camera+mapping+localization+nav).",
    )
    parser.add_argument("--stack", action="append", default=None, help="Stack(s) to test (repeatable).")
    parser.add_argument("--include-offline-sanity", action="store_true", help="Also run offline sanity checks.")
    parser.add_argument("--startup-s", type=float, default=None, help="Seconds to wait after launch before probing.")
    parser.add_argument("--timeout-topics-s", type=float, default=None, help="Timeout for topic presence/type checks.")
    parser.add_argument("--timeout-tf-s", type=float, default=None, help="Timeout for TF edge checks.")
    parser.add_argument(
        "--launch-logs",
        choices=["file", "inherit"],
        default="file",
        help="Where to write ros2 launch output (file keeps the test output clean).",
    )
    parser.add_argument("--log-dir", default="/tmp/rovi_contract_logs", help="Directory for --launch-logs=file.")
    args = parser.parse_args(argv)

    contract_path = Path(args.contract).expanduser()
    doc = _load_yaml(contract_path)
    stacks, offline = _parse_contract(doc)

    robot_modes = args.robot_mode or ["sim"]
    if args.stack:
        stacks_to_test = args.stack
    elif args.profile == "full":
        stacks_to_test = ["teleop", "camera", "mapping", "localization", "nav"]
    else:
        stacks_to_test = ["teleop", "camera"]

    startup_s = float(args.startup_s) if args.startup_s is not None else (6.0 if args.profile == "full" else 4.0)
    timeout_topics_s = float(args.timeout_topics_s) if args.timeout_topics_s is not None else (60.0 if args.profile == "full" else 20.0)
    timeout_tf_s = float(args.timeout_tf_s) if args.timeout_tf_s is not None else (30.0 if args.profile == "full" else 10.0)

    # Ensure requested stacks exist.
    for s in stacks_to_test:
        if s not in stacks:
            print(f"[contract] Unknown stack '{s}' (known: {', '.join(sorted(stacks.keys()))})", file=sys.stderr)
            return 2

    failures: list[str] = []

    log_dir = Path(args.log_dir).expanduser()
    if args.launch_logs == "file":
        log_dir.mkdir(parents=True, exist_ok=True)

    def _start_launch(*, robot_mode: str, stack: str) -> tuple[subprocess.Popen, Path | None, Any | None]:
        label = f"{robot_mode}:{stack}"
        cmd = _launch_cmd(robot_mode=robot_mode, stack=stack)
        if args.launch_logs == "inherit":
            return subprocess.Popen(cmd, start_new_session=True), None, None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f"{label.replace(':', '_')}_{stamp}.log"
        log_f = open(log_path, "wb")
        proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT, start_new_session=True)
        return proc, log_path, log_f

    for robot_mode in robot_modes:
        # Match rovi_env.sh sim behavior for discovery; keep the probe consistent with the launched system.
        # This env var is read by RMW at init time, so it must be set before rclpy.init().
        if robot_mode == "sim":
            os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

        try:
            rclpy.init(args=None)

            for stack in stacks_to_test:
                label = f"{robot_mode}:{stack}"
                print(f"[contract] Launching {label}…", file=sys.stderr)

                proc, log_path, log_f = _start_launch(robot_mode=robot_mode, stack=stack)
                probe: GraphProbe | None = None
                try:
                    time.sleep(startup_s)
                    probe = GraphProbe()

                    stack_contract = stacks[stack]
                    topic_reqs = list(stack_contract.required_topics)
                    topic_reqs.extend(stack_contract.required_topics_by_mode.get(robot_mode, []))

                    missing_topics = probe.wait_topics(topic_reqs, timeout_s=timeout_topics_s)
                    if missing_topics:
                        hint = f" (log: {log_path})" if log_path else ""
                        failures.append(f"{label}: missing topics: {', '.join(missing_topics)}{hint}")

                    missing_tf = probe.wait_tf(stack_contract.required_tf, timeout_s=timeout_tf_s)
                    if missing_tf:
                        hint = f" (log: {log_path})" if log_path else ""
                        failures.append(f"{label}: missing TF: {', '.join(missing_tf)}{hint}")

                except KeyboardInterrupt:
                    hint = f" (log: {log_path})" if log_path else ""
                    failures.append(f"{label}: interrupted by user{hint}")
                    return 130
                except Exception as e:
                    hint = f" (log: {log_path})" if log_path else ""
                    failures.append(f"{label}: exception: {e}{hint}")
                finally:
                    try:
                        if probe is not None:
                            probe.destroy_node()
                    except Exception:
                        pass
                    _terminate(proc)
                    try:
                        if log_f is not None:
                            log_f.close()
                    except Exception:
                        pass

        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    if args.include_offline_sanity and offline is not None:
        print("[contract] Launching offline_sanity…", file=sys.stderr)
        proc = subprocess.Popen(_offline_cmd(), start_new_session=True)
        probe = None
        try:
            rclpy.init(args=None)
            time.sleep(startup_s)
            probe = GraphProbe()
            missing_topics = probe.wait_topics(offline.required_topics, timeout_s=timeout_topics_s)
            if missing_topics:
                failures.append(f"offline_sanity: missing topics: {', '.join(missing_topics)}")
            missing_tf = probe.wait_tf(offline.required_tf, timeout_s=timeout_tf_s)
            if missing_tf:
                failures.append(f"offline_sanity: missing TF: {', '.join(missing_tf)}")
        except KeyboardInterrupt:
            failures.append("offline_sanity: interrupted by user")
            return 130
        except Exception as e:
            failures.append(f"offline_sanity: exception: {e}")
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
            _terminate(proc)

    if failures:
        print("[contract] FAIL", file=sys.stderr)
        for line in failures:
            print(f"  - {line}", file=sys.stderr)
        return 1

    print("[contract] OK", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
