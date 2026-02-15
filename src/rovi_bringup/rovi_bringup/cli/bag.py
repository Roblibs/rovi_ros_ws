from __future__ import annotations

import os
import re
import signal
import shlex
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

SESSION_CURRENT_LAUNCH_REF_TOPIC = "/rovi/session/current_launch_ref"


def _ros_home() -> Path:
    return Path(os.environ.get("ROS_HOME", Path.home() / ".ros")).expanduser()


def _rovi_dir() -> Path:
    return _ros_home() / "rovi"


def _bags_dir() -> Path:
    return _rovi_dir() / "bags"


class _SessionProbe(Node):
    def __init__(self) -> None:
        super().__init__("rovi_bag_session_probe")
        self.value: str | None = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, SESSION_CURRENT_LAUNCH_REF_TOPIC, self._on_msg, qos)

    def _on_msg(self, msg: String) -> None:
        value = str(msg.data or "").strip()
        if value:
            self.value = value


def _wait_current_launch_ref(*, timeout_s: float = 0.0) -> str:
    timeout_s = float(timeout_s)
    if timeout_s < 0:
        raise ValueError("timeout_s must be >= 0")

    rclpy.init(args=None)
    probe: _SessionProbe | None = None
    try:
        probe = _SessionProbe()
        deadline = None if timeout_s == 0 else (time.monotonic() + timeout_s)
        while rclpy.ok() and probe.value is None and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(probe, timeout_sec=0.1)
        if probe.value is None:
            raise RuntimeError(f"Timed out waiting for session topic: {SESSION_CURRENT_LAUNCH_REF_TOPIC}")
        return probe.value
    finally:
        try:
            if probe is not None:
                probe.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


@dataclass(frozen=True)
class LaunchSelector:
    package: str
    key: str


def _selector_from_ref_or_key(value: str) -> LaunchSelector:
    value = value.strip()
    if not value:
        raise ValueError("empty launch selector")

    if "/" in value:
        package, launch_file = value.split("/", 1)
        base = Path(launch_file).name
        key = base.removesuffix(".launch.py").removesuffix(".py")
        if not key:
            raise ValueError(f"could not derive launch key from '{value}'")
        return LaunchSelector(package=package, key=key)

    return LaunchSelector(package="rovi_bringup", key=value)


def _bag_topics_config_path() -> Path:
    candidates: list[Path] = []

    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore

        share = Path(get_package_share_directory("rovi_bringup"))
        candidates.append(share / "config" / "bag_topics.yaml")
    except Exception:
        pass

    ws_dir = os.environ.get("ROVI_ROS_WS_DIR")
    if ws_dir:
        candidates.append(Path(ws_dir) / "src" / "rovi_bringup" / "config" / "bag_topics.yaml")

    for path in candidates:
        if path.is_file():
            return path

    pretty = ", ".join(str(p) for p in candidates) if candidates else "(no candidates)"
    raise FileNotFoundError(f"Missing bag topics config (searched: {pretty})")


def _load_topics_by_key(path: Path) -> dict[str, list[str]]:
    text = path.read_text(encoding="utf-8")

    try:
        import yaml  # type: ignore

        data = yaml.safe_load(text)
        if isinstance(data, dict):
            out: dict[str, list[str]] = {}
            for k, v in data.items():
                if not isinstance(k, str):
                    continue
                if not isinstance(v, list):
                    continue
                topics = [str(x).strip() for x in v if str(x).strip()]
                out[k.strip()] = topics
            if out:
                return out
    except Exception:
        pass

    out: dict[str, list[str]] = {}
    current: str | None = None
    key_re = re.compile(r"^([A-Za-z0-9_-]+)\s*:\s*(?:#.*)?$")
    item_re = re.compile(r"^\s*-\s*(.+?)\s*(?:#.*)?$")

    for raw in text.splitlines():
        line = raw.rstrip("\n")
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        m_key = key_re.match(line)
        if m_key and not line.startswith((" ", "\t")):
            current = m_key.group(1)
            out.setdefault(current, [])
            continue

        m_item = item_re.match(line)
        if m_item and current is not None:
            val = m_item.group(1).strip()
            if (val.startswith('"') and val.endswith('"')) or (val.startswith("'") and val.endswith("'")):
                val = val[1:-1]
            if val:
                out[current].append(val)
            continue

        raise ValueError(f"Unsupported line in {path}: {raw}")

    return out


def _require_ros2_cli() -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError("Missing 'ros2' CLI (is /opt/ros/jazzy/setup.bash sourced?)")


def _require_rosbag_zstd() -> None:
    _require_ros2_cli()
    proc = subprocess.run(
        ["ros2", "pkg", "prefix", "rosbag2_compression_zstd"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            "Missing rosbag2 zstd compression plugin (rosbag2_compression_zstd). "
            "Install via rovi_env.sh: install_ros_deps (or apt: ros-jazzy-rosbag2-compression-zstd)."
        )


def _format_cmd(cmd: list[str]) -> str:
    return " ".join(shlex.quote(c) for c in cmd)


def _cmd_record(argv: list[str]) -> int:
    selector_arg: str | None = None
    pass_args = list(argv)
    if pass_args and not pass_args[0].startswith("-"):
        selector_arg = pass_args.pop(0)

    if selector_arg:
        launch_ref = selector_arg
    else:
        try:
            print(f"[rovi_bag] Waiting for session topic: {SESSION_CURRENT_LAUNCH_REF_TOPIC}", file=sys.stderr)
            launch_ref = _wait_current_launch_ref(timeout_s=0.0)
        except Exception as e:
            print(f"[rovi_bag] {e}", file=sys.stderr)
            return 2
    selector = _selector_from_ref_or_key(launch_ref)

    try:
        cfg_path = _bag_topics_config_path()
    except Exception as e:
        print(f"[rovi_bag] {e}", file=sys.stderr)
        return 2

    try:
        topics_by_key = _load_topics_by_key(cfg_path)
    except Exception as e:
        print(f"[rovi_bag] Failed to parse {cfg_path}: {e}", file=sys.stderr)
        return 2

    topics = topics_by_key.get(selector.key, [])
    if not topics:
        print(
            f"[rovi_bag] No topics configured for '{selector.key}' in {cfg_path}",
            file=sys.stderr,
        )
        return 2

    try:
        _require_rosbag_zstd()
    except Exception as e:
        print(f"[rovi_bag] {e}", file=sys.stderr)
        return 2

    bags_root = _bags_dir()
    bags_root.mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_uri = bags_root / f"{selector.key}-{ts}"

    cmd = [
        "ros2",
        "bag",
        "record",
        "--compression-mode",
        "file",
        "--compression-format",
        "zstd",
        "-o",
        str(out_uri),
        *topics,
        *pass_args,
    ]

    print(f"[rovi_bag] Recording '{selector.key}' → {out_uri}", file=sys.stderr)
    print(f"[rovi_bag] {_format_cmd(cmd)}", file=sys.stderr)

    proc = subprocess.Popen(cmd)
    interrupts = 0
    while True:
        try:
            returncode = proc.wait()
            break
        except KeyboardInterrupt:
            interrupts += 1
            if proc.poll() is not None:
                continue

            if interrupts == 1:
                print("[rovi_bag] Ctrl+C received, stopping recording (SIGINT) and waiting…", file=sys.stderr)
                try:
                    proc.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
            else:
                print("[rovi_bag] Ctrl+C received again, stopping recording (SIGTERM) and waiting…", file=sys.stderr)
                try:
                    proc.terminate()
                except ProcessLookupError:
                    pass

    if returncode < 0:
        return 128 + (-returncode)
    return int(returncode)


def _find_latest_bag(bags_root: Path, key: str | None) -> Path | None:
    if not bags_root.is_dir():
        return None

    candidates: list[Path] = []
    for p in bags_root.iterdir():
        if not p.is_dir():
            continue
        if key and not p.name.startswith(f"{key}-"):
            continue
        if not (p / "metadata.yaml").is_file():
            continue
        candidates.append(p)

    if not candidates:
        return None

    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0]


def _cmd_play(argv: list[str]) -> int:
    selector_arg: str | None = None
    pass_args = list(argv)
    if pass_args and not pass_args[0].startswith("-"):
        selector_arg = pass_args.pop(0)

    try:
        _require_rosbag_zstd()
    except Exception as e:
        print(f"[rovi_bag] {e}", file=sys.stderr)
        return 2

    bags_root = _bags_dir()
    if not bags_root.is_dir():
        print(f"[rovi_bag] Bags directory does not exist: {bags_root}", file=sys.stderr)
        return 1

    target: Path | None = None
    if selector_arg:
        expanded = Path(os.path.expanduser(selector_arg))
        if expanded.is_dir():
            target = expanded
        else:
            direct = bags_root / selector_arg
            if direct.is_dir():
                target = direct
            else:
                target = _find_latest_bag(bags_root, selector_arg)
                if target is None:
                    print(
                        f"[rovi_bag] No rosbags found for key '{selector_arg}' under {bags_root}",
                        file=sys.stderr,
                    )
                    return 1
    else:
        target = _find_latest_bag(bags_root, None)
        if target is None:
            print(f"[rovi_bag] No rosbags found under {bags_root}", file=sys.stderr)
            return 1

    cmd = ["ros2", "bag", "play", str(target), *pass_args]
    print(f"[rovi_bag] Playing bag: {target}", file=sys.stderr)
    print(f"[rovi_bag] {_format_cmd(cmd)}", file=sys.stderr)
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        return int(e.returncode) if e.returncode is not None else 1
    return 0


USAGE = """\
rovi_bag - rosbag helpers

Usage:
  rovi_bag record [<launch_key|pkg/launchfile>] [ros2 bag record args...]
  rovi_bag play   [<launch_key|bag_dir>]        [ros2 bag play args...]

Zero-arg behavior:
  record: waits for /rovi/session/current_launch_ref
  play:   plays the latest bag under ~/.ros/rovi/bags
"""


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)

    if not argv or argv[0] in {"-h", "--help"}:
        print(USAGE)
        return 0 if argv else 2

    cmd = argv.pop(0)
    if cmd == "record":
        return _cmd_record(argv)
    if cmd == "play":
        return _cmd_play(argv)

    print(f"[rovi_bag] Unknown command: {cmd}\n\n{USAGE}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
