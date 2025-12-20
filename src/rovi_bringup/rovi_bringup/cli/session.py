from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def _ros_home() -> Path:
    return Path(os.environ.get("ROS_HOME", Path.home() / ".ros")).expanduser()


def _current_launch_path() -> Path:
    return _ros_home() / "rovi" / "session" / "current_launch"


def _cmd_set(launch_ref: str) -> int:
    launch_ref = launch_ref.strip()
    if not launch_ref:
        print("[rovi_session] Missing launch ref (expected: <pkg>/<launchfile>)", file=sys.stderr)
        return 2

    path = _current_launch_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"{launch_ref}\n", encoding="utf-8")
    return 0


def _cmd_get() -> int:
    path = _current_launch_path()
    if not path.is_file():
        return 1
    value = path.read_text(encoding="utf-8").splitlines()
    if not value:
        return 1
    value0 = value[0].strip()
    if not value0:
        return 1
    print(value0)
    return 0


def _cmd_clear() -> int:
    path = _current_launch_path()
    try:
        path.unlink()
    except FileNotFoundError:
        pass
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rovi_session")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_set = sub.add_parser("set", help="Set current launch ref")
    p_set.add_argument("launch_ref", help="Launch ref like rovi_bringup/nav.launch.py")

    sub.add_parser("get", help="Print current launch ref")
    sub.add_parser("clear", help="Clear current launch ref")

    args = parser.parse_args(argv)

    if args.cmd == "set":
        return _cmd_set(args.launch_ref)
    if args.cmd == "get":
        return _cmd_get()
    if args.cmd == "clear":
        return _cmd_clear()

    print(f"[rovi_session] Unknown command: {args.cmd}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

