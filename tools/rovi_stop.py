#!/usr/bin/env python3

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Proc:
    pid: int
    ppid: int
    args: str


def _run_ps(user: str) -> list[Proc]:
    # pid,ppid,args (args can contain spaces)
    result = subprocess.run(
        ['ps', '-u', user, '-o', 'pid=,ppid=,args='],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"ps failed ({result.returncode}): {result.stderr.strip()}")

    procs: list[Proc] = []
    for raw in result.stdout.splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = line.split(None, 2)
        if len(parts) < 3:
            continue
        try:
            pid = int(parts[0])
            ppid = int(parts[1])
        except ValueError:
            continue
        args = parts[2]
        procs.append(Proc(pid=pid, ppid=ppid, args=args))
    return procs


def _is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        # Can't check; assume alive.
        return True


def _send_signal(pids: Iterable[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            continue
        except PermissionError:
            continue


def _stop_ros2_daemon() -> None:
    try:
        subprocess.run(
            ['ros2', 'daemon', 'stop'],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hard-stop helper: kills ROS 2 / Gazebo / RViz processes for the current user.",
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Only print matched processes; do not send signals.',
    )
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=2.0,
        help='Seconds to wait between SIGINT/SIGTERM/SIGKILL escalation steps.',
    )
    args = parser.parse_args()

    user = os.environ.get('USER') or ''
    if not user:
        print("[rovi_stop] ERROR: USER is not set.", file=sys.stderr)
        return 2

    # Match processes by cmdline. This is intentionally aggressive for "hard stop".
    patterns = [
        r'\bros2\b',
        r'\brviz2\b',
        r'\bgz\s+sim\b',
        r'\bgzserver\b',
        r'\bgzclient\b',
        r'\bgazebo\b',
        r'\bign\s+gazebo\b',
        r'\bros_gz_bridge\b',
        r'\bparameter_bridge\b',
        r'\bros_gz_sim\b',
        r'\bslam_toolbox\b',
        r'\bnav2_',
        r'\brobot_state_publisher\b',
        r'\bjoint_state_publisher\b',
        r'\bekf_node\b',
        r'\btwist_mux\b',
        r'\bteleop_',
        r'\bjoy_node\b',
        r'\brplidar\b',
        r'\brosmaster_driver\b',
        r'\brovi_',
    ]
    matchers = [re.compile(p, flags=re.IGNORECASE) for p in patterns]

    try:
        procs = _run_ps(user)
    except Exception as e:
        print(f"[rovi_stop] ERROR: {e}", file=sys.stderr)
        return 2

    self_pid = os.getpid()
    targets: list[Proc] = []
    for proc in procs:
        if proc.pid == self_pid:
            continue
        if any(m.search(proc.args) for m in matchers):
            targets.append(proc)

    if not targets:
        _stop_ros2_daemon()
        print("[rovi_stop] No ROS/Gazebo processes found.")
        return 0

    targets_sorted = sorted(targets, key=lambda p: p.pid)
    print(f"[rovi_stop] Matched {len(targets_sorted)} processes (user={user}).")
    for proc in targets_sorted:
        print(f"  {proc.pid:<7} {proc.args}")

    if args.dry_run:
        return 0

    pids = [p.pid for p in targets_sorted]

    # Best effort to stop the ROS graph cache early (ignore failures).
    _stop_ros2_daemon()

    for sig, label in [
        (signal.SIGINT, "SIGINT"),
        (signal.SIGTERM, "SIGTERM"),
        (signal.SIGKILL, "SIGKILL"),
    ]:
        _send_signal(pids, sig)
        time.sleep(max(0.0, float(args.timeout_sec)))

        still_alive = [pid for pid in pids if _is_alive(pid)]
        if not still_alive:
            print(f"[rovi_stop] All matched processes stopped after {label}.")
            return 0
        pids = still_alive
        print(f"[rovi_stop] Still alive after {label}: {len(pids)}")

    if pids:
        print(f"[rovi_stop] WARNING: Some processes could not be killed: {pids}", file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

