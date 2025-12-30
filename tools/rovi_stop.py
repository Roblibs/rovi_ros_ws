#!/usr/bin/env python3

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree


@dataclass(frozen=True)
class Proc:
    pid: int
    ppid: int
    pgid: int
    args: str


def _run_ps(user: str) -> list[Proc]:
    # pid,ppid,pgid,args (args can contain spaces)
    result = subprocess.run(
        ['ps', '-u', user, '-o', 'pid=,ppid=,pgid=,args='],
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
        parts = line.split(None, 3)
        if len(parts) < 4:
            continue
        try:
            pid = int(parts[0])
            ppid = int(parts[1])
            pgid = int(parts[2])
        except ValueError:
            continue
        args = parts[3]
        procs.append(Proc(pid=pid, ppid=ppid, pgid=pgid, args=args))
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


def _send_signal_to_pgids(pgids: Iterable[int], sig: int) -> None:
    for pgid in pgids:
        if pgid <= 0:
            continue
        try:
            os.killpg(pgid, sig)
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


def _workspace_root() -> Path:
    env = os.environ.get('ROVI_ROS_WS_DIR')
    if env:
        return Path(env).expanduser().resolve()
    return Path(__file__).resolve().parents[1]


def _discover_workspace_packages(ws_root: Path) -> set[str]:
    src_dir = ws_root / 'src'
    if not src_dir.is_dir():
        return set()

    packages: set[str] = set()
    for package_xml in src_dir.rglob('package.xml'):
        try:
            root = ElementTree.parse(package_xml).getroot()
        except Exception:
            continue

        name_el = root.find('name')
        if name_el is None:
            continue
        name = (name_el.text or '').strip()
        if not name:
            continue
        packages.add(name)

    return packages


_ROS2_VERB_PKG_RE = re.compile(r'\bros2\b\s+(launch|run)\s+([^\s]+)', flags=re.IGNORECASE)


def _ros2_verb_and_package(cmdline: str) -> tuple[str | None, str | None]:
    match = _ROS2_VERB_PKG_RE.search(cmdline)
    if not match:
        return None, None
    return match.group(1).lower(), match.group(2)


def _looks_like_ros_or_gz(cmdline: str) -> bool:
    if '--ros-args' in cmdline:
        return True

    lower = cmdline.lower()
    if re.search(r'\bros2\b', lower):
        return True
    if re.search(r'\brviz2\b', lower):
        return True
    # Direct node executables (ament/colcon install layout), e.g.:
    #   <ws>/install/<pkg>/lib/<pkg>/<exe>
    #   /opt/ros/<distro>/lib/<pkg>/<exe>
    if re.search(r'/(?:install/[^/\s]+|opt/ros/[^/\s]+)/(?:lib|libexec)/[a-z0-9_]+/', lower):
        return True
    if 'gz sim' in lower:
        return True
    if re.search(r'\bgazebo\b', lower):
        return True
    if 'ign gazebo' in lower:
        return True
    if 'ros_gz_' in lower:
        return True
    if 'parameter_bridge' in lower:
        return True

    # Common node executable names.
    markers = (
        'robot_state_publisher',
        'joint_state_publisher',
        'slam_toolbox',
        'nav2_',
        'ekf_node',
        'twist_mux',
        'teleop_',
        'joy_node',
        'rplidar',
    )
    return any(m in lower for m in markers)


def _is_rovi_ros2_root(proc: Proc, workspace_packages: set[str]) -> bool:
    verb, pkg = _ros2_verb_and_package(proc.args)
    if verb not in {'launch', 'run'} or not pkg:
        return False
    return pkg in workspace_packages


def _is_rovi_owned_proc(proc: Proc, ws_root: Path, workspace_packages: set[str]) -> bool:
    ws_root_str = str(ws_root)
    if ws_root_str and ws_root_str in proc.args:
        return True

    _, pkg = _ros2_verb_and_package(proc.args)
    if pkg and pkg in workspace_packages:
        return True

    # Fallback for direct executables that include the package prefix in their argv.
    if re.search(r'\brovi_[a-z0-9_]+\b', proc.args, flags=re.IGNORECASE):
        return True
    if re.search(r'\brosmaster_driver\b', proc.args, flags=re.IGNORECASE):
        return True

    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Stop helper: by default, stops ROS 2 / Gazebo / RViz processes that belong to this workspace. "
            "Use --all for the previous aggressive behavior."
        ),
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Only print matched processes; do not send signals.',
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='Kill any ROS/Gazebo/RViz processes for the current user (very aggressive).',
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

    ws_root = _workspace_root()
    workspace_packages = _discover_workspace_packages(ws_root)

    # Match processes by cmdline. In default mode this is scoped to this workspace.
    aggressive_patterns = [
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
    aggressive_matchers = [re.compile(p, flags=re.IGNORECASE) for p in aggressive_patterns]

    try:
        procs = _run_ps(user)
    except Exception as e:
        print(f"[rovi_stop] ERROR: {e}", file=sys.stderr)
        return 2

    self_pid = os.getpid()
    self_pgid = os.getpgrp()

    rovi_pgids: set[int] = set()
    if not args.all and workspace_packages:
        for proc in procs:
            if proc.pid == self_pid:
                continue
            if _is_rovi_ros2_root(proc, workspace_packages):
                if proc.pgid > 0 and proc.pgid != self_pgid:
                    rovi_pgids.add(proc.pgid)

    target_by_pid: dict[int, Proc] = {}
    for proc in procs:
        if proc.pid == self_pid:
            continue

        if args.all:
            if any(m.search(proc.args) for m in aggressive_matchers):
                target_by_pid[proc.pid] = proc
            continue

        # Default mode: stop only this workspace's processes.
        if proc.pgid in rovi_pgids:
            target_by_pid[proc.pid] = proc
            continue

        if not _looks_like_ros_or_gz(proc.args):
            continue
        if _is_rovi_owned_proc(proc, ws_root, workspace_packages):
            target_by_pid[proc.pid] = proc

    targets = list(target_by_pid.values())

    if not targets:
        if args.all:
            _stop_ros2_daemon()
        print("[rovi_stop] No matching processes found.")
        return 0

    targets_sorted = sorted(targets, key=lambda p: p.pid)
    mode_label = "ALL (aggressive)" if args.all else "workspace (scoped)"
    print(f"[rovi_stop] Matched {len(targets_sorted)} processes (mode={mode_label}, user={user}).")
    for proc in targets_sorted:
        print(f"  {proc.pid:<7} {proc.args}")

    if args.dry_run:
        return 0

    target_pgids: set[int] = set()
    if args.all:
        target_pgids = set()
    else:
        target_pgids = set(p.pgid for p in targets_sorted if p.pgid in rovi_pgids and p.pgid != self_pgid)

    # Always include all PIDs in targeted process groups (to ensure we wait until everything is gone).
    target_pids: set[int] = set()
    for proc in procs:
        if proc.pid == self_pid:
            continue
        if proc.pid in target_by_pid:
            target_pids.add(proc.pid)
        elif proc.pgid in target_pgids:
            target_pids.add(proc.pid)

    # Best effort to stop the ROS graph cache early (ignore failures) in aggressive mode only.
    if args.all:
        _stop_ros2_daemon()

    for sig, label in [
        (signal.SIGINT, "SIGINT"),
        (signal.SIGTERM, "SIGTERM"),
        (signal.SIGKILL, "SIGKILL"),
    ]:
        _send_signal_to_pgids(target_pgids, sig)
        _send_signal(sorted(target_pids), sig)
        time.sleep(max(0.0, float(args.timeout_sec)))

        still_alive = [pid for pid in sorted(target_pids) if _is_alive(pid)]
        if not still_alive:
            print(f"[rovi_stop] All matched processes stopped after {label}.")
            return 0
        target_pids = set(still_alive)
        print(f"[rovi_stop] Still alive after {label}: {len(target_pids)}")

    if target_pids:
        print(
            f"[rovi_stop] WARNING: Some processes could not be killed: {sorted(target_pids)}",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
