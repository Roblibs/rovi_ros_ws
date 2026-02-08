from __future__ import annotations

import subprocess

import psutil


def systemd_is_active(service_name: str) -> str:
    unit = str(service_name).strip()
    if not unit:
        return "unknown"
    if not unit.endswith(".service"):
        unit = unit + ".service"

    try:
        cp = subprocess.run(
            ["systemctl", "is-active", unit],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.5,
        )
    except Exception:  # noqa: BLE001
        return "unknown"

    out = (cp.stdout or "").strip()
    if out:
        return out
    err = (cp.stderr or "").strip()
    return err or "unknown"


def process_is_running(process_name: str, *, service: str | None) -> bool:
    needle = str(process_name).strip()
    if not needle:
        return False

    if service:
        unit = str(service).strip()
        if not unit.endswith(".service"):
            unit = unit + ".service"
        try:
            cp = subprocess.run(
                ["systemctl", "show", unit, "-p", "ControlGroup", "--value"],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.5,
            )
        except Exception:  # noqa: BLE001
            return False

        cgroup = (cp.stdout or "").strip()
        if not cgroup:
            return False

        procs_path = f"/sys/fs/cgroup{cgroup}/cgroup.procs"
        try:
            with open(procs_path, "r", encoding="utf-8") as f:
                pids_text = f.read()
        except OSError:
            return False

        for line in pids_text.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                pid = int(line)
            except ValueError:
                continue
            try:
                proc = psutil.Process(pid)
                cmdline = " ".join(proc.cmdline())
            except Exception:  # noqa: BLE001
                continue
            if needle in cmdline:
                return True
        return False

    for proc in psutil.process_iter(["cmdline"]):
        try:
            cmdline_list = proc.info.get("cmdline") or []
            cmdline = " ".join(cmdline_list)
        except Exception:  # noqa: BLE001
            continue
        if needle in cmdline:
            return True
    return False

