from __future__ import annotations

from dataclasses import dataclass
import re
import subprocess

import psutil


_STACK_RE = re.compile(r"^[a-z0-9][a-z0-9_-]*$")


@dataclass(frozen=True)
class UnitStatus:
    unit: str
    load_state: str
    active_state: str
    sub_state: str
    unit_file_state: str
    result: str
    exec_main_code: int
    exec_main_status: int


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


def stack_to_unit(stack: str, *, unit_prefix: str = "rovi-") -> str:
    name = str(stack).strip().lower()
    if not name or not _STACK_RE.match(name):
        raise ValueError(f"Invalid stack name: {stack!r}")
    if name in {"gateway", "core"}:
        raise ValueError("Refusing to map reserved stack name")
    prefix = str(unit_prefix).strip() or "rovi-"
    if not prefix.endswith("-"):
        # Keep prefix predictable (avoid accidental 'rovi' -> 'rovinav').
        prefix = prefix + "-"
    return f"{prefix}{name}.service"


def get_unit_status(unit: str) -> UnitStatus:
    unit = str(unit).strip()
    if not unit:
        raise ValueError("Empty unit name")
    if not unit.endswith(".service"):
        unit = unit + ".service"

    props = [
        "LoadState",
        "ActiveState",
        "SubState",
        "UnitFileState",
        "Result",
        "ExecMainCode",
        "ExecMainStatus",
    ]

    try:
        cp = subprocess.run(
            ["systemctl", "show", unit, *sum([["-p", p] for p in props], [])],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"Failed to query systemd unit status: {exc}") from exc

    if cp.returncode != 0:
        msg = (cp.stderr or cp.stdout or "").strip() or f"systemctl show failed (rc={cp.returncode})"
        raise RuntimeError(msg)

    values: dict[str, str] = {}
    for line in (cp.stdout or "").splitlines():
        if "=" not in line:
            continue
        k, v = line.split("=", 1)
        values[k.strip()] = v.strip()

    def _get(key: str) -> str:
        return values.get(key, "") or "unknown"

    def _get_int(key: str) -> int:
        raw = values.get(key, "")
        try:
            return int(raw) if raw != "" else 0
        except ValueError:
            return 0

    return UnitStatus(
        unit=unit,
        load_state=_get("LoadState"),
        active_state=_get("ActiveState"),
        sub_state=_get("SubState"),
        unit_file_state=_get("UnitFileState"),
        result=_get("Result"),
        exec_main_code=_get_int("ExecMainCode"),
        exec_main_status=_get_int("ExecMainStatus"),
    )


def control_unit(unit: str, verb: str) -> tuple[bool, str, UnitStatus]:
    """Start/stop/restart a systemd unit (best-effort) and return (ok, message, status)."""
    unit = str(unit).strip()
    verb = str(verb).strip().lower()
    if verb not in {"start", "stop", "restart"}:
        raise ValueError(f"Unsupported verb: {verb}")
    if not unit.endswith(".service"):
        unit = unit + ".service"
    if unit == "rovi-gateway.service":
        raise ValueError("Refusing to control rovi-gateway.service via control API")

    def _unknown_status() -> UnitStatus:
        return UnitStatus(
            unit=unit,
            load_state="unknown",
            active_state="unknown",
            sub_state="unknown",
            unit_file_state="unknown",
            result="unknown",
            exec_main_code=0,
            exec_main_status=0,
        )

    try:
        cp = subprocess.run(
            ["systemctl", verb, "--no-block", unit],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except Exception as exc:  # noqa: BLE001
        try:
            status = get_unit_status(unit)
        except Exception:
            status = _unknown_status()
        return False, f"systemctl {verb} failed: {exc}", status

    ok = cp.returncode == 0
    msg = (cp.stderr or cp.stdout or "").strip()
    if not msg:
        msg = "ok" if ok else f"systemctl {verb} failed (rc={cp.returncode})"

    # Always return fresh status (even if the control command failed).
    try:
        status = get_unit_status(unit)
    except Exception as exc:  # noqa: BLE001
        status = _unknown_status()
        if ok:
            ok = False
            msg = f"{msg}; status query failed: {exc}"
        else:
            msg = f"{msg}; status query failed: {exc}"

    return ok, msg, status


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
