#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class CmdResult:
    name: str
    argv: list[str]
    started_at: str
    ended_at: str
    returncode: int
    stdout: str
    stderr: str
    duration_ms: int
    out_path: str


def _now_local_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _write_text(path: Path, data: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(data, encoding="utf-8", errors="replace")


def _write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _shlex_join(argv: Iterable[str]) -> str:
    # Avoid importing shlex just for join if not needed; on Python 3.11+ it's available anyway.
    try:
        import shlex

        return shlex.join(list(argv))
    except Exception:
        return " ".join(argv)


def _run_cmd(name: str, argv: list[str], out_path: Path) -> CmdResult:
    started_at = _now_local_iso()
    t0 = time.monotonic()
    try:
        p = subprocess.run(
            argv,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError as exc:
        dt_ms = int((time.monotonic() - t0) * 1000)
        stdout = ""
        stderr = f"{exc}\n"
        _write_text(out_path, stderr)
        return CmdResult(
            name=name,
            argv=argv,
            started_at=started_at,
            ended_at=_now_local_iso(),
            returncode=127,
            stdout=stdout,
            stderr=stderr,
            duration_ms=dt_ms,
            out_path=str(out_path),
        )

    dt_ms = int((time.monotonic() - t0) * 1000)
    ended_at = _now_local_iso()
    text_out = p.stdout
    if p.stderr:
        text_out += ("\n" if text_out and not text_out.endswith("\n") else "") + p.stderr
    _write_text(out_path, text_out)
    return CmdResult(
        name=name,
        argv=argv,
        started_at=started_at,
        ended_at=ended_at,
        returncode=int(p.returncode),
        stdout=p.stdout,
        stderr=p.stderr,
        duration_ms=dt_ms,
        out_path=str(out_path),
    )


def _timeout_available() -> bool:
    return shutil.which("timeout") is not None


def _capture_cmd(name: str, argv: list[str], seconds: int, out_path: Path) -> CmdResult:
    seconds = int(seconds)
    if seconds <= 0:
        raise ValueError("seconds must be > 0")

    started_at = _now_local_iso()
    base_argv = argv
    if _timeout_available():
        base_argv = ["timeout", str(seconds), *argv]

    t0 = time.monotonic()
    try:
        p = subprocess.Popen(
            base_argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError as exc:
        dt_ms = int((time.monotonic() - t0) * 1000)
        stderr = f"{exc}\n"
        _write_text(out_path, stderr)
        return CmdResult(
            name=name,
            argv=base_argv,
            started_at=started_at,
            ended_at=_now_local_iso(),
            returncode=127,
            stdout="",
            stderr=stderr,
            duration_ms=dt_ms,
            out_path=str(out_path),
        )

    try:
        stdout, stderr = p.communicate(timeout=seconds + 5)
    except subprocess.TimeoutExpired:
        p.terminate()
        try:
            stdout, stderr = p.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            p.kill()
            stdout, stderr = p.communicate()

    dt_ms = int((time.monotonic() - t0) * 1000)
    ended_at = _now_local_iso()
    text_out = stdout
    if stderr:
        text_out += ("\n" if text_out and not text_out.endswith("\n") else "") + stderr
    _write_text(out_path, text_out)
    return CmdResult(
        name=name,
        argv=base_argv,
        started_at=started_at,
        ended_at=ended_at,
        returncode=int(p.returncode or 0),
        stdout=stdout,
        stderr=stderr,
        duration_ms=dt_ms,
        out_path=str(out_path),
    )


def _default_display_dev() -> str:
    return "/dev/robot_display"


def _tty_name_from_devnode(devnode: str) -> str | None:
    base = os.path.basename(devnode)
    if base.startswith("tty"):
        return base
    return None


def _sysfs_tty_device_path(tty_name: str) -> Path | None:
    p = Path("/sys/class/tty") / tty_name / "device"
    try:
        resolved = p.resolve(strict=True)
    except FileNotFoundError:
        return None
    return resolved


def _find_usb_device_ancestor(sysfs_dev: Path) -> Path | None:
    cur = sysfs_dev
    for _ in range(12):
        if (cur / "busnum").is_file() and (cur / "devnum").is_file():
            return cur
        if cur.parent == cur:
            break
        cur = cur.parent
    return None


def _usb_busnum_for_tty(tty_name: str) -> int | None:
    sysfs_dev = _sysfs_tty_device_path(tty_name)
    if not sysfs_dev:
        return None
    usb_dev = _find_usb_device_ancestor(sysfs_dev)
    if not usb_dev:
        return None
    try:
        return int((usb_dev / "busnum").read_text(encoding="utf-8").strip())
    except Exception:
        return None


def _safe_relpath(path: Path, base: Path) -> str:
    try:
        return str(path.relative_to(base))
    except Exception:
        return str(path)


def _write_blocking_test(devnode: str, *, line_bytes: int, iters: int, sleep_s: float, out_path: Path) -> None:
    import os as _os

    line_bytes = int(line_bytes)
    iters = int(iters)
    sleep_s = float(sleep_s)
    if line_bytes <= 0:
        raise ValueError("line_bytes must be > 0")
    if iters <= 0:
        raise ValueError("iters must be > 0")

    msg = (b"X" * line_bytes) + b"\n"
    lines = []
    lines.append(f"dev={devnode}\n")
    lines.append(f"line_bytes={line_bytes} iters={iters} sleep_s={sleep_s}\n")
    lines.append("iter write_ms\n")

    try:
        fd = _os.open(devnode, _os.O_WRONLY | _os.O_NOCTTY)
    except Exception as exc:
        _write_text(out_path, f"failed to open {devnode}: {exc}\n")
        return

    try:
        for i in range(iters):
            t0 = time.monotonic()
            try:
                _os.write(fd, msg)
            except Exception as exc:
                lines.append(f"{i} ERROR:{exc}\n")
                break
            dt_ms = (time.monotonic() - t0) * 1000.0
            lines.append(f"{i} {dt_ms:.3f}\n")
            time.sleep(sleep_s)
    finally:
        try:
            _os.close(fd)
        except Exception:
            pass

    _write_text(out_path, "".join(lines))


def _ensure_usbmon_ready(raw_dir: Path) -> list[CmdResult]:
    results: list[CmdResult] = []
    # Best-effort: these may fail without root; capture outputs anyway.
    results.append(_run_cmd("modprobe_usbmon", ["modprobe", "usbmon"], _raw_path(raw_dir, "usbmon_modprobe")))
    results.append(
        _run_cmd(
            "mount_debugfs",
            ["mount", "-t", "debugfs", "none", "/sys/kernel/debug"],
            _raw_path(raw_dir, "usbmon_mount_debugfs"),
        )
    )
    results.append(_run_cmd("ls_usbmon_dir", ["ls", "-la", "/sys/kernel/debug/usb/usbmon"], _raw_path(raw_dir, "usbmon_ls")))
    return results


def _require_display_dev(display_dev: str) -> str:
    display_dev = str(display_dev).strip()
    if not display_dev:
        raise RuntimeError("Missing --display-dev")
    if not os.path.exists(display_dev):
        raise RuntimeError(f"Missing display device: {display_dev} (expected stable symlink)")
    return display_dev


def _raw_path(raw_dir: Path, name: str) -> Path:
    stamp = datetime.now().strftime("%H%M%S")
    safe = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in name)
    return raw_dir / f"{stamp}_{safe}.txt"


def _snapshot(raw_dir: Path, display_dev: str) -> list[CmdResult]:
    results: list[CmdResult] = []
    results.append(_run_cmd("lsusb_t", ["lsusb", "-t"], _raw_path(raw_dir, "lsusb_t")))
    results.append(_run_cmd("lsusb", ["lsusb"], _raw_path(raw_dir, "lsusb")))
    results.append(_run_cmd("usb_devices", ["usb-devices"], _raw_path(raw_dir, "usb-devices")))
    results.append(_run_cmd("uname", ["uname", "-a"], _raw_path(raw_dir, "uname")))
    results.append(_run_cmd("lspci", ["lspci", "-nn"], _raw_path(raw_dir, "lspci-nn")))

    results.append(_run_cmd("ls_display_dev", ["ls", "-la", display_dev], _raw_path(raw_dir, "display_dev_ls")))
    results.append(_run_cmd("udev_path", ["udevadm", "info", "-q", "path", "-n", display_dev], _raw_path(raw_dir, "udev_path")))
    results.append(
        _run_cmd(
            "udev_props",
            ["udevadm", "info", "-q", "property", "-n", display_dev],
            _raw_path(raw_dir, "udev_props"),
        )
    )

    tty_name = _tty_name_from_devnode(os.path.realpath(display_dev) or display_dev) or ""
    if tty_name:
        sysfs = _sysfs_tty_device_path(tty_name)
        if sysfs:
            results.append(
                _run_cmd(
                    "sysfs_tty_readlink",
                    ["readlink", "-f", f"/sys/class/tty/{tty_name}/device"],
                    _raw_path(raw_dir, "sysfs_tty_path"),
                )
            )
            # Power-management hints (read-only). Some nodes may not exist; use a shell to allow globs? Keep simple.
            power_dir = sysfs / "power"
            lines = []
            for leaf in ("control", "autosuspend_delay_ms", "runtime_status", "runtime_active_time", "runtime_suspended_time"):
                p = power_dir / leaf
                if p.is_file():
                    try:
                        lines.append(f"{p}: {p.read_text(encoding='utf-8', errors='replace').strip()}\n")
                    except Exception as exc:
                        lines.append(f"{p}: ERROR {exc}\n")
                else:
                    lines.append(f"{p}: (missing)\n")
            sysfs_power_path = _raw_path(raw_dir, "sysfs_power")
            sysfs_power_text = "".join(lines)
            _write_text(sysfs_power_path, sysfs_power_text)
            now = _now_local_iso()
            results.append(
                CmdResult(
                    name="sysfs_power",
                    argv=["(sysfs)"],
                    started_at=now,
                    ended_at=now,
                    returncode=0,
                    stdout=sysfs_power_text,
                    stderr="",
                    duration_ms=0,
                    out_path=str(sysfs_power_path),
                )
            )
    else:
        sysfs_power_path = _raw_path(raw_dir, "sysfs_power")
        sysfs_power_text = f"Could not derive tty name from {display_dev}\n"
        _write_text(sysfs_power_path, sysfs_power_text)
        now = _now_local_iso()
        results.append(
            CmdResult(
                name="sysfs_power",
                argv=["(sysfs)"],
                started_at=now,
                ended_at=now,
                returncode=0,
                stdout=sysfs_power_text,
                stderr="",
                duration_ms=0,
                out_path=str(sysfs_power_path),
            )
        )

    results.append(
        _run_cmd(
            "interrupts_usb",
            ["bash", "-lc", "grep -Ei 'xhci|usb' /proc/interrupts 2>/dev/null || true"],
            _raw_path(raw_dir, "interrupts_usb"),
        )
    )
    results.append(
        _run_cmd(
            "softirqs",
            ["bash", "-lc", "cat /proc/softirqs 2>/dev/null | egrep -n -A2 'NET_RX|TASKLET|HI|TIMER' || true"],
            _raw_path(raw_dir, "softirqs"),
        )
    )
    return results


def _percentile(sorted_values: list[float], pct: float) -> float | None:
    if not sorted_values:
        return None
    if pct <= 0:
        return float(sorted_values[0])
    if pct >= 100:
        return float(sorted_values[-1])
    idx = int(round((pct / 100.0) * (len(sorted_values) - 1)))
    return float(sorted_values[idx])


def _summarize_write_test(path: Path) -> str:
    if not path.is_file():
        return "write_test: (missing)"
    vals: list[float] = []
    err = 0
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if not line or line.startswith(("dev=", "line_bytes=", "iter ")):
            continue
        if "ERROR:" in line:
            err += 1
            continue
        parts = line.split()
        if len(parts) != 2:
            continue
        try:
            vals.append(float(parts[1]))
        except ValueError:
            continue
    vals.sort()
    if not vals and err:
        return f"write_test: errors={err} (no samples)"
    if not vals:
        return "write_test: (no samples)"

    max_ms = vals[-1]
    p50 = _percentile(vals, 50) or 0.0
    p95 = _percentile(vals, 95) or 0.0
    spikes_20 = sum(1 for v in vals if v > 20.0)
    spikes_100 = sum(1 for v in vals if v > 100.0)
    return f"write_test: n={len(vals)} p50={p50:.3f}ms p95={p95:.3f}ms max={max_ms:.3f}ms spikes>20ms={spikes_20} spikes>100ms={spikes_100} errors={err}"


def _summarize_log_keywords(path: Path) -> str:
    if not path.is_file():
        return f"{path.name}: (missing)"
    text = path.read_text(encoding="utf-8", errors="replace").lower()
    keys = [
        "reset",
        "disconnect",
        "over-current",
        "suspend",
        "resume",
        "cdc_acm",
        "ttyacm",
        "xhci",
        "error",
    ]
    counts = {k: text.count(k) for k in keys}
    top = ", ".join(f"{k}={counts[k]}" for k in keys)
    return f"{path.name}: {top}"


def _summarize_usbmon_sniff(path: Path) -> str:
    if not path.is_file():
        return "usbmon: (missing)"
    text = path.read_text(encoding="utf-8", errors="replace")
    if "permission denied" in text.lower():
        return "usbmon: permission denied (need sudo + debugfs)"
    lines = [ln for ln in text.splitlines() if ln.strip()]
    return f"usbmon: lines={len(lines)} bytes={len(text.encode('utf-8', errors='replace'))}"


def _artifact_table(bundle_dir: Path, results: list[CmdResult]) -> str:
    rows = []
    rows.append("| Name | rc | ms | started | cmd | raw |")
    rows.append("|---|---:|---:|---|---|---|")
    for res in results:
        rel = _safe_relpath(Path(res.out_path), bundle_dir)
        raw_link = f"[{rel}]({rel})"
        cmd_s = _shlex_join(res.argv)
        rows.append(f"| `{res.name}` | `{res.returncode}` | `{res.duration_ms}` | `{res.started_at}` | `{cmd_s}` | {raw_link} |")
    return "\n".join(rows) + "\n"


def _render_report(
    bundle_dir: Path,
    *,
    title: str,
    now_iso: str,
    display_dev: str,
    results: list[CmdResult],
    extra_sections: list[tuple[str, str]] | None = None,
) -> None:
    lines: list[str] = []
    lines.append(f"# {title}\n\n")
    lines.append("## Run\n\n")
    lines.append(f"- started_at: `{now_iso}`\n")
    lines.append(f"- host: `{socket.gethostname()}`\n")
    lines.append(f"- kernel: `{platform.release()}`\n")
    lines.append(f"- display_dev: `{display_dev}`\n")
    lines.append(f"- bundle_dir: `{bundle_dir}`\n\n")

    if extra_sections:
        for h, body in extra_sections:
            lines.append(f"## {h}\n\n")
            lines.append(body.rstrip() + "\n\n")

    lines.append("## Quick summary\n\n")
    # Best-effort derived summaries from known artifacts (if present).
    write_test = next((Path(r.out_path) for r in results if r.name == "write_test"), None)
    if write_test:
        lines.append(f"- {_summarize_write_test(write_test)}\n")
    dmesg_watch = next((Path(r.out_path) for r in results if r.name == "dmesg_watch"), None)
    if dmesg_watch:
        lines.append(f"- {_summarize_log_keywords(dmesg_watch)}\n")
    kernel_recent = next((Path(r.out_path) for r in results if r.name == "kernel_log_recent"), None)
    if kernel_recent:
        lines.append(f"- {_summarize_log_keywords(kernel_recent)}\n")
    usbmon_sniff = next((Path(r.out_path) for r in results if r.name == "usbmon_sniff_2s"), None)
    if usbmon_sniff:
        lines.append(f"- {_summarize_usbmon_sniff(usbmon_sniff)}\n")

    lines.append("\n## Artifacts\n\n")
    lines.append("Open `raw/` files for full command output.\n\n")
    lines.append(_artifact_table(bundle_dir, results))

    lines.append("\n## Pasteable bundle reference\n\n")
    lines.append("If you need to share one thing, share this file:\n\n")
    lines.append(f"- `report.md`\n")
    lines.append("If deeper analysis is needed, also attach:\n\n")
    lines.append("- `meta.json`\n")
    lines.append("- `raw/` (or selected `raw/*.txt`)\n")

    _write_text(bundle_dir / "report.md", "".join(lines))


def _usbmon_check(raw_dir: Path, display_dev: str) -> tuple[list[CmdResult], int | None]:
    results: list[CmdResult] = []
    results.extend(_ensure_usbmon_ready(raw_dir))

    tty_name = _tty_name_from_devnode(os.path.realpath(display_dev) or display_dev)
    if not tty_name:
        err_path = _raw_path(raw_dir, "usbmon_error")
        err_text = f"Could not derive tty name from {display_dev}\n"
        _write_text(err_path, err_text)
        now = _now_local_iso()
        results.append(
            CmdResult(
                name="usbmon_error",
                argv=["(internal)"],
                started_at=now,
                ended_at=now,
                returncode=1,
                stdout=err_text,
                stderr="",
                duration_ms=0,
                out_path=str(err_path),
            )
        )
        return results, None

    bus = _usb_busnum_for_tty(tty_name)
    if not bus:
        err_path = _raw_path(raw_dir, "usbmon_error")
        err_text = f"Could not detect USB bus for tty {tty_name}\n"
        _write_text(err_path, err_text)
        now = _now_local_iso()
        results.append(
            CmdResult(
                name="usbmon_error",
                argv=["(internal)"],
                started_at=now,
                ended_at=now,
                returncode=1,
                stdout=err_text,
                stderr="",
                duration_ms=0,
                out_path=str(err_path),
            )
        )
        return results, None

    usbmon_path = f"/sys/kernel/debug/usb/usbmon/{bus}u"
    results.append(_run_cmd("usbmon_path_stat", ["bash", "-lc", f"ls -la {usbmon_path} 2>/dev/null || true"], _raw_path(raw_dir, f"usbmon_bus{bus}_stat")))
    # 2-second sanity read: if usbmon is working, this should produce some traffic once devices are active.
    results.append(_capture_cmd("usbmon_sniff_2s", ["cat", usbmon_path], 2, _raw_path(raw_dir, f"usbmon_bus{bus}_sniff_2s")))
    return results, bus


def _main() -> int:
    parser = argparse.ArgumentParser(
        description="Create timestamped diagnostics bundles for USB display vs camera issues (Pi 5)."
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "--output-root",
            default="output/reports",
            help="Base folder for report bundles (recommended: under gitignored output/).",
        )
        p.add_argument(
            "--display-dev",
            default=_default_display_dev(),
            help="Display devnode (expected stable symlink: /dev/robot_display).",
        )
        p.add_argument("--label", default="", help="Optional label appended to the run directory.")

    p_snapshot = sub.add_parser("snapshot", help="Collect a static snapshot (topology + device state).")
    add_common(p_snapshot)

    p_usbmon = sub.add_parser("usbmon", help="Quick check that usbmon is present/readable for the display bus.")
    add_common(p_usbmon)

    p_capture = sub.add_parser("capture", help="Capture logs for a time window while you start the camera pipeline.")
    add_common(p_capture)
    p_capture.add_argument("--seconds", type=int, default=5, help="Capture duration in seconds.")
    bool_opt = getattr(argparse, "BooleanOptionalAction", None)
    if bool_opt is None:  # pragma: no cover - very old Python fallback
        p_capture.add_argument("--usbmon", action="store_true", default=True, help="Capture usbmon for the display bus (default: enabled; needs root).")
        p_capture.add_argument("--write-test", action="store_true", default=True, help="Run a serial write blocking test (default: enabled).")
    else:
        p_capture.add_argument("--usbmon", action=bool_opt, default=True, help="Capture usbmon for the display bus (default: enabled; needs root).")
        p_capture.add_argument("--write-test", action=bool_opt, default=True, help="Run a serial write blocking test (default: enabled).")
    p_capture.add_argument("--write-line-bytes", type=int, default=306, help="Write-test payload bytes (excluding newline).")
    p_capture.add_argument("--write-iters", type=int, default=200, help="Write-test iterations.")
    p_capture.add_argument("--write-sleep", type=float, default=0.05, help="Write-test sleep between writes (seconds).")

    args = parser.parse_args()
    now_iso = _now_local_iso()

    output_root = Path(args.output_root).expanduser()
    day = datetime.now().strftime("%Y-%m-%d")
    clock = datetime.now().strftime("%H%M%S")
    label = ""
    if args.label:
        label = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in str(args.label))
        label = f"_{label}"
    bundle_dir = output_root / day / str(args.cmd) / f"{clock}{label}"
    raw_dir = bundle_dir / "raw"
    try:
        raw_dir.mkdir(parents=True, exist_ok=True)
    except PermissionError as exc:
        print(f"[rovi_report] ERROR: cannot create reports dir: {raw_dir} ({exc})", file=sys.stderr)
        print(f"[rovi_report] Hint: fix ownership: sudo chown -R $(id -u):$(id -g) {output_root}", file=sys.stderr)
        return 2

    # Print the bundle directory immediately so callers always know where to look,
    # even if a later step fails.
    print(bundle_dir, flush=True)

    results: list[CmdResult] = []
    title = "USB display vs camera diagnostics"

    def maybe_chown_outputs() -> None:
        if os.geteuid() != 0:
            return
        sudo_uid = os.environ.get("SUDO_UID")
        sudo_gid = os.environ.get("SUDO_GID")
        if not sudo_uid or not sudo_gid:
            return
        try:
            uid = int(sudo_uid)
            gid = int(sudo_gid)
        except ValueError:
            return

        def safe_chown(p: Path) -> None:
            try:
                os.chown(p, uid, gid)
            except Exception:
                pass

        # Ensure the output_root path stays writable for the non-root user even
        # when capture runs under sudo.
        for p in (output_root, output_root / day, output_root / day / str(args.cmd), bundle_dir):
            if p.exists():
                safe_chown(p)

        for root, dirs, files in os.walk(bundle_dir):
            safe_chown(Path(root))
            for d in dirs:
                safe_chown(Path(root) / d)
            for f in files:
                safe_chown(Path(root) / f)

    try:
        display_dev = _require_display_dev(args.display_dev)
    except Exception as exc:
        err_path = _raw_path(raw_dir, "error")
        _write_text(err_path, f"{type(exc).__name__}: {exc}\n")
        now = _now_local_iso()
        results.append(
            CmdResult(
                name="error",
                argv=["(startup)"],
                started_at=now,
                ended_at=now,
                returncode=2,
                stdout=f"{type(exc).__name__}: {exc}\n",
                stderr="",
                duration_ms=0,
                out_path=str(err_path),
            )
        )
        meta = {
            "started_at": now_iso,
            "bundle_dir": str(bundle_dir),
            "output_root": str(output_root),
            "display_dev": str(args.display_dev),
            "cmd": args.cmd,
            "argv": sys.argv,
            "uid": os.getuid(),
            "euid": os.geteuid(),
            "python": sys.version,
            "error": str(exc),
        }
        _write_json(bundle_dir / "meta.json", meta)
        _render_report(bundle_dir, title=title + " (error)", now_iso=now_iso, display_dev=str(args.display_dev), results=results)
        maybe_chown_outputs()
        return 2

    meta: dict[str, object] = {
        "started_at": now_iso,
        "bundle_dir": str(bundle_dir),
        "output_root": str(output_root),
        "display_dev": display_dev,
        "cmd": args.cmd,
        "argv": sys.argv,
        "uid": os.getuid(),
        "euid": os.geteuid(),
        "python": sys.version,
    }

    if args.cmd == "snapshot":
        results.extend(_snapshot(raw_dir, display_dev))
        _write_json(bundle_dir / "meta.json", meta)
        _render_report(bundle_dir, title=title + " (snapshot)", now_iso=now_iso, display_dev=display_dev, results=results)
        maybe_chown_outputs()
        return 0

    if args.cmd == "usbmon":
        usbmon_results, bus = _usbmon_check(raw_dir, display_dev)
        results.extend(usbmon_results)
        meta["usbmon_bus"] = bus
        _write_json(bundle_dir / "meta.json", meta)
        _render_report(bundle_dir, title=title + " (usbmon check)", now_iso=now_iso, display_dev=display_dev, results=results)
        maybe_chown_outputs()
        return 0

    if args.cmd == "capture":
        seconds = int(args.seconds)
        results.extend(_snapshot(raw_dir, display_dev))

        extra: list[tuple[str, str]] = []
        extra.append(
            (
                "How to reproduce (during capture window)",
                f"Start your camera pipeline in another terminal now. Capture window: {seconds}s.\n"
                "Suggested commands (examples):\n"
                "- `camera` (rovi_env.sh helper)\n"
                "- `ros2 launch rovi_bringup camera.launch.py robot_mode:=real`\n",
            )
        )

        procs: list[tuple[str, subprocess.Popen[str], Path, str, list[str]]] = []
        started_mono: dict[str, float] = {}

        # Kernel log tail during the capture window.
        dmesg_argv = (["timeout", str(seconds), "dmesg", "-Tw"] if _timeout_available() else ["dmesg", "-Tw"])
        started_mono["dmesg_watch"] = time.monotonic()
        dmesg_started_at = _now_local_iso()
        procs.append(
            (
                "dmesg_watch",
                subprocess.Popen(dmesg_argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True),
                _raw_path(raw_dir, "dmesg_watch"),
                dmesg_started_at,
                [str(x) for x in dmesg_argv],
            )
        )

        usbmon_bus = None
        if args.usbmon:
            usbmon_results, usbmon_bus = _usbmon_check(raw_dir, display_dev)
            results.extend(usbmon_results)
            if usbmon_bus:
                usbmon_path = f"/sys/kernel/debug/usb/usbmon/{usbmon_bus}u"
                started_mono["usbmon_watch"] = time.monotonic()
                usbmon_started_at = _now_local_iso()
                usbmon_argv = (["timeout", str(seconds), "cat", usbmon_path] if _timeout_available() else ["cat", usbmon_path])
                procs.append(
                    (
                        "usbmon_watch",
                        subprocess.Popen(
                            usbmon_argv,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            text=True,
                        ),
                        _raw_path(raw_dir, f"usbmon_bus{usbmon_bus}"),
                        usbmon_started_at,
                        [str(x) for x in usbmon_argv],
                    )
                )

        if args.write_test:
            # Run in-process while watchers are running.
            wt_started = _now_local_iso()
            wt_path = _raw_path(raw_dir, "write_test")
            _write_blocking_test(
                display_dev,
                line_bytes=int(args.write_line_bytes),
                iters=int(args.write_iters),
                sleep_s=float(args.write_sleep),
                out_path=wt_path,
            )
            wt_ended = _now_local_iso()
            if wt_path.is_file():
                wt_text = wt_path.read_text(encoding="utf-8", errors="replace")
                results.append(
                    CmdResult(
                        name="write_test",
                        argv=["python", "-c", "(serial write test)"],
                        started_at=wt_started,
                        ended_at=wt_ended,
                        returncode=0,
                        stdout=wt_text,
                        stderr="",
                        duration_ms=0,
                        out_path=str(wt_path),
                    )
                )

        # If timeout(1) isn't available, manually stop after the window.
        if not _timeout_available():
            time.sleep(seconds)
            for _name, proc, _path, _started_at, _argv in procs:
                try:
                    proc.terminate()
                except Exception:
                    pass

        for name, proc, path, started_at, argv in procs:
            try:
                stdout, stderr = proc.communicate(timeout=(seconds + 5) if _timeout_available() else 5)
            except subprocess.TimeoutExpired:
                proc.kill()
                stdout, stderr = proc.communicate()
            ended_at = _now_local_iso()
            t_start = started_mono.get(name)
            dt_ms = int(((time.monotonic() - t_start) if t_start else 0.0) * 1000)
            text_out = stdout
            if stderr:
                text_out += ("\n" if text_out and not text_out.endswith("\n") else "") + stderr
            _write_text(path, text_out)
            results.append(
                CmdResult(
                    name=name,
                    argv=argv,
                    started_at=started_at,
                    ended_at=ended_at,
                    returncode=int(proc.returncode or 0),
                    stdout=stdout,
                    stderr=stderr,
                    duration_ms=dt_ms,
                    out_path=str(path),
                )
            )

        # Helpful post-capture summaries.
        results.append(
            _run_cmd(
                "kernel_log_recent",
                ["bash", "-lc", "journalctl -k -b --no-pager -n 400 2>/dev/null || dmesg -T | tail -n 400"],
                _raw_path(raw_dir, "kernel_log_recent"),
            )
        )

        meta["capture_seconds"] = seconds
        meta["usbmon_enabled"] = bool(args.usbmon)
        meta["usbmon_bus"] = usbmon_bus
        meta["write_test_enabled"] = bool(args.write_test)

        _write_json(bundle_dir / "meta.json", meta)
        _render_report(bundle_dir, title=title + " (capture)", now_iso=now_iso, display_dev=display_dev, results=results, extra_sections=extra)
        maybe_chown_outputs()
        return 0

    raise AssertionError("unreachable")


if __name__ == "__main__":
    raise SystemExit(_main())
