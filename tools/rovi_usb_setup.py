#!/usr/bin/env python3
"""
Create udev rules for ROVI USB devices and assign stable /dev symlinks.

Targets:
  - /dev/robot_control (Rosmaster control board, CH340)
  - /dev/robot_lidar   (RPLidar, CH340)
  - /dev/robot_display (ESP32-S3 display)

This script requires all devices to be attached so it can positively identify
the Rosmaster board via Rosmaster.get_version() and assign the other CH340 as
the lidar. It does not install rules if devices are missing.

Run with sudo:
  sudo python3 tools/rovi_usb_setup.py
"""

from __future__ import annotations

import glob
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

CH340_VENDOR = "1a86"
CH340_PRODUCT = "7523"
DISPLAY_VENDOR = "303a"
DISPLAY_PRODUCT = "1001"

RULE_PATH = "/etc/udev/rules.d/99-rovi-usb.rules"

LEGACY_RULES = {
    "/etc/udev/rules.d/99-robot-display.rules": (
        'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", '
        'MODE:="0666", SYMLINK+="robot_display"\n'
    ),
    "/etc/udev/rules.d/99-rovi-display.rules": (
        'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", '
        'MODE:="0666", SYMLINK+="rovi_display"\n'
    ),
    "/etc/udev/rules.d/99-rosmaster.rules": (
        'SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", '
        'MODE:="0666", SYMLINK+="my_ros_board"\n'
    ),
}


def _ensure_venv_site() -> Optional[str]:
    repo_root = Path(__file__).resolve().parents[1]
    candidates = []
    venv_root = os.environ.get("VIRTUAL_ENV")
    if venv_root:
        candidates.append(Path(venv_root))
    candidates.append(repo_root / ".venv")
    for root in candidates:
        for site in root.glob("lib/python*/site-packages"):
            if site.is_dir():
                sys.path.insert(0, str(site))
                return str(site)
    return None


def _udev_props(devnode: str) -> Tuple[Dict[str, str], Optional[str]]:
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", devnode],
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        return {}, str(exc)
    props: Dict[str, str] = {}
    for line in out.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        props[key] = value
    return props, None


def _by_path_link(devnode: str) -> str:
    base = Path("/dev/serial/by-path")
    if not base.is_dir():
        return ""
    for entry in base.iterdir():
        try:
            if entry.resolve() == Path(devnode):
                return str(entry)
        except FileNotFoundError:
            continue
    return ""


def _scan_devices(dev_glob: str, vendor: str, product: str) -> List[Dict[str, str]]:
    devices: List[Dict[str, str]] = []
    for devnode in sorted(glob.glob(dev_glob)):
        props, err = _udev_props(devnode)
        if err:
            continue
        if props.get("ID_VENDOR_ID") != vendor or props.get("ID_MODEL_ID") != product:
            continue
        devices.append(
            {
                "devnode": devnode,
                "id_vendor": props.get("ID_VENDOR_ID", ""),
                "id_model": props.get("ID_MODEL_ID", ""),
                "id_serial": props.get("ID_SERIAL", ""),
                "id_path": props.get("ID_PATH", ""),
                "id_path_tag": props.get("ID_PATH_TAG", ""),
                "by_path": _by_path_link(devnode),
            }
        )
    return devices


def _sniff_device(devnode: str, *, baudrate: int = 115200, timeout_s: float = 0.2) -> str:
    try:
        import serial  # type: ignore
    except ModuleNotFoundError:
        _ensure_venv_site()
        try:
            import serial  # type: ignore
        except ModuleNotFoundError as exc:
            return f"pyserial missing: {exc}"
    try:
        with serial.Serial(devnode, baudrate=baudrate, timeout=timeout_s) as ser:
            time.sleep(0.05)
            data = ser.read(64)
        if data:
            return f"{len(data)} bytes (hex={data[:16].hex()})"
        return f"no data within {timeout_s:.1f}s"
    except Exception as exc:  # noqa: BLE001 - surface serial failures
        return f"error: {exc}"


def _load_rosmaster():
    try:
        from Rosmaster_Lib import Rosmaster  # type: ignore
    except ModuleNotFoundError:
        site = _ensure_venv_site()
        if site:
            print(f"[setup] Added venv site-packages: {site}")
        from Rosmaster_Lib import Rosmaster  # type: ignore
    _patch_rosmaster_receive(Rosmaster)
    return Rosmaster


def _patch_rosmaster_receive(Rosmaster) -> None:
    if getattr(Rosmaster, "_rovi_receive_patched", False):
        return
    try:
        import serial  # type: ignore
    except ModuleNotFoundError:
        _ensure_venv_site()
        import serial  # type: ignore
    attr = "_Rosmaster__receive_data"
    if not hasattr(Rosmaster, attr):
        return
    original = getattr(Rosmaster, attr)

    def _safe_receive(self):  # noqa: ANN001
        try:
            original(self)
        except (serial.SerialException, OSError):
            return

    setattr(Rosmaster, attr, _safe_receive)
    setattr(Rosmaster, "_rovi_receive_patched", True)


def _identify_rosmaster(devices: List[Dict[str, str]]) -> Tuple[Optional[Dict[str, str]], Optional[float]]:
    Rosmaster = _load_rosmaster()
    for dev in devices:
        devnode = dev["devnode"]
        print(f"[identify] Probing Rosmaster on {devnode}")
        bot = None
        version = None
        try:
            bot = Rosmaster(com=devnode)
            bot.create_receive_threading()
            time.sleep(0.05)
            version = float(bot.get_version())
            print(f"[identify] {devnode} get_version -> {version}")
        except Exception as exc:  # noqa: BLE001 - surface errors to user
            print(f"[identify] {devnode} error: {exc}")
        finally:
            if bot is not None:
                try:
                    bot.ser.close()
                except Exception:
                    pass
        if version and version > 0:
            return dev, version
    return None, None


def _remove_legacy_rule(path: str, expected: str) -> None:
    if not os.path.exists(path):
        return
    try:
        existing = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        print(f"[warn] Failed to read legacy rule {path}: {exc}")
        return
    if existing == expected:
        try:
            os.remove(path)
            print(f"[cleanup] Removed legacy udev rule: {path}")
        except OSError as exc:
            print(f"[warn] Failed to remove legacy rule {path}: {exc}")
    else:
        print(f"[warn] Legacy rule {path} differs; leaving it in place.")


def main() -> int:
    if os.geteuid() != 0:
        print("Please run as root: sudo python3 tools/rovi_usb_setup.py", file=sys.stderr)
        return 1

    print("[scan] Looking for attached devices...")
    ch340_devices = _scan_devices("/dev/ttyUSB*", CH340_VENDOR, CH340_PRODUCT)
    display_devices = _scan_devices("/dev/ttyACM*", DISPLAY_VENDOR, DISPLAY_PRODUCT)

    print(f"[scan] CH340 devices ({len(ch340_devices)}):")
    for dev in ch340_devices:
        print(
            f"  - {dev['devnode']} id_path={dev['id_path']} id_serial={dev['id_serial']} by_path={dev['by_path']}"
        )
        sniff = _sniff_device(dev["devnode"])
        print(f"    sniff: {sniff}")

    print(f"[scan] Display devices ({len(display_devices)}):")
    for dev in display_devices:
        print(
            f"  - {dev['devnode']} id_path={dev['id_path']} id_serial={dev['id_serial']} by_path={dev['by_path']}"
        )

    if len(ch340_devices) != 2:
        print(
            "[error] Expected exactly 2 CH340 devices (control + lidar). Attach both and re-run."
        )
        return 1

    if len(display_devices) != 1:
        print("[error] Expected exactly 1 display device (ESP32-S3). Attach it and re-run.")
        return 1

    if not all(dev.get("id_path") for dev in ch340_devices):
        print("[error] Missing ID_PATH for CH340 devices; cannot create stable rules.")
        return 1

    rosmaster_dev, version = _identify_rosmaster(ch340_devices)
    if rosmaster_dev is None:
        print("[error] Rosmaster board not detected on any CH340 device.")
        return 1

    lidar_dev = next(dev for dev in ch340_devices if dev is not rosmaster_dev)
    print(
        f"[identify] Rosmaster={rosmaster_dev['devnode']} (version={version}) | "
        f"Lidar={lidar_dev['devnode']}"
    )

    rules = [
        (
            'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", '
            f'ENV{{ID_PATH}}=="{rosmaster_dev["id_path"]}", MODE:="0666", '
            'SYMLINK+="robot_control"\n'
        ),
        (
            'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", '
            f'ENV{{ID_PATH}}=="{lidar_dev["id_path"]}", MODE:="0666", '
            'SYMLINK+="robot_lidar"\n'
        ),
        (
            'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", '
            'MODE:="0666", SYMLINK+="robot_display"\n'
        ),
    ]
    rules_text = "".join(rules)

    if os.path.exists(RULE_PATH):
        try:
            existing = Path(RULE_PATH).read_text(encoding="utf-8")
        except OSError as exc:
            print(f"[error] Failed to read existing rule {RULE_PATH}: {exc}")
            return 1
        if existing == rules_text:
            print(f"[setup] Rule already up to date: {RULE_PATH}")
        else:
            print(f"[setup] Updating rule: {RULE_PATH}")
            try:
                Path(RULE_PATH).write_text(rules_text, encoding="utf-8")
            except OSError as exc:
                print(f"[error] Failed to write {RULE_PATH}: {exc}")
                return 1
    else:
        try:
            Path(RULE_PATH).write_text(rules_text, encoding="utf-8")
            print(f"[setup] Wrote udev rule: {RULE_PATH}")
        except OSError as exc:
            print(f"[error] Failed to write {RULE_PATH}: {exc}")
            return 1

    for legacy_path, legacy_line in LEGACY_RULES.items():
        _remove_legacy_rule(legacy_path, legacy_line)

    cmds = [
        ["udevadm", "control", "--reload-rules"],
        ["udevadm", "trigger", "--attr-match", "idVendor=1a86", "--attr-match", "idProduct=7523"],
        ["udevadm", "trigger", "--attr-match", "idVendor=303a", "--attr-match", "idProduct=1001"],
    ]
    for cmd in cmds:
        try:
            subprocess.run(cmd, check=True)
            print(f"[setup] Ran: {' '.join(cmd)}")
        except (OSError, subprocess.CalledProcessError) as exc:
            print(f"[error] Failed to run {' '.join(cmd)}: {exc}")
            return 1

    print(
        "[setup] Done. Unplug/replug devices to ensure symlinks appear as "
        "/dev/robot_control, /dev/robot_lidar, /dev/robot_display."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
