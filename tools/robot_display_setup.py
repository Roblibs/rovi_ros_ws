#!/usr/bin/env python3
"""
Create a udev rule for the ESP32-S3 display (303a:1001) and reload udev.

This writes /etc/udev/rules.d/99-robot-display.rules with:
  SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE:="0666", SYMLINK+="robot_display"

Run with sudo:
  sudo python3 tools/robot_display_setup.py
Then unplug/replug the device (or power-cycle it).
"""

import os
import subprocess
import sys

RULE_PATH = "/etc/udev/rules.d/99-robot-display.rules"
RULE_LINE = (
    'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", '
    'MODE:="0666", SYMLINK+="robot_display"\n'
)

LEGACY_RULE_PATH = "/etc/udev/rules.d/99-rovi-display.rules"
LEGACY_RULE_LINE = (
    'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", '
    'MODE:="0666", SYMLINK+="rovi_display"\n'
)


def main() -> int:
    if os.geteuid() != 0:
        print("Please run as root: sudo python3 tools/robot_display_setup.py", file=sys.stderr)
        return 1

    # Write rule if missing or different.
    write_rule = True
    if os.path.exists(RULE_PATH):
        try:
            with open(RULE_PATH, "r", encoding="utf-8") as fh:
                existing = fh.read()
            if existing == RULE_LINE:
                write_rule = False
                print(f"Rule already present: {RULE_PATH}")
            else:
                print(f"Updating existing rule at {RULE_PATH}")
        except OSError as exc:
            print(f"Failed to read existing rule: {exc}", file=sys.stderr)

    if write_rule:
        try:
            with open(RULE_PATH, "w", encoding="utf-8") as fh:
                fh.write(RULE_LINE)
            print(f"Wrote udev rule to {RULE_PATH}")
        except OSError as exc:
            print(f"Failed to write rule: {exc}", file=sys.stderr)
            return 1

    # Remove legacy rule if it matches exactly (avoid leaving two symlinks behind).
    if os.path.exists(LEGACY_RULE_PATH):
        try:
            with open(LEGACY_RULE_PATH, "r", encoding="utf-8") as fh:
                existing = fh.read()
            if existing == LEGACY_RULE_LINE:
                os.remove(LEGACY_RULE_PATH)
                print(f"Removed legacy udev rule: {LEGACY_RULE_PATH}")
            else:
                print(f"Legacy rule exists but differs; leaving it in place: {LEGACY_RULE_PATH}")
        except OSError as exc:
            print(f"Failed to remove legacy rule {LEGACY_RULE_PATH}: {exc}", file=sys.stderr)

    # Reload and trigger udev.
    cmds = [
        ["udevadm", "control", "--reload-rules"],
        ["udevadm", "trigger", "--attr-match", "idVendor=303a", "--attr-match", "idProduct=1001"],
    ]
    for cmd in cmds:
        try:
            subprocess.run(cmd, check=True)
            print(f"Ran: {' '.join(cmd)}")
        except (OSError, subprocess.CalledProcessError) as exc:
            print(f"Failed to run {' '.join(cmd)}: {exc}", file=sys.stderr)
            return 1

    print("Done. Unplug/replug the display. It should appear as /dev/robot_display with 0666 perms.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
