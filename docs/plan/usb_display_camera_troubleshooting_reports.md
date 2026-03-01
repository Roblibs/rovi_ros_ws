# Plan: USB display vs camera — reproducible diagnostics reports

## Problem
On Raspberry Pi 5, the CDC-ACM USB serial display can become unreliable (timeouts / partial lines) when the UVC camera pipeline starts streaming. We need **repeatable, timestamped evidence bundles** (commands + outputs) to compare runs and share in issues/handovers.

## Goal
- Provide a **single Python entrypoint** that runs the “most useful first” diagnostics commands and writes a report bundle under a **gitignored** folder (`output/`).
- Reports should be **timestamped** (directory name + metadata) and primarily **Markdown**, with optional `meta.json`.
- Support two workflows:
  1) **Snapshot**: collect topology + device info “as-is”.
  2) **Capture window**: collect logs/usbmon during a time window while the operator starts the camera pipeline in another terminal.

## Proposed deliverable
- New tool: `tools/rovi_usb_cam_display_report.py`
  - `snapshot` subcommand: `lsusb -t`, `lsusb`, `usb-devices`, `/proc/interrupts` summary, sysfs runtime-PM for display.
  - `usbmon` subcommand: fast “is usbmon working for the display bus?” pre-check.
  - `capture` subcommand (optional root): timed `dmesg -Tw` capture; defaults include usbmon capture + serial write-test (can be disabled via `--no-usbmon` / `--no-write-test`).
- Output bundle directory (gitignored):
  - `output/reports/<YYYY-MM-DD>/<command>/<HHMMSS>/`
    - `report.md` (single “paste this” artifact)
    - `meta.json` (host/kernel/tool version/args + timestamp)
    - `raw/` (individual command outputs)

## Success criteria
- One command produces a bundle that contains enough “ground truth” to validate H1/H2/H3 hypotheses (protocol limit vs kernel backpressure vs PM/resets).
- The bundle is stable for sharing (paths and filenames deterministic; includes exact execution time).

## Open points to confirm
- Default display device: `/dev/robot_display` (fixed, by default).
- `capture` defaults to usbmon enabled (but `usbmon` pre-check is available and recommended).
- Default capture window: 30s.
