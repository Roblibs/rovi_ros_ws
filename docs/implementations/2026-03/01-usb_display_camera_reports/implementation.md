## 2026-03-01 — USB display vs camera diagnostics bundles

### Why
- Need a repeatable, timestamped way to capture “ground truth” evidence when the CDC-ACM display becomes unreliable during camera start/streaming on Pi 5.

### What changed
- Added `tools/rovi_usb_cam_display_report.py` to generate shareable bundles under gitignored `output/`:
  - `output/reports/<YYYY-MM-DD>/<command>/<HHMMSS>/report.md`
  - `output/reports/<YYYY-MM-DD>/<command>/<HHMMSS>/meta.json`
  - `output/reports/<YYYY-MM-DD>/<command>/<HHMMSS>/raw/*` (timestamped per-command outputs)
- Added a dedicated `usbmon` pre-check subcommand and `rovi_env.sh` wrappers:
  - `test usb` / `test usb capture ...`
  - `test usbmon`
- Linked the tool from `docs/usb.md`.
- Added a planning note in `docs/plan/usb_display_camera_troubleshooting_reports.md` and indexed it in `plan.md`.

### Usage
```bash
ws

# Static snapshot (topology + device state)
test usb

# Capture window: start camera in another terminal during the window
# (usbmon + dmesg capture usually requires sudo)
test usbmon
test usb capture
```
