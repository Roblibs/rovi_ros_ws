# Reports

This folder is the “human layer” for interpreting report bundles written under gitignored `output/reports/`.

## Generate (Pi)

- `test usb` → snapshot bundle
- `test usbmon` → usbmon pre-check bundle
- `test usb capture` → capture bundle (defaults: 30s, usbmon on, write-test on)

## Interpret (USB display vs camera)

Open the bundle’s `report.md` and focus on:
- `write_test`: if you see spikes above ~`20ms` during camera start/stream, that supports **kernel/USB backpressure** (H2).
- `usbmon`: if it shows very low/no traffic during the failure window, that supports **URB starvation / device stall** (H2/H3).
- If `write_test` stays fast while the display firmware/app times out (often around ~255 bytes), that supports **protocol/firmware framing fragility** (H1).

## Synthesis — 2026-03-01 (Pi 5)

Runs:
- Baseline capture: `output/reports/2026-03-01/capture/193104/report.md`
- “camera” started during capture: `output/reports/2026-03-01/capture/193144/report.md`

Observed:
- Serial write latency stays sub-millisecond in both runs:
  - Baseline: `p95≈0.027ms`, `max≈0.038ms`
  - With camera start: `p95≈0.116ms`, `max≈0.185ms`
- No `>20ms` write spikes in either run.
- usbmon capture volume is essentially unchanged between runs (similar line count / bytes), suggesting the display’s USB traffic is not stalling during these windows.

Conclusion (from these two captures only):
- The data does **not** support “kernel write() to `/dev/robot_display` blocks during camera start” as the primary failure mode.
- The more likely root cause remains **protocol/firmware-side framing/timeouts** (especially around ~255–256 byte line buffering) rather than OS-level USB starvation.

