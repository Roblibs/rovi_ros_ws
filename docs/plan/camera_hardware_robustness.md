# Plan: Camera hardware robustness (real readiness)

Moved out of `docs/plan/camera_preparation.md` (Phase 4). This plan is **not** a prerequisite for depth-aware navigation work; it can be implemented later when real hardware is in hand.

## Scope

### Calibration
- Calibration file currently has no specific camera id; serial usage to be clarified.

### USB enumeration
- Keep RGB pinned to stable `/dev/v4l/by-id/...`.
- Pin depth to a stable device identifier where supported; otherwise document bus-path risk and validate runtime selection.

### Controls & config
- On startup, control `10092545` is failing with an error.
- List all needed controls and evaluate their usage.
- Check if any configuration is needed for performance optimization.

## Acceptance criteria
- Deterministic device selection is documented and validated.
- Startup control failures are either fixed or explicitly gated/ignored with rationale.

## Open points
- OP3: What is the authoritative identity for depth camera selection (serial, URI, bus path)?
- OP4: Where do calibration artifacts live, and how are they selected per device?

