# Plan (index)

This file is an index of planning topics. Details live under `docs/plan/`.

## Camera
- [Camera preparation (depth → nav)](docs/plan/camera_preparation.md): consolidated prep phases (contracts → launch refactor → contract tests → session state → hardware readiness).
- [Floor clearance from depth](docs/plan/floor_clearance_from_depth.md): depth-only derived metric (kept separate from prep work).

## Display
- Resolve startup robustness for UI/display path: `robot_serial_display` initial gRPC connection failure before `ros_ui_bridge` is ready.

## Services
- Follow-ups: expand per-stack process/status catalogs, add richer systemd diagnostics fields, and harden the gRPC control plane (auth/bind separation/audit).

## Improvements
- [Consistent UI stream timestamps](docs/plan/improvements/ui_timestamps.md): standardize on ROS/header time.
- [No launch-time PYTHONPATH mutation](docs/plan/improvements/no_pythonpath_mutation.md): remove environment surgery where possible.
- [Validated mode constants/enums](docs/plan/improvements/mode_constants.md): replace stringly mode checks.
- [Forwarder observability](docs/plan/improvements/forwarder_observability.md): make throttled forwarding failures visible.
- [Rosbag profile hardening](docs/plan/improvements/rosbag_profiles.md): validate stack/profile mappings.

## Refactors
- [Split UI bridge planes](docs/plan/refactors/ui_bridge_planes.md): separate data-plane from transport-plane.
