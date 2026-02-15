# Plan (index)

This file is an index of planning topics. Details live under `docs/plan/`.

## Camera
- [Camera hardware robustness](docs/plan/camera_hardware_robustness.md): bring-up reliability improvements once real camera hardware is in hand.
- [Floor clearance from depth](docs/plan/floor_clearance_from_depth.md): depth-only derived metric (kept separate from camera bringup work).

## Display
- Resolve startup robustness for UI/display path: `robot_serial_display` initial gRPC connection failure before `ros_ui_bridge` is ready.

## Services
- Follow-ups: expand per-stack process/status catalogs, add richer systemd diagnostics fields, and harden the gRPC control plane (auth/bind separation/audit).

## Improvements
- [Consistent UI stream timestamps](docs/plan/improvement-ui_timestamps.md): standardize on ROS/header time.
- [No venv + no launch-time PYTHONPATH mutation](docs/plan/improvement-no_pythonpath_mutation.md): remove environment surgery by relying on system deps and/or workspace-installed libs.
- [Forwarder observability](docs/plan/improvement-forwarder_observability.md): make throttled forwarding failures visible.

## Refactors
- [Split UI bridge planes](docs/plan/refactor-ui_bridge_planes.md): separate data-plane from transport-plane.
