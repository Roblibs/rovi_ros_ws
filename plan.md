# Plan (index)

This file is an index of planning topics. Details live under `docs/plan/`.

## Camera
- [Camera stack](docs/plan/camera.md): calibration policy, deterministic device mapping, baseline warnings, and startup robustness for UI/display.

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
- [Explicit runtime session state](docs/plan/refactors/session_state.md): replace file-based session coupling.
- [Flatten stack composition](docs/plan/refactors/flatten_stack_composition.md): shared launch blocks for mapping/localization/nav.
- [Backend contract schema + tests](docs/plan/refactors/backend_contract_tests.md): enforce real|sim|offline parity.
- [Split UI bridge planes](docs/plan/refactors/ui_bridge_planes.md): separate data-plane from transport-plane.
