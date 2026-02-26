# Plan (index)

This file is an index of planning topics. Details live under `docs/plan/`.

## Camera
- [Camera hardware robustness](docs/plan/camera_hardware_robustness.md): bring-up reliability improvements once real camera hardware is in hand.
- [Camera viewer decorrelation (Wi‑Fi)](docs/plan/camera_viewer_decorrelation.md): introduce throttled/renamed *_view topics and make RViz subscribe to those, not the raw 30 Hz feeds (see `docs/depth_camera_analysis.md` for current-state analysis).
- [Floor diff from depth](docs/plan/floor_diff_from_depth.md): calibrate a per-cell floor LUT and publish `/floor/*` bins/mask (+ optional lasso markers).
- [Nav2 costmap from `/floor/*`](docs/plan/floor_nav2_costmap_layer.md): custom `nav2_costmap_2d` layer that projects `/floor/mask` into the local costmap.

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
