# Generation Log

## 2026-01-01

### Lidar gRPC + Capped Downsampling Refactor

Added lidar streaming to the gRPC interface (`StreamLidar`) and refactored the architecture to use "capped downsampling" — data is forwarded on arrival if rate cap allows, never duplicating stale data. Merged the separate `viz_downsample` node into `ui_bridge`; lidar throttle now republishes to `/viz/scan` for RViz.

**Key changes:**
- New `ThrottledForwarder` utility for rate-capped passthrough
- Queue-based gRPC streaming (`AsyncStreamBroadcaster`) — no caching, event-driven
- Single lidar node handles both gRPC and ROS `/viz/scan` republish
- Removed: `viz_downsample_node.py`, `lidar_store.py`, `robot_state_store.py`
- Config now supports both `rate_hz` and `period_s`
