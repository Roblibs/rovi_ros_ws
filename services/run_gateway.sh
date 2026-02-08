#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_common.sh
source "${SCRIPT_DIR}/run_common.sh"

ensure_runtime_dir
source_ros
source_ws

ws_dir="$(resolve_ws_dir)"
log "Starting gateway from ${ws_dir}"
log "ROVI_ODOM_INTEGRATOR_PUBLISH_TF=${ROVI_ODOM_INTEGRATOR_PUBLISH_TF:-<unset>}"

exec ros2 launch rovi_bringup gateway.launch.py \
  robot_mode:=real

