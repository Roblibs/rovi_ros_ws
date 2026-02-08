#!/usr/bin/env bash
set -euo pipefail

log() {
  echo "[rovi-services] $*" >&2
}

die() {
  log "ERROR: $*"
  exit 1
}

ws_dir_default() {
  echo "${HOME}/dev/rovi_ros_ws"
}

resolve_ws_dir() {
  if [ -n "${ROVI_ROS_WS_DIR:-}" ]; then
    echo "${ROVI_ROS_WS_DIR}"
  else
    ws_dir_default
  fi
}

source_ros() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
  else
    die "Missing /opt/ros/jazzy/setup.bash"
  fi
}

source_ws() {
  local ws_dir
  ws_dir="$(resolve_ws_dir)"
  if [ ! -f "${ws_dir}/install/setup.bash" ]; then
    die "Missing ${ws_dir}/install/setup.bash (run colcon build)"
  fi
  # shellcheck disable=SC1091
  source "${ws_dir}/install/setup.bash"
}

ensure_runtime_dir() {
  if [ -d /run/rovi ]; then
    return 0
  fi
  mkdir -p /run/rovi 2>/dev/null || true
}
