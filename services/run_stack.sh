#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: run_stack.sh <teleop|mapping|nav> [extra ros2 launch args...]" >&2
  exit 2
fi

STACK="$1"
shift || true

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_common.sh
source "${SCRIPT_DIR}/run_common.sh"

ensure_runtime_dir
source_ros
source_ws

case "${STACK}" in
  teleop|mapping|nav) ;;
  *)
    die "Unknown stack: ${STACK}"
    ;;
esac

LOCK_FILE="/run/rovi/stack.lock"
log "Starting stack=${STACK} (exclusive lock: ${LOCK_FILE})"

exec flock -n "${LOCK_FILE}" ros2 launch rovi_bringup rovi.launch.py \
  robot_mode:=real \
  gateway_enabled:=false \
  stack:="${STACK}" \
  rviz:=false \
  "$@"
