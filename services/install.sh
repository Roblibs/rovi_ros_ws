#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

log() { echo "[install] $*" >&2; }
die() { log "ERROR: $*"; exit 1; }

if [ "$(id -u)" -ne 0 ]; then
  die "Run as root (sudo) to install systemd units and polkit rules."
fi

if [ ! -f "${WS_DIR}/.env.robot" ]; then
  die "Missing ${WS_DIR}/.env.robot (create it before installing services)."
fi

install -d /etc/systemd/system
install -m 0644 "${SCRIPT_DIR}/rovi-gateway.service" /etc/systemd/system/rovi-gateway.service
install -m 0644 "${SCRIPT_DIR}/rovi-teleop.service" /etc/systemd/system/rovi-teleop.service
install -m 0644 "${SCRIPT_DIR}/rovi-mapping.service" /etc/systemd/system/rovi-mapping.service
install -m 0644 "${SCRIPT_DIR}/rovi-nav.service" /etc/systemd/system/rovi-nav.service

install -d /etc/polkit-1/rules.d
install -m 0644 "${SCRIPT_DIR}/polkit/10-rovi-stack.rules" /etc/polkit-1/rules.d/10-rovi-stack.rules

if ! getent group rovi-ops >/dev/null 2>&1; then
  groupadd rovi-ops
  log "Created group rovi-ops"
fi

systemctl daemon-reload
systemctl enable rovi-gateway.service
systemctl restart rovi-gateway.service

log "Installed. Add operator users to group rovi-ops to allow stack start/stop without sudo."
