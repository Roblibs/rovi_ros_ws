from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class SerialDisplayConfig:
    gateway_address: str
    serial_port: str
    baudrate: int
    selected_ids: list[str]
    selected_scales: dict[str, float]
    reconnect_delay_s: float


def default_config_path() -> Path:
    share_dir = Path(get_package_share_directory('robot_serial_display'))
    return share_dir / 'config' / 'default.yaml'


def load_config(path: str | Path | None) -> SerialDisplayConfig:
    config_path = Path(path) if path else default_config_path()

    try:
        import yaml  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing dependency 'PyYAML'. Install workspace Python deps (e.g. `uv sync`) or ensure the venv is on PYTHONPATH."
        ) from exc

    try:
        raw_text = config_path.read_text(encoding='utf-8')
    except OSError as exc:
        raise RuntimeError(f"Failed to read config file: {config_path}") from exc

    try:
        data = yaml.safe_load(raw_text) or {}
    except Exception as exc:  # noqa: BLE001 - surface YAML errors to user
        raise RuntimeError(f"Failed to parse YAML config: {config_path}") from exc

    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid config root (expected mapping) in: {config_path}")

    gateway = _read_map(data, 'gateway')
    serial = _read_map(data, 'serial')
    display = _read_map(data, 'display')

    gateway_address = str(gateway.get('address', '127.0.0.1:50051'))
    serial_port = str(serial.get('port', '/dev/robot_display'))
    env_port = os.environ.get('ROVI_DISPLAY_PORT')
    if env_port:
        serial_port = env_port
    baudrate = int(serial.get('baudrate', 256000))
    selected_ids, selected_scales = _read_selected_ids(display.get('selected_ids'))
    reconnect_delay_s = float(data.get('reconnect_delay_s', 2.0))

    if reconnect_delay_s <= 0:
        raise RuntimeError(f"reconnect_delay_s must be > 0 (got {reconnect_delay_s}) in: {config_path}")

    return SerialDisplayConfig(
        gateway_address=gateway_address,
        serial_port=serial_port,
        baudrate=baudrate,
        selected_ids=selected_ids,
        selected_scales=selected_scales,
        reconnect_delay_s=reconnect_delay_s,
    )


def _read_map(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise RuntimeError(f"Invalid '{key}' section (expected mapping)")
    return value


def _read_selected_ids(value: Any) -> tuple[list[str], dict[str, float]]:
    if value is None:
        return [], {}
    if not isinstance(value, list):
        raise RuntimeError("Invalid 'display.selected_ids' (expected list)")
    out: list[str] = []
    scales: dict[str, float] = {}
    for item in value:
        if item is None:
            continue
        if isinstance(item, dict):
            raw_id = item.get('id')
            if raw_id is None:
                continue
            field_id = str(raw_id).strip()
            if not field_id:
                continue
            out.append(field_id)
            if 'scale' in item and item['scale'] is not None:
                try:
                    scales[field_id] = float(item['scale'])
                except (TypeError, ValueError) as exc:
                    raise RuntimeError(f"Invalid scale for display.selected_ids id='{field_id}'") from exc
            continue
        s = str(item).strip()
        if s:
            out.append(s)
    return out, scales
