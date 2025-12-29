from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class SerialDisplayConfig:
    gateway_address: str
    serial_port: str
    baudrate: int
    selected_ids: list[str]
    reconnect_delay_s: float


def default_config_path() -> Path:
    share_dir = Path(get_package_share_directory('rovi_serial_display'))
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
    serial_port = str(serial.get('port', '/dev/rovi_display'))
    baudrate = int(serial.get('baudrate', 256000))
    selected_ids = _read_string_list(display.get('selected_ids'))
    reconnect_delay_s = float(data.get('reconnect_delay_s', 2.0))

    if reconnect_delay_s <= 0:
        raise RuntimeError(f"reconnect_delay_s must be > 0 (got {reconnect_delay_s}) in: {config_path}")

    return SerialDisplayConfig(
        gateway_address=gateway_address,
        serial_port=serial_port,
        baudrate=baudrate,
        selected_ids=selected_ids,
        reconnect_delay_s=reconnect_delay_s,
    )


def _read_map(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise RuntimeError(f"Invalid '{key}' section (expected mapping)")
    return value


def _read_string_list(value: Any) -> list[str]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise RuntimeError("Invalid 'display.selected_ids' (expected list)")
    out: list[str] = []
    for item in value:
        if item is None:
            continue
        s = str(item).strip()
        if s:
            out.append(s)
    return out
