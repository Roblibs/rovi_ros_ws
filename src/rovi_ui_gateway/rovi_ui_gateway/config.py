from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class UiGatewayConfig:
    grpc_bind: str
    update_period_s: float
    voltage_topic: str


def default_config_path() -> Path:
    share_dir = Path(get_package_share_directory('rovi_ui_gateway'))
    return share_dir / 'config' / 'default.yaml'


def load_config(path: str | Path | None) -> UiGatewayConfig:
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

    grpc = _read_map(data, 'grpc')
    ros = _read_map(data, 'ros')

    grpc_bind = str(grpc.get('bind', '0.0.0.0:50051'))
    update_period_s = float(data.get('update_period_s', 3.0))
    voltage_topic = str(ros.get('voltage_topic', 'voltage'))

    if update_period_s <= 0:
        raise RuntimeError(f"update_period_s must be > 0 (got {update_period_s}) in: {config_path}")

    return UiGatewayConfig(
        grpc_bind=grpc_bind,
        update_period_s=update_period_s,
        voltage_topic=voltage_topic,
    )


def _read_map(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise RuntimeError(f"Invalid '{key}' section (expected mapping)")
    return value
