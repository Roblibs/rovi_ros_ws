from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class TopicRateMetricConfig:
    id: str
    topic: str
    target_hz: float | None
    type: str | None
    emit_zero_when_unseen: bool


@dataclass(frozen=True)
class TfRateMetricConfig:
    id: str
    parent: str
    child: str
    target_hz: float | None
    emit_zero_when_unseen: bool


@dataclass(frozen=True)
class RobotModelConfig:
    glb_path: str | None
    chunk_size_bytes: int


@dataclass(frozen=True)
class RobotStateConfig:
    update_hz: float
    odom_topic: str
    joint_states_topic: str
    odom_frame: str
    base_frame: str
    map_frame: str
    wheel_joint_names: list[str]
    map_tf_max_age_s: float


@dataclass(frozen=True)
class UiBridgeConfig:
    grpc_bind: str
    update_period_s: float
    voltage_topic: str
    topic_rates: list[TopicRateMetricConfig]
    tf_rates: list[TfRateMetricConfig]
    robot_state: RobotStateConfig
    robot_model: RobotModelConfig


def default_config_path() -> Path:
    share_dir = Path(get_package_share_directory('ros_ui_bridge'))
    return share_dir / 'config' / 'default.yaml'


def load_config(path: str | Path | None) -> UiBridgeConfig:
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
    metrics = _read_map(data, 'metrics')
    robot_state = _read_map(data, 'robot_state')
    robot_model = _read_map(data, 'robot_model')

    grpc_bind = str(grpc.get('bind', '0.0.0.0:50051'))
    update_period_s = float(data.get('update_period_s', 3.0))
    voltage_topic = str(ros.get('voltage_topic', 'voltage'))
    topic_rates = _parse_topic_rates(metrics.get('rates'))
    tf_rates = _parse_tf_rates(metrics.get('tf_rates'))
    robot_state_cfg = _parse_robot_state(robot_state)
    robot_model_cfg = _parse_robot_model(robot_model)

    if update_period_s <= 0:
        raise RuntimeError(f"update_period_s must be > 0 (got {update_period_s}) in: {config_path}")

    return UiBridgeConfig(
        grpc_bind=grpc_bind,
        update_period_s=update_period_s,
        voltage_topic=voltage_topic,
        topic_rates=topic_rates,
        tf_rates=tf_rates,
        robot_state=robot_state_cfg,
        robot_model=robot_model_cfg,
    )


def _read_map(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise RuntimeError(f"Invalid '{key}' section (expected mapping)")
    return value


def _parse_topic_rates(value: Any) -> list[TopicRateMetricConfig]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise RuntimeError("Invalid 'metrics.rates' section (expected list)")

    out: list[TopicRateMetricConfig] = []
    for entry in value:
        if not isinstance(entry, dict):
            raise RuntimeError("Invalid entry in 'metrics.rates' (expected mapping)")

        metric_id = str(entry.get('id', '')).strip()
        topic = str(entry.get('topic', '')).strip()
        if not metric_id:
            raise RuntimeError("metrics.rates entry is missing non-empty 'id'")
        if not topic:
            raise RuntimeError(f"metrics.rates[{metric_id}] is missing non-empty 'topic'")

        raw_target = entry.get('target_hz')
        target_hz = float(raw_target) if raw_target is not None else None
        raw_type = entry.get('type')
        msg_type = str(raw_type).strip() if raw_type is not None else None
        emit_zero_when_unseen = bool(entry.get('emit_zero_when_unseen', False))

        out.append(
            TopicRateMetricConfig(
                id=metric_id,
                topic=topic,
                target_hz=target_hz,
                type=msg_type if msg_type else None,
                emit_zero_when_unseen=emit_zero_when_unseen,
            )
        )
    return out


def _parse_tf_rates(value: Any) -> list[TfRateMetricConfig]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise RuntimeError("Invalid 'metrics.tf_rates' section (expected list)")

    out: list[TfRateMetricConfig] = []
    for entry in value:
        if not isinstance(entry, dict):
            raise RuntimeError("Invalid entry in 'metrics.tf_rates' (expected mapping)")

        metric_id = str(entry.get('id', '')).strip()
        parent = str(entry.get('parent', '')).strip()
        child = str(entry.get('child', '')).strip()
        if not metric_id:
            raise RuntimeError("metrics.tf_rates entry is missing non-empty 'id'")
        if not parent or not child:
            raise RuntimeError(f"metrics.tf_rates[{metric_id}] must define 'parent' and 'child'")

        raw_target = entry.get('target_hz')
        target_hz = float(raw_target) if raw_target is not None else None
        emit_zero_when_unseen = bool(entry.get('emit_zero_when_unseen', False))

        out.append(
            TfRateMetricConfig(
                id=metric_id,
                parent=parent,
                child=child,
                target_hz=target_hz,
                emit_zero_when_unseen=emit_zero_when_unseen,
            )
        )
    return out


def _parse_robot_state(section: dict[str, Any]) -> RobotStateConfig:
    update_hz = float(section.get('update_hz', 10.0))
    if update_hz <= 0.0:
        update_hz = 10.0

    odom_topic = str(section.get('odom_topic', '/odom_raw'))
    joint_states_topic = str(section.get('joint_states_topic', '/joint_states'))
    odom_frame = str(section.get('odom_frame', 'odom'))
    base_frame = str(section.get('base_frame', 'base_footprint'))
    map_frame = str(section.get('map_frame', 'map'))
    map_tf_max_age_s = float(section.get('map_tf_max_age_s', 1.0))

    wheel_joint_names_raw = section.get('wheel_joint_names')
    wheel_joint_names: list[str]
    if wheel_joint_names_raw is None:
        wheel_joint_names = [
            'front_left_joint',
            'front_right_joint',
            'back_left_joint',
            'back_right_joint',
        ]
    elif isinstance(wheel_joint_names_raw, list):
        wheel_joint_names = [str(name).strip() for name in wheel_joint_names_raw if str(name).strip()]
    else:
        raise RuntimeError("Invalid 'robot_state.wheel_joint_names' (expected list)")

    return RobotStateConfig(
        update_hz=update_hz,
        odom_topic=odom_topic,
        joint_states_topic=joint_states_topic,
        odom_frame=odom_frame,
        base_frame=base_frame,
        map_frame=map_frame,
        wheel_joint_names=wheel_joint_names,
        map_tf_max_age_s=map_tf_max_age_s,
    )


def _parse_robot_model(section: dict[str, Any]) -> RobotModelConfig:
    glb_path_raw = section.get('glb_path', section.get('glb'))
    glb_path = str(glb_path_raw).strip() if glb_path_raw is not None else None
    if glb_path == '':
        glb_path = None

    chunk_size_bytes = int(section.get('chunk_size_bytes', 1024 * 1024))
    if chunk_size_bytes <= 0:
        chunk_size_bytes = 1024 * 1024

    return RobotModelConfig(glb_path=glb_path, chunk_size_bytes=chunk_size_bytes)
