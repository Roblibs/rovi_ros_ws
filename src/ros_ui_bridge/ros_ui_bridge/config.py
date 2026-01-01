from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from ament_index_python.packages import get_package_share_directory


def _parse_rate_or_period(section: dict[str, Any], default_rate_hz: float) -> float:
    """Parse rate_hz or period_s from config, return period in seconds.
    
    If period_s is specified, it takes precedence over rate_hz.
    """
    period_s = section.get('period_s')
    rate_hz = section.get('rate_hz')
    
    if period_s is not None:
        val = float(period_s)
        if val > 0:
            return val
    if rate_hz is not None:
        val = float(rate_hz)
        if val > 0:
            return 1.0 / val
    return 1.0 / default_rate_hz


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
class StatusStreamConfig:
    period_s: float  # Collection/publish period
    rates: list[TopicRateMetricConfig]
    tf_rates: list[TfRateMetricConfig]


@dataclass(frozen=True)
class RobotStateStreamConfig:
    period_s: float  # Max rate cap (forward on arrival, capped)
    odom_topic: str
    joint_states_topic: str
    odom_frame: str
    base_frame: str
    map_frame: str
    wheel_joint_names: list[str]
    map_tf_max_age_s: float


@dataclass(frozen=True)
class LidarStreamConfig:
    period_s: float  # Max rate cap (forward on arrival, capped)
    topic: str
    output_topic: str  # ROS republish destination
    frame_id: str


@dataclass(frozen=True)
class UiBridgeConfig:
    grpc_bind: str
    voltage_topic: str
    status_stream: StatusStreamConfig
    robot_state_stream: RobotStateStreamConfig
    lidar_stream: Optional[LidarStreamConfig]
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
    streams = _read_map(data, 'streams')
    robot_model = _read_map(data, 'robot_model')

    grpc_bind = str(grpc.get('bind', '0.0.0.0:50051'))
    voltage_topic = str(ros.get('voltage_topic', 'voltage'))

    status_stream_cfg = _parse_status_stream(streams.get('status'))
    robot_state_cfg = _parse_robot_state_stream(streams.get('robot_state'))
    lidar_cfg = _parse_lidar_stream(streams.get('lidar'))
    robot_model_cfg = _parse_robot_model(robot_model)

    return UiBridgeConfig(
        grpc_bind=grpc_bind,
        voltage_topic=voltage_topic,
        status_stream=status_stream_cfg,
        robot_state_stream=robot_state_cfg,
        lidar_stream=lidar_cfg,
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
        raise RuntimeError("Invalid 'rates' section (expected list)")

    out: list[TopicRateMetricConfig] = []
    for entry in value:
        if not isinstance(entry, dict):
            raise RuntimeError("Invalid entry in 'rates' (expected mapping)")

        metric_id = str(entry.get('id', '')).strip()
        topic = str(entry.get('topic', '')).strip()
        if not metric_id:
            raise RuntimeError("rates entry is missing non-empty 'id'")
        if not topic:
            raise RuntimeError(f"rates[{metric_id}] is missing non-empty 'topic'")

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
            raise RuntimeError("tf_rates entry is missing non-empty 'id'")
        if not parent or not child:
            raise RuntimeError(f"tf_rates[{metric_id}] must define 'parent' and 'child'")

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


def _parse_status_stream(value: Any) -> StatusStreamConfig:
    section = value if isinstance(value, dict) else {}

    period_s = _parse_rate_or_period(section, default_rate_hz=0.333)  # ~3 second default

    rates = _parse_topic_rates(section.get('rates'))
    tf_rates = _parse_tf_rates(section.get('tf_rates'))

    return StatusStreamConfig(
        period_s=period_s,
        rates=rates,
        tf_rates=tf_rates,
    )


def _parse_robot_state_stream(value: Any) -> RobotStateStreamConfig:
    section = value if isinstance(value, dict) else {}

    period_s = _parse_rate_or_period(section, default_rate_hz=10.0)  # 10 Hz cap default

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
        raise RuntimeError("Invalid 'streams.robot_state.wheel_joint_names' (expected list)")

    return RobotStateStreamConfig(
        period_s=period_s,
        odom_topic=odom_topic,
        joint_states_topic=joint_states_topic,
        odom_frame=odom_frame,
        base_frame=base_frame,
        map_frame=map_frame,
        wheel_joint_names=wheel_joint_names,
        map_tf_max_age_s=map_tf_max_age_s,
    )


def _parse_lidar_stream(value: Any) -> Optional[LidarStreamConfig]:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise RuntimeError("Invalid 'streams.lidar' section (expected mapping)")

    section = value
    period_s = _parse_rate_or_period(section, default_rate_hz=2.0)  # 2 Hz cap default

    topic = str(section.get('topic', '/scan'))
    output_topic = str(section.get('output_topic', '/viz/scan'))
    frame_id = str(section.get('frame_id', 'laser_link'))

    return LidarStreamConfig(
        period_s=period_s,
        topic=topic,
        output_topic=output_topic,
        frame_id=frame_id,
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
