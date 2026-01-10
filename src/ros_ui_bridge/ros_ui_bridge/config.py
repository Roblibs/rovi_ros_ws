from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from ament_index_python.packages import get_package_share_directory


def _parse_downsampling_period_s(section: dict[str, Any]) -> Optional[float]:
    """Parse downsampling period from config.

    Supported keys (in precedence order):
    - downsampling_period_s
    - downsampling_rate_hz
    - period_s (legacy)
    - rate_hz (legacy)

    If no keys are provided or if the provided values are <= 0, returns None
    meaning "no downsampling" (forward 1:1).
    """
    period_s = section.get('downsampling_period_s')
    rate_hz = section.get('downsampling_rate_hz')
    if period_s is None and rate_hz is None:
        # Backward compatibility with older configs.
        period_s = section.get('period_s')
        rate_hz = section.get('rate_hz')

    if period_s is not None:
        val = float(period_s)
        if val > 0:
            return val
        return None

    if rate_hz is not None:
        val = float(rate_hz)
        if val > 0:
            return 1.0 / val
        return None

    return None


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
class StatusFieldSource:
    provider: str  # system | ros
    type: str  # cpu_percent | topic_value | topic_rate | tf_rate
    topic: str | None
    msg_type: str | None
    value_key: str | None
    parent: str | None
    child: str | None
    downsample_period_s: float | None
    stale_after_s: float | None


@dataclass(frozen=True)
class StatusFieldConfig:
    id: str
    unit: str
    min: float | None
    max: float | None
    target: float | None
    source: StatusFieldSource


@dataclass(frozen=True)
class TfDemuxConfig:
    enabled: bool
    prefix: str


@dataclass(frozen=True)
class RobotModelConfig:
    glb_path: str | None
    chunk_size_bytes: int


@dataclass(frozen=True)
class StatusStreamConfig:
    period_s: float  # Collection/publish period
    stale_after_s: float  # Default staleness window for all fields
    debug_log: bool  # Log status publish events
    always_publish: bool  # Publish every period, even if values unchanged
    fields: list[StatusFieldConfig]


@dataclass(frozen=True)
class RobotStateStreamConfig:
    downsampling_period_s: float | None  # Optional max rate cap (forward on arrival, capped)
    odom_topic: str
    joint_states_topic: str
    odom_frame: str
    base_frame: str
    map_frame: str
    wheel_joint_names: list[str]
    map_tf_max_age_s: float


@dataclass(frozen=True)
class LidarStreamConfig:
    downsampling_period_s: float | None  # Optional max rate cap (forward on arrival, capped)
    topic: str
    output_topic: str  # ROS republish destination
    frame_id: str


@dataclass(frozen=True)
class MapStreamConfig:
    downsampling_period_s: float | None  # Optional max rate cap (forward on arrival, capped)
    topic: str


@dataclass(frozen=True)
class UiBridgeConfig:
    grpc_bind: str
    tf_demux: TfDemuxConfig
    status_stream: StatusStreamConfig
    robot_state_stream: RobotStateStreamConfig
    lidar_stream: Optional[LidarStreamConfig]
    map_stream: Optional[MapStreamConfig]
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
    tf_demux_cfg = _parse_tf_demux(ros)

    status_stream_cfg = _parse_status_stream(streams.get('status'))
    robot_state_cfg = _parse_robot_state_stream(streams.get('robot_state'))
    lidar_cfg = _parse_lidar_stream(streams.get('lidar'))
    map_cfg = _parse_map_stream(streams.get('map'))
    robot_model_cfg = _parse_robot_model(robot_model)

    return UiBridgeConfig(
        grpc_bind=grpc_bind,
        tf_demux=tf_demux_cfg,
        status_stream=status_stream_cfg,
        robot_state_stream=robot_state_cfg,
        lidar_stream=lidar_cfg,
        map_stream=map_cfg,
        robot_model=robot_model_cfg,
    )


def _read_map(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise RuntimeError(f"Invalid '{key}' section (expected mapping)")
    return value


def _parse_tf_demux(ros: dict[str, Any]) -> TfDemuxConfig:
    section = _read_map(ros, 'tf_demux')
    enabled = bool(section.get('enabled', True))
    prefix = str(section.get('prefix', '/viz/tf')).strip() or '/viz/tf'
    return TfDemuxConfig(enabled=enabled, prefix=prefix)


def _parse_status_fields(value: Any, default_stale_after_s: float) -> list[StatusFieldConfig]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise RuntimeError("Invalid 'status.fields' section (expected list)")

    out: list[StatusFieldConfig] = []
    for entry in value:
        if not isinstance(entry, dict):
            raise RuntimeError("Invalid entry in 'status.fields' (expected mapping)")

        field_id = str(entry.get('id', '')).strip()
        unit = str(entry.get('unit', '')).strip()
        if not field_id:
            raise RuntimeError("status.fields entry is missing non-empty 'id'")
        if not unit:
            raise RuntimeError(f"status.fields[{field_id}] is missing non-empty 'unit'")

        min_val = entry.get('min')
        max_val = entry.get('max')
        target_val = entry.get('target')
        min_f = float(min_val) if min_val is not None else None
        max_f = float(max_val) if max_val is not None else None
        target_f = float(target_val) if target_val is not None else None

        source_section = _read_map(entry, 'source')
        provider = str(source_section.get('provider', '')).strip().lower()
        type_ = str(source_section.get('type', '')).strip().lower()
        if not provider:
            raise RuntimeError(f"status.fields[{field_id}].source is missing 'provider'")
        if not type_:
            raise RuntimeError(f"status.fields[{field_id}].source is missing 'type'")

        downsample_period_s = source_section.get('downsample_period_s')
        downsample_f = float(downsample_period_s) if downsample_period_s is not None else None
        stale_after = source_section.get('stale_after_s')
        stale_after_f = float(stale_after) if stale_after is not None else default_stale_after_s

        msg_type = source_section.get('msg_type')
        msg_type_str = str(msg_type).strip() if msg_type is not None else None
        value_key = source_section.get('value_key')
        value_key_str = str(value_key).strip() if value_key is not None else None
        topic = source_section.get('topic')
        topic_str = str(topic).strip() if topic is not None else None
        parent = source_section.get('parent')
        child = source_section.get('child')
        parent_str = str(parent).strip() if parent is not None else None
        child_str = str(child).strip() if child is not None else None

        if provider == 'system':
            if type_ != 'cpu_percent':
                raise RuntimeError(f"status.fields[{field_id}] unsupported system source type '{type_}' (expected cpu_percent)")
        elif provider == 'ros':
            if type_ == 'topic_value':
                if not topic_str:
                    raise RuntimeError(f"status.fields[{field_id}] is missing 'source.topic'")
                if not msg_type_str:
                    raise RuntimeError(f"status.fields[{field_id}] is missing 'source.msg_type' for topic_value")
                if not value_key_str:
                    value_key_str = 'data'
            elif type_ == 'topic_rate':
                if not topic_str:
                    raise RuntimeError(f"status.fields[{field_id}] is missing 'source.topic'")
            elif type_ == 'tf_rate':
                if not parent_str or not child_str:
                    raise RuntimeError(f"status.fields[{field_id}] tf_rate requires 'source.parent' and 'source.child'")
            else:
                raise RuntimeError(f"status.fields[{field_id}] unsupported ros source type '{type_}'")
        else:
            raise RuntimeError(f"status.fields[{field_id}] unknown provider '{provider}' (expected system|ros)")

        source = StatusFieldSource(
            provider=provider,
            type=type_,
            topic=topic_str,
            msg_type=msg_type_str,
            value_key=value_key_str,
            parent=parent_str,
            child=child_str,
            downsample_period_s=downsample_f if downsample_f and downsample_f > 0 else None,
            stale_after_s=stale_after_f if stale_after_f and stale_after_f > 0 else default_stale_after_s,
        )

        out.append(
            StatusFieldConfig(
                id=field_id,
                unit=unit,
                min=min_f,
                max=max_f,
                target=target_f,
                source=source,
            )
        )
    return out


def _parse_status_stream(value: Any) -> StatusStreamConfig:
    section = value if isinstance(value, dict) else {}

    period_s = _parse_rate_or_period(section, default_rate_hz=0.333)  # ~3 second default
    stale_after_s = float(section.get('stale_after_s', 7.0))
    if stale_after_s <= 0:
        stale_after_s = 7.0
    debug_log = bool(section.get('debug_log', False))
    always_publish = bool(section.get('always_publish', False))

    fields = _parse_status_fields(section.get('fields'), default_stale_after_s=stale_after_s)

    return StatusStreamConfig(
        period_s=period_s,
        stale_after_s=stale_after_s,
        debug_log=debug_log,
        always_publish=always_publish,
        fields=fields,
    )


def _parse_robot_state_stream(value: Any) -> RobotStateStreamConfig:
    section = value if isinstance(value, dict) else {}

    downsampling_period_s = _parse_downsampling_period_s(section)

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
        downsampling_period_s=downsampling_period_s,
        odom_topic=odom_topic,
        joint_states_topic=joint_states_topic,
        odom_frame=odom_frame,
        base_frame=base_frame,
        map_frame=map_frame,
        wheel_joint_names=wheel_joint_names,
        map_tf_max_age_s=map_tf_max_age_s,
    )


def _parse_map_stream(value: Any) -> Optional[MapStreamConfig]:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise RuntimeError("Invalid 'streams.map' section (expected mapping)")

    downsampling_period_s = _parse_downsampling_period_s(value)
    topic = str(value.get('topic', '/map'))
    return MapStreamConfig(downsampling_period_s=downsampling_period_s, topic=topic)


def _parse_lidar_stream(value: Any) -> Optional[LidarStreamConfig]:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise RuntimeError("Invalid 'streams.lidar' section (expected mapping)")

    section = value
    downsampling_period_s = _parse_downsampling_period_s(section)

    topic = str(section.get('topic', '/scan'))
    output_topic = str(section.get('output_topic', '/viz/scan'))
    frame_id = str(section.get('frame_id', 'laser_link'))

    return LidarStreamConfig(
        downsampling_period_s=downsampling_period_s,
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
