from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo


def _require_key(data: dict[str, Any], key: str) -> Any:
    if key not in data:
        raise KeyError(f"missing key '{key}'")
    return data[key]


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except Exception as e:  # pragma: no cover
        raise RuntimeError(
            "Missing PyYAML dependency (import yaml failed). "
            "Install python deps for this workspace (uv sync) or system python3-yaml."
        ) from e

    if not path.is_file():
        raise FileNotFoundError(str(path))
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping at root of {path}")
    return data


def _matrix_data(data: dict[str, Any], key: str, expected_len: int) -> list[float]:
    node = _require_key(data, key)
    if not isinstance(node, dict):
        raise ValueError(f"Expected mapping for '{key}'")
    arr = _require_key(node, "data")
    if not isinstance(arr, list):
        raise ValueError(f"Expected list for '{key}.data'")
    out = [float(x) for x in arr]
    if len(out) != expected_len:
        raise ValueError(f"Expected {expected_len} values for '{key}.data', got {len(out)}")
    return out


def _vector_data(data: dict[str, Any], key: str) -> list[float]:
    node = _require_key(data, key)
    if not isinstance(node, dict):
        raise ValueError(f"Expected mapping for '{key}'")
    arr = _require_key(node, "data")
    if not isinstance(arr, list):
        raise ValueError(f"Expected list for '{key}.data'")
    return [float(x) for x in arr]


def _build_camera_info(yaml_data: dict[str, Any], *, frame_id: str) -> CameraInfo:
    msg = CameraInfo()

    msg.width = int(_require_key(yaml_data, "image_width"))
    msg.height = int(_require_key(yaml_data, "image_height"))
    msg.distortion_model = str(_require_key(yaml_data, "distortion_model"))

    msg.d = _vector_data(yaml_data, "distortion_coefficients")

    k = _matrix_data(yaml_data, "camera_matrix", 9)
    msg.k = k

    r = _matrix_data(yaml_data, "rectification_matrix", 9)
    msg.r = r

    p = _matrix_data(yaml_data, "projection_matrix", 12)
    msg.p = p

    msg.binning_x = int(yaml_data.get("binning_x", 0))
    msg.binning_y = int(yaml_data.get("binning_y", 0))

    roi = yaml_data.get("roi", {})
    if isinstance(roi, dict):
        msg.roi.x_offset = int(roi.get("x_offset", 0))
        msg.roi.y_offset = int(roi.get("y_offset", 0))
        msg.roi.height = int(roi.get("height", 0))
        msg.roi.width = int(roi.get("width", 0))
        msg.roi.do_rectify = bool(roi.get("do_rectify", False))

    msg.header.frame_id = str(frame_id)
    return msg


class CameraInfoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("rovi_camera_info_pub")

        # Standard ROS param (used implicitly by node clock).
        self.declare_parameter("use_sim_time", False)

        self.declare_parameter("color_yaml", "")
        self.declare_parameter("depth_yaml", "")
        self.declare_parameter("color_topic", "/camera/color/camera_info")
        self.declare_parameter("depth_topic", "/camera/depth/camera_info")
        self.declare_parameter("color_frame_id", "camera_color_optical_frame")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("publish_period_s", 0.0)

        color_yaml_s = str(self.get_parameter("color_yaml").value or "").strip()
        depth_yaml_s = str(self.get_parameter("depth_yaml").value or "").strip()
        if not color_yaml_s:
            raise RuntimeError("color_yaml parameter is required")
        if not depth_yaml_s:
            raise RuntimeError("depth_yaml parameter is required")
        color_yaml = Path(color_yaml_s).expanduser()
        depth_yaml = Path(depth_yaml_s).expanduser()

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._color_pub = self.create_publisher(CameraInfo, str(self.get_parameter("color_topic").value), qos)
        self._depth_pub = self.create_publisher(CameraInfo, str(self.get_parameter("depth_topic").value), qos)

        self._color_msg = _build_camera_info(
            _load_yaml(color_yaml),
            frame_id=str(self.get_parameter("color_frame_id").value),
        )
        self._depth_msg = _build_camera_info(
            _load_yaml(depth_yaml),
            frame_id=str(self.get_parameter("depth_frame_id").value),
        )

        period = float(self.get_parameter("publish_period_s").value or 0.0)
        if period > 0.0:
            self.create_timer(period, self._publish_once)

        self._publish_once()

    def _publish_once(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._color_msg.header.stamp = stamp
        self._depth_msg.header.stamp = stamp
        self._color_pub.publish(self._color_msg)
        self._depth_pub.publish(self._depth_msg)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    try:
        node = CameraInfoPublisher()
    except Exception as e:
        print(f"[rovi_camera_info_pub] {e}", file=sys.stderr)
        rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
