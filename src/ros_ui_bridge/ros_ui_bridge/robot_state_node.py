from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from .throttled_forwarder import AsyncStreamBroadcaster, ThrottledForwarder
from .session_info import resolve_session


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id).lstrip('/')


def _quat_mul(
    ax: float,
    ay: float,
    az: float,
    aw: float,
    bx: float,
    by: float,
    bz: float,
    bw: float,
) -> tuple[float, float, float, float]:
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_conj(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    return (-x, -y, -z, w)


def _rotate_vec_by_quat(vx: float, vy: float, vz: float, qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    # v' = q * (v,0) * q*
    tx, ty, tz, tw = _quat_mul(qx, qy, qz, qw, vx, vy, vz, 0.0)
    cx, cy, cz, cw = _quat_conj(qx, qy, qz, qw)
    rx, ry, rz, _rw = _quat_mul(tx, ty, tz, tw, cx, cy, cz, cw)
    return rx, ry, rz


def _normalize_quat(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0 or math.isinf(norm) or math.isnan(norm):
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return x * inv, y * inv, z * inv, w * inv


@dataclass(frozen=True)
class PoseSnapshot:
    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class JointAngleSnapshot:
    joint_name: str
    position_rad: float


@dataclass(frozen=True)
class RobotStateData:
    """Immutable robot state snapshot for gRPC streaming."""
    timestamp_ms: int
    pose: PoseSnapshot
    wheel_angles: tuple[JointAngleSnapshot, ...]


@dataclass(frozen=True)
class _TransformSnapshot:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    last_seen_monotonic: float


class UiBridgeRobotStateNode(Node):
    """ROS node that subscribes to odom/joints/TF and throttle-forwards to gRPC.
    
    Uses capped downsampling: forwards on arrival if rate cap allows,
    otherwise forwards on next timer tick. Never duplicates stale data.
    """

    def __init__(
        self,
        *,
        odom_topic: str,
        joint_states_topic: str,
        odom_frame: str,
        base_frame: str,
        map_frame: str,
        wheel_joint_names: list[str],
        map_tf_max_age_s: float,
        downsampling_period_s: float | None,
        grpc_broadcaster: AsyncStreamBroadcaster[RobotStateData],
    ) -> None:
        super().__init__('ui_bridge_robot_state')

        self._lock = threading.Lock()

        self._odom_topic = self.resolve_topic_name(str(odom_topic))
        self._joint_states_topic = self.resolve_topic_name(str(joint_states_topic))
        self._odom_frame = _normalize_frame(str(odom_frame))
        self._base_frame = _normalize_frame(str(base_frame))
        self._map_frame = _normalize_frame(str(map_frame))
        self._grpc_broadcaster = grpc_broadcaster

        self._wheel_joint_names = [str(name).strip() for name in wheel_joint_names if str(name).strip()]
        if not self._wheel_joint_names:
            self._wheel_joint_names = [
                'front_left_joint',
                'front_right_joint',
                'back_left_joint',
                'back_right_joint',
            ]

        self._map_tf_max_age_s = float(map_tf_max_age_s)

        self._have_odom = False
        self._odom_pose: Optional[PoseSnapshot] = None
        self._odom_stamp_ns: int = 0
        self._wheel_positions: dict[str, float] = {}
        self._map_to_odom: Optional[_TransformSnapshot] = None

        session = resolve_session()
        self._fixed_frame = session.fixed_frame
        self._session_current_launch_ref = session.current_launch_ref
        self._session_stack = session.stack
        self.get_logger().info(
            f"Fixed frame resolved from session: {self._fixed_frame} (stack={self._session_stack or 'unknown'})"
        )

        self._forwarder: ThrottledForwarder[RobotStateData] | None
        if downsampling_period_s is not None and float(downsampling_period_s) > 0:
            # Throttled forwarder for odom updates
            self._forwarder = ThrottledForwarder(
                period_s=float(downsampling_period_s),
                on_forward=self._on_forward,
            )
        else:
            self._forwarder = None

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, qos_best_effort)
        self._joint_sub = self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, qos_best_effort)
        self._tf_sub = self.create_subscription(TFMessage, '/tf', self._on_tf, qos_best_effort)

        # Timer for throttle (only when downsampling is enabled)
        self._timer = None
        if self._forwarder is not None:
            period_s = self._forwarder.period_s
            self._timer = self.create_timer(period_s, self._on_timer)
            self.get_logger().info(f"Robot state downsampling: {1.0/period_s:.1f} Hz cap")
        else:
            self.get_logger().info("Robot state downsampling: disabled (forward on every odom)")

    @property
    def odom_frame(self) -> str:
        return self._odom_frame

    @property
    def base_frame(self) -> str:
        return self._base_frame

    @property
    def map_frame(self) -> str:
        return self._map_frame

    @property
    def wheel_joint_names(self) -> list[str]:
        return list(self._wheel_joint_names)

    def _on_odom(self, msg: Odometry) -> None:
        """Called on every odom message. Updates state and triggers throttled forward."""
        pose = msg.pose.pose
        q = pose.orientation
        qx, qy, qz, qw = _normalize_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        snap = PoseSnapshot(
            frame_id=self._odom_frame,
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

        with self._lock:
            self._have_odom = True
            self._odom_pose = snap
            # Prefer ROS header stamp for time alignment (sim uses /clock).
            self._odom_stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds

        # Build current state and forward (throttled if enabled)
        state = self._build_state()
        if state is not None:
            if self._forwarder is None:
                self._grpc_broadcaster.publish_sync(state)
            else:
                self._forwarder.on_input(state)

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        with self._lock:
            for i, name in enumerate(msg.name):
                if i >= len(msg.position):
                    break
                joint_name = str(name)
                if joint_name in self._wheel_joint_names:
                    self._wheel_positions[joint_name] = float(msg.position[i])

    def _on_tf(self, msg: TFMessage) -> None:
        now = time.monotonic()
        for transform in msg.transforms:
            parent = _normalize_frame(transform.header.frame_id)
            child = _normalize_frame(transform.child_frame_id)
            if parent != self._map_frame or child != self._odom_frame:
                continue
            t = transform.transform.translation
            r = transform.transform.rotation
            qx, qy, qz, qw = _normalize_quat(float(r.x), float(r.y), float(r.z), float(r.w))
            snap = _TransformSnapshot(
                x=float(t.x),
                y=float(t.y),
                z=float(t.z),
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
                last_seen_monotonic=now,
            )
            with self._lock:
                self._map_to_odom = snap

    def _on_timer(self) -> None:
        """Called periodically. Lets throttler send pending if any."""
        if self._forwarder is not None:
            self._forwarder.on_timer()

    def _on_forward(self, state: RobotStateData) -> None:
        """Called by throttler when it's time to forward state."""
        self._grpc_broadcaster.publish_sync(state)

    def _build_state(self) -> Optional[RobotStateData]:
        """Build current robot state snapshot. Returns None if no odom yet."""
        now = time.monotonic()

        with self._lock:
            have_odom = self._have_odom
            odom_pose = self._odom_pose
            odom_stamp_ns = int(self._odom_stamp_ns)
            map_to_odom = self._map_to_odom
            wheel_positions = dict(self._wheel_positions)

        if odom_pose is None:
            return None

        # Deterministic fixed frame choice is resolved from the launch session.
        # We never publish a mix of odom/map poses in the same stream.
        pose_fixed: Optional[PoseSnapshot]
        if self._fixed_frame == self._odom_frame or self._fixed_frame == 'odom':
            pose_fixed = odom_pose
        else:
            pose_fixed = None
            if have_odom and map_to_odom is not None:
                if self._map_tf_max_age_s <= 0.0 or (now - map_to_odom.last_seen_monotonic) <= self._map_tf_max_age_s:
                    rx, ry, rz = _rotate_vec_by_quat(
                        odom_pose.x,
                        odom_pose.y,
                        odom_pose.z,
                        map_to_odom.qx,
                        map_to_odom.qy,
                        map_to_odom.qz,
                        map_to_odom.qw,
                    )
                    px = map_to_odom.x + rx
                    py = map_to_odom.y + ry
                    pz = map_to_odom.z + rz
                    qx, qy, qz, qw = _quat_mul(
                        map_to_odom.qx,
                        map_to_odom.qy,
                        map_to_odom.qz,
                        map_to_odom.qw,
                        odom_pose.qx,
                        odom_pose.qy,
                        odom_pose.qz,
                        odom_pose.qw,
                    )
                    qx, qy, qz, qw = _normalize_quat(qx, qy, qz, qw)
                    pose_fixed = PoseSnapshot(
                        frame_id=self._map_frame,
                        x=px,
                        y=py,
                        z=pz,
                        qx=qx,
                        qy=qy,
                        qz=qz,
                        qw=qw,
                    )

        if pose_fixed is None:
            return None

        wheel_angles = tuple(
            JointAngleSnapshot(joint_name=name, position_rad=float(wheel_positions.get(name, 0.0)))
            for name in self._wheel_joint_names
        )

        # Use odom header time if present; otherwise fall back to node clock.
        stamp_ns = odom_stamp_ns
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds

        return RobotStateData(
            timestamp_ms=int(stamp_ns // 1_000_000),
            pose=pose_fixed,
            wheel_angles=wheel_angles,
        )

