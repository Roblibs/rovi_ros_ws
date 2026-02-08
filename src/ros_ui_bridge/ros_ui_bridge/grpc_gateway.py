"""gRPC service implementation for UI bridge.

Uses queue-based streaming (AsyncStreamBroadcaster) for robot_state and lidar.
Status stream uses StatusBroadcaster (timer-based collection with staleness filtering done upstream).
"""

from __future__ import annotations

from collections.abc import AsyncIterator, Sequence
from typing import Optional

import grpc

from .api import ui_bridge_pb2, ui_bridge_pb2_grpc
from .lidar_node import LidarScanData
from .map_node import MapData
from .robot_model_provider import RobotModelProvider
from .robot_state_node import JointAngleSnapshot, PoseSnapshot, RobotStateData
from .status_store import StatusBroadcaster, StatusSnapshot
from .throttled_forwarder import AsyncStreamBroadcaster


class UiBridgeService(ui_bridge_pb2_grpc.UiBridgeServicer):
    """gRPC service for UI bridge streams."""

    def __init__(
        self,
        *,
        status_broadcaster: StatusBroadcaster,
        robot_state_broadcaster: AsyncStreamBroadcaster[RobotStateData],
        lidar_broadcaster: Optional[AsyncStreamBroadcaster[LidarScanData]],
        map_broadcaster: Optional[AsyncStreamBroadcaster[MapData]],
        model_provider: RobotModelProvider,
        model_chunk_size_bytes: int,
        odom_frame: str,
        base_frame: str,
        map_frame: str,
        wheel_joint_names: list[str],
    ) -> None:
        self._status_broadcaster = status_broadcaster
        self._robot_state_broadcaster = robot_state_broadcaster
        self._lidar_broadcaster = lidar_broadcaster
        self._map_broadcaster = map_broadcaster
        self._model_provider = model_provider
        self._model_chunk_size_bytes = int(model_chunk_size_bytes)
        self._odom_frame = str(odom_frame)
        self._base_frame = str(base_frame)
        self._map_frame = str(map_frame)
        self._wheel_joint_names = list(wheel_joint_names)

    async def GetStatus(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.StatusRequest,
        context: grpc.aio.ServicerContext,
    ) -> ui_bridge_pb2.StatusSnapshot:
        del request

        snapshot = self._status_broadcaster.latest()
        if snapshot is None:
            snapshot = self._build_empty_snapshot()
        return _snapshot_to_proto(snapshot)

    async def StreamStatus(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.StatusRequest,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[ui_bridge_pb2.StatusUpdate]:
        """Stream status updates (values only)."""
        del request

        last_seq = 0
        latest = self._status_broadcaster.latest()
        if latest is not None:
            last_seq = latest.seq
            yield _snapshot_to_update_proto(latest)

        while True:
            snapshot = await self._status_broadcaster.wait_for_next(last_seq)
            if context.cancelled():
                return
            last_seq = snapshot.seq
            yield _snapshot_to_update_proto(snapshot)

    def _build_empty_snapshot(self) -> StatusSnapshot:
        from builtin_interfaces.msg import Time as RosTime  # Local import to avoid ROS overhead on module load.

        return StatusSnapshot(
            seq=0,
            stamp=RosTime(),
            wall_time_unix_ms=None,
            fields=self._status_broadcaster.fields,
            values=[],
            current_launch_ref=self._status_broadcaster.current_launch_ref,
            stack=self._status_broadcaster.stack,
            fixed_frame=self._status_broadcaster.fixed_frame,
        )

    async def StreamRobotState(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.RobotStateRequest,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[ui_bridge_pb2.RobotStateUpdate]:
        """Stream robot state updates. Queue-based, no stale data on subscribe."""
        del request

        seq = 0
        async for state in self._robot_state_broadcaster.subscribe():
            if context.cancelled():
                return
            seq += 1
            yield _robot_state_to_proto(state, seq)

    async def StreamLidar(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.LidarRequest,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[ui_bridge_pb2.LidarUpdate]:
        """Stream lidar updates. Queue-based, no stale data on subscribe."""
        del request

        if self._lidar_broadcaster is None:
            await context.abort(grpc.StatusCode.UNAVAILABLE, "Lidar stream not configured")
            return

        seq = 0
        async for scan in self._lidar_broadcaster.subscribe():
            if context.cancelled():
                return
            seq += 1
            yield _lidar_to_proto(scan, seq)

    async def StreamMap(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.MapRequest,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[ui_bridge_pb2.MapUpdate]:
        """Stream map updates. Queue-based, no stale data on subscribe."""
        del request

        if self._map_broadcaster is None:
            await context.abort(grpc.StatusCode.UNAVAILABLE, "Map stream not configured")
            return

        seq = 0
        async for m in self._map_broadcaster.subscribe():
            if context.cancelled():
                return
            seq += 1
            yield _map_to_proto(m, seq)

    async def GetRobotModelMeta(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.RobotModelRequest,
        context: grpc.aio.ServicerContext,
    ) -> ui_bridge_pb2.RobotModelMeta:
        del request

        try:
            model_meta = self._model_provider.load_meta()
        except FileNotFoundError as exc:
            await context.abort(grpc.StatusCode.NOT_FOUND, str(exc))
            raise AssertionError("unreachable") from exc
        except Exception as exc:  # noqa: BLE001
            await context.abort(grpc.StatusCode.INTERNAL, f"Failed to load robot model: {exc}")
            raise AssertionError("unreachable") from exc

        return ui_bridge_pb2.RobotModelMeta(
            sha256=model_meta.sha256,
            size_bytes=model_meta.size_bytes,
            wheel_joint_names=self._wheel_joint_names,
            odom_frame=self._odom_frame,
            base_frame=self._base_frame,
            map_frame=self._map_frame,
        )

    async def GetRobotModel(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.RobotModelRequest,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[ui_bridge_pb2.RobotModelChunk]:
        del request

        try:
            glb_path = self._model_provider.resolve_glb_path()
            glb_file = glb_path.open("rb")
        except FileNotFoundError as exc:
            await context.abort(grpc.StatusCode.NOT_FOUND, str(exc))
            return
        except Exception as exc:  # noqa: BLE001
            await context.abort(grpc.StatusCode.INTERNAL, f"Failed to load robot model: {exc}")
            return

        chunk_size = self._model_chunk_size_bytes
        if chunk_size <= 0:
            chunk_size = 1024 * 1024

        chunk_index = 0
        with glb_file as f:
            while True:
                chunk = f.read(chunk_size)
                if not chunk:
                    break
                yield ui_bridge_pb2.RobotModelChunk(chunk=chunk, chunk_index=chunk_index)
                chunk_index += 1


# --- Proto conversion helpers ---


def _time_to_proto(stamp) -> ui_bridge_pb2.Time:
    return ui_bridge_pb2.Time(sec=int(stamp.sec), nanosec=int(stamp.nanosec))


def _field_meta_to_proto(meta) -> ui_bridge_pb2.StatusFieldMeta:
    msg = ui_bridge_pb2.StatusFieldMeta(id=meta.id, unit=meta.unit)
    if getattr(meta, 'value_type', None) == 'text':
        msg.type = ui_bridge_pb2.StatusFieldMeta.StatusFieldType.STATUS_FIELD_TYPE_TEXT
    if meta.min is not None:
        msg.min = meta.min
    if meta.max is not None:
        msg.max = meta.max
    if meta.target is not None:
        msg.target = meta.target
    return msg


def _field_value_to_proto(value) -> ui_bridge_pb2.StatusFieldValue:
    msg = ui_bridge_pb2.StatusFieldValue(id=value.id, value=float(value.value), stamp=_time_to_proto(value.stamp))
    text = getattr(value, 'text', None)
    if text is not None:
        msg.text = str(text)
    return msg


def _snapshot_to_proto(snapshot: StatusSnapshot) -> ui_bridge_pb2.StatusSnapshot:
    msg = ui_bridge_pb2.StatusSnapshot(
        stamp=_time_to_proto(snapshot.stamp),
        seq=snapshot.seq,
    )
    if snapshot.wall_time_unix_ms is not None:
        msg.wall_time_unix_ms = snapshot.wall_time_unix_ms
    msg.fields.extend(_field_meta_to_proto(meta) for meta in snapshot.fields)
    msg.values.extend(_field_value_to_proto(val) for val in snapshot.values)
    if getattr(snapshot, 'current_launch_ref', None):
        msg.current_launch_ref = str(snapshot.current_launch_ref)
    if getattr(snapshot, 'stack', None):
        msg.stack = str(snapshot.stack)
    if getattr(snapshot, 'fixed_frame', None):
        msg.fixed_frame = str(snapshot.fixed_frame)
    return msg


def _snapshot_to_update_proto(snapshot: StatusSnapshot) -> ui_bridge_pb2.StatusUpdate:
    msg = ui_bridge_pb2.StatusUpdate(
        stamp=_time_to_proto(snapshot.stamp),
        seq=snapshot.seq,
    )
    if snapshot.wall_time_unix_ms is not None:
        msg.wall_time_unix_ms = snapshot.wall_time_unix_ms
    msg.values.extend(_field_value_to_proto(val) for val in snapshot.values)
    if getattr(snapshot, 'current_launch_ref', None):
        msg.current_launch_ref = str(snapshot.current_launch_ref)
    if getattr(snapshot, 'stack', None):
        msg.stack = str(snapshot.stack)
    if getattr(snapshot, 'fixed_frame', None):
        msg.fixed_frame = str(snapshot.fixed_frame)
    return msg


def _pose_to_proto(pose: PoseSnapshot) -> ui_bridge_pb2.Pose3D:
    return ui_bridge_pb2.Pose3D(
        frame_id=pose.frame_id,
        x=float(pose.x),
        y=float(pose.y),
        z=float(pose.z),
        qx=float(pose.qx),
        qy=float(pose.qy),
        qz=float(pose.qz),
        qw=float(pose.qw),
    )


def _joint_angles_to_proto(angles: Sequence[JointAngleSnapshot]) -> list[ui_bridge_pb2.JointAngle]:
    return [
        ui_bridge_pb2.JointAngle(joint_name=angle.joint_name, position_rad=float(angle.position_rad))
        for angle in angles
    ]


def _robot_state_to_proto(state: RobotStateData, seq: int) -> ui_bridge_pb2.RobotStateUpdate:
    msg = ui_bridge_pb2.RobotStateUpdate(
        timestamp_unix_ms=state.timestamp_ms,
        seq=seq,
        pose=_pose_to_proto(state.pose),
    )
    msg.wheel_angles.extend(_joint_angles_to_proto(state.wheel_angles))
    return msg


def _lidar_to_proto(scan: LidarScanData, seq: int) -> ui_bridge_pb2.LidarUpdate:
    return ui_bridge_pb2.LidarUpdate(
        timestamp_unix_ms=scan.timestamp_ms,
        seq=seq,
        frame_id=scan.frame_id,
        angle_min=float(scan.angle_min),
        angle_increment=float(scan.angle_increment),
        range_min=float(scan.range_min),
        range_max=float(scan.range_max),
        ranges=list(scan.ranges),
    )


def _map_to_proto(m: MapData, seq: int) -> ui_bridge_pb2.MapUpdate:
    return ui_bridge_pb2.MapUpdate(
        timestamp_unix_ms=m.timestamp_ms,
        seq=seq,
        frame_id=m.frame_id,
        resolution_m_per_px=float(m.resolution_m_per_px),
        width=int(m.width),
        height=int(m.height),
        origin=ui_bridge_pb2.Pose3D(
            frame_id=m.frame_id,
            x=float(m.origin.x),
            y=float(m.origin.y),
            z=float(m.origin.z),
            qx=float(m.origin.qx),
            qy=float(m.origin.qy),
            qz=float(m.origin.qz),
            qw=float(m.origin.qw),
        ),
        png=bytes(m.png),
    )
