"""gRPC service implementation for UI bridge.

Uses queue-based streaming (AsyncStreamBroadcaster) for robot_state and lidar.
Status stream uses StatusBroadcaster (timer-based collection with staleness filtering done upstream).
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Sequence
from typing import Optional

import grpc

from .api import ui_bridge_pb2, ui_bridge_pb2_grpc
from .config import ControlConfig
from .conductor.systemd import UnitStatus, control_unit, get_unit_status, stack_to_unit
from .floor_topology_node import FloorPolylineData, FloorTopologyData
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
        floor_topology_broadcaster: Optional[AsyncStreamBroadcaster[FloorTopologyData]],
        model_provider: RobotModelProvider,
        model_chunk_size_bytes: int,
        odom_frame: str,
        base_frame: str,
        map_frame: str,
        wheel_joint_names: list[str],
        control: ControlConfig,
    ) -> None:
        self._status_broadcaster = status_broadcaster
        self._robot_state_broadcaster = robot_state_broadcaster
        self._lidar_broadcaster = lidar_broadcaster
        self._map_broadcaster = map_broadcaster
        self._floor_topology_broadcaster = floor_topology_broadcaster
        self._model_provider = model_provider
        self._model_chunk_size_bytes = int(model_chunk_size_bytes)
        self._odom_frame = str(odom_frame)
        self._base_frame = str(base_frame)
        self._map_frame = str(map_frame)
        self._wheel_joint_names = list(wheel_joint_names)
        self._control = control

    def _ensure_allowed_stack(self, stack: str) -> str:
        key = str(stack).strip().lower()
        if not key:
            raise ValueError("Missing stack name")
        if self._control.allowed_stacks and key not in set(self._control.allowed_stacks):
            raise ValueError(f"Stack not allowed: {key}")
        return key

    def _stack_unit(self, stack: str) -> str:
        key = self._ensure_allowed_stack(stack)
        return stack_to_unit(key, unit_prefix=self._control.unit_prefix)

    @staticmethod
    def _unit_status_to_proto(stack: str, status: UnitStatus) -> ui_bridge_pb2.StackStatus:
        return ui_bridge_pb2.StackStatus(
            stack=str(stack),
            unit=str(status.unit),
            load_state=str(status.load_state),
            active_state=str(status.active_state),
            sub_state=str(status.sub_state),
            unit_file_state=str(status.unit_file_state),
            result=str(status.result),
            exec_main_code=int(status.exec_main_code),
            exec_main_status=int(status.exec_main_status),
        )

    async def GetStackStatus(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.StackStatusRequest,
        context: grpc.aio.ServicerContext,
    ) -> ui_bridge_pb2.StackStatus:
        try:
            unit = self._stack_unit(request.stack)
        except ValueError as exc:
            await context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))
            raise AssertionError("unreachable") from exc

        try:
            status = await asyncio.to_thread(get_unit_status, unit)
        except Exception as exc:  # noqa: BLE001
            await context.abort(grpc.StatusCode.INTERNAL, str(exc))
            raise AssertionError("unreachable") from exc

        return self._unit_status_to_proto(request.stack, status)

    async def ListStacks(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.ListStacksRequest,
        context: grpc.aio.ServicerContext,
    ) -> ui_bridge_pb2.ListStacksResponse:
        del request

        stacks = list(self._control.allowed_stacks or [])
        resp = ui_bridge_pb2.ListStacksResponse()
        for stack in stacks:
            try:
                unit = self._stack_unit(stack)
                status = await asyncio.to_thread(get_unit_status, unit)
                resp.stacks.append(self._unit_status_to_proto(stack, status))
            except Exception:
                # Best-effort: omit stacks that can't be queried (unit missing, etc.).
                continue
        return resp

    async def StartStack(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.StackControlRequest,
        context: grpc.aio.ServicerContext,
    ) -> ui_bridge_pb2.StackControlResponse:
        return await self._control_stack(request, context, verb="start")

    async def StopStack(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.StackControlRequest,
        context: grpc.aio.ServicerContext,
    ) -> ui_bridge_pb2.StackControlResponse:
        return await self._control_stack(request, context, verb="stop")

    async def RestartStack(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.StackControlRequest,
        context: grpc.aio.ServicerContext,
    ) -> ui_bridge_pb2.StackControlResponse:
        return await self._control_stack(request, context, verb="restart")

    async def _control_stack(
        self,
        request: ui_bridge_pb2.StackControlRequest,
        context: grpc.aio.ServicerContext,
        *,
        verb: str,
    ) -> ui_bridge_pb2.StackControlResponse:
        if not self._control.enabled:
            await context.abort(grpc.StatusCode.FAILED_PRECONDITION, "Stack control is disabled by config")
            raise AssertionError("unreachable")

        try:
            unit = self._stack_unit(request.stack)
        except ValueError as exc:
            await context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))
            raise AssertionError("unreachable") from exc

        ok, message, status = await asyncio.to_thread(control_unit, unit, verb)
        return ui_bridge_pb2.StackControlResponse(
            ok=bool(ok),
            message=str(message),
            status=self._unit_status_to_proto(request.stack, status),
        )

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

    async def StreamFloorTopology(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.FloorTopologyRequest,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[ui_bridge_pb2.FloorTopologyUpdate]:
        """Stream floor topology updates. Queue-based, no stale data on subscribe."""
        del request

        if self._floor_topology_broadcaster is None:
            await context.abort(grpc.StatusCode.UNAVAILABLE, "Floor topology stream not configured")
            return

        seq = 0
        async for topo in self._floor_topology_broadcaster.subscribe():
            if context.cancelled():
                return
            seq += 1
            yield _floor_topology_to_proto(topo, seq)

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


def _floor_topology_to_proto(topo: FloorTopologyData, seq: int) -> ui_bridge_pb2.FloorTopologyUpdate:
    msg = ui_bridge_pb2.FloorTopologyUpdate(
        timestamp_unix_ms=topo.timestamp_ms,
        seq=seq,
    )

    msg.polylines.extend(_floor_polyline_to_proto(pl) for pl in topo.polylines)
    return msg


def _floor_polyline_to_proto(pl: FloorPolylineData) -> ui_bridge_pb2.FloorPolyline:
    msg = ui_bridge_pb2.FloorPolyline(
        ns=str(pl.ns),
        id=int(pl.id),
        frame_id=str(pl.frame_id),
        closed=bool(pl.closed),
    )
    msg.points.extend(ui_bridge_pb2.Point3(x=float(p.x), y=float(p.y), z=float(p.z)) for p in pl.points)
    return msg
