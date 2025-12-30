from __future__ import annotations

from collections.abc import AsyncIterator

import grpc

from .api import ui_bridge_pb2, ui_bridge_pb2_grpc
from .robot_model_provider import RobotModelProvider
from .robot_state_store import JointAngleSnapshot, PoseSnapshot, RobotStateBroadcaster, RobotStateSnapshot
from .status_store import RateMetricSnapshot, SnapshotBroadcaster, StatusSnapshot


class UiBridgeService(ui_bridge_pb2_grpc.UiBridgeServicer):
    def __init__(
        self,
        *,
        status_broadcaster: SnapshotBroadcaster,
        robot_state_broadcaster: RobotStateBroadcaster,
        model_provider: RobotModelProvider,
        model_chunk_size_bytes: int,
        odom_frame: str,
        base_frame: str,
        map_frame: str,
        wheel_joint_names: list[str],
    ) -> None:
        self._status_broadcaster = status_broadcaster
        self._robot_state_broadcaster = robot_state_broadcaster
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
    ) -> AsyncIterator[ui_bridge_pb2.StatusUpdate]:
        last_seq = 0
        latest = self._status_broadcaster.latest()
        if latest is not None:
            last_seq = latest.seq
            yield _snapshot_to_proto(latest)

        while True:
            snapshot = await self._status_broadcaster.wait_for_next(last_seq)
            if context.cancelled():
                return
            last_seq = snapshot.seq
            yield _snapshot_to_proto(snapshot)

    async def StreamRobotState(  # noqa: N802 - gRPC interface name
        self,
        request: ui_bridge_pb2.RobotStateRequest,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[ui_bridge_pb2.RobotStateUpdate]:
        del request

        last_seq = 0
        latest = self._robot_state_broadcaster.latest()
        if latest is not None:
            last_seq = latest.seq
            yield _robot_state_to_proto(latest)

        while True:
            snapshot = await self._robot_state_broadcaster.wait_for_next(last_seq)
            if context.cancelled():
                return
            last_seq = snapshot.seq
            yield _robot_state_to_proto(snapshot)

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


def _snapshot_to_proto(snapshot: StatusSnapshot) -> ui_bridge_pb2.StatusUpdate:
    msg = ui_bridge_pb2.StatusUpdate(
        timestamp_unix_ms=snapshot.timestamp_unix_ms,
        seq=snapshot.seq,
        cpu_percent=snapshot.cpu_percent,
    )
    if snapshot.voltage_v is not None:
        msg.voltage_v = snapshot.voltage_v
    msg.rates.extend(_rate_metrics_to_proto(snapshot.rates))
    return msg


def _rate_metrics_to_proto(metrics: list[RateMetricSnapshot]) -> list[ui_bridge_pb2.RateMetric]:
    out: list[ui_bridge_pb2.RateMetric] = []
    for metric in metrics:
        m = ui_bridge_pb2.RateMetric(id=metric.id, hz=metric.hz)
        if metric.target_hz is not None:
            m.target_hz = metric.target_hz
        out.append(m)
    return out


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


def _joint_angles_to_proto(angles: list[JointAngleSnapshot]) -> list[ui_bridge_pb2.JointAngle]:
    return [
        ui_bridge_pb2.JointAngle(joint_name=angle.joint_name, position_rad=float(angle.position_rad))
        for angle in angles
    ]


def _robot_state_to_proto(snapshot: RobotStateSnapshot) -> ui_bridge_pb2.RobotStateUpdate:
    msg = ui_bridge_pb2.RobotStateUpdate(
        timestamp_unix_ms=snapshot.timestamp_unix_ms,
        seq=snapshot.seq,
        pose_odom=_pose_to_proto(snapshot.pose_odom),
    )
    if snapshot.pose_map is not None:
        msg.pose_map.CopyFrom(_pose_to_proto(snapshot.pose_map))
    msg.wheel_angles.extend(_joint_angles_to_proto(snapshot.wheel_angles))
    return msg
