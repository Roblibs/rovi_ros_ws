from __future__ import annotations

from collections.abc import AsyncIterator

import grpc

from .api import ui_gateway_pb2, ui_gateway_pb2_grpc
from .status_store import SnapshotBroadcaster, StatusSnapshot


class UiGatewayService(ui_gateway_pb2_grpc.UiGatewayServicer):
    def __init__(self, broadcaster: SnapshotBroadcaster) -> None:
        self._broadcaster = broadcaster

    async def GetStatus(  # noqa: N802 - gRPC interface name
        self,
        request: ui_gateway_pb2.StatusRequest,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[ui_gateway_pb2.StatusUpdate]:
        last_seq = 0
        latest = self._broadcaster.latest()
        if latest is not None:
            last_seq = latest.seq
            yield _snapshot_to_proto(latest)

        while True:
            snapshot = await self._broadcaster.wait_for_next(last_seq)
            if context.cancelled():
                return
            last_seq = snapshot.seq
            yield _snapshot_to_proto(snapshot)


def _snapshot_to_proto(snapshot: StatusSnapshot) -> ui_gateway_pb2.StatusUpdate:
    msg = ui_gateway_pb2.StatusUpdate(
        timestamp_unix_ms=snapshot.timestamp_unix_ms,
        seq=snapshot.seq,
        cpu_percent=snapshot.cpu_percent,
    )
    if snapshot.voltage_v is not None:
        msg.voltage_v = snapshot.voltage_v
    return msg

