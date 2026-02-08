from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from typing import Optional

from builtin_interfaces.msg import Time as RosTime


@dataclass(frozen=True)
class StatusFieldMeta:
    id: str
    unit: str
    value_type: str  # float | text (default float)
    min: Optional[float]
    max: Optional[float]
    target: Optional[float]


@dataclass(frozen=True)
class StatusFieldValue:
    id: str
    value: float
    text: Optional[str]
    stamp: RosTime


@dataclass(frozen=True)
class StatusSnapshot:
    seq: int
    stamp: RosTime
    wall_time_unix_ms: Optional[int]
    fields: list[StatusFieldMeta]
    values: list[StatusFieldValue]
    current_launch_ref: Optional[str]
    stack: Optional[str]
    fixed_frame: Optional[str]


class StatusBroadcaster:
    """Caches and broadcasts status snapshots (values only; metadata is fixed at init)."""

    def __init__(
        self,
        *,
        fields: list[StatusFieldMeta],
        current_launch_ref: Optional[str] = None,
        stack: Optional[str] = None,
        fixed_frame: Optional[str] = None,
    ) -> None:
        self._condition = asyncio.Condition()
        self._seq = 0
        self._latest: Optional[StatusSnapshot] = None

        self._fields = list(fields)
        self._current_launch_ref = current_launch_ref
        self._stack = stack
        self._fixed_frame = fixed_frame

    @property
    def fields(self) -> list[StatusFieldMeta]:
        return self._fields

    @property
    def current_launch_ref(self) -> Optional[str]:
        return self._current_launch_ref

    @property
    def stack(self) -> Optional[str]:
        return self._stack

    @property
    def fixed_frame(self) -> Optional[str]:
        return self._fixed_frame

    def latest(self) -> Optional[StatusSnapshot]:
        return self._latest

    async def publish(self, snapshot: StatusSnapshot) -> StatusSnapshot:
        async with self._condition:
            self._seq = snapshot.seq
            self._latest = snapshot
            self._condition.notify_all()
            return snapshot

    async def wait_for_next(self, last_seq: int) -> StatusSnapshot:
        async with self._condition:
            await self._condition.wait_for(lambda: self._latest is not None and self._seq > last_seq)
            assert self._latest is not None
            return self._latest

    def build_snapshot(self, *, seq: int, stamp: RosTime, values: list[StatusFieldValue]) -> StatusSnapshot:
        # Update seq tracking for waiters.
        self._seq = seq
        return StatusSnapshot(
            seq=seq,
            stamp=stamp,
            wall_time_unix_ms=int(time.time() * 1000),
            fields=self._fields,
            values=values,
            current_launch_ref=self._current_launch_ref,
            stack=self._stack,
            fixed_frame=self._fixed_frame,
        )
