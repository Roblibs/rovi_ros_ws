from __future__ import annotations

import asyncio
from dataclasses import dataclass
import time
from typing import Optional


@dataclass(frozen=True)
class RateMetricSnapshot:
    id: str
    hz: float
    target_hz: Optional[float]


@dataclass(frozen=True)
class StatusSnapshot:
    seq: int
    timestamp_unix_ms: int
    cpu_percent: float
    voltage_v: Optional[float]
    rates: list[RateMetricSnapshot]

    # Session context (constant per run; helps UIs understand fixed-frame choice).
    current_launch_ref: Optional[str]
    stack: Optional[str]
    fixed_frame: Optional[str]


class SnapshotBroadcaster:
    def __init__(
        self,
        *,
        current_launch_ref: Optional[str] = None,
        stack: Optional[str] = None,
        fixed_frame: Optional[str] = None,
    ) -> None:
        self._condition = asyncio.Condition()
        self._seq = 0
        self._latest: Optional[StatusSnapshot] = None

        self._current_launch_ref = current_launch_ref
        self._stack = stack
        self._fixed_frame = fixed_frame

    def latest(self) -> Optional[StatusSnapshot]:
        return self._latest

    async def publish(
        self,
        *,
        cpu_percent: float,
        voltage_v: Optional[float],
        rates: list[RateMetricSnapshot],
    ) -> StatusSnapshot:
        async with self._condition:
            self._seq += 1
            snapshot = StatusSnapshot(
                seq=self._seq,
                timestamp_unix_ms=int(time.time() * 1000),
                cpu_percent=cpu_percent,
                voltage_v=voltage_v,
                rates=rates,
                current_launch_ref=self._current_launch_ref,
                stack=self._stack,
                fixed_frame=self._fixed_frame,
            )
            self._latest = snapshot
            self._condition.notify_all()
            return snapshot

    async def wait_for_next(self, last_seq: int) -> StatusSnapshot:
        async with self._condition:
            await self._condition.wait_for(lambda: self._latest is not None and self._seq > last_seq)
            assert self._latest is not None
            return self._latest
