from __future__ import annotations

import asyncio
from dataclasses import dataclass
import time
from typing import Optional


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
class RobotStateSnapshot:
    seq: int
    timestamp_unix_ms: int
    pose_odom: PoseSnapshot
    pose_map: Optional[PoseSnapshot]
    wheel_angles: list[JointAngleSnapshot]


class RobotStateBroadcaster:
    def __init__(self) -> None:
        self._condition = asyncio.Condition()
        self._seq = 0
        self._latest: Optional[RobotStateSnapshot] = None

    def latest(self) -> Optional[RobotStateSnapshot]:
        return self._latest

    async def publish(
        self,
        *,
        pose_odom: PoseSnapshot,
        pose_map: Optional[PoseSnapshot],
        wheel_angles: list[JointAngleSnapshot],
    ) -> RobotStateSnapshot:
        async with self._condition:
            self._seq += 1
            snapshot = RobotStateSnapshot(
                seq=self._seq,
                timestamp_unix_ms=int(time.time() * 1000),
                pose_odom=pose_odom,
                pose_map=pose_map,
                wheel_angles=wheel_angles,
            )
            self._latest = snapshot
            self._condition.notify_all()
            return snapshot

    async def wait_for_next(self, last_seq: int) -> RobotStateSnapshot:
        async with self._condition:
            await self._condition.wait_for(lambda: self._latest is not None and self._seq > last_seq)
            assert self._latest is not None
            return self._latest

