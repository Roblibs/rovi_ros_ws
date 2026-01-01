"""Rate-capped passthrough with minimal latency.

The ThrottledForwarder implements "capped downsampling":
- Forward immediately on arrival if rate cap allows
- Otherwise store as pending and forward on next timer tick
- Never duplicates: only forwards new data
- Never sends stale: pending is cleared after forward
"""

from __future__ import annotations

import asyncio
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, Optional, TypeVar

T = TypeVar('T')


@dataclass
class ThrottleConfig:
    """Configuration for throttled forwarding.
    
    Specify either rate_hz OR period_s. If both given, period_s takes precedence.
    """
    rate_hz: Optional[float] = None
    period_s: Optional[float] = None

    def get_period_s(self) -> float:
        """Return the period in seconds, defaulting to 0.1s if unspecified."""
        if self.period_s is not None and self.period_s > 0:
            return self.period_s
        if self.rate_hz is not None and self.rate_hz > 0:
            return 1.0 / self.rate_hz
        return 0.1  # default 10 Hz


class ThrottledForwarder(Generic[T]):
    """Rate-capped passthrough forwarder.
    
    Usage:
        forwarder = ThrottledForwarder(period_s=0.5, on_forward=my_callback)
        
        # On data arrival (from ROS callback, etc.):
        forwarder.on_input(data)
        
        # From a periodic timer:
        forwarder.on_timer()
    
    The on_forward callback is invoked at most once per period_s, with the latest data.
    If data arrives slower than the rate cap, it's forwarded immediately (minimal latency).
    """

    def __init__(
        self,
        period_s: float,
        on_forward: Callable[[T], None],
    ) -> None:
        self._period_s = float(period_s)
        self._on_forward = on_forward
        self._last_forward_time: float = 0.0
        self._pending: Optional[T] = None
        self._lock = asyncio.Lock()

    @property
    def period_s(self) -> float:
        return self._period_s

    def on_input(self, data: T) -> None:
        """Called when new data arrives. May forward immediately or queue for timer."""
        now = time.monotonic()
        if now - self._last_forward_time >= self._period_s:
            self._forward(data, now)
        else:
            self._pending = data

    def on_timer(self) -> None:
        """Called periodically. Forwards pending data if any."""
        if self._pending is not None:
            self._forward(self._pending, time.monotonic())
            self._pending = None

    def _forward(self, data: T, now: float) -> None:
        self._last_forward_time = now
        self._pending = None
        try:
            self._on_forward(data)
        except Exception:
            pass  # Don't let callback errors break the forwarder


class AsyncStreamBroadcaster(Generic[T]):
    """Broadcasts data to multiple async queue subscribers.
    
    Each subscriber gets their own queue. No caching — if a subscriber
    connects after data was sent, they wait for the next item.
    
    Usage:
        broadcaster = AsyncStreamBroadcaster()
        
        # In gRPC stream handler:
        async for item in broadcaster.subscribe():
            yield item
        
        # When data arrives:
        await broadcaster.publish(data)
    """

    def __init__(self, max_queue_size: int = 1) -> None:
        self._subscribers: set[asyncio.Queue[T]] = set()
        self._lock = asyncio.Lock()
        self._max_queue_size = max_queue_size

    async def subscribe(self):
        """Async generator that yields items as they're published."""
        queue: asyncio.Queue[T] = asyncio.Queue(maxsize=self._max_queue_size)
        async with self._lock:
            self._subscribers.add(queue)
        try:
            while True:
                item = await queue.get()
                yield item
        finally:
            async with self._lock:
                self._subscribers.discard(queue)

    async def publish(self, data: T) -> None:
        """Publish data to all subscribers. Non-blocking, drops if queue full."""
        async with self._lock:
            for queue in self._subscribers:
                try:
                    queue.put_nowait(data)
                except asyncio.QueueFull:
                    # Drop oldest, add new (keep latest)
                    try:
                        queue.get_nowait()
                        queue.put_nowait(data)
                    except (asyncio.QueueEmpty, asyncio.QueueFull):
                        pass

    def publish_sync(self, data: T) -> None:
        """Synchronous publish for use from ROS callbacks. Uses event loop if available."""
        try:
            loop = asyncio.get_running_loop()
            loop.call_soon_threadsafe(self._sync_publish, data)
        except RuntimeError:
            # No event loop running, publish directly (may block)
            self._sync_publish(data)

    def _sync_publish(self, data: T) -> None:
        """Direct sync publish to all queues."""
        for queue in list(self._subscribers):
            try:
                queue.put_nowait(data)
            except asyncio.QueueFull:
                try:
                    queue.get_nowait()
                    queue.put_nowait(data)
                except (asyncio.QueueEmpty, asyncio.QueueFull):
                    pass

    @property
    def subscriber_count(self) -> int:
        return len(self._subscribers)
