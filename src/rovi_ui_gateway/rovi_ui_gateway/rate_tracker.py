from __future__ import annotations

import threading
import time
from typing import Optional


class RateTracker:
    def __init__(self, *, target_hz: float | None, emit_zero_when_unseen: bool) -> None:
        self.target_hz = target_hz
        self.emit_zero_when_unseen = emit_zero_when_unseen

        self._lock = threading.Lock()
        self._ever_seen = False
        self._count_since_sample = 0
        self._last_sample_monotonic = time.monotonic()

    def on_event(self, timestamp_monotonic: float) -> None:
        del timestamp_monotonic
        with self._lock:
            self._ever_seen = True
            self._count_since_sample += 1

    def sample_hz(self, *, now_monotonic: float) -> Optional[float]:
        with self._lock:
            elapsed = now_monotonic - self._last_sample_monotonic
            if elapsed <= 0:
                elapsed = 1e-6
            count = self._count_since_sample
            self._count_since_sample = 0
            self._last_sample_monotonic = now_monotonic
            ever_seen = self._ever_seen

        if not ever_seen and not self.emit_zero_when_unseen:
            return None
        return float(count) / float(elapsed)

