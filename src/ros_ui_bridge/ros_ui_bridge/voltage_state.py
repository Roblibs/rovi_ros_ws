from __future__ import annotations

import threading
import time
from typing import Optional


class VoltageState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value_v: Optional[float] = None
        self._last_update_monotonic: Optional[float] = None

    def update(self, value_v: float) -> None:
        with self._lock:
            self._value_v = value_v
            self._last_update_monotonic = time.monotonic()

    def read(self) -> tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self._value_v, self._last_update_monotonic
