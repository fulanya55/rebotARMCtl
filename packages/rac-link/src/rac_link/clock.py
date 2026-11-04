"""Host clock and the mapping between controller time and host time."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass


def host_now_us() -> int:
    """The timeline clock shared by every process on this computer.

    ``perf_counter`` is the one Python clock that is both high resolution on
    Windows and system-wide, so timestamps taken in the camera process, the
    service and the recorder can be compared directly.
    """
    return time.perf_counter_ns() // 1000


@dataclass
class ClockSample:
    host_mid_us: float
    mcu_mid_us: float
    rtt_us: float


class ClockModel:
    """Tracks controller time as ``mcu = offset + drift * host``.

    Uses NTP-style exchanges. USB round trips have a long upper tail, so only
    the samples with the shortest round trips in the window drive the line fit:
    their midpoint estimate has the smallest asymmetry error.
    """

    def __init__(self, window: int = 64, keep_best: float = 0.5) -> None:
        self._samples: deque[ClockSample] = deque(maxlen=window)
        self._keep_best = keep_best
        self._lock = threading.Lock()
        self._offset = 0.0
        self._drift = 1.0
        self._origin_host = 0.0
        self.best_rtt_us = float("inf")
        self.sample_count = 0

    def add_exchange(self, host_t1: int, mcu_t2: int, mcu_t3: int, host_t4: int) -> None:
        rtt = (host_t4 - host_t1) - (mcu_t3 - mcu_t2)
        sample = ClockSample((host_t1 + host_t4) / 2.0, (mcu_t2 + mcu_t3) / 2.0, float(rtt))
        with self._lock:
            self._samples.append(sample)
            self.sample_count += 1
            self._refit()

    def _refit(self) -> None:
        ordered = sorted(self._samples, key=lambda s: s.rtt_us)
        best = ordered[: max(1, int(len(ordered) * self._keep_best))]
        self.best_rtt_us = best[0].rtt_us
        count = len(best)
        mean_host = sum(s.host_mid_us for s in best) / count
        mean_mcu = sum(s.mcu_mid_us for s in best) / count
        spread = sum((s.host_mid_us - mean_host) ** 2 for s in best)
        host_span = max(s.host_mid_us for s in self._samples) - min(s.host_mid_us for s in self._samples)
        # A drift fit on a short baseline only measures noise. Crystals sit within
        # about 50 ppm, so anything far outside that is rejected too.
        drift = 1.0
        if count >= 8 and spread > 0 and host_span > 2e6:
            cov = sum((s.host_mid_us - mean_host) * (s.mcu_mid_us - mean_mcu) for s in best)
            candidate = cov / spread
            if abs(candidate - 1.0) < 2e-4:
                drift = candidate
        self._drift = drift
        self._origin_host = mean_host
        self._offset = mean_mcu

    @property
    def synced(self) -> bool:
        return self.sample_count >= 4

    def mcu_to_host(self, mcu_us: int | float) -> float:
        with self._lock:
            return self._origin_host + (mcu_us - self._offset) / self._drift

    def host_to_mcu(self, host_us: int | float) -> float:
        with self._lock:
            return self._offset + (host_us - self._origin_host) * self._drift

    def summary(self) -> dict:
        with self._lock:
            return {
                "synced": self.sample_count >= 4,
                "samples": self.sample_count,
                "best_rtt_us": None if self.best_rtt_us == float("inf") else self.best_rtt_us,
                "drift_ppm": (self._drift - 1.0) * 1e6,
                "uncertainty_us": None if self.best_rtt_us == float("inf") else self.best_rtt_us / 2.0,
            }
