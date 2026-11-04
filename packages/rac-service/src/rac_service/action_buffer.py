# Adapted from EVA-Client src/strategy/action_buffer.py at 926cdf4
# (https://github.com/Noietch/EVA-CLIENT), Licensed under the Apache License, 2.0.
# Changes: reduced to the integration logic with a monotonic-time free interface,
# added clear() so a takeover can drop what was buffered, and kept only the
# strategies this platform schedules.
"""Merging overlapping action chunks into one stream of actions."""

from __future__ import annotations

import threading
from collections import deque

import numpy as np


class StreamActionBuffer:
    """Latency trim plus linear blending across the overlap with the previous chunk.

    A new chunk was computed from an observation that is already old by the time
    it arrives. The first ``k`` steps, the ones consumed while it was being
    computed, are dropped, and the rest is cross-faded onto what is still queued.
    """

    name = "stream"

    def __init__(self, min_overlap: int = 8) -> None:
        self._lock = threading.Lock()
        self._queue: deque[np.ndarray] = deque()
        self._consumed_since_chunk = 0
        self._last: np.ndarray | None = None
        self._min_overlap = max(1, min_overlap)

    def integrate(self, chunk: np.ndarray, max_trim: int | None = None) -> None:
        actions = np.asarray(chunk, dtype=np.float32)
        if actions.size == 0:
            return
        with self._lock:
            consumed = self._consumed_since_chunk
            trim = consumed if max_trim is None else min(consumed, max(0, max_trim))
            if trim >= len(actions):
                return
            new = actions[trim:].copy()

            old = list(self._queue)
            if old:
                previous = np.stack(old)
            elif self._last is not None:
                previous = np.repeat(self._last[np.newaxis, :], self._min_overlap, axis=0)
            else:
                previous = None

            if previous is not None:
                overlap = min(len(previous), len(new))
                if overlap > 0:
                    weights = np.linspace(1.0, 0.0, overlap + 2, dtype=np.float32)[1:-1, None]
                    new[:overlap] = weights * previous[:overlap] + (1.0 - weights) * new[:overlap]

            self._queue = deque(new)
            self._consumed_since_chunk = 0

    def pop(self) -> np.ndarray | None:
        with self._lock:
            if not self._queue:
                return None
            action = self._queue.popleft()
            self._consumed_since_chunk += 1
            self._last = action
            return action

    def clear(self) -> int:
        with self._lock:
            dropped = len(self._queue)
            self._queue.clear()
            self._consumed_since_chunk = 0
            self._last = None
            return dropped

    def __len__(self) -> int:
        with self._lock:
            return len(self._queue)


class NaiveActionBuffer(StreamActionBuffer):
    """Replace what is queued with the latency-aligned new chunk, without blending."""

    name = "naive"

    def integrate(self, chunk: np.ndarray, max_trim: int | None = None) -> None:
        actions = np.asarray(chunk, dtype=np.float32)
        if actions.size == 0:
            return
        with self._lock:
            consumed = self._consumed_since_chunk
            trim = consumed if max_trim is None else min(consumed, max(0, max_trim))
            if trim >= len(actions):
                return
            self._queue = deque(actions[trim:].copy())
            self._consumed_since_chunk = 0


class EnsembleActionBuffer(StreamActionBuffer):
    """ACT-style temporal ensembling: exponentially weighted across every chunk that covers a step."""

    name = "ensemble"

    def __init__(self, decay: float = 0.01) -> None:
        super().__init__()
        self._decay = decay
        self._sums: deque[np.ndarray] = deque()
        self._weights: deque[float] = deque()
        self._age = 0

    def integrate(self, chunk: np.ndarray, max_trim: int | None = None) -> None:
        actions = np.asarray(chunk, dtype=np.float32)
        if actions.size == 0:
            return
        with self._lock:
            consumed = self._consumed_since_chunk
            trim = consumed if max_trim is None else min(consumed, max(0, max_trim))
            if trim >= len(actions):
                return
            weight = float(np.exp(-self._decay * self._age))
            self._age += 1
            for index, action in enumerate(actions[trim:]):
                if index < len(self._sums):
                    self._sums[index] = self._sums[index] + action * weight
                    self._weights[index] += weight
                else:
                    self._sums.append(action * weight)
                    self._weights.append(weight)
            self._consumed_since_chunk = 0

    def pop(self) -> np.ndarray | None:
        with self._lock:
            if not self._sums:
                return None
            action = self._sums.popleft() / self._weights.popleft()
            self._consumed_since_chunk += 1
            self._last = action
            return action

    def clear(self) -> int:
        with self._lock:
            dropped = len(self._sums)
            self._sums.clear()
            self._weights.clear()
            self._consumed_since_chunk = 0
            self._age = 0
            self._last = None
            return dropped

    def __len__(self) -> int:
        with self._lock:
            return len(self._sums)


def make_buffer(name: str):
    return {"stream": StreamActionBuffer, "naive": NaiveActionBuffer, "ensemble": EnsembleActionBuffer}[name]()
