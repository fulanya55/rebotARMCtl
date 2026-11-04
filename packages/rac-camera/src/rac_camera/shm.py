"""Shared, timestamped camera frames.

One capture process writes, and any number of readers (preview, recorder, policy)
attach by name. Readers cannot slow the writer down and never own the camera.
A seqlock guards the single frame slot: a reader that catches a write in progress
retries rather than returning a torn image.
"""

from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass
from multiprocessing import shared_memory

import numpy as np

MAGIC = 0x4D414352  # "RCAM"
# magic, width, height, channels, seq, frame_index, host_ts_us, capture_ts_us
_HEADER = struct.Struct("<IIIIQQqq")
HEADER_SIZE = 64


def segment_name(camera: str) -> str:
    return f"rac_cam_{camera}"


@dataclass
class Frame:
    camera: str
    image: np.ndarray  # H x W x 3 RGB, uint8
    frame_index: int
    host_ts_us: int
    capture_ts_us: int


class CameraWriter:
    def __init__(self, camera: str, width: int, height: int, channels: int = 3) -> None:
        self.camera = camera
        self.width, self.height, self.channels = width, height, channels
        self._size = HEADER_SIZE + width * height * channels
        name = segment_name(camera)
        try:
            self._shm = shared_memory.SharedMemory(name=name, create=True, size=self._size)
        except FileExistsError:
            # Left behind by a crashed capture process. It holds nothing worth keeping.
            old = shared_memory.SharedMemory(name=name)
            old.close()
            old.unlink()
            self._shm = shared_memory.SharedMemory(name=name, create=True, size=self._size)
        self._image = np.ndarray((height, width, channels), dtype=np.uint8, buffer=self._shm.buf, offset=HEADER_SIZE)
        self._seq = 0
        self._frame_index = 0
        self._write_header(0, 0, 0)

    def _write_header(self, seq: int, host_ts: int, capture_ts: int) -> None:
        _HEADER.pack_into(self._shm.buf, 0, MAGIC, self.width, self.height, self.channels, seq, self._frame_index, host_ts, capture_ts)

    def write(self, image: np.ndarray, host_ts_us: int, capture_ts_us: int | None = None) -> None:
        self._seq += 1  # odd: write in progress
        self._write_header(self._seq, 0, 0)
        self._image[:] = image
        self._frame_index += 1
        self._seq += 1  # even: stable
        self._write_header(self._seq, host_ts_us, host_ts_us if capture_ts_us is None else capture_ts_us)

    def close(self) -> None:
        del self._image
        self._shm.close()
        try:
            self._shm.unlink()
        except FileNotFoundError:
            pass


class CameraReader:
    def __init__(self, camera: str) -> None:
        self.camera = camera
        self._shm = shared_memory.SharedMemory(name=segment_name(camera))
        # On POSIX the tracker would unlink the writer's segment when this reader
        # exits. Windows has no tracker: a segment lives as long as a handle does.
        if os.name != "nt":
            from multiprocessing import resource_tracker

            try:
                resource_tracker.unregister(self._shm._name, "shared_memory")  # type: ignore[attr-defined]
            except (KeyError, AttributeError, ValueError):
                pass
        magic, self.width, self.height, self.channels, *_ = _HEADER.unpack_from(self._shm.buf, 0)
        if magic != MAGIC:
            raise ValueError(f"segment for {camera} is not a camera")
        self._image = np.ndarray(
            (self.height, self.width, self.channels), dtype=np.uint8, buffer=self._shm.buf, offset=HEADER_SIZE
        )
        self._last_index = -1

    def latest(self, timeout: float = 0.1) -> Frame | None:
        deadline = time.monotonic() + timeout
        while True:
            _, _, _, _, seq, index, host_ts, capture_ts = _HEADER.unpack_from(self._shm.buf, 0)
            if seq % 2 == 0 and index > 0:
                image = self._image.copy()
                _, _, _, _, seq2, *_ = _HEADER.unpack_from(self._shm.buf, 0)
                if seq2 == seq:
                    return Frame(self.camera, image, index, host_ts, capture_ts)
            if time.monotonic() > deadline:
                return None
            time.sleep(0.001)

    def next_frame(self, timeout: float = 1.0) -> Frame | None:
        """The next frame this reader has not seen. Frames arriving meanwhile are skipped."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _, _, _, _, seq, index, _, _ = _HEADER.unpack_from(self._shm.buf, 0)
            if seq % 2 == 0 and index != self._last_index and index > 0:
                frame = self.latest(timeout=0.05)
                if frame is not None:
                    self._last_index = frame.frame_index
                    return frame
            time.sleep(0.001)
        return None

    def close(self) -> None:
        del self._image
        self._shm.close()
