"""Single camera capture process.

    python -m rac_camera.capture --config '{"cameras": {"env": {"source": "sim"}, "wrist": {"source": 0}}}'

Each camera is captured on its own thread, stamped on the shared host clock as soon
as the frame is read, and published to shared memory. It is the only place in the
platform that opens a camera, so recording, inference and preview see the same frames.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import threading
import time

import numpy as np
from rac_link.clock import host_now_us

from .shm import CameraWriter

log = logging.getLogger(__name__)


def synthetic_frame(width: int, height: int, index: int, name: str) -> np.ndarray:
    """A moving test pattern. The frame counter is drawn into it, so tests can read it back."""
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :, 0] = np.linspace(20, 60, width, dtype=np.uint8)
    image[:, :, 1] = (hash(name) % 40) + 20
    image[:, :, 2] = np.linspace(60, 20, height, dtype=np.uint8)[:, None]
    bar = (index * 6) % width
    image[:, max(0, bar - 8) : bar + 8] = (230, 230, 240)
    # 16 cells at the top encode the frame index in binary.
    cell = max(2, width // 32)
    for bit in range(16):
        value = 255 if (index >> bit) & 1 else 0
        image[0 : cell, bit * cell : (bit + 1) * cell] = value
    return image


def read_frame_index(image: np.ndarray) -> int:
    width = image.shape[1]
    cell = max(2, width // 32)
    index = 0
    for bit in range(16):
        if image[cell // 2, bit * cell + cell // 2, 0] > 127:
            index |= 1 << bit
    return index


class CameraThread(threading.Thread):
    def __init__(self, name: str, config: dict, stop: threading.Event) -> None:
        super().__init__(name=f"camera-{name}", daemon=True)
        self.camera = name
        self.config = config
        self.stop_event = stop
        self.width = int(config.get("width", 640))
        self.height = int(config.get("height", 480))
        self.fps = float(config.get("fps", 30))
        self.source = config.get("source", "sim")

    def run(self) -> None:
        if self.source == "sim":
            self._run_synthetic()
        else:
            self._run_opencv()

    def _run_synthetic(self) -> None:
        writer = CameraWriter(self.camera, self.width, self.height)
        period = 1.0 / self.fps
        next_frame = time.perf_counter()
        index = 0
        try:
            while not self.stop_event.is_set():
                image = synthetic_frame(self.width, self.height, index, self.camera)
                writer.write(image, host_now_us())
                index += 1
                next_frame += period
                delay = next_frame - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_frame = time.perf_counter()
        finally:
            writer.close()

    def _run_opencv(self) -> None:
        import cv2

        source = self.source
        if isinstance(source, str) and source.isdigit():
            source = int(source)
        backend = cv2.CAP_DSHOW if isinstance(source, int) and cv2.getBuildInformation().find("DirectShow") >= 0 else cv2.CAP_ANY
        capture = cv2.VideoCapture(source, backend)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        # A one-frame driver queue: an old frame stamped now would corrupt alignment.
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            log.error("camera %s cannot open source %r", self.camera, self.source)
            return
        ok, frame = capture.read()
        if not ok:
            log.error("camera %s opened but returned no frame", self.camera)
            capture.release()
            return
        height, width = frame.shape[:2]
        writer = CameraWriter(self.camera, width, height)
        try:
            while not self.stop_event.is_set():
                ok, frame = capture.read()
                stamp = host_now_us()
                if not ok:
                    time.sleep(0.005)
                    continue
                writer.write(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), stamp)
        finally:
            capture.release()
            writer.close()


def run(config: dict) -> None:
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    threads = [CameraThread(name, cfg, stop) for name, cfg in config.get("cameras", {}).items()]
    for thread in threads:
        thread.start()
    while not stop.is_set():
        time.sleep(0.1)
    for thread in threads:
        thread.join(timeout=2.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True, help="JSON text")
    args = parser.parse_args()
    logging.basicConfig(level="INFO")
    run(json.loads(args.config))


if __name__ == "__main__":
    main()
