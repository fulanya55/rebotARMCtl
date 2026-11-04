"""Recording, independent of who is controlling the arms.

The recorder only subscribes: it reads controller samples and control events through
the service and camera frames from shared memory, and holds no control rights. It can
run across teleoperation, teaching, a policy, a correction or a replay, and never
affects any of them. A failure here cannot stop an arm.
"""

from __future__ import annotations

import json
import logging
import queue
import shutil
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
from rac_camera import CameraReader
from rac_link.client import ControllerClient
from rac_link.clock import host_now_us

log = logging.getLogger(__name__)

JPEG_QUALITY = 95


@dataclass
class RecordingConfig:
    root: str
    repo_id: str
    task: str
    fps: int = 30
    cameras: list[str] = field(default_factory=list)
    follower_arms: list[str] = field(default_factory=lambda: ["left_follower"])
    leader_arms: list[str] = field(default_factory=lambda: ["left_leader"])
    device_config: dict = field(default_factory=dict)
    model_config: dict | None = None


class CameraRecorder(threading.Thread):
    """Saves frames as they arrive. JPEG keeps up in real time, video encoding comes later."""

    def __init__(self, camera: str, directory: Path) -> None:
        super().__init__(name=f"record-{camera}", daemon=True)
        self.camera = camera
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.stop_event = threading.Event()
        self.index: list[dict] = []
        self.error: str | None = None
        self.width = 0
        self.height = 0
        self._queue: queue.Queue = queue.Queue(maxsize=240)
        self._encoder = threading.Thread(target=self._encode_loop, daemon=True)
        self.dropped = 0

    def run(self) -> None:
        try:
            reader = CameraReader(self.camera)
        except (FileNotFoundError, ValueError) as exc:
            self.error = f"camera {self.camera} is not running: {exc}"
            log.error(self.error)
            return
        self.width, self.height = reader.width, reader.height
        self._encoder.start()
        try:
            while not self.stop_event.is_set():
                frame = reader.next_frame(timeout=0.2)
                if frame is None:
                    continue
                try:
                    self._queue.put_nowait(frame)
                except queue.Full:
                    self.dropped += 1
        finally:
            reader.close()
            self._queue.put(None)
            self._encoder.join(timeout=30)

    def _encode_loop(self) -> None:
        while True:
            frame = self._queue.get()
            if frame is None:
                return
            name = f"{len(self.index):06d}.jpg"
            ok, encoded = cv2.imencode(
                ".jpg", cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if not ok:
                self.dropped += 1
                continue
            (self.directory / name).write_bytes(encoded.tobytes())
            self.index.append(
                {"file": name, "host_ts_us": frame.host_ts_us, "capture_ts_us": frame.capture_ts_us, "frame_index": frame.frame_index}
            )


class EpisodeRecording:
    """Raw material for one episode, on disk under a temporary directory."""

    def __init__(self, config: RecordingConfig, client: ControllerClient, directory: Path) -> None:
        self.config = config
        self.client = client
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.samples: dict[str, list[dict]] = {}
        self.events: list[dict] = []
        self.started_us = 0
        self.stopped_us = 0
        self.recording = False
        self.cameras = [CameraRecorder(c, directory / c) for c in config.cameras]
        self.lock = threading.Lock()
        self.status_snapshot: dict | None = None

    def on_state(self, batch: list[dict]) -> None:
        if not self.recording:
            return
        with self.lock:
            for sample in batch:
                self.samples.setdefault(sample["arm"], []).append(sample)

    def on_event(self, event: dict) -> None:
        if self.recording:
            with self.lock:
                self.events.append(event)

    def start(self) -> None:
        self.started_us = host_now_us()
        self.recording = True
        try:
            self.status_snapshot = self.client.status()
        except Exception:
            log.exception("could not capture the device state for this episode")
        for camera in self.cameras:
            camera.start()

    def stop(self) -> None:
        self.recording = False
        self.stopped_us = host_now_us()
        for camera in self.cameras:
            camera.stop_event.set()
        for camera in self.cameras:
            camera.join(timeout=60)
        (self.directory / "raw.json").write_text(json.dumps(self.to_raw()), encoding="utf-8")

    def to_raw(self) -> dict:
        with self.lock:
            return {
                "config": asdict(self.config),
                "started_us": self.started_us,
                "stopped_us": self.stopped_us,
                "samples": self.samples,
                "events": self.events,
                "status": self.status_snapshot,
                "cameras": {
                    c.camera: {"index": c.index, "width": c.width, "height": c.height, "dropped": c.dropped, "error": c.error}
                    for c in self.cameras
                },
            }

    def live_stats(self) -> dict:
        elapsed = max(1e-3, (host_now_us() - self.started_us) / 1e6) if self.started_us else 0.0
        with self.lock:
            arms = {
                arm: {
                    "samples": len(samples),
                    "rate_hz": round(len(samples) / elapsed, 1) if elapsed else 0,
                    "missing": sum(max(0, b["sample_seq"] - a["sample_seq"] - 1) for a, b in zip(samples, samples[1:], strict=False)),
                }
                for arm, samples in self.samples.items()
            }
        cameras = {
            c.camera: {
                "frames": len(c.index),
                "rate_hz": round(len(c.index) / elapsed, 1) if elapsed else 0,
                "dropped": c.dropped,
                "error": c.error,
            }
            for c in self.cameras
        }
        return {"elapsed_s": round(elapsed, 2), "arms": arms, "cameras": cameras}


class Recorder:
    """Session state machine: start, finish, next, append, discard."""

    def __init__(self, client: ControllerClient, writer) -> None:
        self.client = client
        self.writer = writer
        self.config: RecordingConfig | None = None
        self.current: EpisodeRecording | None = None
        self.finished: EpisodeRecording | None = None
        self.lock = threading.RLock()
        client.on_state(self._on_state)
        client.on_event(self._on_event)

    def _on_state(self, batch: list[dict]) -> None:
        current = self.current
        if current:
            current.on_state(batch)

    def _on_event(self, event: dict) -> None:
        current = self.current
        if current:
            current.on_event(event)

    def _raw_root(self, config: RecordingConfig) -> Path:
        # Beside the dataset, not in it: LeRobot refuses to create a dataset in a
        # directory that already exists, and an export should not carry raw frames.
        root = Path(config.root)
        return root.parent / ".rac_raw" / root.name

    def start(self, config: RecordingConfig) -> dict:
        with self.lock:
            if self.current:
                raise RuntimeError("a recording is already running")
            if not config.cameras:
                raise ValueError("choose at least one camera")
            if self.finished:
                raise RuntimeError("save or discard the previous episode first")
            self.config = config
            directory = self._raw_root(config) / uuid.uuid4().hex
            self.current = EpisodeRecording(config, self.client, directory)
            self.current.start()
            return self.status()

    def stop(self) -> dict:
        """End the episode. The result waits for a save or a discard."""
        with self.lock:
            if not self.current:
                raise RuntimeError("nothing is being recorded")
            recording = self.current
            self.current = None
            recording.stop()
            self.finished = recording
            return self.status()

    def save(self, annotation: dict | None = None) -> dict:
        with self.lock:
            if not self.finished:
                raise RuntimeError("there is no finished episode to save")
            recording = self.finished
            self.finished = None
            job_id = self.writer.enqueue(recording.directory, annotation or {})
            return {"job": job_id, **self.status()}

    def discard(self) -> dict:
        with self.lock:
            recording = self.finished
            self.finished = None
            if recording:
                shutil.rmtree(recording.directory, ignore_errors=True)
            return self.status()

    def next(self, annotation: dict | None = None) -> dict:
        """Save this episode and start the next one with the same settings."""
        with self.lock:
            if self.current:
                self.stop()
            if self.finished:
                self.save(annotation)
            if not self.config:
                raise RuntimeError("no recording settings yet")
            return self.start(self.config)

    def status(self) -> dict:
        current = self.current
        return {
            "recording": bool(current),
            "pending_save": bool(self.finished),
            "live": current.live_stats() if current else None,
            "finished": self.finished.live_stats() if self.finished else None,
            "config": asdict(self.config) if self.config else None,
            "writer": self.writer.status(),
        }


def wait_for(predicate, timeout: float = 30.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False
