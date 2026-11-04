"""Streams trajectories to the controller, respecting its queue.

This is the path every computer-side motion takes: teaching, replay, homing and
policy actions all end up here, so flow control and the generation check exist in
one place only.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

import numpy as np
from rac_link.clock import host_now_us
from rac_link.device import ControllerDevice, ControllerError
from rac_link.protocol_constants import CHUNK_POINTS
from rac_link._gen import AckStatus

log = logging.getLogger(__name__)

# How far ahead of now a synchronized start is scheduled. Long enough for the first
# chunk of every arm to arrive before any of them begins.
START_LEAD_US = 60_000


@dataclass
class ArmJob:
    arm: int
    points: np.ndarray
    sent: int = 0
    first_seq: int = 0
    last_seq: int = 0


@dataclass
class Job:
    job_id: int
    source: int
    generation: int
    dt_us: int
    arms: list[ArmJob]
    on_done: object = None
    started_at: float = field(default_factory=time.monotonic)
    done: threading.Event = field(default_factory=threading.Event)
    error: str | None = None
    cancelled: bool = False
    paused: bool = False


class Executor:
    def __init__(self, device_getter, on_finish=None) -> None:
        self._device_getter = device_getter
        self._on_finish = on_finish
        self._lock = threading.Lock()
        self._job: Job | None = None
        self._next_id = 1
        self._next_seq = 1
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="rac-executor", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=1.0)

    def submit(self, arm_points: dict[int, np.ndarray], dt_s: float, source: int, generation: int) -> Job:
        with self._lock:
            arms = []
            for arm, points in arm_points.items():
                first = self._next_seq
                self._next_seq += len(points)
                arms.append(ArmJob(arm=arm, points=points, first_seq=first, last_seq=first + len(points) - 1))
            job = Job(self._next_id, source, generation, int(dt_s * 1e6), arms)
            self._next_id += 1
            if self._job:
                self._job.cancelled = True
                self._job.done.set()
            self._job = job
        self._wake.set()
        return job

    def cancel(self) -> None:
        with self._lock:
            if self._job:
                self._job.cancelled = True
                self._job.done.set()
                self._job = None

    def set_paused(self, paused: bool) -> None:
        with self._lock:
            if self._job:
                self._job.paused = paused

    @property
    def current(self) -> Job | None:
        return self._job

    def progress(self) -> dict | None:
        job = self._job
        if not job:
            return None
        device = self._device_getter()
        arms = []
        for arm_job in job.arms:
            state = device.snapshot.arms.get(arm_job.arm) if device else None
            done_points = 0
            if state and state.exec_seq >= arm_job.first_seq:
                done_points = min(state.exec_seq - arm_job.first_seq + 1, len(arm_job.points))
            arms.append({"arm": arm_job.arm, "total": len(arm_job.points), "sent": arm_job.sent, "executed": done_points})
        return {"job": job.job_id, "dt_s": job.dt_us / 1e6, "paused": job.paused, "arms": arms}

    def _loop(self) -> None:
        while not self._stop.is_set():
            job = self._job
            if job is None:
                self._wake.wait(0.1)
                self._wake.clear()
                continue
            try:
                self._run(job)
            except ControllerError as exc:
                job.error = str(exc)
                log.warning("trajectory stopped: %s", exc)
            with self._lock:
                if self._job is job:
                    self._job = None
            job.done.set()
            if self._on_finish:
                self._on_finish(job)

    def _run(self, job: Job) -> None:
        device: ControllerDevice | None = self._device_getter()
        if device is None:
            job.error = "controller is not connected"
            return

        # Arms of one job share a start time on the controller clock, which is
        # what keeps a bimanual trajectory in step.
        start_at = 0
        if device.clock.synced:
            start_at = int(device.clock.host_to_mcu(host_now_us() + START_LEAD_US))

        first_send = True
        while not job.cancelled and not self._stop.is_set():
            progressed = False
            finished = True
            for arm_job in job.arms:
                if arm_job.sent >= len(arm_job.points):
                    continue
                finished = False
                if job.paused:
                    continue
                slice_points = arm_job.points[arm_job.sent : arm_job.sent + CHUNK_POINTS]
                ack = device.send_points(
                    arm_job.arm,
                    slice_points.tolist(),
                    job.dt_us,
                    arm_job.first_seq + arm_job.sent,
                    job.source,
                    t_start_us=start_at + arm_job.sent * job.dt_us if (first_send and start_at) else 0,
                    generation=job.generation,
                )
                if ack.status == AckStatus.OK:
                    arm_job.sent += len(slice_points)
                    progressed = True
                elif ack.status == AckStatus.QUEUE_FULL:
                    pass
                else:
                    raise ControllerError(f"trajectory rejected: {ack.status.name}", ack.status)
            first_send = False
            if finished:
                break
            if not progressed:
                time.sleep(0.02)

        # Everything is queued: wait for the last point to execute.
        while not job.cancelled and not self._stop.is_set():
            remaining = False
            for arm_job in job.arms:
                state = device.snapshot.arms.get(arm_job.arm)
                if not state or state.exec_seq < arm_job.last_seq:
                    remaining = True
            if not remaining:
                return
            if not device.is_open:
                job.error = "link lost during trajectory"
                return
            if job.generation < device.snapshot.generation:
                # The controller moved on by itself: e-stop or communication loss.
                job.error = "stopped by the controller"
                return
            time.sleep(0.01)
