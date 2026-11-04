"""The model process.

    RAC_POLICY_CONFIG='{"backend": "mock", "arms": ["left_follower"], ...}' python -m rac_policy.worker

The loop: observe, infer, submit a chunk through the unified entry. The token it
submits with was read before it took the observation, so a result computed across a
takeover is refused by the service rather than executed. It never touches the
controller link and has no way to move an arm except by submitting a chunk.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time

import numpy as np
from rac_camera import CameraReader
from rac_link.client import ControllerClient
from rac_link.clock import host_now_us
from rac_link.ipc import ServiceError

from .backends import make_backend
from .spec import IoSpec, Observation

log = logging.getLogger("rac_policy.worker")


class ObservationSource:
    """Latest follower state from the service, latest frames from shared memory."""

    def __init__(self, client: ControllerClient, spec: IoSpec) -> None:
        self.spec = spec
        self.lock = threading.Lock()
        self.states: dict[str, dict] = {}
        self.readers = {key: CameraReader(camera) for key, camera in spec.cameras.items()}
        client.on_state(self._on_state)

    def _on_state(self, batch: list[dict]) -> None:
        with self.lock:
            for sample in batch:
                self.states[sample["arm"]] = sample

    def get(self) -> Observation | None:
        with self.lock:
            samples = [self.states.get(arm) for arm in self.spec.arms]
        if any(s is None or s["valid_mask"] != 0x7F for s in samples):
            return None
        images = {}
        stamps = [s["host_ts_us"] for s in samples]
        for key, reader in self.readers.items():
            frame = reader.latest(timeout=0.2)
            if frame is None:
                return None
            image = frame.image
            if self.spec.image_size:
                import cv2

                height, width = self.spec.image_size
                image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
            images[key] = image
            stamps.append(frame.host_ts_us)
        state = np.concatenate([np.asarray(s["position"], dtype=np.float32) for s in samples])
        # The oldest input decides how stale the result will be.
        return Observation(state=self.spec.state_norm.to_model(state), images=images, task=self.spec.task, timestamp_us=min(stamps))

    def close(self) -> None:
        for reader in self.readers.values():
            reader.close()


def run(config: dict, client: ControllerClient | None = None, stop: threading.Event | None = None) -> dict:
    client = client or ControllerClient()
    stop = stop or threading.Event()
    spec = IoSpec.from_config(config)
    backend = make_backend(spec, config)
    info = backend.connect()
    log.info("backend %s ready: %s", backend.name, info)
    source = ObservationSource(client, spec)
    period = 1.0 / float(config.get("inference_hz", 5.0))
    summary = {"submitted": 0, "discarded": 0, "resets": 0, "errors": 0}
    try:
        while not stop.is_set():
            started = time.perf_counter()
            poll = client.call("policy.poll")
            state = poll["state"]
            if state == "idle":
                break
            if poll["reset"]:
                backend.reset()
                summary["resets"] += 1
            if state != "running":
                time.sleep(0.02)
                continue

            token = poll["token"]  # read before observing: this is what makes staleness detectable
            observation = source.get()
            if observation is None:
                time.sleep(0.02)
                continue
            infer_start = time.perf_counter()
            try:
                actions = spec.check_chunk(backend.infer(observation))
            except (ValueError, RuntimeError, OSError) as exc:
                summary["errors"] += 1
                log.error("inference failed: %s", exc)
                time.sleep(0.5)
                continue
            inference_ms = (time.perf_counter() - infer_start) * 1000.0
            actions = spec.action_norm.from_model(actions)
            try:
                client.policy_submit(actions.tolist(), token, observation.timestamp_us, inference_ms)
                summary["submitted"] += 1
            except ServiceError as exc:
                if exc.code == "policy.stale_token":
                    summary["discarded"] += 1
                    continue
                raise
            remaining = period - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        source.close()
        backend.close()
    return summary


def main() -> None:
    logging.basicConfig(level="INFO", format="%(asctime)s policy %(levelname)s %(message)s")
    config = json.loads(os.environ["RAC_POLICY_CONFIG"])
    summary = run(config)
    log.info("finished at %d: %s", host_now_us(), summary)


if __name__ == "__main__":
    main()
