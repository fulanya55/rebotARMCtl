"""Model backends. Each runs inside the model process, so none of their dependencies
reach the service or the web backend."""

from __future__ import annotations

import logging

import numpy as np

from .spec import IoSpec, Observation, PolicyBackend

log = logging.getLogger(__name__)


class MockBackend(PolicyBackend):
    """A deterministic model for tests and for checking a rig without a checkpoint.

    Returns a slow sine around the current pose. Its ``reset_count`` lets a test prove
    the handback rule: the action cache is cleared and it re-observes.
    """

    name = "mock"

    def __init__(self, spec: IoSpec, config: dict) -> None:
        self.spec = spec
        self.chunk = int(config.get("chunk_size", 30))
        self.amplitude = float(config.get("amplitude", 0.15))
        self.delay_ms = float(config.get("delay_ms", 10))
        self.reset_count = 0
        self.step = 0

    def reset(self) -> None:
        self.reset_count += 1
        self.step = 0

    def infer(self, observation: Observation) -> np.ndarray:
        import time

        time.sleep(self.delay_ms / 1000.0)
        phase = np.arange(self.chunk) + self.step
        self.step += 1
        wave = self.amplitude * np.sin(2 * np.pi * phase / 90.0)
        actions = np.tile(observation.state, (self.chunk, 1))
        actions[:, 0] = observation.state[0] + wave - wave[0]
        return actions


class StarVlaBackend(PolicyBackend):
    """Client of an independently running StarVLA WebSocket server.

    Protocol per starVLA/deployment/model_server: a msgpack-numpy handshake frame, then
    ``{"type": "infer", "payload": {"examples": [...], "unnorm_key": ...}}`` answered by
    ``{"status": "ok", "data": {"actions": ndarray[B, T, D]}}``. The server does its own
    unnormalization. Client shape adapted from EVA-Client src/policy_client/starvla.py
    at 926cdf4 (Apache-2.0).
    """

    name = "starvla"

    def __init__(self, spec: IoSpec, config: dict) -> None:
        self.spec = spec
        self.host = config.get("host", "127.0.0.1")
        self.port = int(config.get("port", 10093))
        self.unnorm_key = config.get("unnorm_key")
        self.camera_key = config.get("camera_key")
        self.timeout = float(config.get("timeout_s", 30.0))
        self._ws = None
        self.metadata: dict = {}

    def connect(self) -> dict:
        from websockets.sync.client import connect

        from .msgpack_numpy import packb, unpackb

        self._packb, self._unpackb = packb, unpackb
        self._ws = connect(f"ws://{self.host}:{self.port}", compression=None, max_size=None, open_timeout=self.timeout)
        self.metadata = unpackb(self._ws.recv())
        return self.metadata

    def _image(self, observation: Observation) -> np.ndarray:
        if self.camera_key:
            return observation.images[self.camera_key]
        return next(iter(observation.images.values()))

    def infer(self, observation: Observation) -> np.ndarray:
        example = {
            "image": self._image(observation),
            "lang": observation.task,
            "state": np.asarray(observation.state, dtype=np.float32),
        }
        payload: dict = {"examples": [example]}
        if self.unnorm_key:
            payload["unnorm_key"] = self.unnorm_key
        self._ws.send(self._packb({"type": "infer", "request_id": "rac", "payload": payload}))
        response = self._ws.recv(timeout=self.timeout)
        if isinstance(response, str):
            raise RuntimeError(f"StarVLA server error: {response[:500]}")
        decoded = self._unpackb(response)
        if decoded.get("status") != "ok":
            raise RuntimeError(f"StarVLA server error: {decoded.get('error')}")
        return np.asarray(decoded["data"]["actions"], dtype=np.float32)

    def close(self) -> None:
        if self._ws:
            self._ws.close()


class LeRobotBackend(PolicyBackend):
    """A LeRobot checkpoint, loaded in this process with LeRobot's own pre and post processors."""

    name = "lerobot"

    def __init__(self, spec: IoSpec, config: dict) -> None:
        self.spec = spec
        self.path = config["checkpoint"]
        self.device = config.get("device", "cuda")
        self.policy = None
        self.pre = None
        self.post = None
        self.torch = None

    def connect(self) -> dict:
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        self.torch = torch
        if self.device == "cuda" and not torch.cuda.is_available():
            log.warning("no CUDA device, running the policy on CPU")
            self.device = "cpu"
        config = PreTrainedConfig.from_pretrained(self.path)
        config.device = self.device
        policy_cls = get_policy_class(config.type)
        self.policy = policy_cls.from_pretrained(self.path, config=config)
        self.policy.to(self.device).eval()
        self.pre, self.post = make_pre_post_processors(policy_cfg=config, pretrained_path=self.path)
        return {"type": config.type, "chunk_size": getattr(config, "chunk_size", None), "device": self.device}

    def reset(self) -> None:
        if self.policy:
            self.policy.reset()
        if self.pre:
            self.pre.reset()
        if self.post:
            self.post.reset()

    def infer(self, observation: Observation) -> np.ndarray:
        torch = self.torch
        batch = {"observation.state": torch.from_numpy(observation.state).float().unsqueeze(0).to(self.device), "task": [observation.task]}
        for key, image in observation.images.items():
            tensor = torch.from_numpy(image).float().permute(2, 0, 1) / 255.0
            batch[f"observation.images.{key}"] = tensor.unsqueeze(0).to(self.device)
        batch = self.pre(batch)
        with torch.inference_mode():
            if hasattr(self.policy, "predict_action_chunk"):
                actions = self.policy.predict_action_chunk(batch)
            else:
                actions = self.policy.select_action(batch).unsqueeze(1)
        actions = self.post(actions)
        return actions.squeeze(0).cpu().numpy()


BACKENDS = {"mock": MockBackend, "starvla": StarVlaBackend, "lerobot": LeRobotBackend}


def make_backend(spec: IoSpec, config: dict) -> PolicyBackend:
    name = config.get("backend", "mock")
    if name not in BACKENDS:
        raise ValueError(f"unknown policy backend {name!r}, choose from {sorted(BACKENDS)}")
    return BACKENDS[name](spec, config)
