"""The unified model interface: what a model sees and what it must return.

Every backend works in the platform's units through this spec. Radians and a 0..1
gripper on the platform side; anything else a checkpoint expects (degrees, a
normalized range) is an explicit conversion here rather than an assumption in a
backend, so a wrong setting shows up as a config error and not as a thrown arm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

JOINTS = 7


@dataclass
class Normalization:
    """Scale and shift between platform values and model values, per dimension."""

    mode: str = "none"  # none, degrees, min_max, mean_std
    a: list[float] | None = None  # min or mean
    b: list[float] | None = None  # max or std

    def to_model(self, values: np.ndarray) -> np.ndarray:
        if self.mode == "none":
            return values
        if self.mode == "degrees":
            out = values.copy()
            mask = np.arange(values.shape[-1]) % JOINTS != JOINTS - 1
            out[..., mask] = out[..., mask] * 180.0 / math.pi
            return out
        a, b = np.asarray(self.a, dtype=np.float32), np.asarray(self.b, dtype=np.float32)
        if self.mode == "min_max":
            return 2.0 * (values - a) / np.maximum(b - a, 1e-6) - 1.0
        if self.mode == "mean_std":
            return (values - a) / np.maximum(b, 1e-6)
        raise ValueError(f"unknown normalization {self.mode}")

    def from_model(self, values: np.ndarray) -> np.ndarray:
        if self.mode == "none":
            return values
        if self.mode == "degrees":
            out = values.copy()
            mask = np.arange(values.shape[-1]) % JOINTS != JOINTS - 1
            out[..., mask] = out[..., mask] * math.pi / 180.0
            return out
        a, b = np.asarray(self.a, dtype=np.float32), np.asarray(self.b, dtype=np.float32)
        if self.mode == "min_max":
            return (values + 1.0) / 2.0 * (b - a) + a
        if self.mode == "mean_std":
            return values * b + a
        raise ValueError(f"unknown normalization {self.mode}")


@dataclass
class IoSpec:
    arms: list[str]
    cameras: dict[str, str] = field(default_factory=dict)  # model input key -> platform camera
    task: str = ""
    state_norm: Normalization = field(default_factory=Normalization)
    action_norm: Normalization = field(default_factory=Normalization)
    image_size: tuple[int, int] | None = None  # height, width expected by the model

    @property
    def width(self) -> int:
        return len(self.arms) * JOINTS

    @classmethod
    def from_config(cls, config: dict) -> IoSpec:
        def norm(key: str) -> Normalization:
            value = config.get(key) or {}
            return Normalization(value.get("mode", "none"), value.get("a"), value.get("b"))

        size = config.get("image_size")
        return cls(
            arms=list(config["arms"]),
            cameras=dict(config.get("cameras", {})),
            task=config.get("task", ""),
            state_norm=norm("state_norm"),
            action_norm=norm("action_norm"),
            image_size=(int(size[0]), int(size[1])) if size else None,
        )

    def check_chunk(self, actions: np.ndarray) -> np.ndarray:
        """Every chunk passes here, whichever backend produced it."""
        actions = np.asarray(actions, dtype=np.float32)
        if actions.ndim == 1:
            actions = actions[None, :]
        if actions.ndim == 3:
            if actions.shape[0] != 1:
                raise ValueError(f"expected a single batch, got shape {actions.shape}")
            actions = actions[0]
        if actions.ndim != 2 or actions.shape[1] != self.width:
            raise ValueError(f"the model returned {actions.shape}, this setup needs T x {self.width}")
        if not np.isfinite(actions).all():
            raise ValueError("the model returned NaN or infinity")
        return actions


@dataclass
class Observation:
    state: np.ndarray
    images: dict[str, np.ndarray]
    task: str
    timestamp_us: int


class PolicyBackend:
    """Implementations return T x width actions in model units; the worker converts them."""

    name = "base"

    def connect(self) -> dict:
        return {}

    def reset(self) -> None:
        """Forget anything cached. Called at the start of a run and after every handback."""

    def infer(self, observation: Observation) -> np.ndarray:
        raise NotImplementedError

    def close(self) -> None:
        pass
