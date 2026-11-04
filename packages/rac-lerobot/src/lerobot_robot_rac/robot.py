"""A LeRobot ``Robot`` backed by the Controller Service.

It holds no bus. Observations come from the service's state stream and the shared
camera module; actions go through the unified execution entry, so LeRobot's scripts
obey the same control ownership, limits and generation checks as everything else.

Units are the platform's: joint keys are radians and the gripper is 0..1.
"""

from __future__ import annotations

import threading
from functools import cached_property

import numpy as np
from lerobot.robots.robot import Robot
from rac_camera import CameraReader
from rac_link.client import ControllerClient
from rac_link.robots import JOINT_NAMES

from .config import RacFollowerConfig


def joint_keys(arms: list[str]) -> list[str]:
    """Fixed order: left before right, joints base to wrist, then the gripper."""
    if len(arms) == 1:
        return [f"{name}.pos" for name in JOINT_NAMES]
    keys = []
    for arm in arms:
        side = arm.split("_")[0]
        keys.extend(f"{side}_{name}.pos" for name in JOINT_NAMES)
    return keys


class RacFollower(Robot):
    config_class = RacFollowerConfig
    name = "rac_follower"

    def __init__(self, config: RacFollowerConfig) -> None:
        super().__init__(config)
        self.config = config
        self._client: ControllerClient | None = None
        self._readers: dict[str, CameraReader] = {}
        self._latest: dict[str, dict] = {}
        self._lock = threading.Lock()

    @cached_property
    def observation_features(self) -> dict:
        features: dict = {key: float for key in joint_keys(self.config.arms)}
        for camera in self.config.shared_cameras:
            reader = self._readers.get(camera)
            if reader is None:
                reader = CameraReader(camera)
                self._readers[camera] = reader
            features[camera] = (reader.height, reader.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict:
        return {key: float for key in joint_keys(self.config.arms)}

    @property
    def is_connected(self) -> bool:
        return self._client is not None

    @property
    def is_calibrated(self) -> bool:
        # Calibration lives in the service, and it is applied whenever the arm is bound.
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        self._client = ControllerClient()
        self._client.on_state(self._on_state)
        for camera in self.config.shared_cameras:
            self._readers.setdefault(camera, CameraReader(camera))

    def _on_state(self, batch: list[dict]) -> None:
        with self._lock:
            for sample in batch:
                self._latest[sample["arm"]] = sample

    def get_observation(self) -> dict:
        with self._lock:
            latest = dict(self._latest)
        observation: dict = {}
        keys = iter(joint_keys(self.config.arms))
        for arm in self.config.arms:
            sample = latest.get(arm)
            if sample is None:
                raise ConnectionError(f"no feedback from {arm} yet")
            for value in sample["position"]:
                observation[next(keys)] = float(value)
        for camera, reader in self._readers.items():
            frame = reader.latest(timeout=0.2)
            if frame is None:
                raise ConnectionError(f"camera {camera} is not producing frames")
            observation[camera] = frame.image
        return observation

    def send_action(self, action: dict) -> dict:
        if self._client is None:
            raise ConnectionError("not connected")
        keys = joint_keys(self.config.arms)
        values = [float(action[key]) for key in keys]
        for index, arm in enumerate(self.config.arms):
            pose = values[index * len(JOINT_NAMES) : (index + 1) * len(JOINT_NAMES)]
            self._client.set_target(arm, pose, self.config.source)
        return action

    def disconnect(self) -> None:
        for reader in self._readers.values():
            reader.close()
        self._readers.clear()
        if self._client:
            # Leaves the arm holding: releasing a brakeless arm is an explicit request.
            self._client.close()
            self._client = None


def as_array(observation: dict, arms: list[str]) -> np.ndarray:
    return np.array([observation[k] for k in joint_keys(arms)], dtype=np.float32)
