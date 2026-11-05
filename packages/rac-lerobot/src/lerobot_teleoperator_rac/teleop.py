"""A LeRobot ``Teleoperator`` backed by the Controller Service.

In this platform the leader to follower loop runs on the controller, so a LeRobot
teleoperation script is an observer of that loop: ``get_action`` reports the target
the follower actually received, which is exactly what the dataset's action column
must hold. Using the leader pose here instead would record what was asked for rather
than what was dispatched.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from functools import cached_property

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot_robot_rac.robot import joint_keys, wait_for_feedback
from rac_link.client import ControllerClient


@TeleoperatorConfig.register_subclass("rac_leader")
@dataclass
class RacLeaderConfig(TeleoperatorConfig):
    # The followers whose dispatched targets this reports.
    arms: list[str] = field(default_factory=lambda: ["left_follower"])
    # Enter and start local teleoperation when connecting, end it when disconnecting.
    manage_session: bool = True
    follow: str = "relative"


class RacLeader(Teleoperator):
    config_class = RacLeaderConfig
    name = "rac_leader"

    def __init__(self, config: RacLeaderConfig) -> None:
        super().__init__(config)
        self.config = config
        self._client: ControllerClient | None = None
        self._latest: dict[str, dict] = {}
        self._lock = threading.Lock()

    @cached_property
    def action_features(self) -> dict:
        return {key: float for key in joint_keys(self.config.arms)}

    @cached_property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._client is not None

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        self._client = ControllerClient()
        self._client.on_state(self._on_state)
        if self.config.manage_session:
            self._client.teleop_enter()
            self._client.teleop_start(follow=self.config.follow)
        wait_for_feedback(self.config.arms, self._latest, self._lock)

    def _on_state(self, batch: list[dict]) -> None:
        with self._lock:
            for sample in batch:
                self._latest[sample["arm"]] = sample

    def get_action(self) -> dict:
        with self._lock:
            latest = dict(self._latest)
        action: dict = {}
        keys = iter(joint_keys(self.config.arms))
        for arm in self.config.arms:
            sample = latest.get(arm)
            if sample is None:
                raise ConnectionError(f"no feedback from {arm} yet")
            for value in sample["target"]:
                action[next(keys)] = float(value)
        return action

    def send_feedback(self, feedback: dict) -> None:
        pass

    def disconnect(self) -> None:
        if self._client:
            if self.config.manage_session:
                self._client.teleop_end()
            self._client.close()
            self._client = None
