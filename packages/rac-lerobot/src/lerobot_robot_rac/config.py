from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("rac_follower")
@dataclass
class RacFollowerConfig(RobotConfig):
    # Logical arms behind the Controller Service: one for single arm, two for bimanual.
    arms: list[str] = field(default_factory=lambda: ["left_follower"])
    # Camera names published by the shared capture module. Opening a camera here
    # would fight the capture module for the device, so these are only read.
    shared_cameras: list[str] = field(default_factory=list)
    # LeRobot's own camera objects are not used: the capture module owns the devices.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    source: str = "policy"
