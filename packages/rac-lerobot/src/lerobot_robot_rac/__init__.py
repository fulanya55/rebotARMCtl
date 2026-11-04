"""LeRobot Robot proxy. LeRobot finds this package by its ``lerobot_robot_`` prefix."""

from .config import RacFollowerConfig
from .robot import RacFollower

__all__ = ["RacFollower", "RacFollowerConfig"]
