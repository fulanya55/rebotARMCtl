"""Model process: unified observation and action interface over LeRobot and StarVLA."""

from .backends import BACKENDS, make_backend
from .spec import IoSpec, Normalization, Observation, PolicyBackend

__all__ = ["BACKENDS", "IoSpec", "Normalization", "Observation", "PolicyBackend", "make_backend"]
