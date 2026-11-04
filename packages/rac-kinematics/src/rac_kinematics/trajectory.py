"""Trajectories for teaching and replay. They only return points to execute."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np


def min_jerk(s: np.ndarray) -> np.ndarray:
    """Quintic blend from 0 to 1 with zero velocity and acceleration at both ends."""
    return s**3 * (10.0 - 15.0 * s + 6.0 * s**2)


def blend(q_from, q_to, duration_s: float, dt_s: float) -> np.ndarray:
    """Sampled min-jerk move, ending exactly at q_to."""
    q_from = np.asarray(q_from, dtype=float)
    q_to = np.asarray(q_to, dtype=float)
    steps = max(1, int(round(duration_s / dt_s)))
    s = min_jerk(np.linspace(0.0, 1.0, steps + 1))[1:]
    return q_from + np.outer(s, q_to - q_from)


def min_duration(q_from, q_to, velocity_limit, minimum_s: float = 0.2) -> float:
    """Shortest min-jerk duration within a velocity limit. Peak speed is 1.875 x mean."""
    delta = np.abs(np.asarray(q_to, dtype=float) - np.asarray(q_from, dtype=float))
    limits = np.maximum(np.asarray(velocity_limit, dtype=float), 1e-3)
    return max(minimum_s, float(np.max(1.875 * delta / limits)))


@dataclass
class TeachPoint:
    name: str
    q: list[float]
    move_time_s: float = 2.0
    dwell_s: float = 0.0


@dataclass
class TeachProgram:
    name: str
    arm: str
    model: str = ""
    points: list[TeachPoint] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> TeachProgram:
        return cls(
            name=data["name"],
            arm=data["arm"],
            model=data.get("model", ""),
            points=[TeachPoint(**p) for p in data.get("points", [])],
        )

    def sample(self, q_start, dt_s: float, velocity_limit=None) -> np.ndarray:
        """Sample the program from a starting pose.

        A segment shorter than the velocity limit allows is stretched rather than
        executed too fast, so the preview shows what the arm will actually do.
        """
        current = np.asarray(q_start, dtype=float)
        parts: list[np.ndarray] = []
        for point in self.points:
            goal = np.asarray(point.q, dtype=float)
            duration = point.move_time_s
            if velocity_limit is not None:
                duration = max(duration, min_duration(current, goal, velocity_limit))
            parts.append(blend(current, goal, duration, dt_s))
            if point.dwell_s > 0:
                parts.append(np.tile(goal, (max(1, int(round(point.dwell_s / dt_s))), 1)))
            current = goal
        if not parts:
            return np.empty((0, len(current)))
        return np.concatenate(parts)


def retime(points: np.ndarray, source_dt_s: float, target_dt_s: float, speed: float = 1.0) -> np.ndarray:
    """Linear resampling, so an episode recorded at 30 Hz can play back at any rate and speed."""
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points.copy()
    if speed <= 0:
        raise ValueError("speed must be positive")
    source_t = np.arange(len(points)) * source_dt_s
    target_t = np.arange(0.0, source_t[-1] + 1e-9, target_dt_s * speed)
    return np.stack([np.interp(target_t, source_t, points[:, j]) for j in range(points.shape[1])], axis=1)


def window(points: np.ndarray, dt_s: float, start_s: float | None, end_s: float | None) -> np.ndarray:
    start = 0 if start_s is None else max(0, int(round(start_s / dt_s)))
    end = len(points) if end_s is None else min(len(points), int(round(end_s / dt_s)) + 1)
    return points[start:end]
