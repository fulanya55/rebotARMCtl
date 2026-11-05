"""Smoothing for targets that change while the arm is moving, and gripper assistance.

Copied from the ReBot RS Digital Twin (Yang-Ci/ReBot_Arm_DigitalTwin_RS at 84c1ea7),
rebotarmcontroller/motion_profiles.py and hardware_manager.py, where they run on a
real B601-RS. The default limits are that project's. The assistance function is
reshaped from a method into a function, and its numbers are unchanged.
"""

from __future__ import annotations

import math

import numpy as np

DEFAULT_ACCELERATION_LIMIT_RAD_S2 = 4.0
DEFAULT_JERK_LIMIT_RAD_S3 = 30.0
DEFAULT_NATURAL_FREQUENCY_RAD_S = 8.0
DEFAULT_MAX_SUBSTEP_S = 0.005
DEFAULT_MAX_ELAPSED_S = 0.1


def advance_jerk_limited_reference(
    position: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    target: np.ndarray,
    velocity_limit: np.ndarray,
    dt: float,
    *,
    acceleration_limit: float = DEFAULT_ACCELERATION_LIMIT_RAD_S2,
    jerk_limit: float = DEFAULT_JERK_LIMIT_RAD_S3,
    natural_frequency: float = DEFAULT_NATURAL_FREQUENCY_RAD_S,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Advance a continuously re-targetable, jerk-limited joint reference.

    Targets may change every few milliseconds while reference position, velocity
    and acceleration stay continuous. The desired acceleration comes from a
    critically damped second-order attractor, then acceleration and jerk limits
    turn it into a bounded third-order profile.
    """
    q = np.asarray(position, dtype=np.float64)
    qd = np.asarray(velocity, dtype=np.float64)
    qdd = np.asarray(acceleration, dtype=np.float64)
    goal = np.asarray(target, dtype=np.float64)
    vlim = np.maximum(np.asarray(velocity_limit, dtype=np.float64), 0.0)
    step_dt = float(np.clip(dt, 0.0005, 0.02))
    amax = max(float(acceleration_limit), 1e-6)
    jmax = max(float(jerk_limit), 1e-6)
    omega = max(float(natural_frequency), 1e-3)

    delta = goal - q
    desired_acceleration = np.clip(omega * omega * delta - 2.0 * omega * qd, -amax, amax)

    max_acceleration_change = jmax * step_dt
    next_acceleration = qdd + np.clip(desired_acceleration - qdd, -max_acceleration_change, max_acceleration_change)
    next_acceleration = np.clip(next_acceleration, -amax, amax)
    next_velocity = qd + 0.5 * (qdd + next_acceleration) * step_dt
    next_velocity = np.clip(next_velocity, -vlim, vlim)
    next_position = q + 0.5 * (qd + next_velocity) * step_dt

    settled = (
        (np.abs(goal - next_position) <= 1e-5)
        & (np.abs(next_velocity) <= 1e-3)
        & (np.abs(next_acceleration) <= max_acceleration_change)
    )
    next_position = np.where(settled, goal, next_position)
    next_velocity = np.where(settled, 0.0, next_velocity)
    next_acceleration = np.where(settled, 0.0, next_acceleration)
    return next_position, next_velocity, next_acceleration


def advance_jerk_limited_reference_over_elapsed(
    position: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    target: np.ndarray,
    velocity_limit: np.ndarray,
    elapsed: float,
    *,
    max_substep: float = DEFAULT_MAX_SUBSTEP_S,
    max_elapsed: float = DEFAULT_MAX_ELAPSED_S,
    acceleration_limit: float = DEFAULT_ACCELERATION_LIMIT_RAD_S2,
    jerk_limit: float = DEFAULT_JERK_LIMIT_RAD_S3,
    natural_frequency: float = DEFAULT_NATURAL_FREQUENCY_RAD_S,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Advance by wall-clock time without turning a late callback into a jump.

    Splitting the elapsed time into bounded substeps keeps the jerk and acceleration
    limits valid while the trajectory stays tied to real time, where integrating one
    capped step would make it run in slow motion.
    """
    safe_max_substep = float(np.clip(max_substep, 0.0005, 0.02))
    safe_max_elapsed = max(float(max_elapsed), safe_max_substep)
    total_dt = float(np.clip(elapsed, 0.0005, safe_max_elapsed))
    step_count = max(1, math.ceil(total_dt / safe_max_substep))
    step_dt = total_dt / step_count

    next_position = np.asarray(position, dtype=np.float64)
    next_velocity = np.asarray(velocity, dtype=np.float64)
    next_acceleration = np.asarray(acceleration, dtype=np.float64)
    for _ in range(step_count):
        next_position, next_velocity, next_acceleration = advance_jerk_limited_reference(
            next_position,
            next_velocity,
            next_acceleration,
            target,
            velocity_limit,
            step_dt,
            acceleration_limit=acceleration_limit,
            jerk_limit=jerk_limit,
            natural_frequency=natural_frequency,
        )
    return next_position, next_velocity, next_acceleration


def gripper_assist_torque(
    position: float,
    velocity: float,
    closed: float,
    opened: float,
    *,
    torque: float = 0.04,
    velocity_threshold: float = 0.02,
    velocity_full: float = 0.22,
    speed_limit: float = 0.8,
    breakaway_fraction: float = 0.30,
    limit_margin: float = 0.12,
) -> float:
    """Torque that helps a hand move the gripper, in raw motor radians.

    It helps only in the direction the gripper is already moving. It is zero at
    rest, near either end of travel and above the speed limit, so it cannot run
    away by itself.
    """
    speed = abs(float(velocity))
    if speed <= velocity_threshold or speed >= speed_limit:
        return 0.0
    direction = 1.0 if velocity > 0.0 else -1.0
    lower, upper = min(closed, opened), max(closed, opened)
    if position <= lower + limit_margin and direction < 0.0:
        return 0.0
    if position >= upper - limit_margin and direction > 0.0:
        return 0.0
    ramp = float(np.clip((speed - velocity_threshold) / max(velocity_full - velocity_threshold, 1e-6), 0.0, 1.0))
    return direction * torque * max(ramp, breakaway_fraction)
