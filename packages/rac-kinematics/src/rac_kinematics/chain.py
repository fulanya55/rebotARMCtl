"""URDF kinematics: forward kinematics, Jacobian, damped IK and gravity torque.

Implemented directly with numpy so it installs everywhere, including Windows where
Pinocchio has no wheels. The IK approach (damped least squares with damping raised
near singularities) follows the method described in the ReBot RS Digital Twin
documentation; that repository declares no license, so none of its code is used.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def transform(xyz, rpy) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    r = np.eye(4)
    r[:3, :3] = [
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
    ]
    return r


def rotation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Rotation vector taking current to target, expressed in the base frame."""
    delta = target @ current.T
    cos_angle = np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if abs(angle - math.pi) < 1e-4:
        # The usual formula degenerates at 180 degrees, so read the axis off the
        # symmetric part instead.
        s = (delta + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(s), 0.0, None))
        pivot = int(np.argmax(axis))
        for i in range(3):
            if i != pivot and s[pivot, i] < 0:
                axis[i] = -axis[i]
        return axis / np.linalg.norm(axis) * angle
    axis = np.array([delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]])
    return axis / (2.0 * math.sin(angle)) * angle


def pose_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    return transform(xyz, rpy)


def matrix_to_rpy(r: np.ndarray) -> tuple[float, float, float]:
    sy = math.hypot(r[0, 0], r[1, 0])
    if sy > 1e-6:
        return (math.atan2(r[2, 1], r[2, 2]), math.atan2(-r[2, 0], sy), math.atan2(r[1, 0], r[0, 0]))
    return (math.atan2(-r[1, 2], r[1, 1]), math.atan2(-r[2, 0], sy), 0.0)


@dataclass
class Link:
    name: str
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float = -math.inf
    upper: float = math.inf


@dataclass
class IkResult:
    q: np.ndarray
    converged: bool
    position_error: float
    orientation_error: float
    iterations: int
    near_singular: bool


class Chain:
    """Serial chain from the URDF root to a named link, plus the rest of the tree for mass."""

    def __init__(self, urdf_path: str | Path, tip: str, joint_names: list[str] | None = None):
        self.path = Path(urdf_path)
        self.links, self.joints = _parse_urdf(self.path)
        children = {j.child: j for j in self.joints.values()}

        chain: list[Joint] = []
        cursor = tip
        while cursor in children:
            joint = children[cursor]
            chain.append(joint)
            cursor = joint.parent
        self.root = cursor
        chain.reverse()
        self.tip = tip
        self.chain = chain
        self.active = [j for j in chain if j.kind in ("revolute", "continuous", "prismatic")]
        if joint_names is not None:
            self.active = [j for j in self.active if j.name in joint_names]
        self.dof = len(self.active)
        self.lower = np.array([j.lower for j in self.active])
        self.upper = np.array([j.upper for j in self.active])
        self._index = {j.name: i for i, j in enumerate(self.active)}
        self._children_of: dict[str, list[Joint]] = {}
        for joint in self.joints.values():
            self._children_of.setdefault(joint.parent, []).append(joint)

    def _joint_transform(self, joint: Joint, q: np.ndarray) -> np.ndarray:
        index = self._index.get(joint.name)
        if index is None or joint.kind == "fixed":
            return joint.origin
        if joint.kind == "prismatic":
            slide = np.eye(4)
            slide[:3, 3] = joint.axis * q[index]
            return joint.origin @ slide
        return joint.origin @ axis_angle(joint.axis, q[index])

    def fk(self, q) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        pose = np.eye(4)
        for joint in self.chain:
            pose = pose @ self._joint_transform(joint, q)
        return pose

    def jacobian(self, q) -> np.ndarray:
        """Geometric Jacobian at the tip, in the base frame: linear rows then angular."""
        q = np.asarray(q, dtype=float)
        frames: list[tuple[int, np.ndarray, np.ndarray, str]] = []
        pose = np.eye(4)
        for joint in self.chain:
            at_joint = pose @ joint.origin
            index = self._index.get(joint.name)
            if index is not None:
                frames.append((index, at_joint[:3, 3].copy(), at_joint[:3, :3] @ joint.axis, joint.kind))
            pose = pose @ self._joint_transform(joint, q)
        tip = pose[:3, 3]
        jac = np.zeros((6, self.dof))
        for index, origin, axis, kind in frames:
            if kind == "prismatic":
                jac[:3, index] = axis
            else:
                jac[:3, index] = np.cross(axis, tip - origin)
                jac[3:, index] = axis
        return jac

    def solve_ik(
        self,
        target: np.ndarray,
        q0,
        position_only: bool = False,
        max_iterations: int = 200,
        position_tolerance: float = 5e-4,
        orientation_tolerance: float = 5e-3,
        max_step: float = 0.15,
        lambda_min: float = 0.018,
        lambda_max: float = 0.075,
    ) -> IkResult:
        """Damped least squares, staying near q0 and inside the joint limits.

        Damping rises from lambda_min towards lambda_max as the smallest singular
        value falls, so motion near a singularity slows down instead of jumping.
        """
        q = np.clip(np.asarray(q0, dtype=float), self.lower, self.upper)
        near_singular = False
        position_error = orientation_error = float("inf")
        best_q = q.copy()
        best_cost = float("inf")
        iteration = 0
        for iteration in range(1, max_iterations + 1):  # noqa: B007
            pose = self.fk(q)
            pos_err = target[:3, 3] - pose[:3, 3]
            rot_err = np.zeros(3) if position_only else rotation_error(pose[:3, :3], target[:3, :3])
            position_error = float(np.linalg.norm(pos_err))
            orientation_error = float(np.linalg.norm(rot_err))
            cost = position_error + 0.2 * orientation_error
            if cost < best_cost:
                best_cost = cost
                best_q = q.copy()
            if position_error < position_tolerance and orientation_error < orientation_tolerance:
                return IkResult(q, True, position_error, orientation_error, iteration, near_singular)

            jac = self.jacobian(q)
            if position_only:
                jac = jac[:3]
                error = pos_err
            else:
                error = np.concatenate([pos_err, rot_err])
            sigma = np.linalg.svd(jac, compute_uv=False)
            smallest = float(sigma[-1])
            threshold = 0.04
            if smallest < threshold:
                near_singular = True
                ratio = 1.0 - smallest / threshold
                damping = lambda_min + (lambda_max - lambda_min) * ratio
            else:
                damping = lambda_min
            size = jac.shape[0]
            dq = jac.T @ np.linalg.solve(jac @ jac.T + (damping**2) * np.eye(size), error)
            largest = float(np.max(np.abs(dq)))
            if largest > max_step:
                dq *= max_step / largest
            q = np.clip(q + dq, self.lower, self.upper)
        return IkResult(best_q, False, position_error, orientation_error, iteration, near_singular)

    def gravity_torque(self, q, gravity: np.ndarray = GRAVITY) -> np.ndarray:
        """Torque per active joint that balances gravity at q. Uses URDF inertial data."""
        q = np.asarray(q, dtype=float)
        torque = np.zeros(self.dof)

        joint_frames: list[tuple[int, np.ndarray, np.ndarray, str]] = []
        masses: list[tuple[float, np.ndarray, frozenset[int]]] = []

        def walk(link_name: str, pose: np.ndarray, moved_by: frozenset[int]) -> None:
            link = self.links.get(link_name)
            if link and link.mass > 0:
                masses.append((link.mass, (pose @ np.append(link.com, 1.0))[:3], moved_by))
            for joint in self._children_of.get(link_name, ()):
                at_joint = pose @ joint.origin
                index = self._index.get(joint.name)
                next_moved = moved_by
                if index is not None:
                    joint_frames.append((index, at_joint[:3, 3].copy(), at_joint[:3, :3] @ joint.axis, joint.kind))
                    next_moved = moved_by | {index}
                walk(joint.child, pose @ self._joint_transform(joint, q), next_moved)

        walk(self.root, np.eye(4), frozenset())
        by_index = {index: (origin, axis, kind) for index, origin, axis, kind in joint_frames}
        for mass, com, moved_by in masses:
            force = mass * gravity
            for index in moved_by:
                origin, axis, kind = by_index[index]
                if kind == "prismatic":
                    torque[index] -= float(axis @ force)
                else:
                    torque[index] -= float(axis @ np.cross(com - origin, force))
        return torque


def _floats(text: str | None, default: list[float]) -> list[float]:
    return [float(v) for v in text.split()] if text else list(default)


def _parse_urdf(path: Path) -> tuple[dict[str, Link], dict[str, Joint]]:
    root = ET.parse(path).getroot()
    links: dict[str, Link] = {}
    for element in root.findall("link"):
        link = Link(name=element.get("name"))
        inertial = element.find("inertial")
        if inertial is not None:
            mass = inertial.find("mass")
            if mass is not None:
                link.mass = float(mass.get("value"))
            origin = inertial.find("origin")
            if origin is not None:
                link.com = np.array(_floats(origin.get("xyz"), [0, 0, 0]))
        links[link.name] = link

    joints: dict[str, Joint] = {}
    for element in root.findall("joint"):
        origin = element.find("origin")
        xyz = _floats(origin.get("xyz") if origin is not None else None, [0, 0, 0])
        rpy = _floats(origin.get("rpy") if origin is not None else None, [0, 0, 0])
        axis_element = element.find("axis")
        axis = np.array(_floats(axis_element.get("xyz") if axis_element is not None else None, [0, 0, 1]))
        norm = np.linalg.norm(axis)
        axis = axis / norm if norm > 0 else axis
        limit = element.find("limit")
        joint = Joint(
            name=element.get("name"),
            kind=element.get("type"),
            parent=element.find("parent").get("link"),
            child=element.find("child").get("link"),
            origin=transform(xyz, rpy),
            axis=axis,
        )
        if limit is not None and limit.get("lower") is not None:
            joint.lower = float(limit.get("lower"))
            joint.upper = float(limit.get("upper"))
        joints[joint.name] = joint
    return links, joints
