"""Computer-side kinematics. Returns targets and trajectories, never executes them."""

from pathlib import Path

from .chain import Chain, IkResult, matrix_to_rpy, pose_from_xyz_rpy
from .trajectory import TeachPoint, TeachProgram, blend, min_duration, retime, window

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def assets_root() -> Path:
    """The repository's robot description directory."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "assets" / "robots"
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError("assets/robots not found above the rac_kinematics package")


def load_chain(urdf_relative: str, end_effector: str) -> Chain:
    """Chain for an arm spec. Only the six arm joints are active: the gripper is not kinematic."""
    return Chain(assets_root() / urdf_relative, end_effector, joint_names=ARM_JOINTS)


__all__ = [
    "ARM_JOINTS",
    "Chain",
    "IkResult",
    "TeachPoint",
    "TeachProgram",
    "assets_root",
    "blend",
    "load_chain",
    "matrix_to_rpy",
    "min_duration",
    "pose_from_xyz_rpy",
    "retime",
    "window",
]
