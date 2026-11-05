"""What each arm model is and can do, and the fixed joint order of the platform.

Joint order is the dataset contract: six joints base to wrist, then the gripper.
Joints are radians, the gripper is travel normalized to 0..1. A bimanual vector is
the left arm followed by the right arm.

Follower limits are the published URDF limits. Star leaders use the ranges the
Seeed leader integration reads, because the URDF ones are placeholders that stop at
a quarter turn on joints that travel further. The mapping between reBot and Star
comes from Seeed's published integration, see default_mapping. None of this has
been run on physical arms here: confirm it joint by joint as docs/bringup.md says.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import radians

from ._gen import JOINTS, MAX_PAIRS, ArmId, ArmModel, PortId, PortKind

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")
assert len(JOINT_NAMES) == JOINTS

ARM_NAMES = {
    ArmId.LEFT_LEADER: "left_leader",
    ArmId.LEFT_FOLLOWER: "left_follower",
    ArmId.RIGHT_LEADER: "right_leader",
    ArmId.RIGHT_FOLLOWER: "right_follower",
}
ARM_BY_NAME = {name: arm for arm, name in ARM_NAMES.items()}

PORT_NAMES = {PortId.CAN1: "CAN1", PortId.CAN2: "CAN2", PortId.UART1: "UART1", PortId.UART2: "UART2"}

PORT_KIND = {
    PortId.CAN1: PortKind.CAN,
    PortId.CAN2: PortKind.CAN,
    PortId.UART1: PortKind.UART,
    PortId.UART2: PortKind.UART,
}

# Pair index: left is 0, right is 1.
PAIR_ARMS = {0: (ArmId.LEFT_LEADER, ArmId.LEFT_FOLLOWER), 1: (ArmId.RIGHT_LEADER, ArmId.RIGHT_FOLLOWER)}
assert len(PAIR_ARMS) == MAX_PAIRS


@dataclass(frozen=True)
class ArmSpec:
    model: ArmModel
    key: str
    label: str
    family: str
    port_kind: PortKind
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    can_execute: bool
    can_hold: bool
    can_guide: bool
    service_period_us: int
    home: tuple[float, ...]
    urdf: str
    end_effector: str
    velocity_limit: tuple[float, ...] = (1.5, 1.5, 1.5, 2.0, 2.0, 2.0, 1.5)
    acceleration_limit: tuple[float, ...] = (10.0, 10.0, 10.0, 15.0, 15.0, 15.0, 15.0)
    kp: tuple[float, ...] = (80.0, 150.0, 150.0, 50.0, 50.0, 50.0, 50.0)
    kd: tuple[float, ...] = (5.0, 10.0, 10.0, 5.0, 4.0, 4.0, 4.0)
    # Applied to the URDF gravity torque of the six arm joints.
    gravity_scale: tuple[float, ...] = (1.0,) * 6
    gripper_raw_closed: float = 0.0
    gripper_raw_open: float = 5.0
    notes: str = ""
    extra: dict = field(default_factory=dict)

    def describe(self) -> dict:
        return {
            "model": int(self.model),
            "key": self.key,
            "label": self.label,
            "family": self.family,
            "port_kind": self.port_kind.name,
            "lower": list(self.lower),
            "upper": list(self.upper),
            "can_execute": self.can_execute,
            "can_hold": self.can_hold,
            "can_guide": self.can_guide,
            "service_period_us": self.service_period_us,
            "home": list(self.home),
            "urdf": self.urdf,
            "end_effector": self.end_effector,
            "notes": self.notes,
        }


REBOT_RS = ArmSpec(
    model=ArmModel.REBOT_B601_RS,
    key="rebot_b601_rs",
    label="reBot Arm B601-RS",
    family="rs",
    port_kind=PortKind.CAN,
    lower=(-2.8, 0.0, 0.0, -1.57, -1.57, -3.14, 0.0),
    upper=(2.8, 3.14, 3.14, 1.57, 1.57, 3.14, 1.0),
    can_execute=True,
    can_hold=True,
    can_guide=True,
    service_period_us=4000,
    home=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    urdf="rebot_b601_rs/urdf/ReBot_Arm_RS.urdf",
    end_effector="gripper_end",
    # Tuned on a real arm by the ReBot RS Digital Twin. Its sweeps measure the model
    # within 5 to 11 percent on the joints that carry load, see tests/test_kinematics.py.
    gravity_scale=(1.0, 0.92, 0.98, 1.0, 1.0, 1.0),
    notes="RS06 on joints 1-3, RS00 on joints 4-6 and the gripper. No brakes: never release away from rest.",
)

_STAR_LOWER = (-2.27, -1.57, -1.57, -1.57, -2.27, -2.27, 0.0)
_STAR_UPPER = (2.27, 1.57, 1.57, 1.57, 2.27, 2.27, 1.0)
# Raw servo travel of a leader from its zero pose, in degrees, gripper excluded.
_STAR_LEADER_LOWER = tuple(radians(v) for v in (-150, -1, -200, -80, -90, -130)) + (0.0,)
_STAR_LEADER_UPPER = tuple(radians(v) for v in (150, 170, 1, 90, 90, 130)) + (1.0,)
# Fashion Star's control script maps 60 degrees of trigger onto 90 degrees of FL gripper.
_STAR_LEADER_GRIPPER_OPEN = radians(60)
_STAR_URDF_FL = "star_arm_102/fl/urdf/star_arm_102_fl.urdf"
_STAR_URDF_LEADER = "star_arm_102/ld_hd/urdf/star_arm_102_ld_hd.urdf"

STAR_FL = ArmSpec(
    model=ArmModel.STAR_102_FL,
    key="star_102_fl",
    label="Star Arm 102-FL",
    family="star",
    port_kind=PortKind.UART,
    lower=_STAR_LOWER,
    upper=_STAR_UPPER,
    can_execute=True,
    can_hold=True,
    can_guide=False,
    service_period_us=8000,
    home=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    urdf=_STAR_URDF_FL,
    end_effector="gripper_tip",
    gripper_raw_open=1.5708,
    notes="Executing follower.",
)

STAR_HD = ArmSpec(
    model=ArmModel.STAR_102_HD,
    key="star_102_hd",
    label="Star Arm 102-HD",
    family="star",
    port_kind=PortKind.UART,
    lower=_STAR_LEADER_LOWER,
    upper=_STAR_LEADER_UPPER,
    can_execute=False,
    can_hold=True,
    can_guide=False,
    service_period_us=8000,
    home=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    urdf=_STAR_URDF_LEADER,
    end_effector="gripper_tip",
    gripper_raw_open=_STAR_LEADER_GRIPPER_OPEN,
    notes="Pose-holding leader. Carries a button board at servo id 7.",
)

STAR_LD = ArmSpec(
    model=ArmModel.STAR_102_LD,
    key="star_102_ld",
    label="Star Arm 102-LD",
    family="star",
    port_kind=PortKind.UART,
    lower=_STAR_LEADER_LOWER,
    upper=_STAR_LEADER_UPPER,
    can_execute=False,
    can_hold=False,
    can_guide=False,
    service_period_us=8000,
    home=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    urdf=_STAR_URDF_LEADER,
    end_effector="gripper_tip",
    gripper_raw_open=_STAR_LEADER_GRIPPER_OPEN,
    notes="Lightweight leader. Read-only: it cannot hold or execute.",
)

SPECS: dict[int, ArmSpec] = {int(s.model): s for s in (REBOT_RS, STAR_FL, STAR_HD, STAR_LD)}
SPEC_BY_KEY: dict[str, ArmSpec] = {s.key: s for s in SPECS.values()}


def spec_of(model: int) -> ArmSpec:
    try:
        return SPECS[int(model)]
    except KeyError:
        raise ValueError(f"no specification for arm model {model}") from None


# Star raw servo angle to reBot RS joint. This is the table in Seeed's integration
# as the ReBot RS Digital Twin uses it. LeRobot's rebot_102_leader has the same table
# negated, because its B601 follower counts the other way, and the joint axes of the
# real URDFs agree once they are compared in the world frame. tests/test_kinematics.py
# checks that. It is its own inverse, so it also serves reBot leading Star.
_STAR_RS_DIRECTION = (1.0, 1.0, -1.0, -1.0, -1.0, 1.0)


def default_mapping(leader: ArmSpec, follower: ArmSpec) -> dict:
    """Leader to follower mapping for a pair of models.

    Every model's zero is its calibration pose, so offsets are zero. Star arms copy
    each other's raw angles, as Fashion Star's control script does, and a reBot and a
    Star differ by direction. The gripper is always mapped 0..1 to 0..1.
    """
    scale = [1.0] * JOINTS
    if leader.family != follower.family:
        scale[: JOINTS - 1] = _STAR_RS_DIRECTION
    return {
        "source_joint": list(range(JOINTS)),
        "scale": scale,
        "offset": [0.0] * JOINTS,
        "gripper_link": True,
    }


def dataset_joint_names(bimanual: bool) -> list[str]:
    if not bimanual:
        return list(JOINT_NAMES)
    return [f"left_{n}" for n in JOINT_NAMES] + [f"right_{n}" for n in JOINT_NAMES]
