"""What each arm model is and can do, and the fixed joint order of the platform.

Joint order is the dataset contract: six joints base to wrist, then the gripper.
Joints are radians, the gripper is travel normalized to 0..1. A bimanual vector is
the left arm followed by the right arm.

Limits are the published URDF limits. Mapping presets between different models
are a starting point derived from those limits and have not been checked against
physical arms: confirm direction and offset joint by joint before following at
speed, as docs/bringup.md describes.
"""

from __future__ import annotations

from dataclasses import dataclass, field

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
    notes="RS06 on joints 1-3, RS00 on joints 4-6 and the gripper. No brakes: never release away from rest.",
)

_STAR_LOWER = (-2.27, -1.57, -1.57, -1.57, -2.27, -2.27, 0.0)
_STAR_UPPER = (2.27, 1.57, 1.57, 1.57, 2.27, 2.27, 1.0)

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
    urdf="star_arm_102/urdf/star_arm_102.urdf",
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
    lower=_STAR_LOWER,
    upper=_STAR_UPPER,
    can_execute=False,
    can_hold=True,
    can_guide=False,
    service_period_us=8000,
    home=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    urdf="star_arm_102/urdf/star_arm_102.urdf",
    end_effector="gripper_tip",
    gripper_raw_open=1.5708,
    notes="Pose-holding leader. Carries a button board at servo id 7.",
)

STAR_LD = ArmSpec(
    model=ArmModel.STAR_102_LD,
    key="star_102_ld",
    label="Star Arm 102-LD",
    family="star",
    port_kind=PortKind.UART,
    lower=_STAR_LOWER,
    upper=_STAR_UPPER,
    can_execute=False,
    can_hold=False,
    can_guide=False,
    service_period_us=8000,
    home=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    urdf="star_arm_102/urdf/star_arm_102.urdf",
    end_effector="gripper_tip",
    gripper_raw_open=1.5708,
    notes="Lightweight leader. Read-only: it cannot hold or execute.",
)

SPECS: dict[int, ArmSpec] = {int(s.model): s for s in (REBOT_RS, STAR_FL, STAR_HD, STAR_LD)}
SPEC_BY_KEY: dict[str, ArmSpec] = {s.key: s for s in SPECS.values()}


def spec_of(model: int) -> ArmSpec:
    try:
        return SPECS[int(model)]
    except KeyError:
        raise ValueError(f"no specification for arm model {model}") from None


def default_mapping(leader: ArmSpec, follower: ArmSpec) -> dict:
    """Leader to follower mapping for a pair of models.

    Same model: identity. Different models: unit scale, with an offset that puts
    the two range midpoints together, and the gripper always mapped 0..1 to 0..1.
    The direction of each joint comes from URDF axes, so a preset is a starting
    point to confirm against real arms, not a measured value.
    """
    scale = [1.0] * JOINTS
    offset = [0.0] * JOINTS
    if leader.family != follower.family:
        for j in range(JOINTS - 1):
            sign = _AXIS_SIGN.get(leader.family, [1] * 6)[j] * _AXIS_SIGN.get(follower.family, [1] * 6)[j]
            scale[j] = float(sign)
            mid_leader = (leader.lower[j] + leader.upper[j]) / 2.0
            mid_follower = (follower.lower[j] + follower.upper[j]) / 2.0
            offset[j] = mid_follower - scale[j] * mid_leader
    return {
        "source_joint": list(range(JOINTS)),
        "scale": scale,
        "offset": offset,
        "gripper_link": True,
    }


# Sign of the URDF axis per joint, read from each published model.
_AXIS_SIGN = {
    "rs": [-1, 1, -1, -1, -1, -1],
    "star": [-1, -1, -1, 1, -1, -1],
}


def dataset_joint_names(bimanual: bool) -> list[str]:
    if not bimanual:
        return list(JOINT_NAMES)
    return [f"left_{n}" for n in JOINT_NAMES] + [f"right_{n}" for n in JOINT_NAMES]
