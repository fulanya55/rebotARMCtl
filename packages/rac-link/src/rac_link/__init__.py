"""RAC-Link: wire protocol, device SDK and service IPC for the sync controller."""

from . import _gen as protocol
from ._gen import (
    JOINTS,
    AckStatus,
    ArmId,
    ArmModel,
    CanDialect,
    ControlSource,
    EstopState,
    EventCode,
    FollowMode,
    Mode,
    PairPhase,
    PortId,
    StopKind,
    TeleopAction,
)
from .client import ControllerClient
from .clock import ClockModel, host_now_us
from .device import ControllerDevice, ControllerError
from .ipc import IpcClient, ServiceError
from .robots import ARM_BY_NAME, ARM_NAMES, JOINT_NAMES, PORT_NAMES, SPECS, ArmSpec, default_mapping, spec_of

__all__ = [
    "ARM_BY_NAME",
    "ARM_NAMES",
    "JOINTS",
    "JOINT_NAMES",
    "PORT_NAMES",
    "SPECS",
    "AckStatus",
    "ArmId",
    "ArmModel",
    "ArmSpec",
    "CanDialect",
    "ClockModel",
    "ControlSource",
    "ControllerClient",
    "ControllerDevice",
    "ControllerError",
    "EstopState",
    "EventCode",
    "FollowMode",
    "IpcClient",
    "Mode",
    "PairPhase",
    "PortId",
    "ServiceError",
    "StopKind",
    "TeleopAction",
    "default_mapping",
    "host_now_us",
    "protocol",
    "spec_of",
]
