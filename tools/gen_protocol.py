#!/usr/bin/env python3
"""Generate the RAC-Link protocol bindings from the single definition below.

Outputs (both committed, so a checkout builds without running this):

* ``firmware/core/rac_link_gen.h``                 packed C structs, enums
* ``packages/rac-link/src/rac_link/_gen.py``       dataclasses with pack/unpack

``tests/test_protocol_gen.py`` regenerates into memory and fails when the
committed files drift from this definition.

Wire format summary (see docs/protocol.md):

    frame   := COBS(header || body || crc16_le) || 0x00
    header  := 24 bytes, little endian, always present
    crc16   := CRC-16/CCITT-FALSE over header || body
"""

from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
C_OUT = ROOT / "firmware" / "core" / "rac_link_gen.h"
PY_OUT = ROOT / "packages" / "rac-link" / "src" / "rac_link" / "_gen.py"

PROTO_VERSION = 1
MAX_ARMS = 4
MAX_PORTS = 4
MAX_PAIRS = 2
JOINTS = 7
MAX_CHUNK_POINTS = 16

# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

ENUMS: dict[str, tuple[str, list[tuple[str, int, str]]]] = {
    "MsgType": (
        "RAC_MSG",
        [
            # Host to controller
            ("HELLO", 0x01, "Identify the device; answered by DEVICE_INFO"),
            ("BIND_PORT", 0x02, "Bind a physical port to a logical arm, model and role"),
            ("DISCOVER", 0x03, "Read-only probe of every port; never energizes a motor"),
            ("SET_CALIBRATION", 0x04, "Per-arm zero, direction, range and gripper range"),
            ("SET_PAIRING", 0x05, "Leader to follower mapping for one side"),
            ("TELEOP_CONTROL", 0x06, "Align, start, pause, resume or end local teleoperation"),
            ("SET_RATES", 0x07, "Motor service, state read and upstream periods"),
            ("SET_LIMITS", 0x08, "Per-arm velocity and acceleration limits and gains"),
            ("SET_MODE", 0x09, "Switch mode; adopts the generation in the header"),
            ("TARGET_CHUNK", 0x0A, "Block of timed joint targets for one arm"),
            ("STOP", 0x0B, "Clear queues and hold or damp according to capability"),
            ("PAUSE_RESUME", 0x0C, "Freeze or continue target playback"),
            ("FEEDFORWARD", 0x0D, "Torque feedforward, used by guide mode"),
            ("TORQUE", 0x0E, "Enable or release motor torque"),
            ("TIME_SYNC", 0x10, "Clock alignment request; answered by TIME_SYNC_REPLY"),
            ("HEARTBEAT", 0x11, "Host liveness; absence triggers a communication stop"),
            ("QUERY_STATUS", 0x12, "Ask for CONTROLLER_STATUS and PORT_STATUS now"),
            ("RESET_ESTOP", 0x13, "Clear a latched emergency stop once the button is released"),
            ("SIL_INJECT", 0x70, "Software-in-the-loop fault and hand motion injection"),
            # Controller to host
            ("ACK", 0x80, "Execution acknowledgement for a host command"),
            ("DEVICE_INFO", 0x81, "Firmware, hardware and capability description"),
            ("PORT_STATUS", 0x82, "Per-port binding and link state"),
            ("ARM_STATE", 0x83, "One sample of one arm with its dispatched target"),
            ("CONTROLLER_STATUS", 0x84, "Mode, generation, e-stop and timing statistics"),
            ("EVENT", 0x85, "Asynchronous control event"),
            ("TIME_SYNC_REPLY", 0x90, "Controller receive and transmit timestamps"),
        ],
    ),
    "AckStatus": (
        "RAC_ACK",
        [
            ("OK", 0, ""),
            ("STALE_GENERATION", 1, "Rejected: generation older than the current one"),
            ("BAD_ARGUMENT", 2, ""),
            ("BUSY", 3, ""),
            ("QUEUE_FULL", 4, ""),
            ("ESTOP_ACTIVE", 5, "Rejected: emergency stop is latched"),
            ("MODE_REJECTED", 6, "Rejected: command not valid in the current mode"),
            ("UNSUPPORTED", 7, "Rejected: arm capability or protocol version"),
            ("CRC_ERROR", 8, ""),
        ],
    ),
    "ArmId": (
        "RAC_ARM",
        [
            ("LEFT_LEADER", 0, ""),
            ("LEFT_FOLLOWER", 1, ""),
            ("RIGHT_LEADER", 2, ""),
            ("RIGHT_FOLLOWER", 3, ""),
            ("NONE", 0xFF, "Device-level message or unbound port"),
        ],
    ),
    "PortId": (
        "RAC_PORT",
        [("CAN1", 0, ""), ("CAN2", 1, ""), ("UART1", 2, ""), ("UART2", 3, "")],
    ),
    "PortKind": ("RAC_PORT_KIND", [("CAN", 0, ""), ("UART", 1, "")]),
    "ArmModel": (
        "RAC_MODEL",
        [
            ("NONE", 0, ""),
            ("REBOT_B601_RS", 1, "RobStride RS06 x3, RS00 x4 on CAN, 48 V"),
            ("STAR_102_FL", 2, "Star Arm 102 follower, can execute"),
            ("STAR_102_HD", 3, "Star Arm 102 pose-holding leader"),
            ("STAR_102_LD", 4, "Star Arm 102 lightweight leader, cannot execute"),
            ("STAR_102_UNKNOWN", 5, "Servos answered; the variant needs user confirmation"),
        ],
    ),
    "CanDialect": (
        "RAC_CAN_DIALECT",
        [
            ("RS_PRIVATE", 0, "29-bit private protocol, the RobStride factory default"),
            ("RS_MIT", 1, "11-bit MIT protocol"),
        ],
    ),
    "Mode": (
        "RAC_MODE",
        [
            ("IDLE", 0, "Reading only, torque released"),
            ("HOLD", 1, "Holding the current pose"),
            ("LOCAL_TELEOP", 2, "Leader sampling and mapping run on the controller"),
            ("HOST_TARGET", 3, "Executing timed targets from the computer"),
            ("GUIDE", 4, "Low impedance or gravity compensated hand guidance"),
            ("ESTOP", 5, "Latched emergency stop"),
        ],
    ),
    "ControlSource": (
        "RAC_SOURCE",
        [
            ("NONE", 0, ""),
            ("TELEOP", 1, ""),
            ("TEACH", 2, ""),
            ("POLICY", 3, ""),
            ("INTERVENTION", 4, "Human correction during a policy run"),
            ("REPLAY", 5, ""),
            ("HOME", 6, ""),
            ("GUIDE", 7, ""),
        ],
    ),
    "TeleopAction": (
        "RAC_TELEOP",
        [("ALIGN", 0, ""), ("START", 1, ""), ("PAUSE", 2, ""), ("RESUME", 3, ""), ("END", 4, "")],
    ),
    "FollowMode": (
        "RAC_FOLLOW",
        [
            ("ABSOLUTE", 0, "The follower converges on the mapped leader pose"),
            ("RELATIVE", 1, "An offset captured at start is kept; joint zeros are untouched"),
        ],
    ),
    "PairPhase": (
        "RAC_PAIR",
        [
            ("OFF", 0, ""),
            ("ALIGNING", 1, ""),
            ("ALIGNED", 2, ""),
            ("FOLLOWING", 3, ""),
            ("PAUSED", 4, ""),
        ],
    ),
    "StopKind": (
        "RAC_STOP",
        [
            ("HOLD", 0, "Hold the current pose where the arm can"),
            ("DAMP", 1, "Velocity damping only"),
            ("RELEASE", 2, "Release torque"),
        ],
    ),
    "EventCode": (
        "RAC_EVT",
        [
            ("BOOT", 1, ""),
            ("ESTOP_PRESSED", 2, ""),
            ("ESTOP_RELEASED", 3, ""),
            ("ESTOP_RESET", 4, ""),
            ("MODE_CHANGED", 5, ""),
            ("HOST_LOST", 6, "Heartbeat timed out; every arm was stopped"),
            ("HOST_RESTORED", 7, ""),
            ("MOTOR_LOST", 8, "arg0: motor mask"),
            ("MOTOR_RESTORED", 9, "arg0: motor mask"),
            ("MOTOR_FAULT", 10, "arg0: motor index, arg1: fault bits"),
            ("LIMIT_CLAMPED", 11, "arg0: joint mask"),
            ("QUEUE_UNDERRUN", 12, ""),
            ("STALE_REJECTED", 13, "arg0: rejected generation, arg1: current generation"),
            ("ALIGN_DONE", 14, "arg0: pair index"),
            ("TELEOP_PHASE", 15, "arg0: pair index, arg1: phase"),
            ("QUEUE_CLEARED", 16, "arg0: points dropped"),
            ("OVERRUN", 17, "arg0: microseconds late"),
            ("LEADER_STALE", 18, "arg0: pair index"),
            ("LEADER_JUMP", 19, "arg0: pair index. The reading jumped, so following stopped"),
        ],
    ),
    "EstopState": (
        "RAC_ESTOP",
        [
            ("CLEAR", 0, ""),
            ("PRESSED", 1, "Button is pressed now"),
            ("LATCHED", 2, "Button released; waiting for RESET_ESTOP"),
        ],
    ),
    "SilOp": (
        "RAC_SIL",
        [
            ("SET_HAND_POSE", 1, "Move a passive arm as a hand would; port in arg, pose in values"),
            ("PRESS_ESTOP", 2, ""),
            ("RELEASE_ESTOP", 3, ""),
            ("DROP_MOTOR", 4, "port in arg, motor mask in arg2"),
            ("RESTORE_MOTOR", 5, ""),
            ("DROP_PORT", 6, ""),
            ("RESTORE_PORT", 7, ""),
            ("HAND_SINE", 8, "Drive a passive arm with a slow sine; amplitude in values[0]"),
            ("SET_TIME_SCALE", 9, "Virtual time rate; values[0]"),
            ("SET_MODEL", 10, "Attach an arm model to a port; arg port, arg2 model"),
        ],
    ),
}

CAP_FLAGS = [
    ("LOCAL_TELEOP", 1 << 0, ""),
    ("GUIDE", 1 << 1, ""),
    ("GRAVITY_FF", 1 << 2, ""),
    ("HW_ESTOP", 1 << 3, ""),
    ("SIL", 1 << 4, "Running as software-in-the-loop, not on a real board"),
]

# Sample validity bits in ArmState.flags.
STATE_FLAGS = [
    ("TARGET_VALID", 1 << 0, "A target was dispatched for this sample"),
    ("LEADER_STALE", 1 << 1, ""),
    ("LIMIT_CLAMPED", 1 << 2, "The dispatched target was clamped"),
    ("TORQUE_ON", 1 << 3, ""),
    ("QUEUE_EMPTY", 1 << 4, ""),
]

# ---------------------------------------------------------------------------
# Messages. Each field is (name, type, count); count 0 means a scalar.
# ---------------------------------------------------------------------------

Field = tuple[str, str, int]

HEADER: list[Field] = [
    ("version", "u8", 0),
    ("msg_type", "u8", 0),
    ("flags", "u16", 0),
    ("device_id", "u16", 0),
    ("arm_id", "u8", 0),
    ("reserved", "u8", 0),
    ("seq", "u32", 0),
    ("generation", "u32", 0),
    ("timestamp_us", "u64", 0),
]

J = JOINTS

MESSAGES: dict[str, list[Field]] = {
    "BindPort": [
        ("port", "u8", 0),
        ("arm_id", "u8", 0),
        ("model", "u8", 0),
        ("can_dialect", "u8", 0),
        ("baudrate", "u32", 0),
    ],
    "SetCalibration": [
        ("version", "u32", 0),
        ("zero", "f32", J),
        ("direction", "f32", J),
        ("lower", "f32", J),
        ("upper", "f32", J),
        ("gripper_closed", "f32", 0),
        ("gripper_open", "f32", 0),
    ],
    "SetPairing": [
        ("pair", "u8", 0),
        ("leader_arm", "u8", 0),
        ("follower_arm", "u8", 0),
        ("enabled", "u8", 0),
        ("gripper_link", "u8", 0),
        ("reserved", "u8", 3),
        ("source_joint", "u8", J),
        ("reserved2", "u8", 0),
        ("scale", "f32", J),
        ("offset", "f32", J),
    ],
    "TeleopControl": [
        ("action", "u8", 0),
        ("follow_mode", "u8", 0),
        ("pair_mask", "u8", 0),
        ("reserved", "u8", 0),
        ("align_speed", "f32", 0),
    ],
    "SetRates": [
        ("motor_service_us", "u32", MAX_PORTS),
        ("state_read_us", "u32", MAX_PORTS),
        ("upstream_us", "u32", 0),
        ("host_timeout_ms", "u32", 0),
    ],
    "SetLimits": [
        ("velocity", "f32", J),
        ("acceleration", "f32", J),
        ("kp", "f32", J),
        ("kd", "f32", J),
    ],
    "SetMode": [
        ("mode", "u8", 0),
        ("arm_mask", "u8", 0),
        ("source", "u8", 0),
        ("reserved", "u8", 0),
    ],
    # Followed by point_count * JOINTS f32 values.
    "TargetChunk": [
        ("exec_seq", "u32", 0),
        ("dt_us", "u32", 0),
        ("t_start_us", "u64", 0),
        ("point_count", "u16", 0),
        ("joint_count", "u8", 0),
        ("source", "u8", 0),
    ],
    "Stop": [("kind", "u8", 0), ("arm_mask", "u8", 0), ("reserved", "u8", 2)],
    "PauseResume": [("pause", "u8", 0), ("arm_mask", "u8", 0), ("reserved", "u8", 2)],
    "Feedforward": [("valid_ms", "u32", 0), ("torque", "f32", J)],
    "Torque": [("enable", "u8", 0), ("arm_mask", "u8", 0), ("reserved", "u8", 2)],
    "Ack": [
        ("acked_seq", "u32", 0),
        ("exec_seq", "u32", 0),
        ("acked_type", "u8", 0),
        ("status", "u8", 0),
        ("queue_free", "u16", 0),
    ],
    "DeviceInfo": [
        ("fw_major", "u8", 0),
        ("fw_minor", "u8", 0),
        ("fw_patch", "u8", 0),
        ("protocol", "u8", 0),
        ("uid", "u8", 12),
        ("hw_revision", "u8", 0),
        ("can_ports", "u8", 0),
        ("uart_ports", "u8", 0),
        ("max_chunk_points", "u8", 0),
        ("capabilities", "u32", 0),
        ("tick_hz", "u32", 0),
        ("queue_points", "u32", 0),
    ],
    "PortStatus": [
        ("port", "u8", 0),
        ("kind", "u8", 0),
        ("arm_id", "u8", 0),
        ("model", "u8", 0),
        ("detected_model", "u8", 0),
        ("can_dialect", "u8", 0),
        ("online_mask", "u8", 0),
        ("expected_mask", "u8", 0),
        ("baudrate", "u32", 0),
        ("tx_errors", "u32", 0),
        ("rx_errors", "u32", 0),
        ("bus_load_permille", "u16", 0),
        ("reserved", "u16", 0),
    ],
    # One message per sample. sample_* describes the feedback read from the
    # motors; target_* describes the target actually dispatched to them. A host
    # that wants dataset `state` reads position, and `action` reads target.
    "ArmState": [
        ("sample_seq", "u32", 0),
        ("sample_ts_us", "u64", 0),
        ("target_seq", "u32", 0),
        ("target_ts_us", "u64", 0),
        ("exec_seq", "u32", 0),
        ("valid_mask", "u8", 0),
        ("flags", "u8", 0),
        ("mode", "u8", 0),
        ("source", "u8", 0),
        ("queue_depth", "u16", 0),
        ("fault_mask", "u8", 0),
        ("phase", "u8", 0),
        ("position", "f32", J),
        ("velocity", "f32", J),
        ("torque", "f32", J),
        ("target", "f32", J),
        ("temperature", "u8", J),
        ("reserved", "u8", 0),
    ],
    "ControllerStatus": [
        ("uptime_us", "u64", 0),
        ("tick_count", "u32", 0),
        ("overrun_count", "u32", 0),
        ("max_tick_us", "u32", 0),
        ("host_rx_errors", "u32", 0),
        ("arm_mode", "u8", MAX_ARMS),
        ("source", "u8", 0),
        ("estop", "u8", 0),
        ("host_alive", "u8", 0),
        ("reserved", "u8", 0),
        ("pair_phase", "u8", MAX_PAIRS),
        ("follow_mode", "u8", MAX_PAIRS),
        ("sync_period_us", "u32", MAX_PAIRS),
        ("sync_latency_us", "u32", MAX_PAIRS),
        ("rel_offset", "f32", J * MAX_PAIRS),
    ],
    "Event": [
        ("event_ts_us", "u64", 0),
        ("arg0", "u32", 0),
        ("arg1", "u32", 0),
        ("code", "u16", 0),
        ("arm", "u8", 0),
        ("severity", "u8", 0),
    ],
    "TimeSyncReply": [
        ("host_t1_us", "u64", 0),
        ("mcu_t2_us", "u64", 0),
        ("mcu_t3_us", "u64", 0),
    ],
    "SilInject": [
        ("op", "u8", 0),
        ("arg", "u8", 0),
        ("arg2", "u8", 0),
        ("reserved", "u8", 0),
        ("values", "f32", J),
    ],
}

CTYPE = {"u8": "uint8_t", "u16": "uint16_t", "u32": "uint32_t", "u64": "uint64_t", "f32": "float"}
FMT = {"u8": "B", "u16": "H", "u32": "I", "u64": "Q", "f32": "f"}
SIZE = {"u8": 1, "u16": 2, "u32": 4, "u64": 8, "f32": 4}


def snake(name: str) -> str:
    out = []
    for i, ch in enumerate(name):
        if ch.isupper() and i:
            out.append("_")
        out.append(ch.lower())
    return "".join(out)


def struct_format(fields: list[Field]) -> str:
    return "<" + "".join(f"{count or ''}{FMT[kind]}" for _, kind, count in fields)


def struct_size(fields: list[Field]) -> int:
    return struct.calcsize(struct_format(fields))


def gen_c() -> str:
    out = [
        "/* Generated by tools/gen_protocol.py. Do not edit. */",
        "#ifndef RAC_LINK_GEN_H",
        "#define RAC_LINK_GEN_H",
        "",
        "#include <stdint.h>",
        "",
        f"#define RAC_PROTO_VERSION {PROTO_VERSION}",
        f"#define RAC_MAX_ARMS {MAX_ARMS}",
        f"#define RAC_MAX_PORTS {MAX_PORTS}",
        f"#define RAC_MAX_PAIRS {MAX_PAIRS}",
        f"#define RAC_JOINTS {JOINTS}",
        f"#define RAC_MAX_CHUNK_POINTS {MAX_CHUNK_POINTS}",
        "",
    ]
    for enum, (prefix, values) in ENUMS.items():
        out.append(f"/* {enum} */")
        for name, value, note in values:
            comment = f" /* {note} */" if note else ""
            out.append(f"#define {prefix}_{name} 0x{value:02X}{comment}")
        out.append("")
    for name, value, note in CAP_FLAGS:
        comment = f" /* {note} */" if note else ""
        out.append(f"#define RAC_CAP_{name} 0x{value:02X}u{comment}")
    for name, value, note in STATE_FLAGS:
        comment = f" /* {note} */" if note else ""
        out.append(f"#define RAC_STATE_{name} 0x{value:02X}u{comment}")
    out += ["", "#pragma pack(push, 1)", ""]

    def emit(c_name: str, fields: list[Field]) -> None:
        out.append("typedef struct {")
        for fname, kind, count in fields:
            suffix = f"[{count}]" if count else ""
            out.append(f"    {CTYPE[kind]} {fname}{suffix};")
        out.append(f"}} {c_name};")
        out.append(
            f"_Static_assert(sizeof({c_name}) == {struct_size(fields)}, " f'"{c_name} size");'
        )
        out.append("")

    emit("rac_header_t", HEADER)
    for name, fields in MESSAGES.items():
        emit(f"rac_{snake(name)}_t", fields)
    out += ["#pragma pack(pop)", "", "#endif /* RAC_LINK_GEN_H */", ""]
    return "\n".join(out)


def _py_default(kind: str, count: int) -> str:
    zero = "0.0" if kind == "f32" else "0"
    if count:
        return f"field(default_factory=lambda: [{zero}] * {count})"
    return zero


def _py_annotation(kind: str, count: int) -> str:
    base = "float" if kind == "f32" else "int"
    return f"list[{base}]" if count else base


def _py_message(name: str, fields: list[Field]) -> list[str]:
    fmt = struct_format(fields)
    out = ["@dataclass", f"class {name}:"]
    for fname, kind, count in fields:
        out.append(f"    {fname}: {_py_annotation(kind, count)} = {_py_default(kind, count)}")
    out += [
        "",
        f'    FORMAT: ClassVar[str] = "{fmt}"',
        f"    SIZE: ClassVar[int] = {struct_size(fields)}",
        "",
        "    def pack(self) -> bytes:",
        "        values: list = []",
    ]
    for fname, _, count in fields:
        if count:
            out.append(f"        values.extend(self.{fname})")
        else:
            out.append(f"        values.append(self.{fname})")
    out += [
        "        return struct.pack(self.FORMAT, *values)",
        "",
        "    @classmethod",
        f'    def unpack(cls, data: bytes | memoryview) -> {name}:',
        "        v = struct.unpack_from(cls.FORMAT, data)",
        "        return cls(",
    ]
    index = 0
    for fname, _, count in fields:
        if count:
            out.append(f"            {fname}=list(v[{index}:{index + count}]),")
            index += count
        else:
            out.append(f"            {fname}=v[{index}],")
            index += 1
    out += ["        )", "", ""]
    return out


def gen_py() -> str:
    out = [
        '"""Generated by tools/gen_protocol.py. Do not edit."""',
        "",
        "from __future__ import annotations",
        "",
        "import struct",
        "from dataclasses import dataclass, field",
        "from enum import IntEnum, IntFlag",
        "from typing import ClassVar",
        "",
        f"PROTO_VERSION = {PROTO_VERSION}",
        f"MAX_ARMS = {MAX_ARMS}",
        f"MAX_PORTS = {MAX_PORTS}",
        f"MAX_PAIRS = {MAX_PAIRS}",
        f"JOINTS = {JOINTS}",
        f"MAX_CHUNK_POINTS = {MAX_CHUNK_POINTS}",
        "",
        "",
    ]
    for enum, (_, values) in ENUMS.items():
        out.append(f"class {enum}(IntEnum):")
        for name, value, _note in values:
            out.append(f"    {name} = 0x{value:02X}")
        out += ["", ""]
    out.append("class Capability(IntFlag):")
    for name, value, _ in CAP_FLAGS:
        out.append(f"    {name} = 0x{value:02X}")
    out += ["", "", "class StateFlag(IntFlag):"]
    for name, value, _ in STATE_FLAGS:
        out.append(f"    {name} = 0x{value:02X}")
    out += ["", ""]
    out += _py_message("Header", HEADER)
    for name, fields in MESSAGES.items():
        out += _py_message(name, fields)
    out.append("MESSAGE_BY_TYPE: dict[int, type] = {")
    pairs = {
        "BIND_PORT": "BindPort",
        "SET_CALIBRATION": "SetCalibration",
        "SET_PAIRING": "SetPairing",
        "TELEOP_CONTROL": "TeleopControl",
        "SET_RATES": "SetRates",
        "SET_LIMITS": "SetLimits",
        "SET_MODE": "SetMode",
        "TARGET_CHUNK": "TargetChunk",
        "STOP": "Stop",
        "PAUSE_RESUME": "PauseResume",
        "FEEDFORWARD": "Feedforward",
        "TORQUE": "Torque",
        "ACK": "Ack",
        "DEVICE_INFO": "DeviceInfo",
        "PORT_STATUS": "PortStatus",
        "ARM_STATE": "ArmState",
        "CONTROLLER_STATUS": "ControllerStatus",
        "EVENT": "Event",
        "TIME_SYNC_REPLY": "TimeSyncReply",
        "SIL_INJECT": "SilInject",
    }
    for key, cls in pairs.items():
        out.append(f"    MsgType.{key}: {cls},")
    out += ["}", ""]
    return "\n".join(out)


@dataclass
class Generated:
    c: str
    py: str


def generate() -> Generated:
    return Generated(c=gen_c(), py=gen_py())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail when committed files are stale")
    args = parser.parse_args()
    gen = generate()
    pairs = [(C_OUT, gen.c), (PY_OUT, gen.py)]
    if args.check:
        stale = [str(p) for p, text in pairs if not p.exists() or p.read_text("utf-8") != text]
        if stale:
            raise SystemExit("stale generated files: " + ", ".join(stale))
        return
    for path, text in pairs:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
