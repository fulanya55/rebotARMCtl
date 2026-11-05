"""The Python and C sides of RAC-Link are generated from one definition, and agree on the wire."""

from __future__ import annotations

import importlib.util
import re
import struct
import sys
from pathlib import Path

import pytest
from rac_link import protocol
from rac_link.clock import ClockModel
from rac_link.wire import Deframer, build_frame, cobs_decode, cobs_encode, crc16

ROOT = Path(__file__).resolve().parents[1]


def load_generator():
    spec = importlib.util.spec_from_file_location("gen_protocol", ROOT / "tools" / "gen_protocol.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["gen_protocol"] = module
    spec.loader.exec_module(module)
    return module


def test_generated_files_are_fresh():
    gen = load_generator().generate()
    assert (ROOT / "firmware/core/rac_link_gen.h").read_text("utf-8") == gen.c
    assert (ROOT / "packages/rac-link/src/rac_link/_gen.py").read_text("utf-8") == gen.py


def test_c_struct_sizes_match_python():
    header = (ROOT / "firmware/core/rac_link_gen.h").read_text("utf-8")
    sizes = dict(re.findall(r"_Static_assert\(sizeof\((\w+)\) == (\d+)", header))
    assert int(sizes["rac_header_t"]) == protocol.Header.SIZE == 24
    assert int(sizes["rac_arm_state_t"]) == protocol.ArmState.SIZE
    assert int(sizes["rac_controller_status_t"]) == protocol.ControllerStatus.SIZE
    assert int(sizes["rac_ack_t"]) == protocol.Ack.SIZE


def test_every_message_round_trips():
    for cls in protocol.MESSAGE_BY_TYPE.values():
        original = cls()
        assert cls.unpack(original.pack()) == original
        assert len(original.pack()) == cls.SIZE


def test_crc_known_value():
    assert crc16(b"123456789") == 0x29B1


@pytest.mark.parametrize(
    "payload",
    [b"", b"\x00", b"\x00\x00\x00", b"\x01\x02", bytes(range(256)), b"\x55" * 254, b"\x55" * 255, b"\x55" * 600, b"\x00" * 300],
)
def test_cobs_round_trip_never_contains_a_delimiter(payload):
    encoded = cobs_encode(payload)
    assert b"\x00" not in encoded
    assert cobs_decode(encoded) == payload


def test_deframer_survives_corruption_and_mid_stream_join():
    good = build_frame(b"hello")
    bad = bytearray(build_frame(b"world"))
    bad[2] ^= 0x40
    other = build_frame(b"again")
    stream = good[3:] + good + bytes(bad) + other  # joined mid frame
    deframer = Deframer()
    frames = []
    # Split at an arbitrary place: a stream gives no promise about read boundaries.
    frames += deframer.feed(stream[:7])
    frames += deframer.feed(stream[7:])
    assert frames == [b"hello", b"again"]
    assert deframer.errors == 2  # the partial first frame and the corrupted one


def test_clock_model_recovers_offset_and_drift():
    model = ClockModel()
    offset = 5_000_000
    drift = 1 + 30e-6
    host = 0
    for step in range(64):
        host = step * 250_000
        # A varying one-way delay with a long tail, as on USB.
        delay = 150 + (step % 7) * 400
        t1 = host
        t2 = int(offset + drift * (t1 + 150))
        t3 = t2 + 20
        t4 = t1 + 150 + 20 + delay
        model.add_exchange(t1, t2, t3, t4)
    assert model.synced
    assert model.summary()["best_rtt_us"] == 300
    probe = host
    expected = offset + drift * probe
    assert abs(model.host_to_mcu(probe) - expected) < 300
    assert abs(model.mcu_to_host(model.host_to_mcu(probe)) - probe) < 1


def test_target_chunk_format():
    head = protocol.TargetChunk(exec_seq=7, dt_us=20000, point_count=2, joint_count=7).pack()
    exec_seq, dt_us, t_start, count, joints, _source = struct.unpack("<IIQHBB", head)
    assert (exec_seq, dt_us, t_start, count, joints) == (7, 20000, 0, 2, 7)
