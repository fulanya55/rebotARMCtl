"""RAC-Link framing: COBS with a CRC, matching firmware/core/rac_wire.c."""

from __future__ import annotations

from ._gen import PROTO_VERSION, Header

MAX_PAYLOAD = 640


def crc16(data: bytes | bytearray | memoryview) -> int:
    """CRC-16/CCITT-FALSE."""
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray(1)
    code_at = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_at] = code
            code = 1
            code_at = len(out)
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_at] = code
                code = 1
                code_at = len(out)
                out.append(0)
    out[code_at] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    """Decode one frame. Returns None when it is malformed."""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            return None
        end = i + code - 1
        if end > n:
            return None
        block = data[i:end]
        if 0 in block:
            return None
        out += block
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def build_frame(payload: bytes) -> bytes:
    """Wrap header and body bytes for the wire."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload of {len(payload)} bytes exceeds {MAX_PAYLOAD}")
    crc = crc16(payload)
    return cobs_encode(payload + bytes((crc & 0xFF, crc >> 8))) + b"\x00"


class Deframer:
    """Reassembles validated payloads from a byte stream.

    Survives joining mid-frame and corruption: anything up to the next delimiter
    is discarded and counted, and the stream resynchronizes on its own.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> list[bytes]:
        frames: list[bytes] = []
        start = 0
        while True:
            delimiter = data.find(0, start)
            if delimiter < 0:
                self._buffer += data[start:]
                if len(self._buffer) > 2 * MAX_PAYLOAD:
                    self._buffer.clear()
                    self.errors += 1
                return frames
            self._buffer += data[start:delimiter]
            start = delimiter + 1
            if not self._buffer:
                continue
            decoded = cobs_decode(bytes(self._buffer))
            self._buffer.clear()
            if decoded is None or len(decoded) < 2:
                self.errors += 1
                continue
            payload, wire_crc = decoded[:-2], decoded[-2] | (decoded[-1] << 8)
            if crc16(payload) != wire_crc:
                self.errors += 1
                continue
            frames.append(payload)


def make_header(msg_type: int, seq: int, generation: int, timestamp_us: int, arm_id: int = 0xFF) -> bytes:
    return Header(
        version=PROTO_VERSION,
        msg_type=int(msg_type),
        arm_id=int(arm_id),
        seq=seq & 0xFFFFFFFF,
        generation=generation & 0xFFFFFFFF,
        timestamp_us=timestamp_us,
    ).pack()
