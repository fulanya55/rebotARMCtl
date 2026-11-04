"""Byte transports to the controller: USB CDC on a board, TCP for the simulator.

URIs:
    serial://COM7          a specific port
    usb://                 the first attached controller, found by USB identity
    sil://127.0.0.1:5760   the software-in-the-loop controller
"""

from __future__ import annotations

import socket
from abc import ABC, abstractmethod
from urllib.parse import urlparse

# USB identity reported by the firmware. pid.codes test range until the board
# has its own allocation, so treat it as a development value.
USB_VID = 0x1209
USB_PID = 0x0001
USB_PRODUCT = "RAC Sync Controller"


class Transport(ABC):
    @abstractmethod
    def read(self, size: int = 4096) -> bytes:
        """Return available bytes, or b"" after a short timeout. Raises OSError when closed."""

    @abstractmethod
    def write(self, data: bytes) -> None: ...

    @abstractmethod
    def close(self) -> None: ...


class TcpTransport(Transport):
    def __init__(self, host: str, port: int, timeout: float = 2.0) -> None:
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(0.05)

    def read(self, size: int = 4096) -> bytes:
        try:
            data = self._sock.recv(size)
        except TimeoutError:
            return b""
        if not data:
            raise OSError("controller closed the connection")
        return data

    def write(self, data: bytes) -> None:
        self._sock.settimeout(0.5)
        try:
            self._sock.sendall(data)
        finally:
            self._sock.settimeout(0.05)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


class SerialTransport(Transport):
    def __init__(self, port: str) -> None:
        import serial

        # The baud rate is meaningless on USB CDC but pyserial requires one.
        self._serial = serial.Serial(port, baudrate=115200, timeout=0.05, write_timeout=0.5)
        self._serial.dtr = True

    def read(self, size: int = 4096) -> bytes:
        waiting = self._serial.in_waiting
        return self._serial.read(min(size, waiting) if waiting else 1)

    def write(self, data: bytes) -> None:
        self._serial.write(data)

    def close(self) -> None:
        try:
            self._serial.close()
        except OSError:
            pass


def list_candidates() -> list[dict]:
    """USB serial ports that could be a controller, best matches first."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    found = []
    for port in list_ports.comports():
        match = port.vid == USB_VID and port.pid == USB_PID
        found.append(
            {
                "uri": f"serial://{port.device}",
                "device": port.device,
                "description": port.description,
                "serial_number": port.serial_number,
                "is_controller": bool(match),
            }
        )
    found.sort(key=lambda c: (not c["is_controller"], c["device"]))
    return found


def open_transport(uri: str) -> Transport:
    parsed = urlparse(uri)
    if parsed.scheme == "sil":
        return TcpTransport(parsed.hostname or "127.0.0.1", parsed.port or 5760)
    if parsed.scheme == "serial":
        return SerialTransport(uri[len("serial://") :])
    if parsed.scheme == "usb":
        for candidate in list_candidates():
            if candidate["is_controller"]:
                return SerialTransport(candidate["device"])
        raise OSError("no RAC controller found on USB")
    raise ValueError(f"unknown transport URI: {uri}")
