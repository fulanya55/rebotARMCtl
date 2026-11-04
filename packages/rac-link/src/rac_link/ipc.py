"""Local IPC between the Controller Service and its clients.

Length-prefixed msgpack over loopback TCP. Loopback TCP rather than a Unix socket
because it behaves the same on Windows and Linux, which matters since the Lab runs
on both. The service binds 127.0.0.1 only and requires the session token it wrote
to its runtime directory, so another user on the machine cannot drive an arm.

    request   {"id": int, "method": str, "params": dict}
    response  {"id": int, "ok": bool, "result": any} or {"id", "ok": False, "error", "code"}
    push      {"topic": str, "data": any}
"""

from __future__ import annotations

import json
import logging
import os
import secrets
import socket
import struct
import threading
from collections import defaultdict
from collections.abc import Callable
from pathlib import Path
from typing import Any

import msgpack

log = logging.getLogger(__name__)

DEFAULT_PORT = 5761
MAX_MESSAGE = 32 * 1024 * 1024


def runtime_dir() -> Path:
    """Where the service leaves its endpoint file and persistent configuration."""
    root = os.environ.get("RAC_HOME")
    path = Path(root) if root else Path.home() / ".rac"
    path.mkdir(parents=True, exist_ok=True)
    return path


def endpoint_file() -> Path:
    return runtime_dir() / "service.json"


def write_endpoint(port: int) -> str:
    token = secrets.token_hex(16)
    path = endpoint_file()
    path.write_text(json.dumps({"port": port, "token": token, "pid": os.getpid()}), encoding="utf-8")
    try:
        path.chmod(0o600)
    except OSError:
        pass
    return token


def read_endpoint() -> dict:
    return json.loads(endpoint_file().read_text(encoding="utf-8"))


def send_message(sock: socket.socket, message: Any) -> None:
    payload = msgpack.packb(message, use_bin_type=True)
    sock.sendall(struct.pack("<I", len(payload)) + payload)


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("peer closed")
        chunks += chunk
    return bytes(chunks)


def recv_message(sock: socket.socket) -> Any:
    size = struct.unpack("<I", recv_exact(sock, 4))[0]
    if size > MAX_MESSAGE:
        raise ConnectionError(f"message of {size} bytes refused")
    return msgpack.unpackb(recv_exact(sock, size), raw=False)


class ServiceError(RuntimeError):
    """The service refused a request. ``code`` follows domain.condition."""

    def __init__(self, message: str, code: str = "service.error") -> None:
        super().__init__(message)
        self.code = code


class IpcClient:
    """Thread-safe connection to the service, with topic subscriptions."""

    def __init__(self, host: str = "127.0.0.1", port: int | None = None, token: str | None = None, timeout: float = 5.0):
        if port is None or token is None:
            endpoint = read_endpoint()
            port = endpoint["port"] if port is None else port
            token = endpoint["token"] if token is None else token
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(None)
        self._timeout = timeout
        self._lock = threading.Lock()
        self._next_id = 1
        self._pending: dict[int, tuple[threading.Event, list]] = {}
        self._handlers: dict[str, list[Callable[[Any], None]]] = defaultdict(list)
        self._closed = False
        self._reader = threading.Thread(target=self._read_loop, name="rac-ipc-rx", daemon=True)
        self._reader.start()
        self.call("auth", token=token)

    def call(self, method: str, timeout: float | None = None, **params: Any) -> Any:
        event = threading.Event()
        slot: list = []
        with self._lock:
            request_id = self._next_id
            self._next_id += 1
            self._pending[request_id] = (event, slot)
            send_message(self._sock, {"id": request_id, "method": method, "params": params})
        if not event.wait(self._timeout if timeout is None else timeout):
            with self._lock:
                self._pending.pop(request_id, None)
            raise TimeoutError(f"no response to {method}")
        if not slot:
            raise ConnectionError("service connection lost")
        response = slot[0]
        if not response.get("ok"):
            raise ServiceError(response.get("error", "request failed"), response.get("code", "service.error"))
        return response.get("result")

    def subscribe(self, topic: str, handler: Callable[[Any], None]) -> None:
        with self._lock:
            first = not self._handlers[topic]
            self._handlers[topic].append(handler)
        if first:
            self.call("subscribe", topic=topic)

    def close(self) -> None:
        self._closed = True
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()

    def _read_loop(self) -> None:
        try:
            while True:
                message = recv_message(self._sock)
                if "topic" in message:
                    for handler in list(self._handlers.get(message["topic"], ())):
                        try:
                            handler(message["data"])
                        except Exception:
                            log.exception("subscriber for %s failed", message["topic"])
                else:
                    with self._lock:
                        pending = self._pending.pop(message.get("id"), None)
                    if pending:
                        pending[1].append(message)
                        pending[0].set()
        except (ConnectionError, OSError, msgpack.exceptions.UnpackException, ValueError) as exc:
            if not self._closed:
                log.info("service connection closed: %s", exc)
        finally:
            with self._lock:
                for event, _ in self._pending.values():
                    event.set()
                self._pending.clear()
