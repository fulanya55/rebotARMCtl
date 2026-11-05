"""End-to-end harness: the real firmware core behind the real service, over real sockets."""

from __future__ import annotations

import atexit
import os
import shutil
import tempfile

# Read when the Lab is imported, so it has to be set before any test imports it.
# Tests never touch the operator's robot records or datasets.
_LAB_HOME = tempfile.mkdtemp(prefix="rac-lab-home-")
os.environ["MAKERMODSLAB_HOME"] = _LAB_HOME
os.environ["HF_LEROBOT_HOME"] = os.path.join(_LAB_HOME, "lerobot")
os.environ.setdefault("MAKERMODSLAB_OUTPUT_ROOT", _LAB_HOME)
atexit.register(shutil.rmtree, _LAB_HOME, ignore_errors=True)

import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIL = ROOT / "firmware" / "build" / ("rac_sil.exe" if os.name == "nt" else "rac_sil")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for_port(port: int, timeout: float = 15.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.02)
    raise TimeoutError(f"nothing listening on {port}")


@pytest.fixture(scope="session")
def sil_binary() -> Path:
    # Rebuilds on source changes, so a test can never pass against a stale controller.
    sources = list((ROOT / "firmware").rglob("*.[ch]"))
    if not SIL.exists() or max(p.stat().st_mtime for p in sources if "build" not in p.parts) > SIL.stat().st_mtime:
        subprocess.run([sys.executable, str(ROOT / "tools" / "build_firmware.py"), "sil"], check=True, cwd=ROOT)
    return SIL


class Rig:
    """A simulated controller, a service in this process, and a client."""

    def __init__(self, sil_binary: Path, models: str, home: Path) -> None:
        from rac_link.client import ControllerClient
        from rac_service.server import Server

        os.environ["RAC_HOME"] = str(home)
        self.sil_port = free_port()
        self.process = subprocess.Popen(
            [str(sil_binary), "--port", str(self.sil_port), "--models", models],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.uri = f"sil://127.0.0.1:{self.sil_port}"
        wait_for_port(self.sil_port)
        self.server = Server(port=0)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.client = ControllerClient(port=self.server.port, token=self.server.token)
        self.events: list[dict] = []
        self.client.on_event(self.events.append)
        self.samples: list[dict] = []
        self.keep_samples = False
        self.client.on_state(lambda batch: self.samples.extend(batch) if self.keep_samples else None)

    def wait(self, predicate, timeout: float = 10.0, what: str = "condition"):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = predicate()
            if last:
                return last
            time.sleep(0.01)
        raise TimeoutError(f"timed out waiting for {what}; last value: {last!r}")

    def status(self) -> dict:
        return self.client.status()

    def arm(self, name: str) -> dict:
        return next(a for a in self.status()["arms"] if a["arm"] == name)

    def event_kinds(self) -> list[str]:
        return [e["kind"] for e in self.events]

    def wait_valid(self, *names: str) -> None:
        def ready():
            status = self.status()
            arms = {a["arm"]: a for a in status["arms"]}
            return all(arms.get(n, {}).get("valid_mask") == 0x7F for n in names)

        self.wait(ready, what=f"feedback from {names}")

    def close(self) -> None:
        self.client.close()
        self.server.close()
        self.process.terminate()
        self.process.wait(timeout=5)


@pytest.fixture
def make_rig(sil_binary, tmp_path):
    rigs = []

    def build(models: str = "rs,rs,hd,hd") -> Rig:
        rig = Rig(sil_binary, models, tmp_path / f"rig{len(rigs)}")
        rigs.append(rig)
        return rig

    yield build
    for rig in rigs:
        rig.close()


@pytest.fixture
def rig(make_rig):
    return make_rig()


@pytest.fixture
def bound_rig(rig):
    """Left pair bound: a Star HD leader driving an RS follower."""
    rig.client.connect(rig.uri)
    rig.client.bind(
        [
            {"port": 0, "arm": "left_follower", "model": "rebot_b601_rs", "serial": "rs-l"},
            {"port": 2, "arm": "left_leader", "model": "star_102_hd", "serial": "hd-l"},
        ]
    )
    rig.wait_valid("left_follower", "left_leader")
    return rig
