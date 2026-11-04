"""One command to run the platform.

    uv run rac up --sim      simulated controller, service and web app
    uv run rac up            real controller on USB, service and web app
    uv run rac service       only the Controller Service
    uv run rac sil           only the simulated controller

Everything starts as a child process of this command and stops with it.
"""

from __future__ import annotations

import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path


def repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "firmware").is_dir() and (parent / "apps").is_dir():
            return parent
    raise SystemExit("run this from a checkout of the repository")


def sil_path(root: Path) -> Path:
    return root / "firmware" / "build" / ("rac_sil.exe" if os.name == "nt" else "rac_sil")


def ensure_sil(root: Path) -> Path:
    path = sil_path(root)
    sources = [p for p in (root / "firmware").rglob("*.[ch]") if "build" not in p.parts and "third_party" not in p.parts]
    if not path.exists() or max(p.stat().st_mtime for p in sources) > path.stat().st_mtime:
        print("building the simulated controller...", flush=True)
        subprocess.run([sys.executable, str(root / "tools" / "build_firmware.py"), "sil"], check=True, cwd=root)
    return path


def wait_port(port: int, timeout: float = 20.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return True
        except OSError:
            time.sleep(0.05)
    return False


def main() -> None:
    parser = argparse.ArgumentParser(prog="rac", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    up = sub.add_parser("up", help="run the whole platform")
    up.add_argument("--sim", action="store_true", help="use the simulated controller instead of USB")
    up.add_argument("--models", default="rs,rs,hd,hd", help="simulated arms on CAN1,CAN2,UART1,UART2: rs, fl, hd, ld or none")
    up.add_argument("--uri", default=None, help="controller address, default usb:// (or the simulator with --sim)")
    up.add_argument("--lab-port", type=int, default=8000)
    up.add_argument("--sil-port", type=int, default=5760)

    service = sub.add_parser("service", help="only the Controller Service")
    service.add_argument("--uri", default=None)

    sil = sub.add_parser("sil", help="only the simulated controller")
    sil.add_argument("--models", default="rs,rs,hd,hd")
    sil.add_argument("--port", type=int, default=5760)

    args = parser.parse_args()
    root = repo_root()

    if args.command == "sil":
        path = ensure_sil(root)
        raise SystemExit(subprocess.run([str(path), "--port", str(args.port), "--models", args.models]).returncode)

    if args.command == "service":
        command = [sys.executable, "-m", "rac_service.server"]
        if args.uri:
            command += ["--uri", args.uri]
        raise SystemExit(subprocess.run(command).returncode)

    children: list[subprocess.Popen] = []

    def stop(*_) -> None:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
        for child in reversed(children):
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    uri = args.uri
    if args.sim:
        path = ensure_sil(root)
        children.append(subprocess.Popen([str(path), "--port", str(args.sil_port), "--models", args.models]))
        if not wait_port(args.sil_port):
            stop()
        uri = uri or f"sil://127.0.0.1:{args.sil_port}"

    command = [sys.executable, "-m", "rac_service.server"]
    if uri:
        command += ["--uri", uri]
    children.append(subprocess.Popen(command))

    dist = root / "apps" / "lab" / "frontend" / "dist"
    if not dist.exists():
        print("The web page is not built yet. Run: cd apps/lab/frontend && npm ci && npm run build", flush=True)
    children.append(
        subprocess.Popen(
            [sys.executable, "-m", "uvicorn", "makermodslab.server:app", "--host", "127.0.0.1", "--port", str(args.lab_port)],
            cwd=root / "apps" / "lab",
        )
    )
    if wait_port(args.lab_port, 90):
        print(f"\nOpen http://127.0.0.1:{args.lab_port}/controller\n", flush=True)

    while True:
        for child in children:
            if child.poll() is not None:
                print(f"a child process exited with {child.returncode}; stopping", flush=True)
                stop()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
