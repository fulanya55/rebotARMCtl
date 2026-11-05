#!/usr/bin/env python3
"""Regenerate the whole board from design.py and check it.

    uv run python hardware/sync-controller/gen/make_all.py [--skip-route]

Schematic, ERC, netlist comparison, layout and routing, DRC, isolation measurement,
then manufacturing files in ../fab. Stops at the first failure. Routing takes a few
minutes; --skip-route keeps the existing layout.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

GEN = Path(__file__).resolve().parent
ROOT = GEN.parent
KICAD = ROOT / "kicad"
FAB = ROOT / "fab"
DOCS = ROOT / "docs"
CLI = os.environ.get("KICAD_CLI", "D:/KiCad/bin/kicad-cli.exe")
KICAD_PY = os.environ.get("KICAD_PYTHON", "D:/KiCad/bin/python.exe")
BOARD = KICAD / "sync-controller.kicad_pcb"


def step(title: str, command: list[str], cwd: Path = GEN) -> None:
    print(f"== {title}", flush=True)
    result = subprocess.run(command, cwd=cwd, text=True, capture_output=True, encoding="utf-8", errors="replace")
    tail = (result.stdout + result.stderr).strip().splitlines()[-4:]
    for line in tail:
        print(f"   {line[:150]}")
    if result.returncode != 0:
        raise SystemExit(f"{title} failed")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--skip-route", action="store_true")
    args = parser.parse_args()

    step("schematic", [sys.executable, "gen_sch.py"])
    step("bill of materials", [sys.executable, "gen_bom.py"])
    step("electrical rules", [sys.executable, "check_erc.py"])
    step("netlist against the design data", [sys.executable, "check_netlist.py"])
    if not args.skip_route:
        step("layout and routing", [KICAD_PY, "gen_pcb.py"])
    step("design rules and schematic parity", [sys.executable, "check_drc.py"])
    step("isolation barriers", [KICAD_PY, "check_isolation.py"])

    if FAB.exists():
        shutil.rmtree(FAB)
    (FAB / "gerber").mkdir(parents=True)
    layers = "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
    step("gerbers", [CLI, "pcb", "export", "gerbers", "-l", layers, "--no-protel-ext", "-o", str(FAB / "gerber"), str(BOARD)])
    step("drill files", [CLI, "pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th", "-o", str(FAB / "gerber") + "/", str(BOARD)])
    step("placement", [CLI, "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--exclude-dnp", "-o", str(FAB / "positions.csv"), str(BOARD)])
    shutil.make_archive(str(FAB / "sync-controller-gerbers"), "zip", FAB / "gerber")
    step("schematic PDF", [CLI, "sch", "export", "pdf", "-o", str(DOCS / "schematic.pdf"), str(KICAD / "sync-controller.kicad_sch")])
    step("board render", [CLI, "pcb", "render", "--side", "top", "--width", "1600", "--height", "1250", "--quality", "basic", "-o", str(DOCS / "board_top.png"), str(BOARD)])
    print("all checks passed")


if __name__ == "__main__":
    main()
