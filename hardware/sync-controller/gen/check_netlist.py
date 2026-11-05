#!/usr/bin/env python3
"""Check KiCad's own reading of the schematic against design.py.

The sheets are written by a script, so a drawing mistake would silently change the
circuit. This exports the netlist through KiCad and compares every pin with the data
it was generated from. It also checks the circuit rules the design depends on.
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

from design import all_parts, build
from sexpr import find, find_all, parse

ROOT = Path(__file__).resolve().parents[1] / "kicad"
CLI = os.environ.get("KICAD_CLI", "D:/KiCad/bin/kicad-cli.exe")


def export_netlist() -> dict[str, dict[str, str]]:
    """{ref: {pin: net name as KiCad resolves it}}"""
    out = Path(tempfile.gettempdir()) / "rac_board.net"
    subprocess.run(
        [CLI, "sch", "export", "netlist", "--format", "kicadsexpr", "-o", str(out), str(ROOT / "sync-controller.kicad_sch")],
        check=True,
        capture_output=True,
    )
    tree = parse(out.read_text("utf-8"))
    pins: dict[str, dict[str, str]] = {}
    for net in find_all(find(tree, "nets"), "net"):
        name = find(net, "name")[1]
        for node in find_all(net, "node"):
            ref = find(node, "ref")[1]
            pin = find(node, "pin")[1]
            pins.setdefault(ref, {})[pin] = name
    return pins


def expected_name(sheet_title: str, net: str, global_nets: set[str]) -> str:
    if net in global_nets or net in ("GND", "+3V3", "+3.3VA", "+5V"):
        return net
    return f"/{sheet_title}/{net}"


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    from design import GLOBALS

    actual = export_netlist()
    errors = []
    total = 0
    for sheet in build():
        for part in sheet.parts:
            got = actual.get(part.ref, {})
            for pin, net in part.pins.items():
                total += 1
                want = expected_name(sheet.title, net, GLOBALS)
                if got.get(pin) != want:
                    errors.append(f"{part.ref}.{pin}: design says {want}, KiCad says {got.get(pin)}")

    # Which pins share a net in the isolated domains: nothing may bridge a barrier.
    by_net: dict[str, set[str]] = {}
    for ref, pins in actual.items():
        for pin, net in pins.items():
            by_net.setdefault(net, set()).add(f"{ref}.{pin}")
    for net in by_net:
        if "ISO_GND" in net and net.count("/") != 2:
            errors.append(f"isolated ground {net} is not local to one port sheet")
    isolated = [n for n in by_net if n.endswith("/ISO_GND")]
    if len(isolated) != 4:
        errors.append(f"expected 4 separate isolated grounds, found {isolated}")

    parts = all_parts()
    refs = [p.ref for p in parts]
    if len(refs) != len(set(refs)):
        errors.append("duplicate reference designators")

    for line in errors[:30]:
        print(line)
    print(f"{total} pins checked across {len(parts)} parts, {len(by_net)} nets, {len(errors)} mismatches")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
