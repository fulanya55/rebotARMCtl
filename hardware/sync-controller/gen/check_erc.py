#!/usr/bin/env python3
"""Run KiCad ERC and print a summary. Exits non-zero on errors or unexplained warnings."""

from __future__ import annotations

import collections
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "kicad"
CLI = os.environ.get("KICAD_CLI", "D:/KiCad/bin/kicad-cli.exe")

# Warnings that are understood and accepted, with the reason.
ACCEPTED = {
    # KiCad's global libraries are not configured for a generated project on every
    # machine, and lib_symbols in each sheet carries the drawing either way.
    "lib_symbol_issues": "symbol libraries are embedded in the sheets",
    "lib_symbol_mismatch": "symbols are flattened copies of the library ones",
}


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    report = Path(tempfile.gettempdir()) / "rac_erc.json"
    subprocess.run(
        [CLI, "sch", "erc", "--severity-all", "--format", "json", "-o", str(report), str(ROOT / "sync-controller.kicad_sch")],
        check=True,
        capture_output=True,
    )
    data = json.loads(report.read_text("utf-8"))
    counts: collections.Counter = collections.Counter()
    bad = 0
    for sheet in data["sheets"]:
        for violation in sheet["violations"]:
            counts[(violation["severity"], violation["type"])] += 1
            if violation["type"] in ACCEPTED:
                continue
            bad += 1
            items = "; ".join(item["description"][:70] for item in violation["items"][:3])
            print(f"{violation['severity']:7} {violation['type']:28} {sheet['path'][:14]:14} {items}")
    print("---")
    for key, count in sorted(counts.items()):
        print(f"{count:4} {key[0]} {key[1]}" + ("  (accepted)" if key[1] in ACCEPTED else ""))
    print(f"unexplained: {bad}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
