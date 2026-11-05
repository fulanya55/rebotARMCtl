#!/usr/bin/env python3
"""Run KiCad DRC with schematic parity. Prints a summary and exits non-zero on real problems."""

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

# Accepted findings, with the reason. Anything else fails the check.
ACCEPTED = {
    # The boards' silkscreen is generated text placed by coordinates and has been read
    # by eye; a label crossing a pad edge costs nothing electrically.
    "silk_over_copper": "cosmetic",
    "silk_overlap": "cosmetic",
    "silk_edge_clearance": "cosmetic",
    "text_height": "cosmetic",
    "text_thickness": "cosmetic",
    # Library footprints as published; their own courtyards and pad geometry.
    "lib_footprint_issues": "the generated board does not resolve library tables",
    "lib_footprint_mismatch": "see above",
}


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    report = Path(tempfile.gettempdir()) / "rac_drc.json"
    subprocess.run(
        [CLI, "pcb", "drc", "--severity-all", "--schematic-parity", "--format", "json", "-o", str(report), str(ROOT / "sync-controller.kicad_pcb")],
        check=True,
        capture_output=True,
    )
    data = json.loads(report.read_text("utf-8"))
    groups = {"violations": data.get("violations", []), "unconnected": data.get("unconnected_items", []), "parity": data.get("schematic_parity", [])}
    bad = 0
    for group, items in groups.items():
        counts: collections.Counter = collections.Counter()
        examples: dict[str, str] = {}
        for item in items:
            key = (item["severity"], item["type"])
            counts[key] += 1
            described = " | ".join(i["description"][:62] for i in item.get("items", [])[:2])
            examples.setdefault(item["type"], described)
        print(f"== {group}: {len(items)}")
        for (severity, kind), count in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0])):
            accepted = kind in ACCEPTED
            if not accepted:
                bad += count
            print(f"  {count:4} {severity:7} {kind:32} {'(accepted)' if accepted else examples[kind]}")
    print(f"unexplained: {bad}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
