#!/usr/bin/env python3
"""Write the BOM from design.py. A blank LCSC number means it was not confirmed when written."""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

from design import all_parts

OUT = Path(__file__).resolve().parents[1] / "docs" / "bom.csv"


def main() -> None:
    groups: dict[tuple, list[str]] = defaultdict(list)
    for part in all_parts():
        key = (part.value, part.footprint.split(":")[-1], part.mpn, part.lcsc, part.dnp)
        groups[key].append(part.ref)

    def natural(ref: str) -> tuple:
        letters = ref.rstrip("0123456789")
        return letters, int(ref[len(letters) :] or 0)

    rows = sorted(groups.items(), key=lambda kv: natural(sorted(kv[1], key=natural)[0]))
    with OUT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["Qty", "References", "Value", "Footprint", "MPN", "LCSC", "Fit"])
        for (value, footprint, mpn, lcsc, dnp), refs in rows:
            refs = sorted(refs, key=natural)
            writer.writerow([len(refs), " ".join(refs), value, footprint, mpn, lcsc, "no" if dnp else "yes"])
    print(f"{len(rows)} lines, {sum(len(r) for r in groups.values())} parts: {OUT}")


if __name__ == "__main__":
    main()
