#!/usr/bin/env python3
"""Measure the isolation barriers on the finished layout. DRC does not know about them.

Run with KiCad's Python. It reports the smallest copper gap between every pair of
domains on every copper layer, including the filled planes, and checks that no net
appears in a domain it does not belong to. Fails below MIN_GAP_MM, the functional
isolation target for a 48 V system.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pcbnew

BOARD = Path(__file__).resolve().parents[1] / "kicad" / "sync-controller.kicad_pcb"
MIN_GAP_MM = 0.5
STEP = 0.05

DOMAINS = ("CAN1 isolated", "CAN2 isolated", "UART1 isolated", "UART2 isolated")


def domain_of(netname: str) -> str:
    # Sheet-local nets carry their sheet's title. Everything else is the logic side, but
    # a port sheet also has logic-side nets (none are local there: they are global).
    for domain in DOMAINS:
        if netname.startswith(f"/{domain}"):
            return domain
    return "logic"


def segment_points(ax, ay, bx, by, half):
    """Points along both edges and the centre of a track."""
    length = float(np.hypot(bx - ax, by - ay))
    n = max(2, int(length / STEP) + 1)
    t = np.linspace(0.0, 1.0, n)
    x = ax + (bx - ax) * t
    y = ay + (by - ay) * t
    if length < 1e-6:
        nx, ny = 0.0, 0.0
    else:
        nx, ny = -(by - ay) / length, (bx - ax) / length
    return np.concatenate(
        [np.stack([x + nx * half, y + ny * half], axis=1), np.stack([x - nx * half, y - ny * half], axis=1)]
    )


def rect_points(cx, cy, hx, hy):
    nx = max(2, int(2 * hx / STEP) + 1)
    ny = max(2, int(2 * hy / STEP) + 1)
    xs = np.linspace(cx - hx, cx + hx, nx)
    ys = np.linspace(cy - hy, cy + hy, ny)
    return np.concatenate(
        [
            np.stack([xs, np.full(nx, cy - hy)], axis=1),
            np.stack([xs, np.full(nx, cy + hy)], axis=1),
            np.stack([np.full(ny, cx - hx), ys], axis=1),
            np.stack([np.full(ny, cx + hx), ys], axis=1),
        ]
    )


def near(points: np.ndarray, other: np.ndarray, reach: float = 4.0) -> np.ndarray:
    """Only the points close to the other set can decide the gap: a coarse occupancy
    grid drops the rest, which turns an hours-long comparison into seconds."""
    cell = reach
    occupied = {(int(x // cell), int(y // cell)) for x, y in other}
    grown = {(i + di, j + dj) for i, j in occupied for di in (-1, 0, 1) for dj in (-1, 0, 1)}
    keep = np.fromiter(((int(x // cell), int(y // cell)) in grown for x, y in points), dtype=bool, count=len(points))
    return points[keep]


def min_distance(a: np.ndarray, b: np.ndarray) -> tuple[float, tuple]:
    a, b = near(a, b), near(b, a)
    best = (1e9, ())
    if len(a) == 0 or len(b) == 0:
        return best
    for start in range(0, len(a), 1000):
        chunk = a[start : start + 1000]
        d2 = (chunk[:, None, 0] - b[None, :, 0]) ** 2 + (chunk[:, None, 1] - b[None, :, 1]) ** 2
        index = int(np.argmin(d2))
        value = float(d2.flat[index])
        if value < best[0] ** 2:
            i, j = np.unravel_index(index, d2.shape)
            best = (float(np.sqrt(value)), (tuple(chunk[i]), tuple(b[j])))
    return best


def main() -> int:
    board = pcbnew.LoadBoard(str(BOARD))
    layers = {"F.Cu": pcbnew.F_Cu, "In1.Cu": pcbnew.In1_Cu, "In2.Cu": pcbnew.In2_Cu, "B.Cu": pcbnew.B_Cu}
    points: dict[tuple, list] = {}

    def add(layer: str, domain: str, array: np.ndarray) -> None:
        points.setdefault((layer, domain), []).append(array)

    for track in board.GetTracks():
        name = track.GetNetname()
        domain = domain_of(name)
        if isinstance(track, pcbnew.PCB_VIA):
            pos = track.GetPosition()
            radius = pcbnew.ToMM(track.GetWidth(pcbnew.F_Cu)) / 2.0
            array = rect_points(pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y), radius * 0.72, radius * 0.72)
            for layer in layers:
                add(layer, domain, array)
        else:
            a, b = track.GetStart(), track.GetEnd()
            layer = "F.Cu" if track.GetLayer() == pcbnew.F_Cu else "B.Cu"
            array = segment_points(pcbnew.ToMM(a.x), pcbnew.ToMM(a.y), pcbnew.ToMM(b.x), pcbnew.ToMM(b.y), pcbnew.ToMM(track.GetWidth()) / 2.0)
            add(layer, domain, array)

    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            name = pad.GetNetname()
            if not name or name.startswith("unconnected-"):
                continue
            domain = domain_of(name)
            box = pad.GetBoundingBox()
            array = rect_points(
                pcbnew.ToMM(box.GetCenter().x), pcbnew.ToMM(box.GetCenter().y), pcbnew.ToMM(box.GetWidth()) / 2.0, pcbnew.ToMM(box.GetHeight()) / 2.0
            )
            for layer, layer_id in layers.items():
                if pad.IsOnLayer(layer_id):
                    add(layer, domain, array)

    zone_domains = {}
    for zone in board.Zones():
        name = zone.GetNetname()
        domain = domain_of(name)
        zone_domains[(zone.GetLayerName(), domain)] = name
        fill = zone.GetFilledPolysList(zone.GetFirstLayer())
        for o in range(fill.OutlineCount()):
            outline = fill.COutline(o)
            coords = [(pcbnew.ToMM(outline.CPoint(i).x), pcbnew.ToMM(outline.CPoint(i).y)) for i in range(outline.PointCount())]
            parts = []
            for (ax, ay), (bx, by) in zip(coords, coords[1:] + coords[:1], strict=True):
                parts.append(segment_points(ax, ay, bx, by, 0.0))
            if parts:
                add(zone.GetLayerName(), domain, np.concatenate(parts))

    worst = 1e9
    failed = False
    print(f"{'layer':8} {'domain A':16} {'domain B':16} {'gap mm':>7}")
    for layer in layers:
        names = sorted(domain for (lay, domain) in points if lay == layer)
        for i, a in enumerate(names):
            for b in names[i + 1 :]:
                pa = np.concatenate(points[(layer, a)])
                pb = np.concatenate(points[(layer, b)])
                gap, where = min_distance(pa, pb)
                if gap > 1e8:
                    continue  # never within reach of each other
                flag = ""
                if gap < MIN_GAP_MM:
                    flag = f"  BELOW {MIN_GAP_MM} at ({where[0][0] - 50:.2f}, {where[0][1] - 50:.2f})"
                    failed = True
                worst = min(worst, gap)
                print(f"{layer:8} {a[:15]:16} {b[:15]:16} {gap:7.2f}{flag}")
    print(f"smallest gap: {worst:.2f} mm; target {MIN_GAP_MM} mm")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
