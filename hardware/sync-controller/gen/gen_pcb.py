#!/usr/bin/env python3
"""Lay out the board: place, fan out planes, route, pour, check.

Run with KiCad's own Python, which has pcbnew and numpy:

    D:/KiCad/bin/python.exe hardware/sync-controller/gen/gen_pcb.py

Stack: F.Cu signals, In1.Cu ground, In2.Cu +3V3, B.Cu signals. Each isolated port has
its own ground area on In1.Cu, and nothing on any layer crosses a barrier except the
isolators and DC/DC converters themselves: a net's routing is confined to its domain
by construction, not just checked afterwards.

The router is a grid A* written here so the layout needs nothing beyond KiCad. It
produces a working first layout that passes DRC. A person should still review it, and
the critical areas (crystal, USB, MCU decoupling) deserve a look by eye.
"""

from __future__ import annotations

import heapq
import json
import math
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

import numpy as np
import pcbnew

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from design import GLOBALS, POWER_NETS, PROJECT, build  # noqa: E402
from sexpr import find, find_all, parse  # noqa: E402

KICAD = HERE.parent / "kicad"
SHARE = Path("D:/KiCad/share/kicad/footprints")
CLI = "D:/KiCad/bin/kicad-cli.exe"

W, H = 116.0, 90.0  # board size
OX, OY = 50.0, 50.0  # board origin on the KiCad page
G = 0.25  # routing grid
BARRIER_Y = 38.1
BARRIER_HALF = 0.5
COLUMNS = {"port_can1": 16.0, "port_can2": 44.0, "port_uart1": 72.0, "port_uart2": 100.0}
COLUMN_HALF = 14.0

CLEARANCE = 0.15
TRACK = 0.2
VIA_D = 0.6
VIA_DRILL = 0.3
MARGIN = 0.03  # guard on the grid's own rounding
HOLE_TO_HOLE = 0.25
EDGE = 0.4

NS = uuid.UUID("7a1c2f3e-5b6d-4f70-8a91-0c2d3e4f5a6b")


def uid(*parts: object) -> str:
    return str(uuid.uuid5(NS, "/".join(str(p) for p in parts)))


def read_netlist():
    """Pad nets and symbol fields as KiCad itself resolves the schematic."""
    out = Path(tempfile.gettempdir()) / "rac_board_pcb.net"
    subprocess.run(
        [CLI, "sch", "export", "netlist", "--format", "kicadsexpr", "-o", str(out), str(KICAD / f"{PROJECT}.kicad_sch")],
        check=True,
        capture_output=True,
    )
    tree = parse(out.read_text("utf-8"))
    pins: dict[str, dict[str, str]] = {}
    for net in find_all(find(tree, "nets"), "net"):
        name = find(net, "name")[1]
        for node in find_all(net, "node"):
            pins.setdefault(find(node, "ref")[1], {})[find(node, "pin")[1]] = name
    fields: dict[str, dict[str, str]] = {}
    for comp in find_all(find(tree, "components"), "comp"):
        ref = find(comp, "ref")[1]
        values = {}
        block = find(comp, "fields")
        if block is not None:
            for field in find_all(block, "field"):
                name = find(field, "name")[1]
                values[name] = field[2] if len(field) > 2 and isinstance(field[2], str) else ""
        datasheet = find(comp, "datasheet")
        if datasheet is not None and len(datasheet) > 1:
            values["Datasheet"] = datasheet[1]
        description = find(comp, "description")
        if description is not None and len(description) > 1:
            values["Description"] = description[1]
        fields[ref] = values
    return pins, fields


def mm(v: float) -> int:
    return pcbnew.FromMM(v)


def vec(x: float, y: float):
    return pcbnew.VECTOR2I(mm(x + OX), mm(y + OY))


# ---------------------------------------------------------------- placement


def placements(sheets) -> dict[str, tuple[float, float, float]]:
    """{ref: (x, y, rotation)} in board coordinates."""
    out: dict[str, tuple[float, float, float]] = {}
    by = {s.name: s.parts for s in sheets}

    def put(sheet: str, index: int, x: float, y: float, rot: float = 0.0) -> None:
        out[by[sheet][index].ref] = (x, y, rot)

    # USB and power, on the right. The connector's mouth faces the board edge.
    p = "power"
    put(p, 0, 111.25, 56.0, 90)
    put(p, 1, 103.0, 61.5)  # CC1
    put(p, 2, 103.0, 50.0)  # CC2
    put(p, 3, 106.0, 66.5)  # shield R
    put(p, 4, 106.0, 69.0)  # shield C
    put(p, 5, 100.0, 56.0)  # USBLC6
    put(p, 6, 103.0, 45.5)  # polyfuse
    put(p, 7, 96.5, 45.5)  # ferrite
    put(p, 8, 91.5, 45.5)  # 10uF
    put(p, 9, 91.5, 48.5)  # 100nF
    put(p, 10, 82.0, 47.0)  # LDO
    put(p, 11, 86.5, 45.5)  # LDO in
    put(p, 12, 82.0, 52.0)  # LDO out 4.7uF
    put(p, 13, 86.5, 49.5)  # LDO out 100nF
    put(p, 14, 66.0, 73.5)  # analog ferrite
    put(p, 15, 66.0, 76.5)
    put(p, 16, 69.5, 73.5)

    # MCU in the middle, decoupling at the pins.
    m = "mcu"
    cx, cy = 58.0, 64.0
    put(m, 0, cx, cy)
    put(m, 1, 49.5, 68.75)  # VDD pin 16
    put(m, 2, 62.25, 72.5, 90)  # VDD pin 32
    put(m, 3, 67.0, 59.5)  # VDD pin 48
    put(m, 4, 54.25, 55.0, 90)  # VDD pin 64
    put(m, 5, 67.0, 56.5)  # bulk
    put(m, 6, 49.5, 58.75)  # VBAT
    put(m, 7, 59.75, 72.5, 90)  # VDDA
    put(m, 8, 47.5, 63.0)  # crystal
    put(m, 9, 43.5, 60.5)
    put(m, 10, 43.5, 65.5)
    put(m, 11, 51.5, 54.5, 90)  # BOOT0 pull-down
    put(m, 12, 44.0, 79.0)  # BOOT key
    put(m, 13, 49.5, 66.25)  # NRST cap
    put(m, 14, 56.0, 79.0)  # RESET key
    put(m, 15, 46.0, 49.0, 90)  # SWD
    x = 74.0
    for i in range(6):
        put(m, 16 + 2 * i, x, 80.0)  # resistor
        put(m, 17 + 2 * i, x, 84.5)  # LED
        x += 6.0

    # Safety block on the left, wires leaving through the left edge.
    s = "safety"
    put(s, 0, 9.5, 48.0, 270)  # ESTOP terminal
    put(s, 1, 14.5, 45.5)  # TVS
    put(s, 2, 18.5, 45.5)  # TVS
    put(s, 3, 19.0, 50.0)  # relay
    put(s, 4, 33.0, 50.5)  # flyback
    put(s, 5, 33.0, 54.0)  # relay LED R
    put(s, 6, 33.0, 57.0)  # relay LED
    put(s, 7, 33.0, 63.0)  # MOSFET
    put(s, 8, 38.0, 63.0)  # gate R
    put(s, 9, 38.0, 66.0)  # gate pull-down
    put(s, 10, 19.0, 79.0)  # STOP key
    put(s, 11, 38.0, 60.0)  # stop sense R
    put(s, 12, 9.5, 66.0, 270)  # relay terminal
    put(s, 13, 42.0, 45.5)  # sense 1 divider
    put(s, 14, 42.0, 48.5)
    put(s, 15, 42.0, 51.5)
    put(s, 16, 37.5, 45.5)  # sense 2
    put(s, 17, 37.5, 48.5)
    put(s, 18, 37.5, 51.5)
    put(s, 19, 30.0, 85.0, 90)  # sync header
    put(s, 20, 36.0, 76.0)
    put(s, 21, 36.0, 79.0)
    put(s, 22, 36.0, 82.0)

    # Isolated ports along the top edge. Isolators and converters straddle the barrier.
    def common_power(name: str, c: float) -> None:
        put(name, 0, c - 8.0, 41.91, 180)  # DC/DC: pins 1,2 south, 3,4 north
        put(name, 1, c - 4.0, 41.25, 90)  # input cap
        put(name, 2, c - 4.5, 35.0, 90)  # 9 V cap
        put(name, 3, c - 4.5, 31.0, 90)  # 2.2k minimum load
        put(name, 4, c - 2.0, 26.0)  # 5 V regulator
        put(name, 5, c + 2.5, 26.0, 90)  # its output cap

    for name in ("port_can1", "port_can2"):
        c = COLUMNS[name]
        common_power(name, c)
        put(name, 6, c + 5.0, BARRIER_Y, 90)  # ISO1044
        put(name, 7, c + 5.0, 43.0)  # logic side 100nF
        put(name, 8, c + 5.0, 33.0)  # bus side 100nF
        put(name, 9, c + 9.5, 26.0, 90)  # 120R
        put(name, 10, c + 10.0, 16.5)  # termination jumper
        put(name, 11, c + 4.0, 17.0)  # TVS, not fitted
        put(name, 12, c + 3.5, 9.5, 180)  # bus terminal, mouth north

    for name in ("port_uart1", "port_uart2"):
        c = COLUMNS[name]
        common_power(name, c)
        put(name, 6, c - 2.0, 19.0)  # 3.3 V regulator
        put(name, 7, c + 2.5, 19.0, 90)
        put(name, 8, c + 5.0, BARRIER_Y, 90)  # ADuM1201
        put(name, 9, c + 5.0, 43.0)
        put(name, 10, c + 5.0, 33.0)
        put(name, 11, c + 10.0, 30.0)  # open drain driver
        put(name, 12, c + 10.0, 21.0)  # receive buffer
        put(name, 13, c + 10.0, 26.5)
        put(name, 14, c + 10.0, 17.5)
        put(name, 15, c + 2.5, 13.0)  # pull-up
        put(name, 16, c - 2.0, 13.0)  # 3.3 V option
        put(name, 17, c - 2.0, 10.5)  # 5 V option, not fitted
        put(name, 18, c + 6.5, 10.0)  # series R
        put(name, 19, c + 10.5, 10.0)  # TVS
        put(name, 20, c - 2.0, 4.5)  # servo connector
    return out


# ---------------------------------------------------------------- net names


def full_net_name(sheet, net: str) -> str:
    if net in GLOBALS or net in POWER_NETS:
        return net
    return f"/{sheet.title}/{net}"


def domain_of(sheet_name: str, net: str) -> str:
    """Which side of the isolation barriers a net lives on."""
    if sheet_name in COLUMNS:
        if net.startswith("ISO_") or net in ("CANH", "CANL", "TERM", "TXI", "RXI", "BUS", "PULLUP", "DATA"):
            return sheet_name
    return "logic"


def track_width(name: str) -> float:
    short = name.rsplit("/", 1)[-1]
    if short in ("+5V", "VBUS_USB", "VBUS_F"):
        return 0.5
    if short in ("ISO_9V", "ISO_5V", "ISO_3V3"):
        return 0.35
    if short.startswith("K_") or short in ("CHAIN1", "COIL_LOW"):
        return 0.35
    return TRACK


PLANE_LOGIC = {"GND", "+3V3"}


def is_plane(name: str) -> bool:
    return name in PLANE_LOGIC or name.endswith("/ISO_GND")


# -------------------------------------------------------------------- router


class Grid:
    """Who owns each cell, per track width class and layer. 0 free, >0 a net, -1 shared."""

    def __init__(self) -> None:
        self.nx = int(W / G) + 1
        self.ny = int(H / G) + 1
        self.shapes: list[tuple] = []
        self.classes = sorted({TRACK, 0.35, 0.5})
        self.rebuild()

    def rebuild(self) -> None:
        self.occ = {w: np.zeros((2, self.ny, self.nx), dtype=np.int32) for w in self.classes}
        self.via = np.zeros((self.ny, self.nx), dtype=np.int32)
        self.holes = np.zeros((self.ny, self.nx), dtype=bool)
        for shape in self.shapes:
            self._apply(shape)

    def add(self, shape: tuple) -> None:
        self.shapes.append(shape)
        self._apply(shape)

    def _mask(self, shape, grow: float):
        kind = shape[0]
        if kind == "rect":
            _, cx, cy, hx, hy = shape[:5]
            x0, x1, y0, y1 = cx - hx - grow, cx + hx + grow, cy - hy - grow, cy + hy + grow
        elif kind == "seg":
            _, ax, ay, bx, by, half = shape[:6]
            r = half + grow
            x0, x1, y0, y1 = min(ax, bx) - r, max(ax, bx) + r, min(ay, by) - r, max(ay, by) + r
        else:  # circle
            _, cx, cy, r0 = shape[:4]
            r = r0 + grow
            x0, x1, y0, y1 = cx - r, cx + r, cy - r, cy + r
        i0, i1 = max(0, int(math.floor(x0 / G))), min(self.nx - 1, int(math.ceil(x1 / G)))
        j0, j1 = max(0, int(math.floor(y0 / G))), min(self.ny - 1, int(math.ceil(y1 / G)))
        if i1 < i0 or j1 < j0:
            return None
        xs = np.arange(i0, i1 + 1) * G
        ys = np.arange(j0, j1 + 1) * G
        gx, gy = np.meshgrid(xs, ys)
        if kind == "rect":
            dx = np.maximum(np.abs(gx - cx) - hx, 0.0)
            dy = np.maximum(np.abs(gy - cy) - hy, 0.0)
            mask = dx * dx + dy * dy <= grow * grow
        elif kind == "seg":
            ex, ey = bx - ax, by - ay
            length2 = ex * ex + ey * ey
            if length2 < 1e-9:
                t = np.zeros_like(gx)
            else:
                t = np.clip(((gx - ax) * ex + (gy - ay) * ey) / length2, 0.0, 1.0)
            dx, dy = gx - (ax + t * ex), gy - (ay + t * ey)
            mask = dx * dx + dy * dy <= r * r
        else:
            mask = (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
        return j0, j1 + 1, i0, i1 + 1, mask

    @staticmethod
    def _stamp(target, slices, net):
        j0, j1, i0, i1, mask = slices
        view = target[j0:j1, i0:i1]
        free = view == 0
        other = (view != 0) & (view != net)
        view[mask & free] = net
        view[mask & other] = -1

    def _apply(self, shape) -> None:
        # (kind, geometry..., layers, net, is_hole)
        layers, net, hole_radius = shape[-3], shape[-2], shape[-1]
        for width in self.classes:
            grown = self._mask(shape, CLEARANCE + width / 2.0 + MARGIN)
            if grown is None:
                continue
            for layer in layers:
                self._stamp(self.occ[width][layer], grown, net)
        grown = self._mask(shape, CLEARANCE + VIA_D / 2.0 + MARGIN)
        if grown is not None:
            self._stamp(self.via, grown, net)
        if hole_radius:
            cx, cy = shape[1], shape[2]
            near = self._mask(("circle", cx, cy, hole_radius), HOLE_TO_HOLE + VIA_DRILL / 2.0)
            if near is not None:
                j0, j1, i0, i1, mask = near
                self.holes[j0:j1, i0:i1] |= mask


DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
STEP = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]
VIA_COST = 14.0


def astar(grid: Grid, net: int, width: float, mask: np.ndarray, sources, goals, max_expand: int = 400000, target=None):
    """Sources and goals are (layer, i, j). Returns [(layer, i, j), ...] or None."""
    nx, ny = grid.nx, grid.ny
    occ = grid.occ[width]
    free = ((occ == 0) | (occ == net)) & mask
    free_f = free[0].tobytes()
    free_b = free[1].tobytes()
    via_ok = (((grid.via == 0) | (grid.via == net)) & ~grid.holes & mask & free[0] & free[1]).tobytes()
    cells = (free_f, free_b)

    goal_set = set(goals)
    if not goal_set:
        return None
    if target is None:
        tx = sum(g[1] for g in goal_set) / len(goal_set)
        ty = sum(g[2] for g in goal_set) / len(goal_set)
    else:
        tx, ty = target

    def h(i, j):
        dx, dy = abs(i - tx), abs(j - ty)
        return (max(dx, dy) + 0.414 * min(dx, dy)) * 0.999

    heap = []
    best: dict[tuple, float] = {}
    parent: dict[tuple, tuple] = {}
    for layer, i, j in sources:
        state = (layer, i, j, -1)
        best[state] = 0.0
        heapq.heappush(heap, (h(i, j), 0.0, state))
    expanded = 0
    while heap:
        _, cost, state = heapq.heappop(heap)
        if cost > best.get(state, 1e18):
            continue
        layer, i, j, d = state
        if (layer, i, j) in goal_set:
            path = [(layer, i, j)]
            while state in parent:
                state = parent[state]
                cell = state[:3]
                if cell != path[-1]:
                    path.append(cell)
            path.reverse()
            return path
        expanded += 1
        if expanded > max_expand:
            return None
        here = cells[layer]
        for index, (dx, dy) in enumerate(DIRS):
            ni, nj = i + dx, j + dy
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            if not here[nj * nx + ni]:
                # A goal pad cell may sit inside its own clearance halo on a tight
                # pitch; it is checked again when written.
                if (layer, ni, nj) not in goal_set:
                    continue
            # A diagonal may not cut between two blocked orthogonal neighbours.
            if dx and dy and not (here[j * nx + ni] or here[nj * nx + i]):
                continue
            step = STEP[index]
            if d >= 0 and d != index:
                ddx, ddy = DIRS[d]
                dot = ddx * dx + ddy * dy
                step += 0.15 if dot > 0 else 0.6
            ns = (layer, ni, nj, index)
            nc = cost + step
            if nc < best.get(ns, 1e18):
                best[ns] = nc
                parent[ns] = state
                heapq.heappush(heap, (nc + h(ni, nj), nc, ns))
        other = 1 - layer
        if via_ok[j * nx + i]:
            ns = (other, i, j, d)
            nc = cost + VIA_COST
            if nc < best.get(ns, 1e18):
                best[ns] = nc
                parent[ns] = state
                heapq.heappush(heap, (nc + h(i, j), nc, ns))
    return None


def simplify(path):
    """Merge straight runs into one segment each."""
    if len(path) < 3:
        return path
    out = [path[0]]
    for a, b, c in zip(path, path[1:], path[2:], strict=False):
        same_layer = a[0] == b[0] == c[0]
        d1 = (b[1] - a[1], b[2] - a[2])
        d2 = (c[1] - b[1], c[2] - b[2])
        if not (same_layer and d1 == d2):
            out.append(b)
    out.append(path[-1])
    return out


class Router:
    def __init__(self, board, pads_by_net, net_id_by_name) -> None:
        self.board = board
        self.pads_by_net = pads_by_net
        self.net_ids = net_id_by_name
        self.grid = Grid()
        self.masks = self._domain_masks()
        self.net_domain: dict[str, str] = {}
        self.base_shapes: list[tuple] = []
        self.fanouts: dict[int, tuple] = {}
        self.routes: dict[str, list] = {}

    def _domain_masks(self):
        nx, ny = self.grid.nx, self.grid.ny
        xs = np.arange(nx) * G
        ys = np.arange(ny) * G
        gx, gy = np.meshgrid(xs, ys)
        edge = (gx >= EDGE + 0.25) & (gx <= W - EDGE - 0.25) & (gy >= EDGE + 0.25) & (gy <= H - EDGE - 0.25)
        masks = {"logic": edge & (gy >= BARRIER_Y + BARRIER_HALF + 0.3)}
        for name, c in COLUMNS.items():
            masks[name] = edge & (gy <= BARRIER_Y - BARRIER_HALF - 0.3) & (gx >= c - COLUMN_HALF + 0.75) & (gx <= c + COLUMN_HALF - 0.75)
        return masks

    def add_pad_shapes(self, pad_records) -> None:
        for record in pad_records:
            self.grid.add(record["shape"])
        self.base_shapes = list(self.grid.shapes)


def cell_of(x: float, y: float) -> tuple[int, int]:
    return int(round(x / G)), int(round(y / G))


def pad_entries(grid: Grid, record, width: float, net: int, mask: np.ndarray):
    """Grid cells a track may start or end on for this pad."""
    x, y, hx, hy = record["x"], record["y"], record["hx"], record["hy"]
    out = []
    i0, i1 = int(math.ceil((x - hx) / G)), int(math.floor((x + hx) / G))
    j0, j1 = int(math.ceil((y - hy) / G)), int(math.floor((y + hy) / G))
    candidates = [(i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)]
    if not candidates:
        candidates = [cell_of(x, y)]
    occ = grid.occ[width]
    for layer in record["layers"]:
        for i, j in candidates:
            if 0 <= j < grid.ny and 0 <= i < grid.nx and mask[j, i] and occ[layer, j, i] in (0, net):
                out.append((layer, i, j))
    if not out:
        # Tight pitch: the centre itself, even where a neighbour's halo reaches it.
        i, j = cell_of(x, y)
        for layer in record["layers"]:
            out.append((layer, i, j))
    return out


# ------------------------------------------------------------------ the board


def load_footprint(part, position, net_lookup, board, sheet, pins, fields):
    lib, name = part.footprint.split(":")
    fp = pcbnew.FootprintLoad(str(SHARE / f"{lib}.pretty"), name)
    if fp is None:
        raise RuntimeError(f"no footprint {part.footprint}")
    fp.SetReference(part.ref)
    fp.SetValue(part.value)
    fp.SetFPIDAsString(part.footprint)
    x, y, rot = position
    fp.SetPosition(vec(x, y))
    fp.SetOrientationDegrees(rot)
    fp.Reference().SetVisible(False)
    fp.Value().SetVisible(False)
    fp.SetAttributes(fp.GetAttributes())
    if part.dnp:
        fp.SetDNP(True)
    sheet_uuid = uid("sheet", sheet.name)
    sym_uuid = uid(sheet.name, "sym", part.ref)
    fp.SetPath(pcbnew.KIID_PATH(f"/{sheet_uuid}/{sym_uuid}"))
    fp.SetSheetname(sheet.title)
    fp.SetSheetfile(f"{sheet.name}.kicad_sch")
    for name, value in fields.items():
        if name in ("Footprint", "Reference", "Value", "Sheetname", "Sheetfile"):
            continue
        fp.SetField(name, value)
        field = fp.GetField(name)
        if field is not None:
            field.SetVisible(False)
    board.Add(fp)
    for pad in fp.Pads():
        net = pins.get(pad.GetNumber())
        if net is not None:
            pad.SetNet(net_lookup[net])
    return fp


def pad_record(pad, ref):
    position = pad.GetPosition()
    x = pcbnew.ToMM(position.x) - OX
    y = pcbnew.ToMM(position.y) - OY
    box = pad.GetBoundingBox()
    hx = pcbnew.ToMM(box.GetWidth()) / 2.0
    hy = pcbnew.ToMM(box.GetHeight()) / 2.0
    through = pad.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
    on_back = pad.IsOnLayer(pcbnew.B_Cu)
    on_front = pad.IsOnLayer(pcbnew.F_Cu)
    layers = []
    if through or on_front:
        layers.append(0)
    if through or on_back:
        layers.append(1)
    netcode = pad.GetNetCode()
    drill = pcbnew.ToMM(pad.GetDrillSizeX()) / 2.0 if through else 0.0
    if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
        netcode = -1
        layers = [0, 1]
    return {
        "x": x,
        "y": y,
        "hx": hx,
        "hy": hy,
        "layers": layers,
        "net": netcode,
        "through": through,
        "ref": ref,
        "number": pad.GetNumber(),
        "pad": pad,
        "shape": ("rect", x, y, hx, hy, tuple(layers), netcode if netcode else -1, drill),
    }


def add_track(board, net, width, layer, ax, ay, bx, by):
    track = pcbnew.PCB_TRACK(board)
    track.SetStart(vec(ax, ay))
    track.SetEnd(vec(bx, by))
    track.SetWidth(mm(width))
    track.SetLayer(pcbnew.F_Cu if layer == 0 else pcbnew.B_Cu)
    track.SetNet(net)
    board.Add(track)


def add_via(board, net, x, y):
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(vec(x, y))
    via.SetDrill(mm(VIA_DRILL))
    via.SetWidth(mm(VIA_D))
    via.SetNet(net)
    board.Add(via)


def add_zone(board, net, layer, x0, y0, x1, y1):
    zone = pcbnew.ZONE(board)
    zone.SetLayer(layer)
    zone.SetIsRuleArea(False)
    zone.SetNet(net)
    zone.SetLocalClearance(mm(0.25))
    zone.SetMinThickness(mm(0.2))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    zone.SetThermalReliefGap(mm(0.25))
    zone.SetThermalReliefSpokeWidth(mm(0.3))
    outline = zone.Outline()
    outline.NewOutline()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        outline.Append(mm(x + OX), mm(y + OY))
    zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    board.Add(zone)
    return zone


def add_text(board, text, x, y, size=1.2, layer=None, angle=0.0, thickness=0.15):
    item = pcbnew.PCB_TEXT(board)
    item.SetText(text)
    item.SetPosition(vec(x, y))
    item.SetLayer(pcbnew.F_SilkS if layer is None else layer)
    item.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    item.SetTextThickness(mm(thickness))
    item.SetTextAngleDegrees(angle)
    board.Add(item)


def write_project_rules() -> None:
    path = KICAD / f"{PROJECT}.kicad_pro"
    project = json.loads(path.read_text("utf-8")) if path.exists() else {}
    project.setdefault("meta", {"filename": f"{PROJECT}.kicad_pro", "version": 3})
    project["net_settings"] = {
        "classes": [
            {
                "name": "Default",
                "clearance": CLEARANCE,
                "track_width": TRACK,
                "via_diameter": VIA_D,
                "via_drill": VIA_DRILL,
                "microvia_diameter": 0.3,
                "microvia_drill": 0.1,
                "diff_pair_width": 0.2,
                "diff_pair_gap": 0.2,
                "diff_pair_via_gap": 0.25,
                "wire_width": 6,
                "bus_width": 12,
                "line_style": 0,
                "schematic_color": "rgba(0, 0, 0, 0.000)",
                "pcb_color": "rgba(0, 0, 0, 0.000)",
                "priority": 2147483647,
            }
        ],
        "meta": {"version": 3},
        "net_colors": None,
        "netclass_assignments": None,
        "netclass_patterns": [],
    }
    board_settings = project.setdefault("board", {})
    settings = board_settings.setdefault("design_settings", {})
    settings["rules"] = {
        "min_clearance": CLEARANCE,
        "min_track_width": 0.15,
        "min_via_diameter": 0.45,
        "min_via_annular_width": 0.1,
        "min_through_hole_diameter": 0.2,
        "min_hole_clearance": 0.2,
        "min_hole_to_hole": HOLE_TO_HOLE,
        "min_copper_edge_clearance": 0.3,
        "min_silk_clearance": 0.0,
        "min_text_height": 0.8,
        "min_text_thickness": 0.08,
        "solder_mask_to_copper_clearance": 0.0,
    }
    path.write_text(json.dumps(project, indent=2), encoding="utf-8", newline=chr(10))


def main() -> int:
    started = time.time()
    sheets = build()
    positions = placements(sheets)
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)

    settings = board.GetDesignSettings()
    settings.SetBoardThickness(mm(1.6))

    # Nets
    netlist, comp_fields = read_netlist()
    names: dict[str, str] = {}
    for sheet in sheets:
        for part in sheet.parts:
            for net in part.pins.values():
                full = full_net_name(sheet, net)
                names[full] = domain_of(sheet.name, net)
            for net in netlist.get(part.ref, {}).values():
                # Pins marked no-connect come back as their own "unconnected-" nets.
                names.setdefault(net, "logic")
    net_lookup = {}
    for name in sorted(names):
        item = pcbnew.NETINFO_ITEM(board, name)
        board.Add(item)
        net_lookup[name] = item

    # Outline
    corners = [(0, 0), (W, 0), (W, H), (0, H)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1], strict=True):
        edge = pcbnew.PCB_SHAPE(board)
        edge.SetShape(pcbnew.SHAPE_T_SEGMENT)
        edge.SetStart(vec(ax, ay))
        edge.SetEnd(vec(bx, by))
        edge.SetLayer(pcbnew.Edge_Cuts)
        edge.SetWidth(mm(0.1))
        board.Add(edge)

    # Footprints
    missing = []
    footprints = {}
    for sheet in sheets:
        for part in sheet.parts:
            if part.ref not in positions:
                missing.append(part.ref)
                continue
            footprints[part.ref] = load_footprint(
                part, positions[part.ref], net_lookup, board, sheet, netlist.get(part.ref, {}), comp_fields.get(part.ref, {})
            )
    if missing:
        raise RuntimeError(f"no placement for {missing}")

    # Mounting holes are board-only: the schematic does not know about them.
    hole_shapes = []
    for hx, hy in ((4.5, 4.5), (W - 4.5, 4.5), (4.5, H - 4.5), (W - 4.5, H - 4.5)):
        fp = pcbnew.FootprintLoad(str(SHARE / "MountingHole.pretty"), "MountingHole_3.2mm_M3")
        fp.SetReference("H")
        fp.SetPosition(vec(hx, hy))
        fp.Reference().SetVisible(False)
        fp.Value().SetVisible(False)
        fp.SetAttributes(pcbnew.FP_BOARD_ONLY | pcbnew.FP_EXCLUDE_FROM_POS_FILES | pcbnew.FP_EXCLUDE_FROM_BOM)
        board.Add(fp)
        hole_shapes.append(("circle", hx, hy, 1.6, (0, 1), -1, 1.6))

    # Pads as the router sees them
    records = []
    by_net: dict[str, list] = {}
    for ref, fp in footprints.items():
        for pad in fp.Pads():
            record = pad_record(pad, ref)
            records.append(record)
            name = pad.GetNetname()
            if name:
                by_net.setdefault(name, []).append(record)

    router = Router(board, by_net, net_lookup)
    grid = router.grid
    for record in records:
        grid.add(record["shape"])
    for shape in hole_shapes:
        grid.add(shape)
    base_shapes = list(grid.shapes)

    # Order: the USB pair first, then planes' fan-out, then shortest nets.
    def span(name: str) -> float:
        xs = [r["x"] for r in by_net[name]]
        ys = [r["y"] for r in by_net[name]]
        return (max(xs) - min(xs)) + (max(ys) - min(ys))

    signal_nets = [n for n in by_net if not is_plane(n) and len(by_net[n]) > 1 and not n.startswith("unconnected-")]
    plane_nets = [n for n in by_net if is_plane(n)]
    print(f"{len(signal_nets)} signal nets, {len(plane_nets)} plane nets, {sum(1 for n in by_net if n.startswith('unconnected-'))} marked unconnected", flush=True)
    priority = {"USB_DP": 0, "USB_DM": 1, "HSE_IN": 2, "HSE_OUT": 3}
    order = sorted(signal_nets, key=lambda n: (priority.get(n, 10), span(n)))

    best_result = None
    for attempt in range(8):
        grid.shapes = list(base_shapes)
        grid.rebuild()
        result = route_all(grid, by_net, names, order, plane_nets, net_lookup, router.masks)
        failed = result["failed"]
        print(f"pass {attempt + 1}: {len(result['tracks'])} segments, {len(result['vias'])} vias, {len(failed)} failed {failed[:8]} ({time.time() - started:.0f}s)", flush=True)
        if best_result is None or len(failed) < len(best_result["failed"]):
            best_result = result
        if not failed:
            break
        order = failed + [n for n in order if n not in failed]

    for name, width, layer, ax, ay, bx, by in best_result["tracks"]:
        add_track(board, net_lookup[name], width, layer, ax, ay, bx, by)
    for name, x, y in best_result["vias"]:
        add_via(board, net_lookup[name], x, y)

    # Planes
    gap = BARRIER_HALF
    logic = (0.25, BARRIER_Y + gap, W - 0.25, H - 0.25)
    zones = [add_zone(board, net_lookup["GND"], pcbnew.In1_Cu, *logic)]
    zones.append(add_zone(board, net_lookup["+3V3"], pcbnew.In2_Cu, *logic))
    for sheet in sheets:
        if sheet.name in COLUMNS:
            c = COLUMNS[sheet.name]
            net = net_lookup[f"/{sheet.title}/ISO_GND"]
            x0 = max(0.25, c - COLUMN_HALF + gap)
            x1 = min(W - 0.25, c + COLUMN_HALF - gap)
            zones.append(add_zone(board, net, pcbnew.In1_Cu, x0, 0.25, x1, BARRIER_Y - gap))
            zones.append(add_zone(board, net, pcbnew.In2_Cu, x0, 0.25, x1, BARRIER_Y - gap))
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())

    # Labels. Each pin is named where it physically is: a connector turned 180 degrees
    # reads backwards, and a wrong label here means a wrong wire later.
    for sheet in sheets:
        if sheet.name not in COLUMNS:
            continue
        c = COLUMNS[sheet.name]
        title = sheet.name.replace("port_", "").upper()
        if sheet.name.startswith("port_can"):
            add_text(board, title, c - 10.0, 15.5, 1.6, thickness=0.2)
            # Pin 1 is at c + 3.5 after the rotation, pin 3 at c - 3.5.
            add_text(board, "CANH", c + 3.5, 14.0, 0.8)
            add_text(board, "CANL", c, 14.0, 0.8)
            add_text(board, "GND", c - 3.5, 14.0, 0.8)
            add_text(board, "120R", c + 10.0, 13.5, 0.8)
        else:
            add_text(board, title, c - 9.5, 12.5, 1.6, thickness=0.2)
            # Pins run left to right here: 1 at c - 2, 3 at c + 2.
            add_text(board, "DATA  NC  GND", c - 9.5, 4.5, 0.8)
    add_text(board, "RAC SYNC CONTROLLER  REV A", 95.0, 76.0, 1.2)
    back_label = pcbnew.PCB_TEXT(board)
    back_label.SetText("ISOLATION BARRIER")
    back_label.SetPosition(vec(44.0, 38.1))
    back_label.SetLayer(pcbnew.B_SilkS)
    back_label.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
    back_label.SetTextThickness(mm(0.12))
    back_label.SetMirrored(True)
    board.Add(back_label)
    # Terminals at the left edge: pin 1 at the top after the rotation.
    for i, label in enumerate(("+5V", "CH1", "CH2", "GND")):
        add_text(board, label, 14.5, 48.0 + 3.5 * i, 0.8)
    add_text(board, "ESTOP", 7.0, 62.5, 1.0)
    for i, label in enumerate(("A COM", "A NO", "B COM", "B NO")):
        add_text(board, label, 15.0, 66.0 + 3.5 * i, 0.8)
    add_text(board, "RELAY", 7.0, 80.5, 1.0)
    add_text(board, "STOP", 22.0, 86.0, 1.0)
    add_text(board, "BOOT", 47.0, 86.0, 1.0)
    add_text(board, "RESET", 59.0, 86.0, 1.0)
    add_text(board, "SYNC", 33.0, 87.5, 1.0)
    add_text(board, "SWD", 52.0, 46.5, 1.0)
    x = 74.0
    for label in ("CAN1", "CAN2", "UART1", "UART2", "RUN", "FAULT"):
        add_text(board, label, x, 87.5, 0.8)
        x += 6.0

    KICAD.mkdir(parents=True, exist_ok=True)
    write_project_rules()
    out = KICAD / f"{PROJECT}.kicad_pcb"
    pcbnew.SaveBoard(str(out), board)
    print(f"saved {out} in {time.time() - started:.0f}s; failed nets: {best_result['failed']}", flush=True)
    return 0 if not best_result["failed"] else 2


def route_all(grid, by_net, names, order, plane_nets, net_lookup, masks):
    tracks: list[tuple] = []
    vias: list[tuple] = []
    failed: list[str] = []

    def commit_path(name, width, net_id, path, start_record=None, end_record=None):
        cells = simplify(path)
        for a, b in zip(cells, cells[1:], strict=False):
            ax, ay, bx, by = a[1] * G, a[2] * G, b[1] * G, b[2] * G
            if a[0] != b[0]:
                vias.append((name, ax, ay))
                grid.add(("circle", ax, ay, VIA_D / 2.0, (0, 1), net_id, VIA_DRILL / 2.0))
            else:
                tracks.append((name, width, a[0], ax, ay, bx, by))
                grid.add(("seg", ax, ay, bx, by, width / 2.0, (a[0],), net_id, 0.0))

    def stub(name, width, record, cell, net_id):
        """Join a grid cell to the exact pad centre when they differ."""
        layer, i, j = cell
        x, y = i * G, j * G
        if abs(x - record["x"]) > 1e-6 or abs(y - record["y"]) > 1e-6:
            tracks.append((name, width, layer, x, y, record["x"], record["y"]))

    # Plane fan-out: every surface mount pad on a plane net gets a via beside it.
    for name in plane_nets:
        net_id = net_lookup[name].GetNetCode()
        mask = masks[names[name]]
        for record in by_net[name]:
            if record["through"]:
                continue
            nearby = []
            cx, cy = cell_of(record["x"], record["y"])
            for dj in range(-14, 15):
                for di in range(-14, 15):
                    i, j = cx + di, cy + dj
                    if not (0 <= i < grid.nx and 0 <= j < grid.ny):
                        continue
                    if not mask[j, i] or grid.holes[j, i] or grid.via[j, i] not in (0, net_id):
                        continue
                    if grid.occ[TRACK][0, j, i] not in (0, net_id) or grid.occ[TRACK][1, j, i] not in (0, net_id):
                        continue
                    nearby.append((0, i, j))
            sources = pad_entries(grid, record, TRACK, net_id, mask)
            path = astar(grid, net_id, TRACK, mask, sources, nearby, max_expand=4000, target=(cx, cy)) if nearby else None
            if path is None:
                failed.append(f"{name}:{record['ref']}.{record['number']}")
                continue
            stub(name, TRACK, record, path[0], net_id)
            commit_path(name, TRACK, net_id, path)
            end = path[-1]
            vias.append((name, end[1] * G, end[2] * G))
            grid.add(("circle", end[1] * G, end[2] * G, VIA_D / 2.0, (0, 1), net_id, VIA_DRILL / 2.0))

    # Signal nets: grow a tree, nearest unconnected pad first.
    for name in order:
        net_id = net_lookup[name].GetNetCode()
        width = track_width(name)
        mask = masks[names[name]]
        pads = by_net[name]
        remaining = list(pads)
        first = remaining.pop(0)
        tree = set(pad_entries(grid, first, width, net_id, mask))
        # A cell can be inside a pad's bounding box yet outside a round pad's copper,
        # so a path touching any pad's cell is always joined to that pad's centre.
        owner = {cell: first for cell in tree}
        ok = True
        while remaining:
            tree_list = list(tree)
            cx = sum(t[1] for t in tree_list) / len(tree_list) * G
            cy = sum(t[2] for t in tree_list) / len(tree_list) * G
            remaining.sort(key=lambda r: abs(r["x"] - cx) + abs(r["y"] - cy))
            record = remaining.pop(0)
            goals = pad_entries(grid, record, width, net_id, mask)
            path = astar(grid, net_id, width, mask, tree_list, goals)
            if path is None:
                ok = False
                break
            if path[0] in owner:
                stub(name, width, owner[path[0]], path[0], net_id)
            stub(name, width, record, path[-1], net_id)
            commit_path(name, width, net_id, path)
            tree.update(path)
            entries = pad_entries(grid, record, width, net_id, mask)
            tree.update(entries)
            for cell in entries:
                owner.setdefault(cell, record)
        if not ok:
            failed.append(name)
    return {"tracks": tracks, "vias": vias, "failed": failed}


if __name__ == "__main__":
    sys.exit(main())
