#!/usr/bin/env python3
"""Write the KiCad schematic from design.py.

Connections are drawn as a short stub and a label on each pin. That is a choice:
every connection is then explicit in the data file, and the generated sheets are
regenerated rather than hand edited.

    python hardware/sync-controller/gen/gen_sch.py [--kicad-share D:/KiCad/share/kicad]
"""

from __future__ import annotations

import argparse
import copy
import os
import uuid
from pathlib import Path

from design import GLOBALS, POWER_NETS, PROJECT, Part, Sheet, build
from sexpr import Sym, dump, find, find_all, parse

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "kicad"

NS = uuid.UUID("7a1c2f3e-5b6d-4f70-8a91-0c2d3e4f5a6b")
STUB = 2.54
FONT = [Sym("font"), [Sym("size"), 1.27, 1.27]]


def uid(*parts: object) -> str:
    """Stable identifiers, so regenerating does not churn every line of the files."""
    return str(uuid.uuid5(NS, "/".join(str(p) for p in parts)))


class Library:
    def __init__(self, share: Path) -> None:
        self.dir = share / "symbols"
        self.cache: dict[str, list] = {}

    def _lib(self, name: str) -> list:
        if name not in self.cache:
            self.cache[name] = parse((self.dir / f"{name}.kicad_sym").read_text("utf-8"))
        return self.cache[name]

    def symbol(self, lib_id: str) -> list:
        """A flattened copy under its lib_id: derived symbols take their parent's drawing."""
        lib, name = lib_id.split(":")
        tree = self._lib(lib)
        symbols = {s[1]: s for s in find_all(tree, "symbol")}
        own = symbols[name]
        extends = find(own, "extends")
        if extends is None:
            result = copy.deepcopy(own)
            base = name
        else:
            parent = symbols[extends[1]]
            base = extends[1]
            result = copy.deepcopy(parent)
            # The child's properties replace the parent's; its drawing is inherited.
            result = [item for item in result if not (isinstance(item, list) and item and item[0] == "property")]
            properties = [copy.deepcopy(p) for p in find_all(own, "property")]
            at = 2
            for prop in properties:
                result.insert(at, prop)
                at += 1
        result[1] = lib_id
        for sub in find_all(result, "symbol"):
            if sub[1].startswith(base + "_"):
                sub[1] = name + sub[1][len(base) :]
        return result


def pins_of(symbol: list, unit: int = 1) -> dict[str, tuple[float, float, int, str]]:
    """Pin number -> (x, y, angle, electrical type) in symbol coordinates."""
    out: dict[str, tuple[float, float, int, str]] = {}
    for sub in find_all(symbol, "symbol"):
        sub_unit = int(sub[1].rsplit("_", 2)[-2])
        if sub_unit not in (0, unit):
            continue
        for pin in find_all(sub, "pin"):
            at = find(pin, "at")
            number = find(pin, "number")[1]
            out[number] = (float(at[1]), float(at[2]), int(float(at[3])) if len(at) > 3 else 0, str(pin[1]))
    return out


def rotate(x: float, y: float, angle: int) -> tuple[float, float]:
    """Symbol coordinates, Y up, turned counterclockwise by angle."""
    a = angle % 360
    if a == 0:
        return x, y
    if a == 90:
        return -y, x
    if a == 180:
        return -x, -y
    return y, -x


class Writer:
    def __init__(self, sheet: Sheet, library: Library, root_uuid: str, sheet_uuid: str, page: int) -> None:
        self.sheet = sheet
        self.lib = library
        self.root_uuid = root_uuid
        self.sheet_uuid = sheet_uuid
        self.page = page
        self.items: list = []
        self.used_libs: dict[str, list] = {}
        self.counter = 0
        self.instance_path = f"/{root_uuid}/{sheet_uuid}"
        # Everything drawn on a net, so two nets touching by accident is caught here.
        self.segments: list[tuple[float, float, float, float, str, str]] = []

    def next_id(self, kind: str) -> str:
        self.counter += 1
        return uid(self.sheet.name, kind, self.counter)

    def use(self, lib_id: str) -> list:
        if lib_id not in self.used_libs:
            self.used_libs[lib_id] = self.lib.symbol(lib_id)
        return self.used_libs[lib_id]

    def wire(self, x1: float, y1: float, x2: float, y2: float) -> None:
        self.items.append(
            [
                Sym("wire"),
                [Sym("pts"), [Sym("xy"), x1, y1], [Sym("xy"), x2, y2]],
                [Sym("stroke"), [Sym("width"), 0], [Sym("type"), Sym("default")]],
                [Sym("uuid"), self.next_id("wire")],
            ]
        )

    def label(self, net: str, x: float, y: float, direction: str) -> None:
        angle = {"left": 180, "right": 0, "up": 90, "down": 270}[direction]
        justify = {"left": [Sym("justify"), Sym("right"), Sym("bottom")], "right": [Sym("justify"), Sym("left"), Sym("bottom")], "up": [Sym("justify"), Sym("left"), Sym("bottom")], "down": [Sym("justify"), Sym("right"), Sym("bottom")]}[direction]
        if net in GLOBALS:
            self.items.append(
                [
                    Sym("global_label"),
                    net,
                    [Sym("shape"), Sym("bidirectional")],
                    [Sym("at"), x, y, angle],
                    [Sym("fields_autoplaced"), Sym("yes")],
                    [Sym("effects"), FONT, [Sym("justify"), Sym("right" if direction in ("left", "down") else "left")]],
                    [Sym("uuid"), self.next_id("glabel")],
                    [
                        Sym("property"),
                        "Intersheetrefs",
                        "${INTERSHEET_REFS}",
                        [Sym("at"), x, y, angle],
                        [Sym("effects"), FONT, [Sym("hide"), Sym("yes")]],
                    ],
                ]
            )
        else:
            self.items.append(
                [
                    Sym("label"),
                    net,
                    [Sym("at"), x, y, angle],
                    [Sym("effects"), FONT, justify],
                    [Sym("uuid"), self.next_id("label")],
                ]
            )

    def power_symbol(self, net: str, x: float, y: float, direction: str) -> None:
        lib_id = f"power:{net}"
        self.use(lib_id)
        # GND draws downward at angle 0 and supplies draw upward.
        if net == "GND":
            angle = {"down": 0, "up": 180, "left": 270, "right": 90}[direction]
        else:
            angle = {"up": 0, "down": 180, "left": 90, "right": 270}[direction]
        self._instance(lib_id, f"#PWR_{self.sheet.name}", net, "", x, y, angle, hidden_ref=True, pwr=True)

    def flag(self, x: float, y: float, net: str) -> None:
        """PWR_FLAG: says a net really is driven, for nets ERC cannot see a source on."""
        lib_id = "power:PWR_FLAG"
        self.use(lib_id)
        self._instance(lib_id, f"#FLG_{self.sheet.name}", "PWR_FLAG", "", x, y, 0, hidden_ref=True, pwr=True)
        self.wire(x, y, x, y + STUB)
        self.place_net(net, x, y + STUB, "down")

    def _instance(self, lib_id, ref, value, footprint, x, y, angle, hidden_ref=False, pwr=False, part: Part | None = None):
        symbol = self.use(lib_id)
        if pwr:
            self.counter += 1
            ref = f"{ref}_{self.counter}"
        instance = [
            Sym("symbol"),
            [Sym("lib_id"), lib_id],
            [Sym("at"), x, y, angle],
            [Sym("unit"), 1],
            [Sym("exclude_from_sim"), Sym("no")],
            [Sym("in_bom"), Sym("no" if pwr else "yes")],
            [Sym("on_board"), Sym("no" if pwr else "yes")],
            [Sym("dnp"), Sym("yes" if (part and part.dnp) else "no")],
            [Sym("uuid"), uid(self.sheet.name, "sym", ref)],
        ]
        for prop in find_all(symbol, "property"):
            name = prop[1]
            at = find(prop, "at")
            px, py = rotate(float(at[1]), float(at[2]), angle)
            text = prop[2]
            if name == "Reference":
                text = ref
            elif name == "Value":
                text = value
            elif name == "Footprint":
                text = footprint
            elif name not in ("Datasheet", "Description"):
                continue
            effects = copy.deepcopy(find(prop, "effects"))
            text_angle = float(at[3]) if len(at) > 3 else 0
            field = [Sym("property"), name, text, [Sym("at"), round(x + px, 3), round(y - py, 3), text_angle]]
            # Only the reference and value are drawn. KiCad keeps this flag on the field itself.
            if name not in ("Reference", "Value") or (hidden_ref and name == "Reference"):
                field.append([Sym("hide"), Sym("yes")])
            field.append(effects)
            instance.append(field)
        if part:
            for name, text in (("MPN", part.mpn), ("LCSC", part.lcsc)):
                if text:
                    instance.append([Sym("property"), name, text, [Sym("at"), x, y, 0], [Sym("hide"), Sym("yes")], [Sym("effects"), FONT]])
        for number in pins_of(symbol):
            instance.append([Sym("pin"), number, [Sym("uuid"), uid(self.sheet.name, ref, "pin", number)]])
        instance.append(
            [
                Sym("instances"),
                [Sym("project"), PROJECT, [Sym("path"), self.instance_path, [Sym("reference"), ref], [Sym("unit"), 1]]],
            ]
        )
        self.items.append(instance)

    def place_net(self, net: str, x: float, y: float, direction: str) -> None:
        if net in POWER_NETS:
            self.power_symbol(net, x, y, direction)
        else:
            self.label(net, x, y, direction)

    def part(self, part: Part) -> None:
        symbol = self.use(part.lib)
        self._instance(part.lib, part.ref, part.value, part.footprint, part.x, part.y, part.rot, part=part)
        pins = pins_of(symbol)
        missing = set(part.pins) - set(pins)
        if missing:
            raise ValueError(f"{part.ref} ({part.lib}) has no pin {sorted(missing)}; it has {sorted(pins)}")
        for number, (px, py, pin_angle, _type) in pins.items():
            sx, sy = rotate(px, py, part.rot)
            # Sheet coordinates: Y grows downward.
            x = round(part.x + sx, 3)
            y = round(part.y - sy, 3)
            outward = {0: "left", 90: "down", 180: "right", 270: "up"}[(pin_angle + part.rot) % 360]
            dx, dy = {"left": (-STUB, 0), "right": (STUB, 0), "up": (0, -STUB), "down": (0, STUB)}[outward]
            if number in part.pins:
                ex, ey = round(x + dx, 3), round(y + dy, 3)
                self.wire(x, y, ex, ey)
                self.segments.append((x, y, ex, ey, part.pins[number], f"{part.ref}.{number}"))
                self.place_net(part.pins[number], ex, ey, outward)
            elif number in part.no_connect:
                self.items.append([Sym("no_connect"), [Sym("at"), x, y], [Sym("uuid"), self.next_id("nc")]])
            else:
                # A forgotten pin must be an error here, not a quiet ERC warning later.
                raise ValueError(f"{part.ref} pin {number} is neither connected nor marked no-connect")

    def check_collisions(self) -> None:
        """Generated drawings connect by coordinates, so an overlap is a silent short."""

        def on(px, py, seg):
            x1, y1, x2, y2, *_ = seg
            return min(x1, x2) - 1e-6 <= px <= max(x1, x2) + 1e-6 and min(y1, y2) - 1e-6 <= py <= max(y1, y2) + 1e-6

        for a in self.segments:
            for b in self.segments:
                if a[4] == b[4] or a is b:
                    continue
                for px, py in ((b[0], b[1]), (b[2], b[3])):
                    if on(px, py, a):
                        raise ValueError(
                            f"sheet {self.sheet.name}: {b[5]} ({b[4]}) touches {a[5]} ({a[4]}) at {px}, {py}"
                        )

    def text(self, x: float, y: float, content: str) -> None:
        self.items.append(
            [
                Sym("text"),
                content,
                [Sym("exclude_from_sim"), Sym("no")],
                [Sym("at"), x, y, 0],
                [Sym("effects"), [Sym("font"), [Sym("size"), 1.5, 1.5]], [Sym("justify"), Sym("left"), Sym("bottom")]],
                [Sym("uuid"), self.next_id("text")],
            ]
        )

    def render(self) -> str:
        for part in self.sheet.parts:
            self.part(part)
        for x, y, net in self.sheet.flags:
            self.segments.append((x, y, x, y + STUB, net, "PWR_FLAG"))
            self.flag(x, y, net)
        self.check_collisions()
        for x, y, content in self.sheet.notes:
            self.text(x, y, content)
        lib_symbols = [Sym("lib_symbols")] + [self.used_libs[k] for k in sorted(self.used_libs)]
        document = [
            Sym("kicad_sch"),
            [Sym("version"), 20250114],
            [Sym("generator"), "rac_gen_sch"],
            [Sym("generator_version"), "1.0"],
            [Sym("uuid"), self.sheet_uuid],
            [Sym("paper"), "A3"],
            [
                Sym("title_block"),
                [Sym("title"), self.sheet.title],
                [Sym("date"), "2026-11-05"],
                [Sym("rev"), "A"],
                [Sym("company"), "rebotARMCtl sync controller"],
                [Sym("comment"), 1, "Generated by hardware/sync-controller/gen/gen_sch.py from design.py"],
            ],
            lib_symbols,
            *self.items,
            [Sym("embedded_fonts"), Sym("no")],
        ]
        return dump(document) + chr(10)


def root_document(root_uuid: str, sheets: list[tuple[Sheet, str, int]]) -> str:
    items = []
    x, y = 25.4, 40.64
    for sheet, sheet_uuid, page in sheets:
        items.append(
            [
                Sym("sheet"),
                [Sym("at"), x, y],
                [Sym("size"), 45.72, 25.4],
                [Sym("fields_autoplaced"), Sym("yes")],
                [Sym("stroke"), [Sym("width"), 0.1524], [Sym("type"), Sym("solid")]],
                [Sym("fill"), [Sym("color"), 0, 0, 0, 0.0]],
                [Sym("uuid"), sheet_uuid],
                [Sym("property"), "Sheetname", sheet.title, [Sym("at"), x, y - 0.71, 0], [Sym("effects"), FONT, [Sym("justify"), Sym("left"), Sym("bottom")]]],
                [Sym("property"), "Sheetfile", f"{sheet.name}.kicad_sch", [Sym("at"), x, y + 26.0, 0], [Sym("effects"), FONT, [Sym("justify"), Sym("left"), Sym("top")]]],
                [Sym("instances"), [Sym("project"), PROJECT, [Sym("path"), f"/{root_uuid}", [Sym("page"), str(page)]]]],
            ]
        )
        x += 60.96
        if x > 300:
            x = 25.4
            y += 45.72
    document = [
        Sym("kicad_sch"),
        [Sym("version"), 20250114],
        [Sym("generator"), "rac_gen_sch"],
        [Sym("generator_version"), "1.0"],
        [Sym("uuid"), root_uuid],
        [Sym("paper"), "A3"],
        [
            Sym("title_block"),
            [Sym("title"), "RAC sync controller"],
            [Sym("date"), "2026-11-05"],
            [Sym("rev"), "A"],
            [Sym("company"), "rebotARMCtl"],
            [Sym("comment"), 1, "2 x isolated CAN, 2 x isolated servo UART, emergency stop, USB-C. Not yet built or tested."],
        ],
        [Sym("lib_symbols")],
        *items,
        [Sym("sheet_instances"), [Sym("path"), "/", [Sym("page"), "1"]]],
        [Sym("embedded_fonts"), Sym("no")],
    ]
    return dump(document) + chr(10)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kicad-share", default=os.environ.get("KICAD_SHARE", "D:/KiCad/share/kicad"))
    args = parser.parse_args()

    library = Library(Path(args.kicad_share))
    OUT.mkdir(parents=True, exist_ok=True)
    root_uuid = uid("root")
    placed = []
    for page, sheet in enumerate(build(), start=2):
        sheet_uuid = uid("sheet", sheet.name)
        writer = Writer(sheet, library, root_uuid, sheet_uuid, page)
        (OUT / f"{sheet.name}.kicad_sch").write_text(writer.render(), encoding="utf-8", newline=chr(10))
        placed.append((sheet, sheet_uuid, page))
    (OUT / f"{PROJECT}.kicad_sch").write_text(root_document(root_uuid, placed), encoding="utf-8", newline=chr(10))
    import json

    # Only the sheet list is ours: gen_pcb.py keeps the design rules in this file too.
    project_path = OUT / f"{PROJECT}.kicad_pro"
    project = json.loads(project_path.read_text("utf-8")) if project_path.exists() else {}
    project.setdefault("meta", {"filename": f"{PROJECT}.kicad_pro", "version": 3})
    project.setdefault("board", {"design_settings": {"defaults": {}}})
    project["sheets"] = [[root_uuid, "Root"]] + [[u, s.title] for s, u, _ in placed]
    project_path.write_text(json.dumps(project, indent=2), encoding="utf-8", newline=chr(10))
    print(f"wrote {len(placed) + 1} sheets to {OUT}")


if __name__ == "__main__":
    main()
