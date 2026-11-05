"""The committed board against its checks. Skipped on machines without KiCad."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
HW = ROOT / "hardware" / "sync-controller"
GEN = HW / "gen"
CLI = os.environ.get("KICAD_CLI", "D:/KiCad/bin/kicad-cli.exe")
KICAD_PY = os.environ.get("KICAD_PYTHON", "D:/KiCad/bin/python.exe")

sys.path.insert(0, str(GEN))

needs_kicad = pytest.mark.skipif(not (Path(CLI).exists() or shutil.which(CLI)), reason="KiCad is not installed")


def run(script: str, python: str = sys.executable) -> str:
    env = {**os.environ, "PYTHONUTF8": "1"}
    result = subprocess.run([python, script], cwd=GEN, capture_output=True, text=True, encoding="utf-8", errors="replace", env=env)
    assert result.returncode == 0, result.stdout[-2000:] + result.stderr[-1000:]
    return result.stdout


@needs_kicad
def test_erc_is_clean():
    assert "unexplained: 0" in run("check_erc.py")


@needs_kicad
def test_schematic_matches_the_design_data():
    assert ", 0 mismatches" in run("check_netlist.py")


@needs_kicad
def test_drc_is_clean_and_matches_the_schematic():
    assert "unexplained: 0" in run("check_drc.py")


@needs_kicad
def test_isolation_gaps_meet_the_target():
    run("check_isolation.py", KICAD_PY)


class TestDesignRules:
    """Circuit properties the safety and isolation arguments depend on."""

    @pytest.fixture(scope="class")
    def sheets(self):
        from design import build

        return {s.name: s for s in build()}

    def _part(self, sheet, value):
        return next(p for p in sheet.parts if p.value == value)

    def test_every_port_has_its_own_ground(self, sheets):
        for name in ("port_can1", "port_can2", "port_uart1", "port_uart2"):
            sheet = sheets[name]
            converter = self._part(sheet, "B0509S-1WR3")
            assert converter.pins["1"] == "GND" and converter.pins["2"] == "+5V"
            assert converter.pins["3"] == "ISO_GND" and converter.pins["4"] == "ISO_9V"

    def test_can_transceiver_pins_follow_the_datasheet(self, sheets):
        """TI SLLSFB0A: 5 CANL, 6 CANH, 7 GND2, 8 VCC2. Easy to misremember."""
        iso = self._part(sheets["port_can1"], "ISO1044BD")
        assert iso.pins["5"] == "CANL"
        assert iso.pins["6"] == "CANH"
        assert iso.pins["7"] == "ISO_GND"
        assert iso.pins["8"] == "ISO_5V", "VCC2 must be the regulated rail: its limit is 5.5 V"

    def test_servo_connector_leaves_the_arms_supply_unconnected(self, sheets):
        connector = next(p for p in sheets["port_uart1"].parts if p.footprint.endswith("PH_B3B-PH-K_1x03_P2.00mm_Vertical"))
        assert "2" not in connector.pins
        assert "2" in connector.no_connect

    def test_the_estop_contact_is_in_series_with_the_relay_coil(self, sheets):
        safety = sheets["safety"]
        terminal = next(p for p in safety.parts if p.value == "ESTOP")
        relay = next(p for p in safety.parts if p.lib == "Relay:G5V-2")
        mosfet = next(p for p in safety.parts if p.value == "2N7002")
        assert terminal.pins["1"] == "+5V"
        assert relay.pins["1"] == terminal.pins["2"], "coil is fed through the contact"
        assert relay.pins["16"] == mosfet.pins["3"]

    def test_the_relay_gate_is_pulled_down(self, sheets):
        """A reset MCU leaves its pin floating: this is what then drops the relay."""
        safety = sheets["safety"]
        pulldowns = [p for p in safety.parts if p.value == "100k" and set(p.pins.values()) == {"SAFETY_GATE", "GND"}]
        assert len(pulldowns) == 1

    def test_relay_outputs_are_normally_open(self, sheets):
        terminal = next(p for p in sheets["safety"].parts if p.value == "RELAY")
        assert set(terminal.pins.values()) == {"K_A_COM", "K_A_NO", "K_B_COM", "K_B_NO"}

    def test_no_port_signal_bypasses_an_isolator(self, sheets):
        """On a port sheet only the isolator and converter may touch both domains."""
        from design import GLOBALS, POWER_NETS

        for name in ("port_can1", "port_can2", "port_uart1", "port_uart2"):
            for part in sheets[name].parts:
                nets = set(part.pins.values())
                logic = nets & (GLOBALS | POWER_NETS)
                isolated = nets - logic
                if logic and isolated:
                    assert part.value in ("B0509S-1WR3", "ISO1044BD", "ADuM1201BRZ"), f"{part.ref} {part.value} joins {nets}"

    def test_bom_lists_every_part(self):
        import csv

        from design import all_parts

        with (HW / "docs" / "bom.csv").open(encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        assert sum(int(r["Qty"]) for r in rows) == len(all_parts())


def test_manufacturing_files_exist():
    fab = HW / "fab"
    if not fab.exists():
        pytest.skip("run make_all.py first")
    names = {p.name for p in (fab / "gerber").iterdir()}
    for layer in ("F_Cu", "In1_Cu", "In2_Cu", "B_Cu", "Edge_Cuts"):
        assert any(layer in n for n in names), layer
    assert any(n.endswith(".drl") for n in names)
    assert (fab / "positions.csv").exists()
    assert (fab / "sync-controller-gerbers.zip").exists()
