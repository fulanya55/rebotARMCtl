"""What can be checked about the STM32 image without a board.

It has not run on silicon, so this proves the build is well formed and consistent
with the hardware design, not that it works. docs/bringup.md covers the rest.
"""

from __future__ import annotations

import re
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
ELF = ROOT / "firmware" / "build" / "stm32g4" / "rac_controller.elf"
BIN = ELF.with_suffix(".bin")

FLASH = (0x08000000, 512 * 1024)
RAM = (0x20000000, 96 * 1024)

sys.path.insert(0, str(ROOT / "hardware" / "sync-controller" / "gen"))


def parse_elf(path: Path) -> tuple[dict[str, int], list[dict]]:
    data = path.read_bytes()
    assert data[:4] == b"\x7fELF" and data[4] == 1, "expected a 32 bit ELF"
    shoff = struct.unpack_from("<I", data, 32)[0]
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", data, 46)
    sections = []
    for i in range(shnum):
        name, kind, flags, addr, offset, size, link = struct.unpack_from("<IIIIIII", data, shoff + i * shentsize)
        sections.append({"name": name, "type": kind, "flags": flags, "addr": addr, "offset": offset, "size": size, "link": link})
    names = sections[shstrndx]
    for section in sections:
        start = names["offset"] + section["name"]
        section["name"] = data[start : data.index(b"\0", start)].decode()
    symbols: dict[str, int] = {}
    for section in sections:
        if section["type"] != 2:
            continue
        strtab = sections[section["link"]]
        for i in range(section["size"] // 16):
            name, value, _size, _info, _other, _shndx = struct.unpack_from("<IIIBBH", data, section["offset"] + i * 16)
            if name:
                start = strtab["offset"] + name
                symbols[data[start : data.index(b"\0", start)].decode()] = value
    return symbols, sections


@pytest.fixture(scope="module")
def image():
    if not (ROOT / "firmware" / "third_party" / "tinyusb").exists():
        pytest.skip("firmware dependencies not fetched: tools/fetch_firmware_deps.py")
    subprocess.run([sys.executable, str(ROOT / "tools" / "build_firmware.py"), "stm32"], check=True, capture_output=True)
    symbols, sections = parse_elf(ELF)
    return symbols, sections, BIN.read_bytes()


def vector(blob: bytes, index: int) -> int:
    return struct.unpack_from("<I", blob, index * 4)[0]


def header_value(name: str) -> int:
    header = (ROOT / "firmware" / "third_party" / "cmsis_device_g4" / "Include" / "stm32g474xx.h").read_text("utf-8", "replace")
    match = re.search(rf"{name}\s*=\s*(-?\d+)", header)
    assert match, name
    return int(match.group(1))


def test_the_flash_image_holds_what_the_elf_says(image):
    """Every loaded segment, byte for byte, at its physical address.

    Added after a conversion tool wrote the first segment as zeros: the vector table
    was gone, and nothing short of reading the output would have shown it.
    """
    _, _, blob = image
    data = ELF.read_bytes()
    phoff = struct.unpack_from("<I", data, 28)[0]
    phentsize, phnum = struct.unpack_from("<HH", data, 42)
    checked = 0
    for i in range(phnum):
        kind, offset, _vaddr, paddr, filesz, *_ = struct.unpack_from("<8I", data, phoff + i * phentsize)
        if kind == 1 and filesz:
            assert blob[paddr - FLASH[0] : paddr - FLASH[0] + filesz] == data[offset : offset + filesz], hex(paddr)
            checked += 1
    assert checked >= 2


def test_intel_hex_decodes_back_to_the_image(image):
    _, _, blob = image
    text = ELF.with_suffix(".hex").read_text("ascii")
    decoded = bytearray(b"\xff" * len(blob))
    upper = 0
    for line in text.splitlines():
        record = bytes.fromhex(line[1:])
        assert sum(record) & 0xFF == 0, "hex record checksum"
        count, address, kind = record[0], (record[1] << 8) | record[2], record[3]
        payload = record[4 : 4 + count]
        if kind == 4:
            upper = int.from_bytes(payload, "big") << 16
        elif kind == 0:
            start = upper + address - FLASH[0]
            decoded[start : start + count] = payload
    assert bytes(decoded) == blob


def test_stack_and_reset_vectors(image):
    symbols, _, blob = image
    assert vector(blob, 0) == RAM[0] + RAM[1], "the initial stack pointer is the top of RAM"
    reset = vector(blob, 1)
    assert reset & 1, "Cortex-M vectors must have the Thumb bit set"
    assert (reset & ~1) == symbols["Reset_Handler"] & ~1
    assert FLASH[0] <= reset < FLASH[0] + FLASH[1]


@pytest.mark.parametrize(
    "irq,handler",
    [
        ("USB_HP_IRQn", "USB_HP_IRQHandler"),
        ("USB_LP_IRQn", "USB_LP_IRQHandler"),
        ("USBWakeUp_IRQn", "USBWakeUp_IRQHandler"),
        ("USART2_IRQn", "USART2_IRQHandler"),
        ("USART3_IRQn", "USART3_IRQHandler"),
        ("FDCAN2_IT0_IRQn", "FDCAN2_IT0_IRQHandler"),
        ("FDCAN3_IT0_IRQn", "FDCAN3_IT0_IRQHandler"),
    ],
)
def test_interrupts_reach_their_handlers(image, irq, handler):
    """A weak alias left in place would compile silently and hang at the first interrupt."""
    symbols, _, blob = image
    entry = vector(blob, 16 + header_value(irq))
    assert entry & ~1 == symbols[handler] & ~1
    assert entry & ~1 != symbols["Default_Handler"] & ~1, f"{handler} was not linked over the weak default"


def test_unused_interrupts_fall_to_the_default(image):
    symbols, _, blob = image
    entry = vector(blob, 16 + header_value("TIM3_IRQn"))
    assert entry & ~1 == symbols["Default_Handler"] & ~1


def test_it_fits(image):
    _, sections, blob = image
    assert len(blob) < FLASH[1] * 0.5
    ram_used = sum(s["size"] for s in sections if s["name"] in (".data", ".bss"))
    assert ram_used < RAM[1] - 8 * 1024, f"{ram_used} bytes leave too little for the stack"


def test_no_host_code_leaked_in(image):
    symbols, _, _ = image
    assert "sim_advance" not in symbols
    assert "rac_core_tick" in symbols
    assert "tud_task_ext" in symbols


class TestAgainstTheBoard:
    """The firmware's pins and the schematic's nets are two statements of one fact."""

    def _mcu_pins(self):
        from design import build

        return next(p for s in build() for p in s.parts if p.lib.startswith("MCU_ST"))

    # LQFP-64 positions, from ST's STM32_open_pin_data.
    PIN_NUMBER = {
        "PA2": "14",
        "PA3": "17",
        "PB10": "30",
        "PB11": "33",
        "PB12": "34",
        "PB13": "35",
        "PA8": "42",
        "PA15": "51",
        "PA11": "45",
        "PA12": "46",
        "PC0": "8",
        "PC1": "9",
        "PC2": "10",
        "PC3": "11",
        "PC6": "38",
        "PC7": "39",
        "PC8": "40",
        "PC9": "41",
        "PB14": "36",
        "PB15": "37",
        "PC10": "52",
        "PC11": "53",
    }

    EXPECTED = {
        "PA2": "UART1_TX",
        "PA3": "UART1_RX",
        "PB10": "UART2_TX",
        "PB11": "UART2_RX",
        "PB12": "CAN1_RX",
        "PB13": "CAN1_TX",
        "PA8": "CAN2_RX",
        "PA15": "CAN2_TX",
        "PA11": "USB_DM",
        "PA12": "USB_DP",
        "PC0": "ESTOP_SENSE1",
        "PC1": "ESTOP_SENSE2",
        "PC2": "SAFETY_EN",
        "PC3": "STOP_SENSE",
        "PC6": "LED_CAN1",
        "PC7": "LED_CAN2",
        "PC8": "LED_UART1",
        "PC9": "LED_UART2",
        "PB14": "LED_RUN",
        "PB15": "LED_FAULT",
        "PC10": "SYNC_OUT",
        "PC11": "SYNC_IN",
    }

    def test_schematic_nets_match_the_pin_map(self):
        mcu = self._mcu_pins()
        for pin, net in self.EXPECTED.items():
            assert mcu.pins[self.PIN_NUMBER[pin]] == net, f"{pin}"

    def test_firmware_drives_the_pins_the_board_has(self):
        source = (ROOT / "firmware" / "port" / "stm32g4" / "port_g4.c").read_text("utf-8")
        board = (ROOT / "firmware" / "port" / "stm32g4" / "board.h").read_text("utf-8")
        # Alternate functions as ST's data lists them.
        for call in ("gpio_af(GPIOB, 12u, 9u)", "gpio_af(GPIOB, 13u, 9u)", "gpio_af(GPIOA, 8u, 11u)", "gpio_af(GPIOA, 15u, 11u)", "gpio_af(GPIOA, 2u, 7u)", "gpio_af(GPIOA, 3u, 7u)", "gpio_af(GPIOB, 10u, 7u)", "gpio_af(GPIOB, 11u, 7u)"):
            assert call in source, call
        for define, port, pin in (
            ("ESTOP_CH1", "GPIOC", 0),
            ("ESTOP_CH2", "GPIOC", 1),
            ("SAFETY_EN", "GPIOC", 2),
            ("STOP_SENSE", "GPIOC", 3),
            ("SYNC_OUT", "GPIOC", 10),
            ("SYNC_IN", "GPIOC", 11),
        ):
            assert re.search(rf"{define}_PORT {port}\b", board), define
            assert re.search(rf"{define}_PIN {pin}u", board), define

    def test_can_bit_timing_is_exactly_one_megabit(self):
        source = (ROOT / "firmware" / "port" / "stm32g4" / "port_g4.c").read_text("utf-8")
        brp = int(re.search(r"\((\d+)u << FDCAN_NBTP_NBRP_Pos\)", source).group(1)) + 1
        seg1 = int(re.search(r"\((\d+)u << FDCAN_NBTP_NTSEG1_Pos\)", source).group(1)) + 1
        seg2 = int(re.search(r"\((\d+)u << FDCAN_NBTP_NTSEG2_Pos\)", source).group(1)) + 1
        kernel_hz = 8_000_000 / 2 * 85 / 4  # HSE / PLLM * PLLN / PLLQ
        assert kernel_hz / brp / (1 + seg1 + seg2) == pytest.approx(1_000_000, abs=1)
        assert 0.75 <= (1 + seg1) / (1 + seg1 + seg2) <= 0.875

    def test_message_ram_offsets_match_the_header(self):
        """Hardware layout from RM0440: 212 words per instance, so 0x350 bytes apart."""
        assert 212 * 4 == 0x350
        board = (ROOT / "firmware" / "port" / "stm32g4" / "board.h").read_text("utf-8")
        assert "CAN1_RAM_OFFSET 0x350u" in board  # FDCAN2 is the second instance
        assert "CAN2_RAM_OFFSET 0x6A0u" in board  # FDCAN3 is the third
