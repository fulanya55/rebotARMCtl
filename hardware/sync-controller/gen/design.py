"""The sync controller board as data: one source for schematic, BOM and layout.

Pin numbers come from the KiCad libraries, which were checked against the vendor
datasheets where it matters (see docs/selection.md). Nets:

    GND, +3V3, +3.3VA, +5V        power symbols
    UPPERCASE_WITH_UNDERSCORE     global labels when listed in GLOBALS, across sheets
    anything else                 local to its sheet, which is what keeps each
                                  isolated port's ISO_GND, ISO_5V and so on separate
"""

from __future__ import annotations

from dataclasses import dataclass, field

PROJECT = "sync-controller"

R0603 = "Resistor_SMD:R_0603_1608Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
LED0603 = "LED_SMD:LED_0603_1608Metric"
SOD323 = "Diode_SMD:D_SOD-323"
L0805 = "Inductor_SMD:L_0805_2012Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
SOT235 = "Package_TO_SOT_SMD:SOT-23-5"
SOT236 = "Package_TO_SOT_SMD:SOT-23-6"
SOT89 = "Package_TO_SOT_SMD:SOT-89-3"
SOIC8 = "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm"
TACT = "Button_Switch_THT:SW_PUSH_6mm"
DCDC = "Converter_DCDC:Converter_DCDC_Murata_CRE1xxxxxxSC_THT"
TERM3 = "Connector_Phoenix_MC:PhoenixContact_MC_1,5_3-G-3.5_1x03_P3.50mm_Horizontal"
TERM4 = "Connector_Phoenix_MC:PhoenixContact_MC_1,5_4-G-3.5_1x04_P3.50mm_Horizontal"
PH3 = "Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical"
HDR2 = "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"
HDR3 = "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical"
HDR6 = "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical"

POWER_NETS = {"GND", "+3V3", "+3.3VA", "+5V"}

# Signals that cross sheets.
GLOBALS = {
    "USB_DP",
    "USB_DM",
    "CAN1_TX",
    "CAN1_RX",
    "CAN2_TX",
    "CAN2_RX",
    "UART1_TX",
    "UART1_RX",
    "UART2_TX",
    "UART2_RX",
    "ESTOP_SENSE1",
    "ESTOP_SENSE2",
    "SAFETY_EN",
    "STOP_SENSE",
    "SYNC_OUT",
    "SYNC_IN",
}


@dataclass
class Part:
    ref: str
    lib: str
    value: str
    footprint: str
    pins: dict[str, str]
    x: float
    y: float
    mpn: str = ""
    lcsc: str = ""
    dnp: bool = False
    no_connect: tuple[str, ...] = ()
    rot: int = 0


@dataclass
class Sheet:
    name: str
    title: str
    parts: list[Part] = field(default_factory=list)
    notes: list[tuple[float, float, str]] = field(default_factory=list)
    flags: list[tuple[float, float, str]] = field(default_factory=list)  # PWR_FLAG placements


class Refs:
    """Reference designators, unique across the board."""

    def __init__(self) -> None:
        self.counts: dict[str, int] = {}

    def __call__(self, prefix: str) -> str:
        self.counts[prefix] = self.counts.get(prefix, 0) + 1
        return f"{prefix}{self.counts[prefix]}"


def passive(ref, lib, value, footprint, a, b, x, y, **kw) -> Part:
    return Part(ref, lib, value, footprint, {"1": a, "2": b}, x, y, **kw)


def build() -> list[Sheet]:
    r = Refs()
    sheets = [power_sheet(r), mcu_sheet(r), safety_sheet(r)]
    sheets.append(can_sheet(r, 1))
    sheets.append(can_sheet(r, 2))
    sheets.append(uart_sheet(r, 1))
    sheets.append(uart_sheet(r, 2))
    return sheets


def power_sheet(r: Refs) -> Sheet:
    s = Sheet("power", "USB and power")
    s.notes.append((25.4, 25.4, "USB-C input. 5.1k on CC1 and CC2 asks the host for default USB power."))
    usb = {
        "A1": "GND",
        "A12": "GND",
        "B1": "GND",
        "B12": "GND",
        "A4": "VBUS_USB",
        "A9": "VBUS_USB",
        "B4": "VBUS_USB",
        "B9": "VBUS_USB",
        "A5": "CC1",
        "B5": "CC2",
        "A6": "USB_DP",
        "B6": "USB_DP",
        "A7": "USB_DM",
        "B7": "USB_DM",
        "SH": "USB_SHIELD",
    }
    s.parts.append(
        Part(
            r("J"),
            "Connector:USB_C_Receptacle_USB2.0_16P",
            "USB-C",
            "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
            usb,
            50.8,
            76.2,
            mpn="HRO TYPE-C-31-M-12",
            lcsc="C165948",
            no_connect=("A8", "B8"),
        )
    )
    s.parts.append(passive(r("R"), "Device:R", "5.1k", R0603, "CC1", "GND", 101.6, 53.34))
    s.parts.append(passive(r("R"), "Device:R", "5.1k", R0603, "CC2", "GND", 114.3, 53.34))
    s.parts.append(passive(r("R"), "Device:R", "1M", R0603, "USB_SHIELD", "GND", 127.0, 53.34))
    s.parts.append(passive(r("C"), "Device:C", "4.7nF", C0603, "USB_SHIELD", "GND", 139.7, 53.34))

    s.parts.append(
        Part(
            r("U"),
            "Power_Protection:USBLC6-2SC6",
            "USBLC6-2SC6",
            SOT236,
            {"1": "USB_DP", "6": "USB_DP", "3": "USB_DM", "4": "USB_DM", "2": "GND", "5": "VBUS_USB"},
            114.3,
            88.9,
            mpn="USBLC6-2SC6",
        )
    )

    s.notes.append((154.94, 25.4, "5 V path, then the 3.3 V regulator. Ferrite keeps converter noise off VBUS."))
    s.parts.append(passive(r("F"), "Device:Polyfuse", "750mA", "Fuse:Fuse_1206_3216Metric", "VBUS_USB", "VBUS_F", 165.1, 53.34, mpn="SMD1206 750mA hold"))
    s.parts.append(passive(r("FB"), "Device:FerriteBead_Small", "600R@100MHz 1A", L0805, "VBUS_F", "+5V", 177.8, 53.34))
    s.parts.append(passive(r("C"), "Device:C", "10uF", C0805, "+5V", "GND", 190.5, 53.34))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "+5V", "GND", 203.2, 53.34))

    s.parts.append(
        Part(
            r("U"),
            "Regulator_Linear:AP2112K-3.3",
            "AP2112K-3.3",
            SOT235,
            {"1": "+5V", "2": "GND", "3": "+5V", "5": "+3V3"},
            175.26,
            88.9,
            mpn="AP2112K-3.3TRG1",
            no_connect=("4",),
        )
    )
    s.parts.append(passive(r("C"), "Device:C", "1uF", C0603, "+5V", "GND", 152.4, 88.9))
    s.parts.append(passive(r("C"), "Device:C", "4.7uF", C0805, "+3V3", "GND", 198.12, 88.9))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "+3V3", "GND", 210.82, 88.9))

    s.notes.append((154.94, 111.76, "Filtered analog supply for the MCU."))
    s.parts.append(passive(r("FB"), "Device:FerriteBead_Small", "600R@100MHz", "Inductor_SMD:L_0603_1608Metric", "+3V3", "+3.3VA", 165.1, 129.54))
    s.parts.append(passive(r("C"), "Device:C", "1uF", C0603, "+3.3VA", "GND", 177.8, 129.54))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "+3.3VA", "GND", 190.5, 129.54))

    # Only nets no power output pin drives: +3V3 and the isolated rails have regulators.
    s.flags += [(215.9, 129.54, "GND"), (228.6, 129.54, "+5V"), (254.0, 129.54, "+3.3VA")]
    return s


def mcu_sheet(r: Refs) -> Sheet:
    s = Sheet("mcu", "STM32G474RET6")
    pins = {
        "16": "+3V3",
        "32": "+3V3",
        "48": "+3V3",
        "64": "+3V3",
        "1": "+3V3",
        "15": "GND",
        "31": "GND",
        "47": "GND",
        "63": "GND",
        "27": "GND",
        "28": "+3.3VA",
        "29": "+3.3VA",
        "7": "NRST",
        "5": "HSE_IN",
        "6": "HSE_OUT",
        "45": "USB_DM",
        "46": "USB_DP",
        "49": "SWDIO",
        "50": "SWCLK",
        "56": "SWO",
        "61": "BOOT0",
        "34": "CAN1_RX",
        "35": "CAN1_TX",
        "42": "CAN2_RX",
        "51": "CAN2_TX",
        "14": "UART1_TX",
        "17": "UART1_RX",
        "30": "UART2_TX",
        "33": "UART2_RX",
        "8": "ESTOP_SENSE1",
        "9": "ESTOP_SENSE2",
        "10": "SAFETY_EN",
        "11": "STOP_SENSE",
        "52": "SYNC_OUT",
        "53": "SYNC_IN",
        "38": "LED_CAN1",
        "39": "LED_CAN2",
        "40": "LED_UART1",
        "41": "LED_UART2",
        "36": "LED_RUN",
        "37": "LED_FAULT",
    }
    all_pins = {str(i) for i in range(1, 65)}
    unused = tuple(sorted(all_pins - set(pins), key=int))
    s.parts.append(
        Part(
            r("U"),
            "MCU_ST_STM32G4:STM32G474RETx",
            "STM32G474RET6",
            "Package_QFP:LQFP-64_10x10mm_P0.5mm",
            pins,
            101.6,
            101.6,
            mpn="STM32G474RET6",
            no_connect=unused,
        )
    )

    # Decoupling: one 100 nF per VDD pin, kept near the pin in layout.
    x = 165.1
    for _ in range(4):
        s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "+3V3", "GND", x, 35.56))
        x += 10.16
    s.parts.append(passive(r("C"), "Device:C", "4.7uF", C0805, "+3V3", "GND", x, 35.56))
    x += 10.16
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "+3V3", "GND", x, 35.56))  # VBAT
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "+3.3VA", "GND", 165.1, 60.96))

    s.notes.append((165.1, 76.2, "8 MHz crystal, CL 12 pF: C = 2 x (CL - 3 pF stray) = 18 pF."))
    s.parts.append(
        Part(
            r("Y"),
            "Device:Crystal_GND24",
            "8MHz CL12pF",
            "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
            {"1": "HSE_IN", "3": "HSE_OUT", "2": "GND", "4": "GND"},
            177.8,
            91.44,
            mpn="8 MHz 3225 12 pF",
        )
    )
    s.parts.append(passive(r("C"), "Device:C", "18pF", C0603, "HSE_IN", "GND", 165.1, 101.6))
    s.parts.append(passive(r("C"), "Device:C", "18pF", C0603, "HSE_OUT", "GND", 190.5, 101.6))

    s.notes.append((165.1, 116.84, "BOOT: hold while resetting to enter the ROM USB DFU loader."))
    s.parts.append(passive(r("R"), "Device:R", "10k", R0603, "BOOT0", "GND", 165.1, 134.62))
    s.parts.append(passive(r("SW"), "Switch:SW_Push", "BOOT", TACT, "BOOT0", "+3V3", 180.34, 134.62))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "NRST", "GND", 203.2, 134.62))
    s.parts.append(passive(r("SW"), "Switch:SW_Push", "RESET", TACT, "NRST", "GND", 218.44, 134.62))

    s.notes.append((25.4, 25.4, "Debug: SWD plus SWO."))
    s.parts.append(
        Part(
            r("J"),
            "Connector_Generic:Conn_01x06",
            "SWD",
            HDR6,
            {"1": "+3V3", "2": "SWCLK", "3": "GND", "4": "SWDIO", "5": "NRST", "6": "SWO"},
            30.48,
            50.8,
        )
    )

    s.notes.append((25.4, 91.44, "Indicators, active high."))
    y = 96.52
    for net, color in (
        ("LED_CAN1", "green"),
        ("LED_CAN2", "green"),
        ("LED_UART1", "green"),
        ("LED_UART2", "green"),
        ("LED_RUN", "blue"),
        ("LED_FAULT", "red"),
    ):
        led_net = f"{net}_A"
        s.parts.append(Part(r("R"), "Device:R", "1k", R0603, {"1": net, "2": led_net}, 30.48, y, rot=90))
        s.parts.append(Part(r("D"), "Device:LED", color, LED0603, {"1": "GND", "2": led_net}, 50.8, y, rot=0))
        y += 15.24
    return s


def safety_sheet(r: Refs) -> Sheet:
    s = Sheet("safety", "Emergency stop and sync")
    s.notes.append((25.4, 20.32, "E-stop: fails safe without firmware. Contact 1 is in series with the relay coil, so"))
    s.notes.append((25.4, 25.4, "pressing it drops the relay whatever the MCU does. Contact 2 is read back; a difference"))
    s.notes.append((25.4, 30.48, "between the two is a wiring fault. A broken wire looks the same as pressing the button."))
    s.notes.append((25.4, 35.56, "Relay contacts drive an external contactor coil or a supply's remote enable. Not rated for 48 V motor current."))
    s.parts.append(
        Part(
            r("J"),
            "Connector_Generic:Conn_01x04",
            "ESTOP",
            TERM4,
            {"1": "+5V", "2": "CHAIN1", "3": "SENSE2_IN", "4": "GND"},
            33.02,
            63.5,
        )
    )
    s.parts.append(passive(r("D"), "Device:D_TVS", "PESD5V0", SOD323, "CHAIN1", "GND", 58.42, 55.88))
    s.parts.append(passive(r("D"), "Device:D_TVS", "PESD5V0", SOD323, "SENSE2_IN", "GND", 76.2, 55.88))

    # Coil, flyback, low side switch.
    # Contacts: the normally closed ones are unused, so a dropped relay opens both outputs.
    relay = {"1": "CHAIN1", "16": "COIL_LOW", "4": "K_A_COM", "8": "K_A_NO", "13": "K_B_COM", "9": "K_B_NO"}
    s.parts.append(
        Part(r("K"), "Relay:G5V-2", "G5V-2-DC5", "Relay_THT:Relay_DPDT_Omron_G5V-2", relay, 127.0, 71.12, mpn="Omron G5V-2 DC5", no_connect=("6", "11"))
    )
    s.parts.append(passive(r("D"), "Device:D", "1N4148W", SOD323, "CHAIN1", "COIL_LOW", 96.52, 66.04))
    s.parts.append(passive(r("R"), "Device:R", "1k", R0603, "CHAIN1", "RELAY_LED", 96.52, 91.44))
    s.parts.append(Part(r("D"), "Device:LED", "green", LED0603, {"1": "COIL_LOW", "2": "RELAY_LED"}, 111.76, 91.44))
    s.parts.append(Part(r("Q"), "Transistor_FET:2N7002", "2N7002", SOT23, {"1": "SAFETY_GATE", "2": "GND", "3": "COIL_LOW"}, 96.52, 116.84, mpn="2N7002"))
    s.parts.append(passive(r("R"), "Device:R", "1k", R0603, "SAFETY_EN", "SAFETY_GATE", 73.66, 116.84))
    s.parts.append(passive(r("R"), "Device:R", "100k", R0603, "SAFETY_GATE", "GND", 60.96, 116.84))
    s.parts.append(passive(r("SW"), "Switch:SW_Push", "STOP", TACT, "SAFETY_GATE", "GND", 48.26, 116.84))
    s.parts.append(passive(r("R"), "Device:R", "10k", R0603, "SAFETY_GATE", "STOP_SENSE", 35.56, 116.84))

    s.parts.append(
        Part(
            r("J"),
            "Connector_Generic:Conn_01x04",
            "RELAY",
            TERM4,
            {"1": "K_A_COM", "2": "K_A_NO", "3": "K_B_COM", "4": "K_B_NO"},
            175.26,
            71.12,
        )
    )

    # Sense channels.
    s.parts.append(passive(r("R"), "Device:R", "10k", R0603, "CHAIN1", "ESTOP_SENSE1", 35.56, 149.86))
    s.parts.append(passive(r("R"), "Device:R", "20k", R0603, "ESTOP_SENSE1", "GND", 48.26, 149.86))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "ESTOP_SENSE1", "GND", 60.96, 149.86))
    s.parts.append(passive(r("R"), "Device:R", "10k", R0603, "+3V3", "ESTOP_SENSE2", 81.28, 149.86))
    s.parts.append(passive(r("R"), "Device:R", "1k", R0603, "ESTOP_SENSE2", "SENSE2_IN", 93.98, 149.86))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "ESTOP_SENSE2", "GND", 106.68, 149.86))

    # Camera sync.
    s.notes.append((142.24, 134.62, "Camera sync. PC10 and PC11 are 5 V tolerant."))
    s.parts.append(
        Part(
            r("J"),
            "Connector_Generic:Conn_01x03",
            "SYNC",
            HDR3,
            {"1": "GND", "2": "SYNC_OUT_C", "3": "SYNC_IN_C"},
            147.32,
            152.4,
        )
    )
    s.parts.append(passive(r("R"), "Device:R", "100", R0603, "SYNC_OUT", "SYNC_OUT_C", 170.18, 149.86))
    s.parts.append(passive(r("R"), "Device:R", "1k", R0603, "SYNC_IN", "SYNC_IN_C", 182.88, 149.86))
    s.parts.append(passive(r("R"), "Device:R", "100k", R0603, "SYNC_IN", "GND", 195.58, 149.86))
    return s


def _iso_power(s: Sheet, r: Refs, x0: float, y0: float) -> None:
    s.notes.append((x0, y0 - 25.4, "Isolated supply: B0509S (9 V, unregulated) then a 5 V regulator, because a B0505S"))
    s.notes.append((x0, y0 - 20.32, "can exceed the 5.5 V transceiver limit at light load. 2.2k keeps its 12 mA minimum load."))
    s.parts.append(
        Part(
            r("U"),
            "Converter_DCDC_Isolated:CRE1S0505SC",
            "B0509S-1WR3",
            DCDC,
            {"1": "GND", "2": "+5V", "3": "ISO_GND", "4": "ISO_9V"},
            x0 + 38.1,
            y0,
            mpn="Mornsun B0509S-1WR3",
        )
    )
    s.parts.append(passive(r("C"), "Device:C", "4.7uF", C0805, "+5V", "GND", x0 + 7.62, y0))
    s.parts.append(passive(r("C"), "Device:C", "10uF 25V", C0805, "ISO_9V", "ISO_GND", x0 + 66.04, y0))
    s.parts.append(passive(r("R"), "Device:R", "2.2k", R0603, "ISO_9V", "ISO_GND", x0 + 78.74, y0))
    s.parts.append(
        Part(
            r("U"),
            "Regulator_Linear:HT75xx-1-SOT89",
            "HT7550-1",
            SOT89,
            {"1": "ISO_GND", "2": "ISO_9V", "3": "ISO_5V"},
            x0 + 111.76,
            y0,
            mpn="HT7550-1",
        )
    )
    s.parts.append(passive(r("C"), "Device:C", "4.7uF", C0805, "ISO_5V", "ISO_GND", x0 + 137.16, y0))


def can_sheet(r: Refs, port: int) -> Sheet:
    s = Sheet(f"port_can{port}", f"CAN{port} isolated port")
    _iso_power(s, r, 25.4, 55.88)

    s.notes.append((25.4, 91.44, "Termination: fit the jumper at the end of the bus. Bus TVS footprint not fitted:"))
    s.notes.append((25.4, 96.52, "the transceiver survives +/-58 V, and a 24 V TVS would fail first on a 48 V short."))
    s.parts.append(
        Part(
            r("U"),
            "Interface_CAN_LIN:ISO1044BD",
            "ISO1044BD",
            SOIC8,
            {"1": "+3V3", "2": f"CAN{port}_TX", "3": f"CAN{port}_RX", "4": "GND", "5": "CANL", "6": "CANH", "7": "ISO_GND", "8": "ISO_5V"},
            76.2,
            124.46,
            mpn="ISO1044BD",
            lcsc="C2878198",
        )
    )
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "+3V3", "GND", 40.64, 124.46))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "ISO_5V", "ISO_GND", 106.68, 124.46))
    s.parts.append(passive(r("R"), "Device:R", "120", R0603, "CANH", "TERM", 127.0, 124.46))
    s.parts.append(Part(r("J"), "Connector_Generic:Conn_01x02", "TERM", HDR2, {"1": "TERM", "2": "CANL"}, 147.32, 124.46))
    s.parts.append(
        Part(
            r("D"),
            "Power_Protection:NUP2105L",
            "NUP2105L",
            "Package_TO_SOT_SMD:SOT-23",
            {"1": "CANH", "2": "CANL", "3": "ISO_GND"},
            175.26,
            124.46,
            dnp=True,
        )
    )
    s.parts.append(
        Part(
            r("J"),
            "Connector_Generic:Conn_01x03",
            f"CAN{port}",
            TERM3,
            {"1": "CANH", "2": "CANL", "3": "ISO_GND"},
            203.2,
            124.46,
        )
    )
    return s


def uart_sheet(r: Refs, port: int) -> Sheet:
    s = Sheet(f"port_uart{port}", f"UART{port} isolated servo port")
    _iso_power(s, r, 25.4, 55.88)
    s.parts.append(
        Part(
            r("U"),
            "Regulator_Linear:HT75xx-1-SOT89",
            "HT7533-1",
            SOT89,
            {"1": "ISO_GND", "2": "ISO_5V", "3": "ISO_3V3"},
            229.87,
            55.88,
            mpn="HT7533-1",
        )
    )
    s.parts.append(passive(r("C"), "Device:C", "4.7uF", C0805, "ISO_3V3", "ISO_GND", 251.46, 55.88))

    s.notes.append((25.4, 91.44, "Open drain bus: the pull-up cannot fight a servo driving the line, so no direction signal exists."))
    s.notes.append((25.4, 96.52, "Firmware disables its receiver while sending. Fit R to choose 3.3 V or 5 V logic, as on a UC-01."))
    s.notes.append((25.4, 101.6, "Pin 2 of the servo connector carries the arm's 12 V and stays unconnected here."))
    s.parts.append(
        Part(
            r("U"),
            "Isolator:ADuM1201BR",
            "ADuM1201BRZ",
            SOIC8,
            {
                "1": "+3V3",
                "2": f"UART{port}_RX",
                "3": f"UART{port}_TX",
                "4": "GND",
                "5": "ISO_GND",
                "6": "TXI",
                "7": "RXI",
                "8": "ISO_3V3",
            },
            76.2,
            129.54,
            mpn="ADuM1201BRZ",
        )
    )
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "+3V3", "GND", 40.64, 129.54))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "ISO_3V3", "ISO_GND", 106.68, 129.54))
    s.parts.append(
        Part(
            r("U"),
            "74xGxx:74LVC1G07",
            "74LVC1G07",
            SOT235,
            {"2": "TXI", "3": "ISO_GND", "4": "BUS", "5": "ISO_3V3"},
            134.62,
            111.76,
            mpn="SN74LVC1G07DBVR",
            no_connect=("1",),
        )
    )
    s.parts.append(
        Part(
            r("U"),
            "74xGxx:74LVC1G17",
            "74LVC1G17",
            SOT235,
            {"2": "BUS", "3": "ISO_GND", "4": "RXI", "5": "ISO_3V3"},
            134.62,
            144.78,
            mpn="SN74LVC1G17DBVR",
            no_connect=("1",),
        )
    )
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "ISO_3V3", "ISO_GND", 160.02, 111.76))
    s.parts.append(passive(r("C"), "Device:C", "100nF", C0603, "ISO_3V3", "ISO_GND", 160.02, 144.78))
    s.parts.append(passive(r("R"), "Device:R", "680", R0603, "BUS", "PULLUP", 175.26, 111.76))
    s.parts.append(passive(r("R"), "Device:R", "0", R0603, "ISO_3V3", "PULLUP", 187.96, 111.76))
    s.parts.append(passive(r("R"), "Device:R", "0", R0603, "ISO_5V", "PULLUP", 200.66, 111.76, dnp=True))
    s.parts.append(passive(r("R"), "Device:R", "100", R0603, "BUS", "DATA", 175.26, 144.78))
    s.parts.append(passive(r("D"), "Device:D_TVS", "PESD5V0", SOD323, "DATA", "ISO_GND", 187.96, 144.78))
    s.parts.append(
        Part(
            r("J"),
            "Connector_Generic:Conn_01x03",
            f"UART{port}",
            PH3,
            {"1": "DATA", "3": "ISO_GND"},
            210.82,
            144.78,
            no_connect=("2",),
        )
    )
    return s


def all_parts() -> list[Part]:
    return [p for sheet in build() for p in sheet.parts]
