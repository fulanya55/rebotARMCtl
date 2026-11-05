#!/usr/bin/env python3
"""Build the firmware targets.

    uv run python tools/build_firmware.py sil      host executable with simulated arms
    uv run python tools/build_firmware.py test     host unit tests, then run them
    uv run python tools/build_firmware.py stm32    STM32G474 image

Every target uses ``zig cc`` from the ``ziglang`` wheel, so nothing is needed beyond
``uv sync``. The STM32 target also needs tools/fetch_firmware_deps.py to have run.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "firmware"
BUILD = FW / "build"

CORE = ["rac_wire.c", "drv_robstride.c", "drv_fashionstar.c", "rac_core.c"]
SIL = ["sim_plant.c"]
WARN = ["-Wall", "-Wextra", "-Wshadow", "-Wno-unused-parameter", "-Wdouble-promotion"]


def exe_suffix() -> str:
    return ".exe" if os.name == "nt" else ""


def zig_cc() -> list[str]:
    return [sys.executable, "-m", "ziglang", "cc"]


def host_flags() -> list[str]:
    return ["-std=c11", "-O2", "-g", "-D_DEFAULT_SOURCE", *WARN]


def core_sources() -> list[Path]:
    return [FW / "core" / name for name in CORE]


def sil_sources() -> list[Path]:
    return [FW / "port" / "sil" / name for name in SIL]


def includes() -> list[str]:
    return [f"-I{FW / 'core'}", f"-I{FW / 'port' / 'sil'}"]


def run(cmd: list[str | Path]) -> None:
    subprocess.run([str(c) for c in cmd], check=True, cwd=ROOT)


def build_sil() -> Path:
    BUILD.mkdir(parents=True, exist_ok=True)
    out = BUILD / f"rac_sil{exe_suffix()}"
    libs = ["-lws2_32", "-lwinmm"] if os.name == "nt" else ["-lm"]
    run(
        [
            *zig_cc(),
            *host_flags(),
            *includes(),
            *core_sources(),
            *sil_sources(),
            FW / "port" / "sil" / "main_sil.c",
            "-o",
            out,
            *libs,
        ]
    )
    return out


def build_tests() -> Path:
    BUILD.mkdir(parents=True, exist_ok=True)
    out = BUILD / f"rac_tests{exe_suffix()}"
    libs = [] if os.name == "nt" else ["-lm"]
    run(
        [
            *zig_cc(),
            *host_flags(),
            *includes(),
            *core_sources(),
            *sil_sources(),
            FW / "tests" / "test_core.c",
            "-o",
            out,
            *libs,
        ]
    )
    return out


def build_stm32() -> Path:
    """Cross compile for the Cortex-M4F with the same Zig toolchain as the host builds."""
    port = FW / "port" / "stm32g4"
    tp = FW / "third_party"
    missing = [d for d in ("cmsis_core", "cmsis_device_g4", "tinyusb") if not (tp / d).exists()]
    if missing:
        raise SystemExit(f"missing {missing}. Run: uv run python tools/fetch_firmware_deps.py")

    out_dir = BUILD / "stm32g4"
    out_dir.mkdir(parents=True, exist_ok=True)

    tusb = tp / "tinyusb" / "src"
    ours = [*core_sources(), *sorted(port.glob("*.c"))]
    theirs = [
        tusb / "tusb.c",
        tusb / "common" / "tusb_fifo.c",
        tusb / "device" / "usbd.c",
        tusb / "device" / "usbd_control.c",
        tusb / "class" / "cdc" / "cdc_device.c",
        tusb / "portable" / "st" / "stm32_fsdev" / "dcd_stm32_fsdev.c",
    ]
    target = ["-target", "thumb-freestanding-eabihf", "-mcpu=cortex_m4+vfp4d16sp"]
    common = [
        *target,
        "-std=gnu11",  # TinyUSB's FSDEV driver uses the plain asm keyword
        "-O2",
        "-g",
        "-ffreestanding",
        "-fno-math-errno",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-sanitize=undefined",
        "-DSTM32G474xx",
        f"-I{FW / 'core'}",
        f"-I{port}",
        f"-I{port / 'libc'}",
        f"-I{tusb}",
        f"-I{tp / 'cmsis_core' / 'Include'}",
        f"-I{tp / 'cmsis_device_g4' / 'Include'}",
    ]
    objects: list[Path] = []
    for src in ours:
        obj = out_dir / (src.stem + ".o")
        run([*zig_cc(), *common, *WARN, "-c", src, "-o", obj])
        objects.append(obj)
    for src in theirs:
        obj = out_dir / (src.stem + ".o")
        run([*zig_cc(), *common, "-w", "-c", src, "-o", obj])
        objects.append(obj)

    elf = out_dir / "rac_controller.elf"
    run(
        [
            *zig_cc(),
            *target,
            "-nostdlib",
            f"-T{port / 'stm32g474.ld'}",
            "-Wl,--gc-sections",
            "-Wl,--entry=Reset_Handler",
            *objects,
            "-o",
            elf,
        ]
    )
    for flag, name in (("binary", "rac_controller.bin"), ("hex", "rac_controller.hex")):
        run([sys.executable, "-m", "ziglang", "objcopy", "-O", flag, elf, out_dir / name])
    return elf


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("target", choices=["sil", "test", "stm32"])
    args = parser.parse_args()
    if args.target == "sil":
        print(build_sil())
    elif args.target == "test":
        exe = build_tests()
        raise SystemExit(subprocess.run([str(exe)], cwd=ROOT).returncode)
    else:
        print(build_stm32())


if __name__ == "__main__":
    main()
