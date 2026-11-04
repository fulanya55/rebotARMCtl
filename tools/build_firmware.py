#!/usr/bin/env python3
"""Build the firmware targets.

    uv run python tools/build_firmware.py sil      host executable with simulated arms
    uv run python tools/build_firmware.py test     host unit tests, then run them
    uv run python tools/build_firmware.py stm32    STM32G474 image (needs arm-none-eabi-gcc)

Host builds use ``zig cc`` from the ``ziglang`` wheel, so they need nothing
beyond ``uv sync``. The Arm toolchain is looked up on PATH and then in .tools/.
"""

from __future__ import annotations

import argparse
import os
import shutil
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


def find_arm_gcc() -> str:
    found = shutil.which("arm-none-eabi-gcc")
    if found:
        return found
    for candidate in (ROOT / ".tools").glob("**/bin/arm-none-eabi-gcc*"):
        if candidate.suffix in ("", ".exe"):
            return str(candidate)
    raise SystemExit(
        "arm-none-eabi-gcc not found. Run: uv run python tools/fetch_firmware_deps.py --toolchain"
    )


def build_stm32(profile: str = "release") -> Path:
    gcc = find_arm_gcc()
    objcopy = str(Path(gcc).with_name(Path(gcc).name.replace("gcc", "objcopy")))
    size = str(Path(gcc).with_name(Path(gcc).name.replace("gcc", "size")))
    port = FW / "port" / "stm32g4"
    tp = FW / "third_party"
    missing = [d for d in ("cmsis_core", "cmsis_device_g4", "tinyusb") if not (tp / d).exists()]
    if missing:
        raise SystemExit(f"missing {missing}. Run: uv run python tools/fetch_firmware_deps.py")

    out_dir = BUILD / "stm32g4"
    out_dir.mkdir(parents=True, exist_ok=True)

    tusb = tp / "tinyusb" / "src"
    sources = [
        *core_sources(),
        *sorted(port.glob("*.c")),
        port / "startup_stm32g474.s",
        tp / "cmsis_device_g4" / "Source" / "Templates" / "system_stm32g4xx.c",
        tusb / "tusb.c",
        tusb / "common" / "tusb_fifo.c",
        tusb / "device" / "usbd.c",
        tusb / "device" / "usbd_control.c",
        tusb / "class" / "cdc" / "cdc_device.c",
        tusb / "portable" / "st" / "stm32_fsdev" / "dcd_stm32_fsdev.c",
    ]
    cpu = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    cflags = [
        *cpu,
        "-std=c11",
        "-O2",
        "-g3",
        "-ffunction-sections",
        "-fdata-sections",
        "-DSTM32G474xx",
        "-DCFG_TUSB_MCU=OPT_MCU_STM32G4",
        *WARN,
        f"-I{FW / 'core'}",
        f"-I{port}",
        f"-I{tusb}",
        f"-I{tp / 'cmsis_core' / 'Include'}",
        f"-I{tp / 'cmsis_device_g4' / 'Include'}",
    ]
    elf = out_dir / "rac_controller.elf"
    objects: list[Path] = []
    for src in sources:
        obj = out_dir / (src.stem + ".o")
        # Third-party sources are compiled with their warnings left alone.
        flags = [f for f in cflags if not (f.startswith("-W") and tp in src.parents)]
        run([gcc, *flags, "-c", src, "-o", obj])
        objects.append(obj)
    run(
        [
            gcc,
            *cpu,
            f"-T{port / 'stm32g474.ld'}",
            "--specs=nano.specs",
            "--specs=nosys.specs",
            "-Wl,--gc-sections",
            f"-Wl,-Map={out_dir / 'rac_controller.map'}",
            *objects,
            "-lm",
            "-o",
            elf,
        ]
    )
    run([objcopy, "-O", "binary", elf, out_dir / "rac_controller.bin"])
    run([objcopy, "-O", "ihex", elf, out_dir / "rac_controller.hex"])
    run([size, elf])
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
