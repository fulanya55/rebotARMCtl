#!/usr/bin/env python3
"""Fetch the firmware's source dependencies, pinned, into firmware/third_party.

These are source trees compiled into the image, not tools:

    cmsis_core        Arm CMSIS headers, Apache-2.0
    cmsis_device_g4   ST register definitions for the STM32G4, Apache-2.0
    tinyusb           USB device stack, MIT
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TP = ROOT / "firmware" / "third_party"

DEPS = {
    "cmsis_core": ("https://github.com/STMicroelectronics/cmsis_core.git", "v5.9.0"),
    "cmsis_device_g4": ("https://github.com/STMicroelectronics/cmsis_device_g4.git", "v1.2.6"),
    "tinyusb": ("https://github.com/hathach/tinyusb.git", "0.18.0"),
}


def main() -> None:
    TP.mkdir(parents=True, exist_ok=True)
    for name, (url, tag) in DEPS.items():
        target = TP / name
        if target.exists():
            print(f"{name}: present")
            continue
        subprocess.run(["git", "clone", "--depth", "1", "--branch", tag, "--quiet", url, str(target)], check=True)
        # Only the source is wanted. Dropping history also means no nested repository.
        shutil.rmtree(target / ".git", ignore_errors=True)
        print(f"{name}: {tag}")


if __name__ == "__main__":
    main()
