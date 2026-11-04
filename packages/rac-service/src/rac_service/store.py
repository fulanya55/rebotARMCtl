"""Persistent configuration: bindings, pairing and versioned calibrations.

Calibrations are keyed by the arm's serial, not by the port or logical role, so a
calibrated arm keeps its calibration when it moves to another port. Versions are
never overwritten: every recorded episode names the version it was taken with, and
that reference has to stay meaningful.
"""

from __future__ import annotations

import json
import os
import re
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from rac_link.ipc import runtime_dir
from rac_link.robots import ArmSpec

_SAFE_SERIAL = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")


def _atomic_write(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, path)


def check_serial(serial: str) -> str:
    if not _SAFE_SERIAL.match(serial):
        raise ValueError(f"arm serial may only use letters, digits, '.', '_' and '-': {serial!r}")
    return serial


@dataclass
class Calibration:
    serial: str
    model: str
    version: int = 0
    zero: list[float] = field(default_factory=lambda: [0.0] * 7)
    direction: list[float] = field(default_factory=lambda: [1.0] * 7)
    lower: list[float] = field(default_factory=lambda: [-3.14] * 7)
    upper: list[float] = field(default_factory=lambda: [3.14] * 7)
    gripper_closed: float = 0.0
    gripper_open: float = 1.0
    created_at: float = 0.0
    note: str = ""
    steps_done: list[str] = field(default_factory=list)

    @classmethod
    def default_for(cls, serial: str, spec: ArmSpec) -> Calibration:
        return cls(
            serial=serial,
            model=spec.key,
            lower=list(spec.lower),
            upper=list(spec.upper),
            gripper_closed=spec.gripper_raw_closed,
            gripper_open=spec.gripper_raw_open,
        )

    def to_dict(self) -> dict:
        return asdict(self)


class Store:
    def __init__(self, root: Path | None = None) -> None:
        self.root = root or runtime_dir()
        self.config_path = self.root / "config.json"
        self.config: dict = {"bindings": {}, "pairs": {}, "rates": {}, "cameras": {}, "bimanual": False}
        if self.config_path.exists():
            self.config.update(json.loads(self.config_path.read_text(encoding="utf-8")))

    def save_config(self) -> None:
        _atomic_write(self.config_path, self.config)

    def _cal_dir(self, serial: str) -> Path:
        return self.root / "calibration" / check_serial(serial)

    def list_versions(self, serial: str) -> list[int]:
        directory = self._cal_dir(serial)
        if not directory.exists():
            return []
        return sorted(int(p.stem[1:]) for p in directory.glob("v*.json"))

    def load_calibration(self, serial: str, version: int | None = None) -> Calibration | None:
        versions = self.list_versions(serial)
        if not versions:
            return None
        chosen = versions[-1] if version is None else version
        if chosen not in versions:
            raise ValueError(f"arm {serial} has no calibration version {chosen}")
        data = json.loads((self._cal_dir(serial) / f"v{chosen:04d}.json").read_text(encoding="utf-8"))
        return Calibration(**data)

    def save_calibration(self, calibration: Calibration) -> Calibration:
        versions = self.list_versions(calibration.serial)
        calibration.version = (versions[-1] if versions else 0) + 1
        calibration.created_at = time.time()
        _atomic_write(self._cal_dir(calibration.serial) / f"v{calibration.version:04d}.json", calibration.to_dict())
        return calibration

    def list_calibrations(self) -> list[dict]:
        base = self.root / "calibration"
        if not base.exists():
            return []
        out = []
        for directory in sorted(p for p in base.iterdir() if p.is_dir()):
            versions = self.list_versions(directory.name)
            latest = self.load_calibration(directory.name)
            out.append(
                {
                    "serial": directory.name,
                    "versions": versions,
                    "latest": latest.version if latest else None,
                    "model": latest.model if latest else None,
                    "created_at": latest.created_at if latest else None,
                    "note": latest.note if latest else "",
                }
            )
        return out
