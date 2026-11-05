#!/usr/bin/env python3
"""Copy the real Star Arm 102 models from a checkout of servodevelop/Star-Arm-102.

Usage: python tools/import_star_urdf.py <path to Star-Arm-102>

HD and LD share one model upstream, so they share one here. Three things change
on the way in, and everything else is the published file:

- mesh paths become relative, the same way the reBot model does it;
- a tip frame is added, because the upstream models have no end-effector link;
- two joints are corrected to agree with what the arms physically do:

  joint5 on HD and LD is declared fixed, but servo 4 exists and turns. It becomes
  revolute about the axis FL declares for the same joint.

  joint4 on FL has its axis flipped against HD and LD. Fashion Star's own control
  script copies raw leader angles to FL unchanged on every joint, so the two have
  to agree. Flipping it makes the model match the raw servo frame.

In this frame a positive raw servo angle is a positive URDF angle on every joint,
which is what lets the 3D view and inverse kinematics take calibrated joints as is.
tests/test_kinematics.py checks the result against the mapping Seeed publishes.
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets" / "robots" / "star_arm_102"

# Tip frames in the link6 frame, from the farthest vertex of each finger mesh with the
# joints at zero: between the fingertips on FL, between the trigger ends on HD and LD.
VARIANTS = {
    "fl": {
        "source": "hardware/102-fl/urdf/stararm102-fl-description",
        "urdf": "stararm102_fl_description.urdf",
        "package": "stararm102_fl_description",
        "tip_xyz": "0.0013 0.0004 0",
    },
    "ld_hd": {
        "source": "hardware/102-ld/urdf/stararm102-ld-hd-description",
        "urdf": "stararm102_ld_hd_description.urdf",
        "package": "stararm102_ld_hd_description",
        "tip_xyz": "-0.0037 0.0768 -0.0046",
    },
}

TIP = """  <link name="gripper_tip"/>
  <joint name="gripper_tip_joint" type="fixed">
    <origin xyz="{xyz}" rpy="0 0 0"/>
    <parent link="link6"/>
    <child link="gripper_tip"/>
  </joint>
</robot>"""


def joint_block(text: str, name: str) -> re.Match[str]:
    match = re.search(rf'<joint\s+name="{name}".*?</joint>', text, flags=re.S)
    if not match:
        raise SystemExit(f"joint {name} not found: the upstream model changed, review this script")
    return match


def sub_in_joint(text: str, name: str, pattern: str, replacement: str) -> str:
    block = joint_block(text, name)
    changed, count = re.subn(pattern, replacement, block.group(0), count=1, flags=re.S)
    if count != 1:
        raise SystemExit(f"{pattern!r} not found in joint {name}: review this script")
    return text[: block.start()] + changed + text[block.end() :]


def convert(text: str, key: str, package: str, tip_xyz: str) -> str:
    text = text.replace(f"package://{package}/meshes/", "../meshes/")
    if key == "ld_hd":
        text = sub_in_joint(text, "joint5", r'type="fixed"', 'type="revolute"')
        text = sub_in_joint(text, "joint5", r'<axis\s+xyz="0 0 0"\s*/>', '<axis xyz="0 0 -1"/>')
    else:
        text = sub_in_joint(text, "joint4", r'<axis\s+xyz="0 0 1"\s*/>', '<axis xyz="0 0 -1"/>')
    head, closing, _ = text.rpartition("</robot>")
    if not closing:
        raise SystemExit("no closing robot tag")
    return head + TIP.format(xyz=tip_xyz) + "\n"


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    upstream = Path(sys.argv[1])
    for key, variant in VARIANTS.items():
        source = upstream / variant["source"]
        target = OUT / key
        (target / "urdf").mkdir(parents=True, exist_ok=True)
        (target / "meshes").mkdir(parents=True, exist_ok=True)
        for mesh in sorted((source / "meshes").iterdir()):
            shutil.copyfile(mesh, target / "meshes" / mesh.name)
        text = (source / "urdf" / variant["urdf"]).read_text(encoding="utf-8")
        output = target / "urdf" / f"star_arm_102_{key}.urdf"
        output.write_text(convert(text, key, variant["package"], variant["tip_xyz"]), encoding="utf-8", newline="\n")
        print(f"wrote {output.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
