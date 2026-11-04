"""The dataset contract: LeRobot Dataset v3 with a fixed joint order.

    observation.state        follower measured feedback
    action                   target the controller confirmed dispatched to the follower
    observation.leader_state leader measured feedback, stored separately
    observation.images.*     cameras

Joints are radians and the gripper is 0..1. A bimanual vector is left then right.
"""

from __future__ import annotations

from rac_link.robots import dataset_joint_names

CONTROL_NONE = 0
CONTROL_LABELS = {0: "none", 1: "teleop", 2: "teach", 3: "policy", 4: "intervention", 5: "replay", 6: "home", 7: "guide"}


def build_features(bimanual: bool, cameras: dict[str, dict]) -> dict:
    names = dataset_joint_names(bimanual)
    size = len(names)
    features: dict = {
        "observation.state": {"dtype": "float32", "shape": (size,), "names": names},
        "action": {"dtype": "float32", "shape": (size,), "names": names},
        "observation.leader_state": {"dtype": "float32", "shape": (size,), "names": names},
        # The bookkeeping that makes each frame traceable to the raw timeline.
        "rac.exec_seq": {"dtype": "int64", "shape": (1,), "names": None},
        "rac.control_source": {"dtype": "int64", "shape": (1,), "names": None},
        "rac.intervention": {"dtype": "int64", "shape": (1,), "names": None},
        "rac.state_valid": {"dtype": "int64", "shape": (1,), "names": None},
        "rac.mcu_ts_us": {"dtype": "int64", "shape": (1,), "names": None},
        "rac.host_ts_us": {"dtype": "int64", "shape": (1,), "names": None},
    }
    for camera, info in cameras.items():
        features[f"observation.images.{camera}"] = {
            "dtype": "video",
            "shape": (int(info["height"]), int(info["width"]), 3),
            "names": ["height", "width", "channels"],
        }
    return features


def definition_of(info: dict) -> dict:
    """The part of a dataset's info.json that decides whether two datasets can merge."""
    features = info.get("features", {})
    return {
        "fps": info.get("fps"),
        "robot_type": info.get("robot_type"),
        "joints": features.get("observation.state", {}).get("names"),
        "cameras": sorted(k.split(".", 2)[2] for k in features if k.startswith("observation.images.")),
        "camera_shapes": {
            k.split(".", 2)[2]: list(v.get("shape", [])) for k, v in features.items() if k.startswith("observation.images.")
        },
    }
