"""Constants that belong to the host's use of the protocol, not to the wire."""

from ._gen import MAX_CHUNK_POINTS

CHUNK_POINTS = MAX_CHUNK_POINTS

SOURCE_NAMES = {
    "none": 0,
    "teleop": 1,
    "teach": 2,
    "policy": 3,
    "intervention": 4,
    "replay": 5,
    "home": 6,
    "guide": 7,
}
SOURCE_BY_VALUE = {value: name for name, value in SOURCE_NAMES.items()}
