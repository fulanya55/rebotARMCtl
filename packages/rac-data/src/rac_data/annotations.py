"""Episode and segment annotations, kept beside the dataset and linked by episode index.

Annotations are never written into the sampled data, so reviewing and editing them
cannot change a recorded value.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

RESULTS = ("success", "failure", "incomplete", "unlabeled")
FILE = "meta/rac/annotations.json"


class AnnotationStore:
    def __init__(self, root: str | Path) -> None:
        self.path = Path(root) / FILE

    def load(self) -> dict:
        if not self.path.exists():
            return {"episodes": {}}
        return json.loads(self.path.read_text(encoding="utf-8"))

    def _save(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")
        os.replace(tmp, self.path)

    def get_episode(self, index: int) -> dict:
        return self.load()["episodes"].get(str(index), {"result": "unlabeled", "note": "", "segments": []})

    def set_episode(self, index: int, fields: dict) -> dict:
        data = self.load()
        episode = data["episodes"].setdefault(str(index), {"result": "unlabeled", "note": "", "segments": []})
        if "result" in fields and fields["result"] not in RESULTS:
            raise ValueError(f"result must be one of {RESULTS}")
        episode.update(fields)
        self._save(data)
        return episode

    def add_segment(self, index: int, start_s: float, end_s: float, label: str, note: str = "", kind: str = "phase") -> dict:
        if end_s < start_s:
            raise ValueError("a segment cannot end before it starts")
        if kind not in ("phase", "correction", "invalid"):
            raise ValueError("kind must be phase, correction or invalid")
        data = self.load()
        episode = data["episodes"].setdefault(str(index), {"result": "unlabeled", "note": "", "segments": []})
        segments = episode.setdefault("segments", [])
        segment = {"id": max((s["id"] for s in segments), default=0) + 1, "start_s": start_s, "end_s": end_s, "label": label, "note": note, "kind": kind}
        segments.append(segment)
        segments.sort(key=lambda s: s["start_s"])
        self._save(data)
        return segment

    def update_segment(self, index: int, segment_id: int, fields: dict) -> dict:
        data = self.load()
        for segment in data["episodes"].get(str(index), {}).get("segments", []):
            if segment["id"] == segment_id:
                segment.update({k: v for k, v in fields.items() if k in ("start_s", "end_s", "label", "note", "kind")})
                self._save(data)
                return segment
        raise KeyError(f"episode {index} has no segment {segment_id}")

    def delete_segment(self, index: int, segment_id: int) -> None:
        data = self.load()
        episode = data["episodes"].get(str(index), {})
        before = len(episode.get("segments", []))
        episode["segments"] = [s for s in episode.get("segments", []) if s["id"] != segment_id]
        if len(episode["segments"]) == before:
            raise KeyError(f"episode {index} has no segment {segment_id}")
        self._save(data)
