"""Read side of a dataset: find, inspect, filter, export, merge. Never touches the arms."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from .annotations import AnnotationStore
from .schema import CONTROL_LABELS, definition_of


def read_info(root: str | Path) -> dict:
    return json.loads((Path(root) / "meta" / "info.json").read_text(encoding="utf-8"))


def is_dataset(path: Path) -> bool:
    return (path / "meta" / "info.json").exists()


def list_datasets(base: str | Path) -> list[dict]:
    """Datasets at or below ``base``, at most two levels down (owner/name)."""
    base = Path(base)
    if not base.exists():
        return []
    candidates = []
    if is_dataset(base):
        candidates.append(base)
    for child in sorted(p for p in base.iterdir() if p.is_dir() and not p.name.startswith(".")):
        if is_dataset(child):
            candidates.append(child)
        else:
            candidates.extend(sorted(g for g in child.iterdir() if g.is_dir() and is_dataset(g)))
    out = []
    for path in candidates:
        info = read_info(path)
        out.append(
            {
                "path": str(path),
                "name": path.name,
                "episodes": info.get("total_episodes", 0),
                "frames": info.get("total_frames", 0),
                "fps": info.get("fps"),
                "robot_type": info.get("robot_type"),
                "codebase_version": info.get("codebase_version"),
                "definition": definition_of(info),
            }
        )
    return out


def _episodes_table(root: Path) -> list[dict]:
    rows: list[dict] = []
    for file in sorted((root / "meta" / "episodes").rglob("*.parquet")):
        rows.extend(pq.read_table(file).to_pylist())
    rows.sort(key=lambda r: r["episode_index"])
    return rows


def _tasks(root: Path) -> dict[int, str]:
    path = root / "meta" / "tasks.parquet"
    if not path.exists():
        return {}
    table = pq.read_table(path)
    data = table.to_pydict()
    names = data.get("task") or data.get("__index_level_0__") or []
    return {int(i): str(n) for i, n in zip(data.get("task_index", []), names, strict=False)}


def list_episodes(root: str | Path, task: str | None = None, result: str | None = None, source: str | None = None) -> list[dict]:
    """Episodes with their annotations, optionally filtered by task, result or control source."""
    root = Path(root)
    annotations = AnnotationStore(root).load()["episodes"]
    out = []
    for row in _episodes_table(root):
        index = int(row["episode_index"])
        annotation = annotations.get(str(index), {})
        tasks = row.get("tasks") or []
        intervals = annotation.get("control_intervals", [])
        labels = {CONTROL_LABELS.get(i["source"], "none") for i in intervals} - {"none"}
        if any(i.get("intervention") for i in intervals):
            labels.add("intervention")
        sources = sorted(labels)
        item = {
            "episode_index": index,
            "length": int(row["length"]),
            "tasks": list(tasks),
            "result": annotation.get("result", "unlabeled"),
            "note": annotation.get("note", ""),
            "sources": sources,
            "control_intervals": intervals,
            "segments": annotation.get("segments", []),
            "quality": annotation.get("quality"),
            "calibration_versions": annotation.get("calibration_versions"),
            "excluded": annotation.get("excluded", False),
        }
        if task and task not in tasks:
            continue
        if result and item["result"] != result:
            continue
        if source and source not in sources:
            continue
        out.append(item)
    return out


def _episode_row(root: Path, episode_index: int) -> dict:
    for row in _episodes_table(root):
        if int(row["episode_index"]) == episode_index:
            return row
    raise KeyError(f"no episode {episode_index}")


def episode_data(root: str | Path, episode_index: int, stride: int = 1) -> dict:
    """State, action and timing of one episode, for synchronized plots."""
    root = Path(root)
    row = _episode_row(root, episode_index)
    info = read_info(root)
    file = root / info["data_path"].format(chunk_index=row["data/chunk_index"], file_index=row["data/file_index"])
    table = pq.read_table(file, filters=[("episode_index", "=", episode_index)])
    data = table.to_pydict()
    length = len(data["frame_index"])
    picks = list(range(0, length, max(1, stride)))
    pick = lambda key: [data[key][i] for i in picks]  # noqa: E731

    def optional(key: str) -> list:
        # Only this platform's recordings have these: a dataset from the Lab or an
        # imported one has plain state and action.
        if key not in data:
            return []
        return [v[0] if isinstance(v, list) and len(v) == 1 else v for v in pick(key)]

    videos = {}
    for key, spec in info["features"].items():
        if spec.get("dtype") != "video":
            continue
        chunk = row[f"videos/{key}/chunk_index"]
        index = row[f"videos/{key}/file_index"]
        videos[key.split(".", 2)[2]] = {
            "path": info["video_path"].format(video_key=key, chunk_index=chunk, file_index=index),
            "from_s": float(row[f"videos/{key}/from_timestamp"]),
            "to_s": float(row[f"videos/{key}/to_timestamp"]),
        }
    return {
        "episode_index": episode_index,
        "fps": info["fps"],
        "length": length,
        "joints": info["features"]["observation.state"]["names"],
        "timestamp": pick("timestamp"),
        "state": pick("observation.state"),
        "action": pick("action"),
        "leader": optional("observation.leader_state"),
        "control_source": optional("rac.control_source"),
        "intervention": optional("rac.intervention"),
        "state_valid": optional("rac.state_valid"),
        "videos": videos,
        "annotation": AnnotationStore(root).get_episode(episode_index),
    }


def trajectory_for_replay(root: str | Path, episode_index: int, column: str = "action") -> dict:
    """Points to replay on the real arm. The execution itself is the runtime's job."""
    data = episode_data(root, episode_index)
    if column not in ("action", "state"):
        raise ValueError("column must be action or state")
    points = np.asarray(data[column], dtype=float)
    if points.ndim != 2 or points.shape[1] not in (7, 14):
        raise ValueError(f"these arms replay 7 or 14 joints, and this dataset has {points.shape[-1]}")
    bimanual = points.shape[1] == 14
    return {
        "fps": data["fps"],
        "dt_s": 1.0 / data["fps"],
        "bimanual": bimanual,
        "left": points[:, :7].tolist(),
        "right": points[:, 7:].tolist() if bimanual else None,
        "length": len(points),
    }


def check_mergeable(roots: list[str | Path]) -> dict:
    """Datasets can merge only with the same joints, cameras, image sizes and rate."""
    if len(roots) < 2:
        return {"ok": False, "problems": ["choose at least two datasets"], "definition": None}
    definitions = [definition_of(read_info(r)) for r in roots]
    reference = definitions[0]
    problems = []
    for root, definition in zip(roots[1:], definitions[1:], strict=True):
        for key in ("fps", "joints", "cameras", "camera_shapes"):
            if definition[key] != reference[key]:
                problems.append(f"{Path(root).name}: {key} is {definition[key]}, expected {reference[key]}")
    return {"ok": not problems, "problems": problems, "definition": reference}


def merge_datasets(roots: list[str | Path], output: str | Path) -> dict:
    """Merge via LeRobot's aggregator after checking the definitions, carrying annotations across."""
    check = check_mergeable(roots)
    if not check["ok"]:
        raise ValueError("; ".join(check["problems"]))
    output = Path(output)
    if output.exists():
        raise ValueError(f"{output} already exists")
    from lerobot.datasets.aggregate import aggregate_datasets

    paths = [Path(r) for r in roots]
    aggregate_datasets(repo_ids=[p.name for p in paths], aggr_repo_id=output.name, roots=paths, aggr_root=output)

    merged = AnnotationStore(output)
    combined: dict = {"episodes": {}}
    offset = 0
    for path in paths:
        store = AnnotationStore(path).load()["episodes"]
        total = read_info(path).get("total_episodes", 0)
        for index, annotation in store.items():
            combined["episodes"][str(int(index) + offset)] = annotation
        sidecars = path / "meta" / "rac"
        if sidecars.exists():
            for sidecar in sidecars.glob("episode_*"):
                new_index = int(sidecar.name.split("_")[1]) + offset
                shutil.copytree(sidecar, output / "meta" / "rac" / f"episode_{new_index:06d}")
        offset += total
    merged._save(combined)
    return {"path": str(output), "episodes": read_info(output).get("total_episodes"), "frames": read_info(output).get("total_frames")}


def export_dataset(root: str | Path, destination: str | Path) -> dict:
    """Copy a dataset to another location, sidecars and annotations included."""
    root, destination = Path(root), Path(destination)
    if destination.exists():
        raise ValueError(f"{destination} already exists")
    ignore = shutil.ignore_patterns(".rac_raw", "images")
    shutil.copytree(root, destination, ignore=ignore)
    return {"path": str(destination)}


def import_dataset(source: str | Path, base: str | Path) -> dict:
    """Bring a dataset into the managed location, after checking it really is one."""
    source, base = Path(source), Path(base)
    if not is_dataset(source):
        raise ValueError(f"{source} is not a LeRobot dataset: no meta/info.json")
    info = read_info(source)
    if not str(info.get("codebase_version", "")).startswith("v3"):
        raise ValueError(f"{source} is {info.get('codebase_version')}, this platform uses v3")
    destination = base / source.name
    if destination.exists():
        raise ValueError(f"{destination} already exists")
    shutil.copytree(source, destination)
    return {"path": str(destination), "episodes": info.get("total_episodes")}
