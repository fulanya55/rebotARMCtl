"""Background conversion of raw episodes into LeRobot Dataset v3.

Encoding video takes a while, and recording must not wait for it, so saved episodes
queue here and a single worker writes them one at a time. The raw timeline is kept
next to the dataset as a sidecar, because the v3 frame table holds resampled values.
"""

from __future__ import annotations

import json
import logging
import queue
import shutil
import threading
import time
import traceback
import uuid
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from .align import ArmTrack, CameraTrack, align_episode, control_intervals
from .annotations import AnnotationStore
from .schema import build_features

log = logging.getLogger(__name__)

SIDECAR_DIR = "meta/rac"


@dataclass
class WriteJob:
    job_id: str
    directory: Path
    annotation: dict
    state: str = "queued"
    error: str | None = None
    episode_index: int | None = None
    quality: dict | None = None
    created: float = 0.0
    finished: float | None = None
    progress: float = 0.0


def _intervention_spans(events: list[dict], started_us: int, stopped_us: int) -> list[tuple[int, int]]:
    spans: list[tuple[int, int]] = []
    start: int | None = None
    for event in sorted(events, key=lambda e: e.get("host_ts_us", 0)):
        if event.get("kind") == "intervention_begin":
            start = event["host_ts_us"]
        elif event.get("kind") == "intervention_end":
            spans.append((start if start is not None else started_us, event["host_ts_us"]))
            start = None
    if start is not None:
        spans.append((start, stopped_us))
    return spans


def _write_sidecar(root: Path, episode_index: int, raw: dict, aligned) -> None:
    """Keep what resampling loses: every raw sample, event and original timestamp."""
    directory = root / SIDECAR_DIR / f"episode_{episode_index:06d}"
    directory.mkdir(parents=True, exist_ok=True)
    for arm, samples in raw["samples"].items():
        if not samples:
            continue
        table = pa.table(
            {
                "sample_seq": [s["sample_seq"] for s in samples],
                "mcu_ts_us": [s["sample_ts_us"] for s in samples],
                "host_ts_us": [s["host_ts_us"] for s in samples],
                "target_seq": [s["target_seq"] for s in samples],
                "target_ts_us": [s["target_ts_us"] for s in samples],
                "exec_seq": [s["exec_seq"] for s in samples],
                "valid_mask": [s["valid_mask"] for s in samples],
                "flags": [s["flags"] for s in samples],
                "mode": [s["mode"] for s in samples],
                "source": [s["source"] for s in samples],
                "position": [s["position"] for s in samples],
                "velocity": [s["velocity"] for s in samples],
                "torque": [s["torque"] for s in samples],
                "target": [s["target"] for s in samples],
            }
        )
        pq.write_table(table, directory / f"{arm}.parquet")
    for name, track in raw["cameras"].items():
        index = track["index"]
        if index:
            pq.write_table(
                pa.table(
                    {
                        "frame_index": [f["frame_index"] for f in index],
                        "host_ts_us": [f["host_ts_us"] for f in index],
                        "capture_ts_us": [f["capture_ts_us"] for f in index],
                    }
                ),
                directory / f"camera_{name}.parquet",
            )
    (directory / "events.jsonl").write_text(
        "\n".join(json.dumps(e, ensure_ascii=False) for e in raw["events"]), encoding="utf-8"
    )
    summary = {
        "started_host_us": raw["started_us"],
        "stopped_host_us": raw["stopped_us"],
        "quality": aligned.quality,
        "device": raw.get("status"),
        "recording_config": raw["config"],
        "time_sources": "state and action timestamps originate on the controller clock; host_ts_us maps them through the clock model; camera frames are stamped on the host clock when read",
    }
    (directory / "episode.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")


def open_or_create_dataset(root: Path, repo_id: str, fps: int, features: dict, robot_type: str):
    """Open a dataset for appending, or create it. The import is here because it pulls torch."""
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # H.264 plays in every browser, which the data browser relies on.
    encoder = RGBEncoderConfig(vcodec="h264")
    if (root / "meta" / "info.json").exists():
        info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
        existing = info.get("features", {})
        for key, spec in features.items():
            if key in existing and list(existing[key]["shape"]) != list(spec["shape"]):
                raise ValueError(f"{key} is {existing[key]['shape']} in this dataset but {spec['shape']} here")
        if set(features) != set(existing) - {"timestamp", "frame_index", "episode_index", "index", "task_index"}:
            raise ValueError("this dataset has different features; use a new dataset")
        if info.get("fps") != fps:
            raise ValueError(f"this dataset is {info.get('fps')} fps, recording is {fps}")
        return LeRobotDataset.resume(repo_id, root=root, rgb_encoder=encoder)
    return LeRobotDataset.create(repo_id, fps=fps, features=features, root=root, robot_type=robot_type, use_videos=True, rgb_encoder=encoder)


class DatasetWriter:
    def __init__(self) -> None:
        self.jobs: dict[str, WriteJob] = {}
        self._queue: queue.Queue[str] = queue.Queue()
        self._thread = threading.Thread(target=self._loop, name="rac-writer", daemon=True)
        self._thread.start()

    def enqueue(self, directory: Path, annotation: dict) -> str:
        job = WriteJob(uuid.uuid4().hex[:12], directory, annotation, created=time.time())
        self.jobs[job.job_id] = job
        self._queue.put(job.job_id)
        return job.job_id

    def status(self) -> dict:
        recent = sorted(self.jobs.values(), key=lambda j: j.created)[-10:]
        return {
            "pending": sum(1 for j in self.jobs.values() if j.state in ("queued", "writing")),
            "jobs": [
                {
                    "job": j.job_id,
                    "state": j.state,
                    "error": j.error,
                    "episode_index": j.episode_index,
                    "progress": round(j.progress, 3),
                    "quality": j.quality,
                }
                for j in recent
            ],
        }

    def wait(self, job_id: str, timeout: float = 300.0) -> WriteJob:
        deadline = time.monotonic() + timeout
        job = self.jobs[job_id]
        while job.state in ("queued", "writing"):
            if time.monotonic() > deadline:
                raise TimeoutError(f"job {job_id} is still {job.state}")
            time.sleep(0.05)
        return job

    def _loop(self) -> None:
        while True:
            job = self.jobs[self._queue.get()]
            job.state = "writing"
            try:
                self._write(job)
                job.state = "done"
                shutil.rmtree(job.directory, ignore_errors=True)
            except Exception as exc:
                # The raw episode stays on disk, so a failed encode loses nothing.
                job.state = "failed"
                job.error = f"{exc}"
                log.error("writing %s failed: %s\n%s", job.directory, exc, traceback.format_exc())
            job.finished = time.time()

    def _write(self, job: WriteJob) -> None:
        raw = json.loads((job.directory / "raw.json").read_text(encoding="utf-8"))
        config = raw["config"]
        fps = int(config["fps"])
        root = Path(config["root"])

        followers = []
        for arm in config["follower_arms"]:
            samples = raw["samples"].get(arm, [])
            if not samples:
                raise ValueError(f"nothing was recorded from {arm}")
            followers.append(ArmTrack.from_samples(samples))
        leaders = []
        for arm in config.get("leader_arms", []):
            samples = raw["samples"].get(arm, [])
            leaders.append(ArmTrack.from_samples(samples) if samples else None)

        cameras = {}
        for name, track in raw["cameras"].items():
            if track.get("error"):
                raise ValueError(track["error"])
            index = track["index"]
            cameras[name] = CameraTrack(
                host_ts=np.array([f["host_ts_us"] for f in index], dtype=np.int64),
                capture_ts=np.array([f["capture_ts_us"] for f in index], dtype=np.int64),
                frame_index=np.array([f["frame_index"] for f in index], dtype=np.int64),
                files=[str(job.directory / name / f["file"]) for f in index],
            )

        spans = _intervention_spans(raw["events"], raw["started_us"], raw["stopped_us"])
        aligned = align_episode(fps, followers, leaders, cameras, spans)
        job.quality = aligned.quality

        bimanual = len(config["follower_arms"]) == 2
        camera_shapes = {n: {"width": t["width"], "height": t["height"]} for n, t in raw["cameras"].items()}
        features = build_features(bimanual, camera_shapes)
        follower_models = [
            a.get("binding", {}).get("model", "unknown") for a in (raw.get("status") or {}).get("arms", []) if a["arm"] in config["follower_arms"]
        ]
        robot_type = "rac_" + ("bi_" if bimanual else "") + (follower_models[0] if follower_models else "arm")
        dataset = open_or_create_dataset(root, config["repo_id"], fps, features, robot_type)

        total = aligned.state.shape[0]
        for i in range(total):
            frame = {
                "task": config["task"],
                "observation.state": aligned.state[i],
                "action": aligned.action[i],
                "observation.leader_state": aligned.leader[i],
                "rac.exec_seq": np.array([aligned.exec_seq[i]], dtype=np.int64),
                "rac.control_source": np.array([aligned.control_source[i]], dtype=np.int64),
                "rac.intervention": np.array([aligned.intervention[i]], dtype=np.int64),
                "rac.state_valid": np.array([aligned.state_valid[i]], dtype=np.int64),
                "rac.mcu_ts_us": np.array([aligned.mcu_ts_us[i]], dtype=np.int64),
                "rac.host_ts_us": np.array([aligned.frame_times_us[i]], dtype=np.int64),
            }
            for name, files in aligned.camera_files.items():
                image = cv2.imread(files[i], cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f"could not read {files[i]}")
                frame[f"observation.images.{name}"] = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            dataset.add_frame(frame)
            job.progress = 0.8 * (i + 1) / total
        episode_index = dataset.meta.total_episodes
        dataset.save_episode()
        dataset.finalize()
        job.episode_index = episode_index
        job.progress = 0.95

        _write_sidecar(root, episode_index, raw, aligned)
        store = AnnotationStore(root)
        store.set_episode(
            episode_index,
            {
                "task": config["task"],
                "result": job.annotation.get("result", "unlabeled"),
                "note": job.annotation.get("note", ""),
                "calibration_versions": {
                    a["arm"]: a.get("calibration_version") for a in (raw.get("status") or {}).get("arms", []) if a.get("bound")
                },
                "model_config": config.get("model_config"),
                "control_intervals": control_intervals(aligned.frame_times_us, aligned.control_source, aligned.intervention),
                "quality": aligned.quality,
                "segments": [],
            },
        )
        job.progress = 1.0
