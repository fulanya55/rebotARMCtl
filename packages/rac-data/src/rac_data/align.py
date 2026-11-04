"""Turn raw recordings into frames on one uniform timeline.

Everything is compared on the host clock: camera frames are stamped there when read,
and controller samples carry their original MCU time plus the host time the clock
model maps it to. The original values are kept too, in the episode's sidecar.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

JOINTS = 7


@dataclass
class ArmTrack:
    """All samples of one arm, ordered by time."""

    host_ts: np.ndarray
    mcu_ts: np.ndarray
    sample_seq: np.ndarray
    position: np.ndarray
    target: np.ndarray
    target_valid: np.ndarray
    exec_seq: np.ndarray
    source: np.ndarray
    valid_mask: np.ndarray

    @classmethod
    def from_samples(cls, samples: list[dict]) -> ArmTrack:
        # Controller order is the truth. Host times come through a clock model that
        # refits as it runs, so they can step backwards by a few microseconds; sorting
        # on them would scramble the sequence and report samples that were never lost.
        ordered = sorted(samples, key=lambda s: s["sample_seq"])
        mcu = np.array([s["sample_ts_us"] for s in ordered], dtype=np.int64)
        host = np.array([s["host_ts_us"] for s in ordered], dtype=np.int64)
        return cls(
            host_ts=_smooth_host_time(mcu, host),
            mcu_ts=mcu,
            sample_seq=np.array([s["sample_seq"] for s in ordered], dtype=np.int64),
            position=np.array([s["position"] for s in ordered], dtype=np.float32).reshape(-1, JOINTS),
            target=np.array([s["target"] for s in ordered], dtype=np.float32).reshape(-1, JOINTS),
            target_valid=np.array([bool(s["flags"] & 1) for s in ordered]),
            exec_seq=np.array([s["exec_seq"] for s in ordered], dtype=np.int64),
            source=np.array([s["source"] for s in ordered], dtype=np.int64),
            valid_mask=np.array([s["valid_mask"] for s in ordered], dtype=np.int64),
        )

    def __len__(self) -> int:
        return len(self.host_ts)


@dataclass
class CameraTrack:
    host_ts: np.ndarray
    capture_ts: np.ndarray
    frame_index: np.ndarray
    files: list[str]


@dataclass
class AlignedEpisode:
    fps: int
    frame_times_us: np.ndarray
    state: np.ndarray
    action: np.ndarray
    leader: np.ndarray
    exec_seq: np.ndarray
    control_source: np.ndarray
    intervention: np.ndarray
    state_valid: np.ndarray
    mcu_ts_us: np.ndarray
    camera_files: dict[str, list[str]]
    camera_frame_host_ts: dict[str, np.ndarray]
    quality: dict = field(default_factory=dict)


def _smooth_host_time(mcu: np.ndarray, host: np.ndarray) -> np.ndarray:
    """Fit one line from controller time to host time across the episode.

    Within an episode the controller clock is far steadier than any single clock
    exchange, so a single fit is more accurate than the per-sample values and is
    strictly increasing. The per-sample values stay in the sidecar.
    """
    if len(mcu) < 2 or mcu[-1] == mcu[0]:
        return host
    origin = mcu[0]
    slope, intercept = np.polyfit((mcu - origin).astype(np.float64), host.astype(np.float64), 1)
    fitted = np.rint(intercept + slope * (mcu - origin).astype(np.float64)).astype(np.int64)
    return np.maximum.accumulate(fitted)


def _interpolate(times: np.ndarray, source_t: np.ndarray, values: np.ndarray) -> np.ndarray:
    return np.stack([np.interp(times, source_t, values[:, j]) for j in range(values.shape[1])], axis=1).astype(np.float32)


def _hold_index(times: np.ndarray, source_t: np.ndarray) -> np.ndarray:
    """Index of the latest sample at or before each time."""
    return np.clip(np.searchsorted(source_t, times, side="right") - 1, 0, len(source_t) - 1)


def _rate_of(ts: np.ndarray) -> float:
    if len(ts) < 2:
        return 0.0
    span = (ts[-1] - ts[0]) / 1e6
    return float((len(ts) - 1) / span) if span > 0 else 0.0


def quality_of(track: ArmTrack | None, expected_hz: float | None = None) -> dict:
    if track is None or len(track) == 0:
        return {"samples": 0, "rate_hz": 0.0, "missing": 0, "invalid": 0}
    gaps = np.diff(track.sample_seq)
    return {
        "samples": len(track),
        "rate_hz": round(_rate_of(track.host_ts), 2),
        "missing": int(np.sum(np.maximum(gaps - 1, 0))) if len(gaps) else 0,
        "invalid": int(np.sum(track.valid_mask != 0x7F)),
        "expected_hz": expected_hz,
    }


def align_episode(
    fps: int,
    followers: list[ArmTrack],
    leaders: list[ArmTrack | None],
    cameras: dict[str, CameraTrack],
    intervention_spans_us: list[tuple[int, int]],
) -> AlignedEpisode:
    """Sample every source at ``fps`` on the span where all of them have data.

    Measured values interpolate, since they are continuous. Dispatched targets and
    labels hold the last value, since that is what was actually in force at that time.
    A camera whose nearest frame is further than 1.5 periods away is counted as a
    missed frame in the quality report instead of silently stretching time.
    """
    if not followers or any(len(t) == 0 for t in followers):
        raise ValueError("no follower samples were recorded")
    if not cameras or any(len(c.host_ts) == 0 for c in cameras.values()):
        raise ValueError("no camera frames were recorded")

    start = max(max(int(t.host_ts[0]) for t in followers), max(int(c.host_ts[0]) for c in cameras.values()))
    end = min(min(int(t.host_ts[-1]) for t in followers), min(int(c.host_ts[-1]) for c in cameras.values()))
    if end <= start:
        raise ValueError("the sources do not overlap in time")
    period_us = 1e6 / fps
    frame_times = (start + np.arange(int((end - start) / period_us) + 1) * period_us).astype(np.int64)
    count = len(frame_times)

    states, actions, leaders_out = [], [], []
    for index, follower in enumerate(followers):
        states.append(_interpolate(frame_times, follower.host_ts, follower.position))
        held = _hold_index(frame_times, follower.host_ts)
        # Before anything was dispatched the action is the pose itself, so the
        # dataset never contains a target that was not in force.
        action = follower.target[held].copy()
        no_target = ~follower.target_valid[held]
        never = np.maximum.accumulate(follower.target_valid[held]) == 0
        action[no_target & never] = states[-1][no_target & never]
        actions.append(action)
        leader = leaders[index] if index < len(leaders) else None
        if leader is not None and len(leader) > 0:
            leaders_out.append(_interpolate(frame_times, leader.host_ts, leader.position))
        else:
            leaders_out.append(np.zeros((count, JOINTS), dtype=np.float32))

    primary = followers[0]
    held = _hold_index(frame_times, primary.host_ts)
    all_valid = np.ones(count, dtype=bool)
    for follower in followers:
        h = _hold_index(frame_times, follower.host_ts)
        # A sample older than 20 periods is not evidence about this frame.
        fresh = np.abs(follower.host_ts[h] - frame_times) < 20 * period_us
        all_valid &= (follower.valid_mask[h] == 0x7F) & fresh

    intervention = np.zeros(count, dtype=np.int64)
    for span_start, span_end in intervention_spans_us:
        intervention[(frame_times >= span_start) & (frame_times <= span_end)] = 1

    camera_files: dict[str, list[str]] = {}
    camera_times: dict[str, np.ndarray] = {}
    camera_quality: dict[str, dict] = {}
    for name, track in cameras.items():
        positions = np.clip(np.searchsorted(track.host_ts, frame_times), 0, len(track.host_ts) - 1)
        before = np.clip(positions - 1, 0, len(track.host_ts) - 1)
        use_before = np.abs(track.host_ts[before] - frame_times) < np.abs(track.host_ts[positions] - frame_times)
        nearest = np.where(use_before, before, positions)
        distance = np.abs(track.host_ts[nearest] - frame_times)
        camera_files[name] = [track.files[i] for i in nearest]
        camera_times[name] = track.host_ts[nearest]
        indices = track.frame_index
        camera_quality[name] = {
            "frames": len(track.host_ts),
            "rate_hz": round(_rate_of(track.host_ts), 2),
            "dropped_by_camera": int(np.sum(np.maximum(np.diff(indices) - 1, 0))) if len(indices) > 1 else 0,
            "late_frames": int(np.sum(distance > 1.5 * period_us)),
            "max_offset_ms": round(float(distance.max()) / 1000.0, 2),
        }

    return AlignedEpisode(
        fps=fps,
        frame_times_us=frame_times,
        state=np.concatenate(states, axis=1),
        action=np.concatenate(actions, axis=1),
        leader=np.concatenate(leaders_out, axis=1),
        exec_seq=primary.exec_seq[held],
        control_source=primary.source[held],
        intervention=intervention,
        state_valid=all_valid.astype(np.int64),
        mcu_ts_us=primary.mcu_ts[held],
        camera_files=camera_files,
        camera_frame_host_ts=camera_times,
        quality={
            "frames": count,
            "duration_s": round((end - start) / 1e6, 3),
            "cameras": camera_quality,
            "followers": [quality_of(t) for t in followers],
            "leaders": [quality_of(t) for t in leaders],
            "invalid_frames": int(np.sum(~all_valid)),
        },
    )


def control_intervals(frame_times_us: np.ndarray, control_source: np.ndarray, intervention: np.ndarray) -> list[dict]:
    """Runs of one control source, the basis of automatic model and human labels."""
    if len(frame_times_us) == 0:
        return []
    label = control_source.astype(np.int64) * 2 + intervention.astype(np.int64)
    boundaries = np.flatnonzero(np.diff(label)) + 1
    starts = np.concatenate([[0], boundaries])
    ends = np.concatenate([boundaries, [len(label)]])
    origin = frame_times_us[0]
    return [
        {
            "start_frame": int(s),
            "end_frame": int(e - 1),
            "start_s": round(float(frame_times_us[s] - origin) / 1e6, 4),
            "end_s": round(float(frame_times_us[e - 1] - origin) / 1e6, 4),
            "source": int(control_source[s]),
            "intervention": bool(intervention[s]),
        }
        for s, e in zip(starts, ends, strict=True)
    ]
