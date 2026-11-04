# Copyright 2026 fulanya.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""HTTP and WebSocket surface of the sync controller pages.

Everything that moves an arm is forwarded to the Controller Service. This module
adds the parts that belong in the web backend: kinematics for 3D teaching, teach
programs, the recorder, camera previews and the offline data browser. The web
state shown to users always comes from controller feedback relayed by the service.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
from fastapi import APIRouter, Depends, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel, Field

log = logging.getLogger(__name__)



def is_local(host: str | None) -> bool:
    return host in ("127.0.0.1", "::1", "localhost", "testclient")


def local_only(request: Request) -> None:
    """These routes move arms and the Lab has no user accounts.

    The Lab can be bound to a LAN address for its other features, so the controller
    surface answers only the local machine unless that is explicitly turned off.
    """
    if os.environ.get("RAC_ALLOW_REMOTE") == "1":
        return
    if not is_local(request.client.host if request.client else None):
        raise HTTPException(403, detail={"code": "auth.local_only", "message": "controller routes only answer this computer; set RAC_ALLOW_REMOTE=1 to change that"})


router = APIRouter(prefix="/controller", tags=["controller"], dependencies=[Depends(local_only)])

# A WebSocket cannot take the HTTP dependency above, so it checks for itself.
ws_router = APIRouter(prefix="/controller", tags=["controller"])


class Bridge:
    """Lazily connects to the service, so the Lab works without a controller installed."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.client = None
        self.recorder = None
        self.writer = None
        self.camera_process: subprocess.Popen | None = None
        self.camera_config: dict = {}
        self.latest_status: dict | None = None
        self.latest_samples: dict[str, dict] = {}
        self.events: list[dict] = []
        self.event_total = 0
        self.chains: dict[str, Any] = {}

    def get_client(self):
        with self.lock:
            if self.client is not None:
                return self.client
            from rac_link.client import ControllerClient

            try:
                client = ControllerClient()
            except (OSError, ValueError, KeyError) as exc:
                raise HTTPException(
                    503,
                    detail={"code": "service.unavailable", "message": f"the Controller Service is not running: {exc}"},
                ) from exc
            client.on_status(self._on_status)
            client.on_event(self._on_event)
            client.on_state(self._on_state)
            self.client = client
            return client

    def _on_status(self, status: dict) -> None:
        self.latest_status = status

    def _on_state(self, batch: list[dict]) -> None:
        for sample in batch:
            self.latest_samples[sample["arm"]] = sample

    def _on_event(self, event: dict) -> None:
        self.events.append(event)
        self.event_total += 1
        del self.events[:-200]

    def get_recorder(self):
        with self.lock:
            if self.recorder is None:
                from rac_data.recorder import Recorder
                from rac_data.writer import DatasetWriter

                self.writer = DatasetWriter()
                self.recorder = Recorder(self.get_client(), self.writer)
            return self.recorder

    def chain(self, model_key: str):
        chain = self.chains.get(model_key)
        if chain is None:
            from rac_kinematics import load_chain
            from rac_link.robots import SPEC_BY_KEY

            spec = SPEC_BY_KEY[model_key]
            chain = load_chain(spec.urdf, spec.end_effector)
            self.chains[model_key] = chain
        return chain


bridge = Bridge()


def call(method: str, **params: Any) -> Any:
    """Forward to the service, mapping its refusals onto HTTP errors with their code."""
    from rac_link.ipc import ServiceError

    client = bridge.get_client()
    try:
        return client.call(method, **params)
    except ServiceError as exc:
        status = 409 if exc.code.startswith(("control.", "safety.")) else 400
        raise HTTPException(status, detail={"code": exc.code, "message": str(exc)}) from exc
    except (ConnectionError, TimeoutError) as exc:
        bridge.client = None
        raise HTTPException(503, detail={"code": "service.unavailable", "message": str(exc)}) from exc


def safe_call(fn, *args: Any, **kwargs: Any) -> Any:
    try:
        return fn(*args, **kwargs)
    except (ValueError, RuntimeError, KeyError, FileNotFoundError, FileExistsError) as exc:
        raise HTTPException(400, detail={"code": "data.invalid", "message": str(exc)}) from exc


# ------------------------------------------------------------------ 1.1 device


class ConnectBody(BaseModel):
    uri: str


class BindBody(BaseModel):
    bindings: list[dict]


class RatesBody(BaseModel):
    motor_service_us: list[int] | None = None
    state_read_us: list[int] | None = None
    upstream_us: int | None = None
    host_timeout_ms: int | None = None


@router.get("/status")
def controller_status():
    return call("device.status")


@router.get("/candidates")
def controller_candidates():
    return call("device.candidates")


@router.get("/models")
def controller_models():
    return call("device.models")


@router.post("/connect")
def controller_connect(body: ConnectBody):
    return call("device.connect", uri=body.uri, timeout=10.0)


@router.post("/disconnect")
def controller_disconnect():
    return call("device.disconnect")


@router.post("/discover")
def controller_discover():
    return call("device.discover", timeout=5.0)


@router.post("/bind")
def controller_bind(body: BindBody):
    return call("device.bind", bindings=body.bindings)


@router.post("/rates")
def controller_rates(body: RatesBody):
    return call("device.set_rates", **body.model_dump(exclude_none=True))


# ----------------------------------------------------------------- 1.1 cameras


class CamerasBody(BaseModel):
    cameras: dict[str, dict]


@router.get("/cameras")
def cameras_status():
    process = bridge.camera_process
    return {"running": bool(process and process.poll() is None), "config": bridge.camera_config}


@router.get("/cameras/probe")
def cameras_probe():
    from rac_camera import probe_cameras

    # The capture process owns the devices once it runs: probing would fight it.
    if bridge.camera_process and bridge.camera_process.poll() is None:
        raise HTTPException(409, detail={"code": "camera.in_use", "message": "stop capture before probing cameras"})
    return safe_call(probe_cameras)


@router.post("/cameras/start")
def cameras_start(body: CamerasBody):
    stop_cameras()
    bridge.camera_config = body.cameras
    bridge.camera_process = subprocess.Popen(
        [sys.executable, "-m", "rac_camera.capture", "--config", json.dumps({"cameras": body.cameras})]
    )
    return {"running": True, "config": body.cameras}


def stop_cameras() -> None:
    process = bridge.camera_process
    if process and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    bridge.camera_process = None


@router.post("/cameras/stop")
def cameras_stop():
    stop_cameras()
    return {"running": False}


@router.get("/cameras/{name}/stream")
async def camera_stream(name: str):
    """MJPEG preview read from shared memory, so a preview never opens the device."""
    import cv2
    from rac_camera import CameraReader

    if not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", name):
        raise HTTPException(400, detail={"code": "request.bad_argument", "message": "bad camera name"})
    try:
        reader = CameraReader(name)
    except (FileNotFoundError, ValueError) as exc:
        raise HTTPException(404, detail={"code": "camera.not_running", "message": f"camera {name} is not running"}) from exc

    async def frames():
        try:
            while True:
                frame = await asyncio.to_thread(reader.next_frame, 1.0)
                if frame is None:
                    continue
                ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                await asyncio.sleep(0.03)
        finally:
            reader.close()

    return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")


# ------------------------------------------------------------- 1.2 calibration


class ArmBody(BaseModel):
    arm: str
    joints: list[int] | None = None
    joint: int | None = None
    direction: float | None = None
    which: str | None = None
    note: str = ""
    version: int | None = None
    amplitude: float = 0.1


@router.get("/calibration/list")
def calibration_list():
    return call("calibration.list")


@router.get("/calibration/{arm}")
def calibration_get(arm: str):
    return call("calibration.get", arm=arm)


@router.post("/calibration/capture-zero")
def calibration_capture_zero(body: ArmBody):
    return call("calibration.capture_zero", arm=body.arm, joints=body.joints)


@router.post("/calibration/direction")
def calibration_direction(body: ArmBody):
    return call("calibration.set_direction", arm=body.arm, joint=body.joint, direction=body.direction)


@router.post("/calibration/range-start")
def calibration_range_start(body: ArmBody):
    return call("calibration.range_start", arm=body.arm)


@router.post("/calibration/range-stop")
def calibration_range_stop(body: ArmBody):
    return call("calibration.range_stop", arm=body.arm)


@router.post("/calibration/gripper")
def calibration_gripper(body: ArmBody):
    return call("calibration.capture_gripper", arm=body.arm, which=body.which)


@router.post("/calibration/save")
def calibration_save(body: ArmBody):
    return call("calibration.save", arm=body.arm, note=body.note)


@router.post("/calibration/load")
def calibration_load(body: ArmBody):
    return call("calibration.load", arm=body.arm, version=body.version)


@router.post("/calibration/verify")
def calibration_verify(body: ArmBody):
    return call("calibration.verify_joint", arm=body.arm, joint=body.joint, amplitude=body.amplitude, timeout=30.0)


# ------------------------------------------------------------------ 1.3 teleop


class PairsBody(BaseModel):
    pairs: list[dict]
    bimanual: bool | None = None


class TeleopBody(BaseModel):
    pairs: list[int] | None = None
    speed: float = 0.5
    follow: str = "absolute"


@router.get("/teleop/config")
def teleop_config():
    return call("teleop.config")


@router.post("/teleop/config")
def teleop_configure(body: PairsBody):
    return call("teleop.configure", pairs=body.pairs, bimanual=body.bimanual)


@router.post("/teleop/enter")
def teleop_enter(body: TeleopBody):
    return call("teleop.enter", pairs=body.pairs)


@router.post("/teleop/align")
def teleop_align(body: TeleopBody):
    return call("teleop.align", pairs=body.pairs, speed=body.speed)


@router.post("/teleop/start")
def teleop_start(body: TeleopBody):
    return call("teleop.start", pairs=body.pairs, follow=body.follow)


@router.post("/teleop/pause")
def teleop_pause(body: TeleopBody):
    return call("teleop.pause", pairs=body.pairs)


@router.post("/teleop/resume")
def teleop_resume(body: TeleopBody):
    return call("teleop.resume", pairs=body.pairs)


@router.post("/teleop/end")
def teleop_end(body: TeleopBody):
    return call("teleop.end", pairs=body.pairs)


# ---------------------------------------------------------------- 1.4 teaching


class IkBody(BaseModel):
    model: str
    xyz: list[float] = Field(min_length=3, max_length=3)
    rpy: list[float] = Field(min_length=3, max_length=3)
    seed: list[float] = Field(min_length=6, max_length=6)
    position_only: bool = False


class FkBody(BaseModel):
    model: str
    q: list[float] = Field(min_length=6, max_length=6)


class TargetBody(BaseModel):
    arm: str
    pose: list[float]
    source: str = "teach"
    duration_s: float = 2.0


class ProgramBody(BaseModel):
    name: str
    arm: str
    model: str = ""
    points: list[dict]


class PreviewBody(BaseModel):
    program: ProgramBody
    start: list[float] | None = None
    dt_s: float = 0.02


class ExecuteProgramBody(BaseModel):
    program: ProgramBody
    dt_s: float = 0.02


class GuideBody(BaseModel):
    arm: str
    gravity: bool = True


def _programs_dir() -> Path:
    from rac_link.ipc import runtime_dir

    directory = runtime_dir() / "programs"
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _program_path(name: str) -> Path:
    if not re.fullmatch(r"[^\\/:*?\"<>|\x00-\x1f]{1,64}", name) or name.startswith("."):
        raise HTTPException(400, detail={"code": "request.bad_argument", "message": "invalid program name"})
    return _programs_dir() / f"{name}.json"


def _model_chain(key: str):
    from rac_link.robots import SPEC_BY_KEY

    if key not in SPEC_BY_KEY:
        raise HTTPException(400, detail={"code": "request.bad_model", "message": f"unknown model {key}"})
    return bridge.chain(key)


@router.post("/kinematics/fk")
def kinematics_fk(body: FkBody):
    from rac_kinematics import matrix_to_rpy

    chain = _model_chain(body.model)
    pose = chain.fk(np.array(body.q))
    return {"xyz": pose[:3, 3].tolist(), "rpy": list(matrix_to_rpy(pose[:3, :3]))}


@router.post("/kinematics/ik")
def kinematics_ik(body: IkBody):
    """Solved on the computer. Returns joint targets; it does not move anything."""
    from rac_kinematics import pose_from_xyz_rpy

    chain = _model_chain(body.model)
    result = chain.solve_ik(pose_from_xyz_rpy(body.xyz, body.rpy), np.array(body.seed), position_only=body.position_only)
    return {
        "q": result.q.tolist(),
        "converged": bool(result.converged),
        "position_error": result.position_error,
        "orientation_error": result.orientation_error,
        "near_singular": bool(result.near_singular),
        "iterations": result.iterations,
    }


@router.post("/motion/target")
def motion_target(body: TargetBody):
    return call("motion.set_target", arm=body.arm, pose=body.pose, source=body.source)


@router.post("/motion/move-to")
def motion_move_to(body: TargetBody):
    return call("motion.move_to", arm=body.arm, pose=body.pose, duration_s=body.duration_s, source=body.source)


@router.post("/motion/home")
def motion_home():
    return call("motion.home")


@router.get("/programs")
def programs_list():
    out = []
    for path in sorted(_programs_dir().glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        out.append({"name": data["name"], "arm": data["arm"], "model": data.get("model", ""), "points": len(data["points"])})
    return out


@router.get("/programs/{name}")
def programs_get(name: str):
    path = _program_path(name)
    if not path.exists():
        raise HTTPException(404, detail={"code": "program.not_found", "message": name})
    return json.loads(path.read_text(encoding="utf-8"))


@router.post("/programs")
def programs_save(body: ProgramBody):
    _program_path(body.name).write_text(json.dumps(body.model_dump(), ensure_ascii=False, indent=1), encoding="utf-8")
    return {"ok": True}


@router.delete("/programs/{name}")
def programs_delete(name: str):
    path = _program_path(name)
    if path.exists():
        path.unlink()
    return {"ok": True}


def _sample_program(program: ProgramBody, start: list[float] | None, dt_s: float) -> np.ndarray:
    from rac_kinematics import TeachPoint, TeachProgram
    from rac_link.robots import SPEC_BY_KEY

    if not program.points:
        raise HTTPException(400, detail={"code": "program.empty", "message": "the program has no points"})
    parsed = TeachProgram(
        name=program.name,
        arm=program.arm,
        model=program.model,
        points=[TeachPoint(**{k: p[k] for k in ("name", "q", "move_time_s", "dwell_s") if k in p}) for p in program.points],
    )
    spec = SPEC_BY_KEY.get(program.model)
    limits = spec.velocity_limit if spec else None
    begin = start if start is not None else parsed.points[0].q
    return parsed.sample(begin, dt_s, velocity_limit=limits)


@router.post("/programs/preview")
def programs_preview(body: PreviewBody):
    points = _sample_program(body.program, body.start, body.dt_s)
    stride = max(1, len(points) // 400)
    return {"dt_s": body.dt_s * stride, "duration_s": len(points) * body.dt_s, "points": points[::stride].tolist()}


@router.post("/programs/execute")
def programs_execute(body: ExecuteProgramBody):
    status = call("device.status")
    arm = next((a for a in status["arms"] if a["arm"] == body.program.arm), None)
    if not arm or "position" not in arm:
        raise HTTPException(400, detail={"code": "arm.no_feedback", "message": f"{body.program.arm} is not reporting"})
    points = _sample_program(body.program, arm["position"], body.dt_s)
    return call(
        "motion.execute",
        arm=body.program.arm,
        points=points.tolist(),
        dt_s=body.dt_s,
        source="teach",
        start_from_current=False,
    )


@router.post("/guide/start")
def guide_start(body: GuideBody):
    return call("guide.start", arm=body.arm, gravity=body.gravity)


@router.post("/guide/stop")
def guide_stop(body: GuideBody):
    return call("guide.stop", arm=body.arm)


# ------------------------------------------------------------- 1.8 run control


class StopBody(BaseModel):
    kind: str = "hold"


class ReleaseBody(BaseModel):
    arms: list[str] | None = None


@router.post("/run/stop")
def run_stop(body: StopBody):
    return call("run.stop", kind=body.kind)


@router.post("/run/pause")
def run_pause():
    return call("run.pause")


@router.post("/run/resume")
def run_resume():
    return call("run.resume")


@router.post("/run/reset-estop")
def run_reset_estop():
    return call("run.reset_estop")


@router.post("/run/release")
def run_release(body: ReleaseBody):
    return call("run.release", arms=body.arms)


# --------------------------------------------------------- 1.5, 1.6 deployment


class PolicyBody(BaseModel):
    config: dict


class TakeOverBody(BaseModel):
    follow: str = "relative"
    speed: float = 0.5


class MarkBody(BaseModel):
    label: str
    note: str = ""


@router.get("/policy/status")
def policy_status():
    return call("policy.status")


@router.post("/policy/start")
def policy_start(body: PolicyBody):
    return call("policy.start", config=body.config, timeout=30.0)


@router.post("/policy/stop")
def policy_stop():
    return call("policy.stop", timeout=15.0)


@router.post("/policy/pause")
def policy_pause():
    return call("policy.pause")


@router.post("/policy/resume")
def policy_resume():
    return call("policy.resume")


@router.post("/intervention/take-over")
def intervention_take_over(body: TakeOverBody):
    return call("intervention.take_over", follow=body.follow, speed=body.speed)


@router.post("/intervention/confirm-aligned")
def intervention_confirm():
    return call("intervention.confirm_aligned")


@router.post("/intervention/hand-back")
def intervention_hand_back():
    return call("intervention.hand_back")


@router.post("/intervention/mark")
def intervention_mark(body: MarkBody):
    return call("intervention.mark", label=body.label, note=body.note)


# --------------------------------------------------------------- 1.7 replay


class ReplayBody(BaseModel):
    root: str
    episode: int
    start_s: float | None = None
    end_s: float | None = None
    speed: float = 1.0
    column: str = "action"
    blend_s: float = 2.0


class ProgramReplayBody(BaseModel):
    name: str
    speed: float = 1.0


@router.post("/replay/episode")
def replay_episode(body: ReplayBody):
    """Move smoothly to the pose at the chosen time, then follow the recording from there."""
    from rac_data.browse import trajectory_for_replay
    from rac_kinematics import retime, window

    if not 0.05 <= body.speed <= 4.0:
        raise HTTPException(400, detail={"code": "request.bad_argument", "message": "speed must be between 0.05 and 4"})
    data = safe_call(trajectory_for_replay, body.root, body.episode, body.column)
    trajectories = {}
    dt_s = 0.02
    for side, arm in (("left", "left_follower"), ("right", "right_follower")):
        points = data.get(side)
        if points is None:
            continue
        clipped = window(np.asarray(points), data["dt_s"], body.start_s, body.end_s)
        if len(clipped) == 0:
            raise HTTPException(400, detail={"code": "request.bad_argument", "message": "that range of the episode is empty"})
        trajectories[arm] = retime(clipped, data["dt_s"], dt_s, body.speed).tolist()
    return call("motion.execute_many", trajectories=trajectories, dt_s=dt_s, source="replay", blend_s=body.blend_s, timeout=30.0)


# -------------------------------------------------------------- 2.1, 2.3 record


class RecordBody(BaseModel):
    root: str
    repo_id: str = "local/rac_dataset"
    task: str
    fps: int = 30
    cameras: list[str]
    follower_arms: list[str] = ["left_follower"]
    leader_arms: list[str] = ["left_leader"]
    model_config_data: dict | None = None


class SaveBody(BaseModel):
    result: str = "unlabeled"
    note: str = ""


@router.get("/record/status")
def record_status():
    return bridge.get_recorder().status()


@router.post("/record/start")
def record_start(body: RecordBody):
    from rac_data.recorder import RecordingConfig

    config = RecordingConfig(
        root=body.root,
        repo_id=body.repo_id,
        task=body.task,
        fps=body.fps,
        cameras=body.cameras,
        follower_arms=body.follower_arms,
        leader_arms=body.leader_arms,
        model_config=body.model_config_data,
    )
    return safe_call(bridge.get_recorder().start, config)


@router.post("/record/stop")
def record_stop():
    return safe_call(bridge.get_recorder().stop)


@router.post("/record/next")
def record_next(body: SaveBody):
    return safe_call(bridge.get_recorder().next, body.model_dump())


@router.post("/record/save")
def record_save(body: SaveBody):
    return safe_call(bridge.get_recorder().save, body.model_dump())


@router.post("/record/discard")
def record_discard():
    return safe_call(bridge.get_recorder().discard)


# -------------------------------------------------- 2.3, 2.4, 2.5 data browser


class ExportBody(BaseModel):
    root: str
    destination: str


class ImportBody(BaseModel):
    source: str
    base: str


class MergeBody(BaseModel):
    roots: list[str]
    output: str


class EpisodeAnnotationBody(BaseModel):
    root: str
    episode: int
    result: str | None = None
    note: str | None = None
    task: str | None = None
    excluded: bool | None = None


class SegmentBody(BaseModel):
    root: str
    episode: int
    start_s: float
    end_s: float
    label: str
    note: str = ""
    kind: str = "phase"


class SegmentEditBody(BaseModel):
    root: str
    episode: int
    segment: int
    fields: dict


def default_data_root() -> str:
    from rac_link.ipc import runtime_dir

    return os.environ.get("RAC_DATA_ROOT", str(runtime_dir() / "datasets"))


@router.get("/data/root")
def data_root():
    return {"root": default_data_root()}


@router.get("/data/datasets")
def data_datasets(base: str | None = None):
    from rac_data.browse import list_datasets

    return safe_call(list_datasets, base or default_data_root())


@router.get("/data/episodes")
def data_episodes(root: str, task: str | None = None, result: str | None = None, source: str | None = None):
    from rac_data.browse import list_episodes

    return safe_call(list_episodes, root, task, result, source)


@router.get("/data/episode")
def data_episode(root: str, episode: int, stride: int = Query(1, ge=1, le=100)):
    from rac_data.browse import episode_data

    return safe_call(episode_data, root, episode, stride)


@router.get("/data/video")
def data_video(root: str, path: str):
    """Serve a video file of a dataset. The path has to stay inside the dataset's videos."""
    base = (Path(root) / "videos").resolve()
    target = (Path(root) / path).resolve()
    if base not in target.parents or target.suffix != ".mp4" or not target.exists():
        raise HTTPException(404, detail={"code": "data.not_found", "message": "no such video"})
    return FileResponse(target, media_type="video/mp4")


@router.post("/data/annotate")
def data_annotate(body: EpisodeAnnotationBody):
    from rac_data import AnnotationStore

    fields = body.model_dump(exclude={"root", "episode"}, exclude_none=True)
    return safe_call(AnnotationStore(body.root).set_episode, body.episode, fields)


@router.post("/data/segments")
def data_segment_add(body: SegmentBody):
    from rac_data import AnnotationStore

    return safe_call(AnnotationStore(body.root).add_segment, body.episode, body.start_s, body.end_s, body.label, body.note, body.kind)


@router.post("/data/segments/update")
def data_segment_update(body: SegmentEditBody):
    from rac_data import AnnotationStore

    return safe_call(AnnotationStore(body.root).update_segment, body.episode, body.segment, body.fields)


@router.post("/data/segments/delete")
def data_segment_delete(body: SegmentEditBody):
    from rac_data import AnnotationStore

    safe_call(AnnotationStore(body.root).delete_segment, body.episode, body.segment)
    return {"ok": True}


@router.post("/data/check-merge")
def data_check_merge(body: MergeBody):
    from rac_data.browse import check_mergeable

    return safe_call(check_mergeable, body.roots)


@router.post("/data/merge")
def data_merge(body: MergeBody):
    from rac_data.browse import merge_datasets

    return safe_call(merge_datasets, body.roots, body.output)


@router.post("/data/export")
def data_export(body: ExportBody):
    from rac_data.browse import export_dataset

    return safe_call(export_dataset, body.root, body.destination)


@router.post("/data/import")
def data_import(body: ImportBody):
    from rac_data.browse import import_dataset

    return safe_call(import_dataset, body.source, body.base)


# ----------------------------------------------------------------- simulation


class SimBody(BaseModel):
    op: str
    arg: int = 0
    arg2: int = 0
    values: list[float] | None = None


@router.post("/sim/inject")
def sim_inject(body: SimBody):
    return call("sil.inject", op=body.op, arg=body.arg, arg2=body.arg2, values=body.values)


# ------------------------------------------------------------------ install


def install_controller(app) -> None:
    """Register the arm family, the routes and the robot descriptions on the Lab."""
    from fastapi.staticfiles import StaticFiles

    from .arms import registry
    from .arms.controller import CONTROLLER

    if CONTROLLER.id not in registry.ids():
        registry.register(CONTROLLER, provided_by="rebotarmctl")
    app.include_router(router, prefix="/api/v1")
    app.include_router(ws_router, prefix="/api/v1")
    assets = Path(__file__).resolve().parents[3] / "assets" / "robots"
    if assets.is_dir():
        app.mount("/robots", StaticFiles(directory=assets), name="robots")


# ------------------------------------------------------------------ live state


def _compact_sample(sample: dict) -> dict:
    keys = ("arm", "sample_seq", "position", "target", "velocity", "torque", "temperature", "mode", "source", "phase", "valid_mask", "flags", "queue_depth", "exec_seq", "fault_mask")
    return {k: sample[k] for k in keys if k in sample}


@ws_router.websocket("/ws")
async def controller_ws(websocket: WebSocket):
    """Status, the latest samples and new events at about 30 Hz."""
    if os.environ.get("RAC_ALLOW_REMOTE") != "1" and not is_local(websocket.client.host if websocket.client else None):
        await websocket.close(code=1008)
        return
    await websocket.accept()
    events_seen = bridge.event_total
    last_status_sent = 0.0
    try:
        await asyncio.to_thread(bridge.get_client)
    except HTTPException as exc:
        await websocket.send_json({"type": "error", "detail": exc.detail})
    try:
        while True:
            now = time.monotonic()
            payload: dict = {"type": "state", "samples": {k: _compact_sample(v) for k, v in bridge.latest_samples.items()}}
            if now - last_status_sent > 0.1 and bridge.latest_status is not None:
                payload["status"] = bridge.latest_status
                last_status_sent = now
            total = bridge.event_total
            if total != events_seen:
                # The list is trimmed, so index from its end by how many are new.
                payload["events"] = bridge.events[-min(total - events_seen, len(bridge.events)) :]
                events_seen = total
            await websocket.send_json(payload)
            await asyncio.sleep(1 / 30)
    except (WebSocketDisconnect, RuntimeError):
        return
