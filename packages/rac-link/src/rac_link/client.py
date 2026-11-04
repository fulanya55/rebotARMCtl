"""The Controller SDK every module uses to reach the Controller Service.

Each method is one service request. Long documentation lives on the service
handlers; this class exists so callers get names and arguments instead of strings.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .ipc import IpcClient


class ControllerClient:
    def __init__(self, ipc: IpcClient | None = None, **kwargs: Any) -> None:
        self.ipc = ipc or IpcClient(**kwargs)

    def close(self) -> None:
        self.ipc.close()

    def call(self, method: str, **params: Any) -> Any:
        return self.ipc.call(method, **params)

    # ---------------------------------------------------------- subscriptions

    def on_state(self, handler: Callable[[dict], None]) -> None:
        """Every arm sample, as dictionaries with both controller and host time."""
        self.ipc.subscribe("state", handler)

    def on_event(self, handler: Callable[[dict], None]) -> None:
        """Control events from the controller and the service, on one timeline."""
        self.ipc.subscribe("event", handler)

    def on_status(self, handler: Callable[[dict], None]) -> None:
        """Whole-system status, published several times a second."""
        self.ipc.subscribe("status", handler)

    # ------------------------------------------------------------------ device

    def candidates(self) -> list[dict]:
        return self.call("device.candidates")

    def connect(self, uri: str) -> dict:
        return self.call("device.connect", uri=uri, timeout=10.0)

    def disconnect(self) -> None:
        self.call("device.disconnect")

    def status(self) -> dict:
        return self.call("device.status")

    def discover(self) -> dict:
        return self.call("device.discover", timeout=5.0)

    def bind(self, bindings: list[dict]) -> dict:
        return self.call("device.bind", bindings=bindings)

    def set_rates(self, **rates: Any) -> dict:
        return self.call("device.set_rates", **rates)

    # ------------------------------------------------------------- calibration

    def calibration_state(self, arm: str) -> dict:
        return self.call("calibration.get", arm=arm)

    def calibration_list(self) -> list[dict]:
        return self.call("calibration.list")

    def calibration_capture_zero(self, arm: str, joints: list[int] | None = None) -> dict:
        return self.call("calibration.capture_zero", arm=arm, joints=joints)

    def calibration_set_direction(self, arm: str, joint: int, direction: float) -> dict:
        return self.call("calibration.set_direction", arm=arm, joint=joint, direction=direction)

    def calibration_range_start(self, arm: str) -> dict:
        return self.call("calibration.range_start", arm=arm)

    def calibration_range_stop(self, arm: str) -> dict:
        return self.call("calibration.range_stop", arm=arm)

    def calibration_capture_gripper(self, arm: str, which: str) -> dict:
        return self.call("calibration.capture_gripper", arm=arm, which=which)

    def calibration_save(self, arm: str, note: str = "") -> dict:
        return self.call("calibration.save", arm=arm, note=note)

    def calibration_load(self, arm: str, version: int | None = None) -> dict:
        return self.call("calibration.load", arm=arm, version=version)

    def calibration_verify_joint(self, arm: str, joint: int, amplitude: float = 0.1) -> dict:
        return self.call("calibration.verify_joint", arm=arm, joint=joint, amplitude=amplitude, timeout=30.0)

    # ------------------------------------------------------------------ teleop

    def teleop_configure(self, pairs: list[dict], bimanual: bool | None = None) -> dict:
        return self.call("teleop.configure", pairs=pairs, bimanual=bimanual)

    def teleop_config(self) -> dict:
        return self.call("teleop.config")

    def teleop_enter(self, pairs: list[int] | None = None) -> dict:
        return self.call("teleop.enter", pairs=pairs)

    def teleop_align(self, pairs: list[int] | None = None, speed: float = 0.5) -> dict:
        return self.call("teleop.align", pairs=pairs, speed=speed)

    def teleop_start(self, pairs: list[int] | None = None, follow: str = "absolute") -> dict:
        return self.call("teleop.start", pairs=pairs, follow=follow)

    def teleop_pause(self, pairs: list[int] | None = None) -> dict:
        return self.call("teleop.pause", pairs=pairs)

    def teleop_resume(self, pairs: list[int] | None = None) -> dict:
        return self.call("teleop.resume", pairs=pairs)

    def teleop_end(self, pairs: list[int] | None = None) -> dict:
        return self.call("teleop.end", pairs=pairs)

    # ------------------------------------------------- the one execution entry

    def execute(
        self,
        arm: str,
        points: list[list[float]],
        dt_s: float,
        source: str,
        start_from_current: bool = True,
        blend_s: float = 1.0,
        wait: bool = False,
    ) -> dict:
        return self.call(
            "motion.execute",
            arm=arm,
            points=points,
            dt_s=dt_s,
            source=source,
            start_from_current=start_from_current,
            blend_s=blend_s,
            wait=wait,
            timeout=None if not wait else max(60.0, len(points) * dt_s + 30.0),
        )

    def execute_bimanual(self, trajectories: dict[str, list[list[float]]], dt_s: float, source: str, blend_s: float = 1.0) -> dict:
        return self.call("motion.execute_many", trajectories=trajectories, dt_s=dt_s, source=source, blend_s=blend_s)

    def move_to(self, arm: str, pose: list[float], duration_s: float = 2.0, source: str = "teach") -> dict:
        return self.call("motion.move_to", arm=arm, pose=pose, duration_s=duration_s, source=source)

    def set_target(self, arm: str, pose: list[float], source: str = "teach") -> dict:
        return self.call("motion.set_target", arm=arm, pose=pose, source=source)

    def home(self, arms: list[str] | None = None, duration_s: float = 3.0) -> dict:
        return self.call("motion.home", arms=arms, duration_s=duration_s)

    def guide_start(self, arm: str, gravity: bool = True) -> dict:
        return self.call("guide.start", arm=arm, gravity=gravity)

    def guide_stop(self, arm: str) -> dict:
        return self.call("guide.stop", arm=arm)

    # ------------------------------------------------------------- run control

    def stop(self, kind: str = "hold") -> dict:
        return self.call("run.stop", kind=kind)

    def pause(self) -> dict:
        return self.call("run.pause")

    def resume(self) -> dict:
        return self.call("run.resume")

    def reset_estop(self) -> dict:
        return self.call("run.reset_estop")

    def release(self, arms: list[str] | None = None) -> dict:
        return self.call("run.release", arms=arms)

    # ------------------------------------------------------------------ policy

    def policy_start(self, config: dict) -> dict:
        return self.call("policy.start", config=config, timeout=30.0)

    def policy_stop(self) -> dict:
        return self.call("policy.stop", timeout=15.0)

    def policy_pause(self) -> dict:
        return self.call("policy.pause")

    def policy_resume(self) -> dict:
        return self.call("policy.resume")

    def policy_status(self) -> dict:
        return self.call("policy.status")

    def policy_submit(self, chunk: list[list[float]], token: int, observation_ts_us: int, inference_ms: float) -> dict:
        """What a model process calls. A token from before a takeover is refused."""
        return self.call(
            "policy.submit", chunk=chunk, token=token, observation_ts_us=observation_ts_us, inference_ms=inference_ms
        )

    def intervention_take_over(self, follow: str = "relative", speed: float = 0.5) -> dict:
        return self.call("intervention.take_over", follow=follow, speed=speed)

    def intervention_hand_back(self) -> dict:
        return self.call("intervention.hand_back")

    def intervention_mark(self, label: str, note: str = "") -> dict:
        return self.call("intervention.mark", label=label, note=note)

    def intervention_status(self) -> dict:
        return self.call("intervention.status")

    # -------------------------------------------------------------- simulation

    def sil_inject(self, op: str, arg: int = 0, arg2: int = 0, values: list[float] | None = None) -> dict:
        return self.call("sil.inject", op=op, arg=arg, arg2=arg2, values=values)
