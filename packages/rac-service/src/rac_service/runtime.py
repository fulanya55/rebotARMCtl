"""The unified runtime: control ownership, one execution entry, one state view.

Rules this class enforces:

* Only this object holds the controller link.
* Exactly one source controls the arms at a time. A change of source takes a new
  control generation, and the controller refuses anything from an older one.
* Kinematics and models produce targets; they all execute through ``_execute``.
* What the UI shows comes from controller feedback, not from what was requested.
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Callable
from pathlib import Path

import numpy as np
from rac_kinematics import blend, load_chain, min_duration
from rac_link._gen import (
    JOINTS,
    AckStatus,
    ArmId,
    ArmModel,
    CanDialect,
    ControlSource,
    EstopState,
    EventCode,
    FollowMode,
    Mode,
    PairPhase,
    PortId,
    SetCalibration,
    SetLimits,
    SetPairing,
    SetRates,
    SilOp,
    StopKind,
    TeleopAction,
)
from rac_link.clock import host_now_us
from rac_link.device import ControllerDevice, ControllerError
from rac_link.ipc import ServiceError
from rac_link.protocol_constants import SOURCE_BY_VALUE, SOURCE_NAMES
from rac_link.robots import (
    ARM_BY_NAME,
    ARM_NAMES,
    PAIR_ARMS,
    PORT_KIND,
    PORT_NAMES,
    SPEC_BY_KEY,
    SPECS,
    default_mapping,
    spec_of,
)
from rac_link.transport import list_candidates

from .action_buffer import make_buffer
from .executor import Executor, Job
from .store import Calibration, Store

log = logging.getLogger(__name__)

STATUS_PERIOD_S = 0.1
GUIDE_PERIOD_S = 0.01
FF_VALID_MS = 200
DISCOVER_SETTLE_S = 0.15


def _err(code: str, message: str) -> ServiceError:
    return ServiceError(message, code)


class Runtime:
    def __init__(self, store: Store, publish: Callable[[str, object], None]) -> None:
        self.store = store
        self.publish = publish
        self.lock = threading.RLock()
        self.device: ControllerDevice | None = None
        self.uri: str | None = None

        self.source = int(ControlSource.NONE)
        self.executor = Executor(lambda: self.device, self._on_job_finished)
        self.drafts: dict[str, Calibration] = {}
        self.applied: dict[int, Calibration] = {}
        self.range_tracking: dict[int, dict[str, np.ndarray]] = {}
        self._chains: dict[str, object] = {}

        self._sample_batch: deque[dict] = deque(maxlen=4096)
        self._batch_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._status_loop, name="rac-status", daemon=True),
            threading.Thread(target=self._sample_flush_loop, name="rac-samples", daemon=True),
            threading.Thread(target=self._guide_loop, name="rac-guide", daemon=True),
            threading.Thread(target=self._policy_loop, name="rac-policy", daemon=True),
        ]
        self.guide_arms: dict[int, bool] = {}  # arm -> gravity compensation on

        self.teleop_active = False
        self.teleop_pairs: list[int] = []
        self.follow_mode = "absolute"
        self.paused = False

        # Policy run state
        self.policy_config: dict | None = None
        self.policy_token = 0
        self.policy_state = "idle"
        self.policy_buffer = None
        self.policy_arms: list[int] = []
        self.policy_stats: dict = {}
        self.policy_process: subprocess.Popen | None = None
        self.policy_exec_seq = 1
        self.policy_reset_requested = False

        # Intervention state
        self.intervention_active = False
        self.intervention_started_us = 0
        self.intervention_segments: list[dict] = []

        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        self._stop.set()
        self.executor.close()
        self._terminate_policy_process()
        if self.device:
            self.device.close()

    # ---------------------------------------------------------------- helpers

    def _dev(self) -> ControllerDevice:
        if not self.device or not self.device.is_open:
            raise _err("device.not_connected", "the controller is not connected")
        return self.device

    def _arm_id(self, name: str | int) -> int:
        if isinstance(name, int):
            return name
        try:
            return int(ARM_BY_NAME[name])
        except KeyError:
            raise _err("request.bad_arm", f"unknown arm {name!r}") from None

    def _binding_of_arm(self, arm: int) -> dict | None:
        name = ARM_NAMES[ArmId(arm)]
        for port, binding in self.store.config["bindings"].items():
            if binding.get("arm") == name:
                return {"port": int(port), **binding}
        return None

    def _spec_of_arm(self, arm: int):
        binding = self._binding_of_arm(arm)
        if not binding:
            raise _err("arm.not_bound", f"{ARM_NAMES[ArmId(arm)]} is not bound to a port")
        return SPEC_BY_KEY[binding["model"]]

    def _chain(self, spec):
        chain = self._chains.get(spec.key)
        if chain is None:
            chain = load_chain(spec.urdf, spec.end_effector)
            self._chains[spec.key] = chain
        return chain

    def _emit(self, kind: str, /, **fields) -> None:
        """Publish a service-level control event on the shared timeline."""
        self.publish("event", {"origin": "service", "kind": kind, "host_ts_us": host_now_us(), **fields})

    def _set_source(self, source: int) -> None:
        if source != self.source:
            previous = self.source
            self.source = source
            self._emit("source_changed", source=SOURCE_BY_VALUE.get(source), previous=SOURCE_BY_VALUE.get(previous))

    def _claim(self, source: int, preempt: tuple[int, ...] = ()) -> None:
        """Take control. Refuses when another source is running, unless named in preempt."""
        status = self._dev().snapshot.status
        if status and status.estop != EstopState.CLEAR:
            raise _err("safety.estop", "the emergency stop is active")
        if self.source not in (ControlSource.NONE, source, *preempt):
            raise _err(
                "control.busy",
                f"{SOURCE_BY_VALUE.get(self.source)} is in control; stop it before starting {SOURCE_BY_VALUE.get(source)}",
            )

    def _guard(self, fn):
        """Map controller refusals onto coded errors the UI can act on."""
        try:
            return fn()
        except ControllerError as exc:
            codes = {
                AckStatus.ESTOP_ACTIVE: "safety.estop",
                AckStatus.STALE_GENERATION: "control.stale",
                AckStatus.UNSUPPORTED: "arm.unsupported",
                AckStatus.MODE_REJECTED: "control.mode_rejected",
                AckStatus.BUSY: "control.busy",
                AckStatus.BAD_ARGUMENT: "request.bad_argument",
            }
            raise _err(codes.get(exc.status, "device.error"), str(exc)) from exc

    def _mask(self, arms: list[int]) -> int:
        mask = 0
        for arm in arms:
            mask |= 1 << arm
        return mask

    # ----------------------------------------------------------------- device

    def candidates(self) -> list[dict]:
        found = list_candidates()
        return found

    def connect(self, uri: str) -> dict:
        with self.lock:
            if self.device and self.device.is_open:
                if self.uri == uri:
                    return self.status()
                raise _err("device.already_connected", f"already connected to {self.uri}")
            device = ControllerDevice(uri)
            device.on_state = self._on_state
            device.on_event = self._on_event
            device.on_disconnect = self._on_disconnect
            try:
                device.open()
            except (ControllerError, OSError, ValueError) as exc:
                raise _err("device.connect_failed", f"cannot open {uri}: {exc}") from exc
            self.device = device
            self.uri = uri
            self._emit("device_connected", uri=uri)
            self._restore_bindings()
            return self.status()

    def _restore_bindings(self) -> None:
        """Re-apply saved bindings and calibrations. Communication only: torque stays off."""
        bindings = self.store.config.get("bindings", {})
        for port, binding in bindings.items():
            try:
                self._apply_binding(int(port), binding)
            except (ControllerError, ServiceError) as exc:
                log.warning("could not restore binding of port %s: %s", port, exc)
        self._push_pairs()
        rates = self.store.config.get("rates")
        if rates:
            try:
                self._apply_rates(rates)
            except ControllerError as exc:
                log.warning("could not restore rates: %s", exc)

    def disconnect(self) -> dict:
        with self.lock:
            if self.device:
                self._stop_everything(StopKind.HOLD)
                self.device.close()
                self.device = None
                self._emit("device_disconnected")
            return {"connected": False}

    def _on_disconnect(self, reason: str) -> None:
        self.source = int(ControlSource.NONE)
        self.executor.cancel()
        self.teleop_active = False
        self._emit("device_lost", reason=reason)

    def discover(self) -> dict:
        device = self._dev()
        self._guard(device.discover)
        time.sleep(DISCOVER_SETTLE_S)
        device.query_status()
        time.sleep(0.03)
        return self.ports()

    def ports(self) -> dict:
        device = self.device
        out = []
        for port_id in PortId:
            status = device.snapshot.ports.get(int(port_id)) if device else None
            binding = self.store.config["bindings"].get(str(int(port_id)))
            item = {
                "port": int(port_id),
                "name": PORT_NAMES[port_id],
                "kind": PORT_KIND[port_id].name,
                "binding": binding,
                "detected_model": None,
                "online_mask": 0,
                "expected_mask": 0,
                "bus_load": 0.0,
                "tx_errors": 0,
                "rx_errors": 0,
            }
            if status:
                detected = ArmModel(status.detected_model)
                item.update(
                    {
                        "detected_model": None if detected == ArmModel.NONE else detected.name,
                        "online_mask": status.online_mask,
                        "expected_mask": status.expected_mask,
                        "bus_load": status.bus_load_permille / 10.0,
                        "tx_errors": status.tx_errors,
                        "rx_errors": status.rx_errors,
                        "can_dialect": CanDialect(status.can_dialect).name,
                    }
                )
            out.append(item)
        return {"ports": out}

    def _apply_binding(self, port: int, binding: dict) -> None:
        device = self._dev()
        spec = SPEC_BY_KEY[binding["model"]]
        arm = ARM_BY_NAME[binding["arm"]]
        dialect = CanDialect[binding.get("dialect", "RS_PRIVATE")]
        device.bind_port(port, arm, spec.model, dialect, binding.get("baudrate", 1_000_000))
        limits = SetLimits(
            velocity=list(spec.velocity_limit),
            acceleration=list(spec.acceleration_limit),
            kp=list(spec.kp),
            kd=list(spec.kd),
        )
        device.set_limits(arm, limits)
        serial = binding.get("serial") or f"{spec.key}-{PORT_NAMES[PortId(port)].lower()}"
        calibration = self.store.load_calibration(serial) or Calibration.default_for(serial, spec)
        self._push_calibration(arm, calibration)

    def bind(self, bindings: list[dict]) -> dict:
        """Bind ports to roles. Validates the whole request before changing anything."""
        with self.lock:
            device = self._dev()
            if self.source != ControlSource.NONE:
                raise _err("control.busy", "stop the current run before changing bindings")
            seen_arms: set[str] = set()
            parsed: dict[int, dict | None] = {}
            for entry in bindings:
                port = int(entry["port"])
                if port not in (0, 1, 2, 3):
                    raise _err("request.bad_argument", f"no port {port}")
                arm = entry.get("arm")
                if not arm:
                    parsed[port] = None
                    continue
                if arm not in ARM_BY_NAME:
                    raise _err("request.bad_arm", f"unknown role {arm!r}")
                if arm in seen_arms:
                    raise _err("request.bad_argument", f"{arm} is bound twice")
                seen_arms.add(arm)
                model = entry.get("model")
                if model not in SPEC_BY_KEY:
                    raise _err("request.bad_model", f"unknown arm model {model!r}")
                spec = SPEC_BY_KEY[model]
                if spec.port_kind != PORT_KIND[PortId(port)]:
                    raise _err(
                        "arm.unsupported",
                        f"{spec.label} uses {spec.port_kind.name}, {PORT_NAMES[PortId(port)]} is {PORT_KIND[PortId(port)].name}",
                    )
                if arm.endswith("follower") and not spec.can_execute:
                    raise _err("arm.unsupported", f"{spec.label} cannot execute, so it cannot be a follower")
                parsed[port] = {
                    "arm": arm,
                    "model": model,
                    "serial": entry.get("serial") or f"{model}-{PORT_NAMES[PortId(port)].lower()}",
                    "dialect": entry.get("dialect", "RS_PRIVATE"),
                }

            for port, binding in parsed.items():
                if binding is None:
                    self._guard(lambda p=port: device.bind_port(p, ArmId.NONE, ArmModel.NONE))
                    self.store.config["bindings"].pop(str(port), None)
                else:
                    self._guard(lambda p=port, b=binding: self._apply_binding(p, b))
                    self.store.config["bindings"][str(port)] = binding
            self.store.save_config()
            self._push_pairs()
            self._emit("bindings_changed", bindings=self.store.config["bindings"])
            return self.ports()

    def _apply_rates(self, rates: dict) -> None:
        device = self._dev()
        device.set_rates(
            SetRates(
                motor_service_us=list(rates.get("motor_service_us", [4000, 4000, 8000, 8000])),
                state_read_us=list(rates.get("state_read_us", [10000, 10000, 8000, 8000])),
                upstream_us=int(rates.get("upstream_us", 4000)),
                host_timeout_ms=int(rates.get("host_timeout_ms", 500)),
            )
        )

    def set_rates(self, **rates) -> dict:
        with self.lock:
            self._guard(lambda: self._apply_rates(rates))
            self.store.config["rates"] = {**self.store.config.get("rates", {}), **rates}
            self.store.save_config()
            return self.store.config["rates"]

    def status(self) -> dict:
        device = self.device
        status = device.snapshot.status if device else None
        info = device.snapshot.info if device else None
        arms = []
        if device:
            for arm in ArmId:
                if arm == ArmId.NONE:
                    continue
                state = device.snapshot.arms.get(int(arm))
                binding = self._binding_of_arm(int(arm))
                stats = device.stats.get(int(arm))
                entry: dict = {"arm": ARM_NAMES[arm], "bound": bool(binding), "binding": binding}
                if status:
                    entry["mode"] = Mode(status.arm_mode[int(arm)]).name
                if state:
                    entry.update(
                        {
                            "position": state.position,
                            "velocity": state.velocity,
                            "torque": state.torque,
                            "target": state.target,
                            "temperature": state.temperature,
                            "valid_mask": state.valid_mask,
                            "fault_mask": state.fault_mask,
                            "flags": state.flags,
                            "queue_depth": state.queue_depth,
                            "exec_seq": state.exec_seq,
                            "torque_on": bool(state.flags & 0x08),
                            "source": SOURCE_BY_VALUE.get(state.source),
                        }
                    )
                if stats:
                    entry["rate_hz"] = round(stats.rate_hz, 1)
                    entry["missing"] = stats.missing
                    entry["invalid"] = stats.invalid
                calibration = self.applied.get(int(arm))
                if calibration:
                    entry["calibration_version"] = calibration.version
                    entry["serial"] = calibration.serial
                arms.append(entry)

        pairs = []
        if status:
            for index in range(2):
                pairs.append(
                    {
                        "pair": index,
                        "phase": PairPhase(status.pair_phase[index]).name,
                        "follow_mode": FollowMode(status.follow_mode[index]).name,
                        "sync_period_us": status.sync_period_us[index],
                        "sync_latency_us": status.sync_latency_us[index],
                        "rel_offset": status.rel_offset[index * JOINTS : (index + 1) * JOINTS],
                    }
                )
        return {
            "connected": bool(device and device.is_open),
            "uri": self.uri,
            "simulated": bool(info and info.capabilities & 0x10),
            "device": None
            if not info
            else {
                "firmware": f"{info.fw_major}.{info.fw_minor}.{info.fw_patch}",
                "protocol": info.protocol,
                "uid": bytes(info.uid).hex(),
                "hw_revision": info.hw_revision,
                "tick_hz": info.tick_hz,
                "queue_points": info.queue_points,
                "capabilities": info.capabilities,
            },
            "generation": device.generation if device else 0,
            "source": SOURCE_BY_VALUE.get(self.source),
            "estop": EstopState(status.estop).name if status else None,
            "host_alive": bool(status.host_alive) if status else False,
            "paused": self.paused,
            "uptime_s": status.uptime_us / 1e6 if status else 0,
            "overruns": status.overrun_count if status else 0,
            "max_tick_us": status.max_tick_us if status else 0,
            "clock": device.clock.summary() if device else None,
            "arms": arms,
            "pairs": pairs,
            "ports": self.ports()["ports"],
            "execution": self.executor.progress(),
            "teleop": {"active": self.teleop_active, "pairs": self.teleop_pairs, "follow": self.follow_mode},
            "guide": {ARM_NAMES[ArmId(a)]: gravity for a, gravity in self.guide_arms.items()},
            "policy": self.policy_status(),
            "intervention": self.intervention_status(),
        }

    # --------------------------------------------------------------- callbacks

    def _on_state(self, arm: int, state, host_ts: float) -> None:
        tracked = self.range_tracking.get(arm)
        if tracked is not None and state.valid_mask == 0x7F:
            position = np.asarray(state.position)
            tracked["min"] = np.minimum(tracked["min"], position)
            tracked["max"] = np.maximum(tracked["max"], position)
        sample = {
            "arm": ARM_NAMES.get(ArmId(arm)) if arm in (0, 1, 2, 3) else None,
            "sample_seq": state.sample_seq,
            "sample_ts_us": state.sample_ts_us,
            "host_ts_us": int(host_ts),
            "target_seq": state.target_seq,
            "target_ts_us": state.target_ts_us,
            "exec_seq": state.exec_seq,
            "valid_mask": state.valid_mask,
            "flags": state.flags,
            "mode": state.mode,
            "source": state.source,
            "queue_depth": state.queue_depth,
            "fault_mask": state.fault_mask,
            "phase": state.phase,
            "position": state.position,
            "velocity": state.velocity,
            "torque": state.torque,
            "target": state.target,
            "temperature": state.temperature,
        }
        with self._batch_lock:
            self._sample_batch.append(sample)

    def _on_event(self, event, host_ts: float) -> None:
        try:
            name = EventCode(event.code).name
        except ValueError:
            name = f"UNKNOWN_{event.code}"
        self.publish(
            "event",
            {
                "origin": "controller",
                "kind": name.lower(),
                "host_ts_us": int(host_ts),
                "mcu_ts_us": event.event_ts_us,
                "arm": ARM_NAMES.get(ArmId(event.arm)) if event.arm in (0, 1, 2, 3) else None,
                "severity": event.severity,
                "arg0": event.arg0,
                "arg1": event.arg1,
            },
        )
        # Anything the controller stopped on its own takes control away here too.
        if event.code in (EventCode.ESTOP_PRESSED, EventCode.HOST_LOST):
            self.executor.cancel()
            self.teleop_active = False
            self.guide_arms.clear()
            self._suspend_policy("stopped by controller")
            self._set_source(int(ControlSource.NONE))

    def _sample_flush_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.01)
            with self._batch_lock:
                if not self._sample_batch:
                    continue
                batch = list(self._sample_batch)
                self._sample_batch.clear()
            self.publish("state", batch)

    def _status_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(STATUS_PERIOD_S)
            try:
                self.publish("status", self.status())
            except Exception:
                log.exception("status publication failed")

    def _guide_loop(self) -> None:
        """Gravity torque for guided arms, computed here and refreshed on the controller."""
        while not self._stop.is_set():
            time.sleep(GUIDE_PERIOD_S)
            device = self.device
            if not device or not device.is_open:
                continue
            status = device.snapshot.status
            if not status or status.estop != EstopState.CLEAR:
                continue
            for arm in ArmId:
                if arm == ArmId.NONE:
                    continue
                state = device.snapshot.arms.get(int(arm))
                binding = self._binding_of_arm(int(arm))
                if not state or not binding:
                    continue
                spec = SPEC_BY_KEY[binding["model"]]
                if spec.family != "rs":
                    continue
                mode = status.arm_mode[int(arm)]
                guided = int(arm) in self.guide_arms
                torque_on = bool(state.flags & 0x08)
                # Position-controlled RS arms get it too: it removes the droop.
                carrying = torque_on and mode in (Mode.HOLD, Mode.HOST_TARGET, Mode.LOCAL_TELEOP, Mode.GUIDE)
                if not carrying:
                    continue
                if guided and not self.guide_arms[int(arm)]:
                    tau = [0.0] * JOINTS
                else:
                    gravity = self._chain(spec).gravity_torque(np.array(state.position[:6]))
                    tau = [float(v) for v in gravity] + [0.0]
                try:
                    device.feedforward(int(arm), tau, FF_VALID_MS)
                except ControllerError:
                    break

    # ------------------------------------------------------------- calibration

    def _push_calibration(self, arm: int, calibration: Calibration) -> None:
        device = self._dev()
        device.set_calibration(
            arm,
            SetCalibration(
                version=calibration.version,
                zero=list(calibration.zero),
                direction=list(calibration.direction),
                lower=list(calibration.lower),
                upper=list(calibration.upper),
                gripper_closed=calibration.gripper_closed,
                gripper_open=calibration.gripper_open,
            ),
        )
        self.applied[arm] = calibration

    def _draft(self, arm: int) -> Calibration:
        binding = self._binding_of_arm(arm)
        if not binding:
            raise _err("arm.not_bound", f"{ARM_NAMES[ArmId(arm)]} is not bound to a port")
        serial = binding["serial"]
        draft = self.drafts.get(serial)
        if draft is None:
            spec = SPEC_BY_KEY[binding["model"]]
            applied = self.applied.get(arm)
            source = applied or self.store.load_calibration(serial) or Calibration.default_for(serial, spec)
            draft = Calibration(**source.to_dict())
            self.drafts[serial] = draft
        return draft

    def _raw_position(self, arm: int) -> list[float]:
        """Recover raw motor positions by inverting the calibration currently applied."""
        device = self._dev()
        state = device.snapshot.arms.get(arm)
        if not state or state.valid_mask != 0x7F:
            raise _err("arm.no_feedback", "not every joint of this arm is reporting")
        applied = self.applied[arm]
        raw = []
        for j in range(JOINTS - 1):
            direction = applied.direction[j] or 1.0
            raw.append(applied.zero[j] + state.position[j] / direction)
        raw.append(applied.gripper_closed + state.position[-1] * (applied.gripper_open - applied.gripper_closed))
        return raw

    def _require_released(self, arm: int) -> None:
        device = self._dev()
        state = device.snapshot.arms.get(arm)
        if state and state.flags & 0x08:
            raise _err("control.busy", "release torque on this arm before calibrating it")

    def calibration_get(self, arm: str) -> dict:
        arm_id = self._arm_id(arm)
        draft = self._draft(arm_id)
        state = self.device.snapshot.arms.get(arm_id) if self.device else None
        return {
            "draft": draft.to_dict(),
            "applied_version": self.applied[arm_id].version if arm_id in self.applied else None,
            "versions": self.store.list_versions(draft.serial),
            "tracking_range": arm_id in self.range_tracking,
            "position": state.position if state else None,
        }

    def calibration_capture_zero(self, arm: str, joints: list[int] | None = None) -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            self._require_released(arm_id)
            raw = self._raw_position(arm_id)
            draft = self._draft(arm_id)
            for j in joints if joints is not None else range(JOINTS - 1):
                if not 0 <= j < JOINTS - 1:
                    raise _err("request.bad_argument", f"joint {j} has no zero: the gripper uses a range")
                draft.zero[j] = raw[j]
            if "zero" not in draft.steps_done:
                draft.steps_done.append("zero")
            self._guard(lambda: self._push_calibration(arm_id, draft))
            return self.calibration_get(arm)

    def calibration_set_direction(self, arm: str, joint: int, direction: float) -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            self._require_released(arm_id)
            if direction not in (1, -1, 1.0, -1.0):
                raise _err("request.bad_argument", "direction must be 1 or -1")
            if not 0 <= joint < JOINTS - 1:
                raise _err("request.bad_argument", "direction applies to joints 0 to 5")
            draft = self._draft(arm_id)
            draft.direction[joint] = float(direction)
            if "direction" not in draft.steps_done:
                draft.steps_done.append("direction")
            self._guard(lambda: self._push_calibration(arm_id, draft))
            return self.calibration_get(arm)

    def calibration_range_start(self, arm: str) -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            self._require_released(arm_id)
            state = self._dev().snapshot.arms.get(arm_id)
            if not state:
                raise _err("arm.no_feedback", "this arm is not reporting")
            position = np.asarray(state.position)
            self.range_tracking[arm_id] = {"min": position.copy(), "max": position.copy()}
            return self.calibration_get(arm)

    def calibration_range_stop(self, arm: str, margin: float = 0.02) -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            tracked = self.range_tracking.pop(arm_id, None)
            if tracked is None:
                raise _err("calibration.not_started", "range recording was not started")
            draft = self._draft(arm_id)
            spec = self._spec_of_arm(arm_id)
            for j in range(JOINTS - 1):
                low, high = float(tracked["min"][j]), float(tracked["max"][j])
                # A joint nobody moved keeps the published range rather than freezing.
                if high - low < 0.05:
                    continue
                draft.lower[j] = max(low + margin, spec.lower[j])
                draft.upper[j] = min(high - margin, spec.upper[j])
            if "range" not in draft.steps_done:
                draft.steps_done.append("range")
            self._guard(lambda: self._push_calibration(arm_id, draft))
            result = self.calibration_get(arm)
            result["observed"] = {"min": tracked["min"].tolist(), "max": tracked["max"].tolist()}
            return result

    def calibration_capture_gripper(self, arm: str, which: str) -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            self._require_released(arm_id)
            if which not in ("open", "closed"):
                raise _err("request.bad_argument", "which must be open or closed")
            raw = self._raw_position(arm_id)[JOINTS - 1]
            draft = self._draft(arm_id)
            if which == "open":
                draft.gripper_open = raw
            else:
                draft.gripper_closed = raw
            if abs(draft.gripper_open - draft.gripper_closed) > 0.01:
                if "gripper" not in draft.steps_done:
                    draft.steps_done.append("gripper")
                self._guard(lambda: self._push_calibration(arm_id, draft))
            return self.calibration_get(arm)

    def calibration_save(self, arm: str, note: str = "") -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            draft = self._draft(arm_id)
            self._require_released(arm_id)
            if abs(draft.gripper_open - draft.gripper_closed) < 0.01:
                raise _err("calibration.incomplete", "the gripper range is empty; capture open and closed first")
            draft.note = note
            saved = self.store.save_calibration(Calibration(**draft.to_dict()))
            self.drafts[draft.serial] = Calibration(**saved.to_dict())
            self._guard(lambda: self._push_calibration(arm_id, saved))
            self._emit("calibration_saved", arm=arm, serial=saved.serial, version=saved.version)
            return self.calibration_get(arm)

    def calibration_load(self, arm: str, version: int | None = None) -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            self._require_released(arm_id)
            binding = self._binding_of_arm(arm_id)
            if not binding:
                raise _err("arm.not_bound", "this arm is not bound")
            try:
                calibration = self.store.load_calibration(binding["serial"], version)
            except ValueError as exc:
                raise _err("calibration.no_version", str(exc)) from exc
            if calibration is None:
                raise _err("calibration.no_version", f"no saved calibration for {binding['serial']}")
            self.drafts[binding["serial"]] = Calibration(**calibration.to_dict())
            self._guard(lambda: self._push_calibration(arm_id, calibration))
            return self.calibration_get(arm)

    def calibration_list(self) -> list[dict]:
        return self.store.list_calibrations()

    def calibration_verify_joint(self, arm: str, joint: int, amplitude: float = 0.1) -> dict:
        """Jog one joint and report whether it moved the commanded way."""
        arm_id = self._arm_id(arm)
        spec = self._spec_of_arm(arm_id)
        if not spec.can_execute:
            raise _err("arm.unsupported", f"{spec.label} cannot execute: move this joint by hand and watch the readout")
        with self.lock:
            self._claim(int(ControlSource.TEACH))
            device = self._dev()
            state = device.snapshot.arms.get(arm_id)
            if not state or state.valid_mask != 0x7F:
                raise _err("arm.no_feedback", "not every joint of this arm is reporting")
            start = list(state.position)
            lower, upper = self.applied[arm_id].lower[joint], self.applied[arm_id].upper[joint]
            sign = 1.0 if start[joint] + amplitude <= upper else -1.0
            goal = list(start)
            goal[joint] = max(lower, min(upper, start[joint] + sign * amplitude))
        job = self._execute({arm_id: np.vstack([blend(start, goal, 1.0, 0.02), blend(goal, start, 1.0, 0.02)])}, 0.02, int(ControlSource.TEACH))
        peak = start[joint]
        deadline = time.monotonic() + 8.0
        while not job.done.is_set() and time.monotonic() < deadline:
            current = device.snapshot.arms[arm_id].position[joint]
            if abs(current - start[joint]) > abs(peak - start[joint]):
                peak = current
            time.sleep(0.01)
        moved = peak - start[joint]
        return {
            "joint": joint,
            "commanded": goal[joint] - start[joint],
            "moved": moved,
            "direction_ok": moved * (goal[joint] - start[joint]) > 0,
            "tracking_ok": abs(moved - (goal[joint] - start[joint])) < 0.3 * amplitude + 0.03,
            "error": job.error,
        }

    # ------------------------------------------------------------------ pairing

    def _pair_config(self, index: int) -> dict:
        return self.store.config.get("pairs", {}).get(str(index), {})

    def _push_pairs(self) -> None:
        device = self.device
        if not device or not device.is_open:
            return
        for index, (leader, follower) in PAIR_ARMS.items():
            config = self._pair_config(index)
            if not config:
                continue
            enabled = config.get("enabled", False) and self._binding_of_arm(int(leader)) and self._binding_of_arm(int(follower))
            try:
                device.set_pairing(
                    SetPairing(
                        pair=index,
                        leader_arm=int(leader),
                        follower_arm=int(follower),
                        enabled=1 if enabled else 0,
                        gripper_link=1 if config.get("gripper_link", True) else 0,
                        source_joint=list(config.get("source_joint", range(JOINTS))),
                        scale=list(config.get("scale", [1.0] * JOINTS)),
                        offset=list(config.get("offset", [0.0] * JOINTS)),
                    )
                )
            except ControllerError as exc:
                log.warning("could not push pair %d: %s", index, exc)

    def teleop_config(self) -> dict:
        out = {"pairs": {}, "bimanual": self.store.config.get("bimanual", False)}
        for index, (leader, follower) in PAIR_ARMS.items():
            config = dict(self._pair_config(index))
            leader_binding = self._binding_of_arm(int(leader))
            follower_binding = self._binding_of_arm(int(follower))
            if leader_binding and follower_binding:
                # Fields never set read as the preset, so a partial update still gives
                # a complete mapping. What was never configured at all is flagged.
                if "scale" not in config:
                    config["preset"] = True
                preset = default_mapping(SPEC_BY_KEY[leader_binding["model"]], SPEC_BY_KEY[follower_binding["model"]])
                config = {**preset, **config}
            config["leader"] = leader_binding
            config["follower"] = follower_binding
            config.setdefault("enabled", bool(leader_binding and follower_binding))
            out["pairs"][str(index)] = config
        return out

    def teleop_configure(self, pairs: list[dict], bimanual: bool | None = None) -> dict:
        with self.lock:
            if self.teleop_active:
                raise _err("control.busy", "end teleoperation before changing the mapping")
            for entry in pairs:
                index = int(entry["pair"])
                if index not in PAIR_ARMS:
                    raise _err("request.bad_argument", f"no pair {index}")
                for key, size in (("scale", JOINTS), ("offset", JOINTS), ("source_joint", JOINTS)):
                    if key in entry and len(entry[key]) != size:
                        raise _err("request.bad_argument", f"{key} needs {size} values")
                if "source_joint" in entry and any(not 0 <= int(v) < JOINTS for v in entry["source_joint"]):
                    raise _err("request.bad_argument", "source_joint values must name a joint")
                stored = self.store.config.setdefault("pairs", {}).setdefault(str(index), {})
                for key in ("enabled", "gripper_link", "scale", "offset", "source_joint"):
                    if key in entry:
                        stored[key] = entry[key]
            if bimanual is not None:
                self.store.config["bimanual"] = bimanual
            self.store.save_config()
            self._push_pairs()
            return self.teleop_config()

    def _resolve_pairs(self, pairs: list[int] | None) -> list[int]:
        if pairs is None:
            config = self.teleop_config()["pairs"]
            pairs = [int(i) for i, c in config.items() if c.get("enabled") and c.get("leader") and c.get("follower")]
        if not pairs:
            raise _err("teleop.no_pair", "no leader and follower pair is configured")
        return pairs

    def _ensure_pair_defaults(self, pairs: list[int]) -> None:
        """Store the preset for a pair that was never configured, so it is what runs."""
        config = self.teleop_config()["pairs"]
        changed = False
        for index in pairs:
            stored = self.store.config.setdefault("pairs", {}).setdefault(str(index), {})
            if "scale" not in stored:
                preset = config[str(index)]
                for key in ("scale", "offset", "source_joint", "gripper_link"):
                    stored[key] = preset[key]
                changed = True
            stored["enabled"] = True
        if changed:
            self.store.save_config()
        self._push_pairs()

    def teleop_enter(self, pairs: list[int] | None = None, source: int = int(ControlSource.TELEOP), preempt: tuple[int, ...] = ()) -> dict:
        """Put the pair in local teleoperation. The follower holds until align or start."""
        with self.lock:
            self._claim(source, preempt)
            device = self._dev()
            chosen = self._resolve_pairs(pairs)
            self._ensure_pair_defaults(chosen)
            arms = [int(a) for index in chosen for a in PAIR_ARMS[index]]
            for arm in arms:
                if not self._binding_of_arm(arm):
                    raise _err("arm.not_bound", f"{ARM_NAMES[ArmId(arm)]} is not bound to a port")
            self.executor.cancel()
            generation = device.next_generation()
            self._guard(lambda: device.set_mode(Mode.LOCAL_TELEOP, self._mask(arms), source, generation=generation))
            # An RS leader has no brakes: the firmware holds it in guide with this torque.
            for index in chosen:
                leader = int(PAIR_ARMS[index][0])
                if self._spec_of_arm(leader).can_guide:
                    self.guide_arms[leader] = True
            self.teleop_active = True
            self.teleop_pairs = chosen
            self.paused = False
            self._set_source(source)
            self._emit("teleop_entered", pairs=chosen, generation=generation)
            return {"pairs": chosen, "generation": generation}

    def _pair_mask(self, pairs: list[int] | None) -> int:
        if not self.teleop_active:
            raise _err("teleop.not_active", "enter teleoperation first")
        return self._mask(pairs if pairs is not None else self.teleop_pairs)

    def teleop_align(self, pairs: list[int] | None = None, speed: float = 0.5) -> dict:
        with self.lock:
            if speed <= 0:
                raise _err("request.bad_argument", "alignment speed must be positive")
            mask = self._pair_mask(pairs)
            self._guard(lambda: self._dev().teleop(TeleopAction.ALIGN, mask, align_speed=speed))
            self._emit("teleop_align", speed=speed)
            return {"ok": True}

    def teleop_start(self, pairs: list[int] | None = None, follow: str = "absolute") -> dict:
        with self.lock:
            if follow not in ("absolute", "relative"):
                raise _err("request.bad_argument", "follow must be absolute or relative")
            mask = self._pair_mask(pairs)
            mode = FollowMode.RELATIVE if follow == "relative" else FollowMode.ABSOLUTE
            self._guard(lambda: self._dev().teleop(TeleopAction.START, mask, follow_mode=mode))
            self.follow_mode = follow
            self._emit("teleop_started", follow=follow)
            return {"ok": True}

    def teleop_pause(self, pairs: list[int] | None = None) -> dict:
        with self.lock:
            mask = self._pair_mask(pairs)
            self._guard(lambda: self._dev().teleop(TeleopAction.PAUSE, mask))
            self.paused = True
            self._emit("teleop_paused")
            return {"ok": True}

    def teleop_resume(self, pairs: list[int] | None = None) -> dict:
        with self.lock:
            mask = self._pair_mask(pairs)
            self._guard(lambda: self._dev().teleop(TeleopAction.RESUME, mask))
            self.paused = False
            self._emit("teleop_resumed")
            return {"ok": True}

    def teleop_end(self, pairs: list[int] | None = None) -> dict:
        with self.lock:
            if not self.teleop_active:
                return {"ok": True}
            device = self._dev()
            mask = self._pair_mask(pairs)
            chosen = pairs if pairs is not None else self.teleop_pairs
            self._guard(lambda: device.teleop(TeleopAction.END, mask))
            generation = device.next_generation()
            followers = [int(PAIR_ARMS[i][1]) for i in chosen]
            self._guard(lambda: device.set_mode(Mode.HOLD, self._mask(followers), ControlSource.NONE, generation=generation))
            for index in chosen:
                leader = int(PAIR_ARMS[index][0])
                self.guide_arms.pop(leader, None)
                if self._binding_of_arm(leader):
                    self._guard(lambda a=leader: device.set_mode(Mode.IDLE, 1 << a, ControlSource.NONE, generation=generation))
            self.teleop_active = False
            self.teleop_pairs = []
            self._set_source(int(ControlSource.NONE))
            self._emit("teleop_ended")
            return {"ok": True}

    # ---------------------------------------------------------------- execution

    def _on_job_finished(self, job: Job) -> None:
        self._emit("execution_finished", job=job.job_id, error=job.error, cancelled=job.cancelled, source=SOURCE_BY_VALUE.get(job.source))
        with self.lock:
            if job.cancelled or self.source != job.source or job.source == ControlSource.POLICY:
                return
            # A finished move leaves the arm holding under no one's control. The
            # controller keeps the running target through this, so nothing steps.
            device = self.device
            if device and device.is_open and not job.error:
                try:
                    generation = device.next_generation()
                    mask = self._mask([a.arm for a in job.arms])
                    device.set_mode(Mode.HOLD, mask, ControlSource.NONE, generation=generation)
                except ControllerError as exc:
                    log.warning("could not settle after a trajectory: %s", exc)
            self._set_source(int(ControlSource.NONE))

    def _execute(self, arm_points: dict[int, np.ndarray], dt_s: float, source: int) -> Job:
        """The one execution entry for computer-side targets."""
        with self.lock:
            device = self._dev()
            self._claim(source)
            for arm, points in arm_points.items():
                spec = self._spec_of_arm(arm)
                if not spec.can_execute:
                    raise _err("arm.unsupported", f"{spec.label} cannot execute")
                if points.ndim != 2 or points.shape[1] != JOINTS:
                    raise _err("request.bad_argument", f"each point needs {JOINTS} values")
                if not np.isfinite(points).all():
                    raise _err("request.bad_argument", "trajectory contains NaN or infinity")
            generation = device.next_generation()
            mask = self._mask(list(arm_points))
            self._guard(lambda: device.set_mode(Mode.HOST_TARGET, mask, source, generation=generation))
            self.paused = False
            self._set_source(source)
            job = self.executor.submit(arm_points, dt_s, source, generation)
            self._emit(
                "execution_started",
                job=job.job_id,
                source=SOURCE_BY_VALUE.get(source),
                generation=generation,
                arms=[ARM_NAMES[ArmId(a)] for a in arm_points],
                points=max(len(p) for p in arm_points.values()),
            )
            return job

    def _current_pose(self, arm: int) -> list[float]:
        device = self._dev()
        state = device.snapshot.arms.get(arm)
        if not state or state.valid_mask != 0x7F:
            raise _err("arm.no_feedback", f"not every joint of {ARM_NAMES[ArmId(arm)]} is reporting")
        return list(state.position)

    def _prepare_points(self, arm: int, points, dt_s: float, start_from_current: bool, blend_s: float) -> np.ndarray:
        array = np.asarray(points, dtype=float)
        if array.size == 0:
            raise _err("request.bad_argument", "the trajectory is empty")
        if dt_s < 0.002:
            raise _err("request.bad_argument", "point spacing must be at least 2 ms")
        if start_from_current:
            spec = self._spec_of_arm(arm)
            current = self._current_pose(arm)
            duration = max(blend_s, min_duration(current, array[0], spec.velocity_limit))
            array = np.vstack([blend(current, array[0], duration, dt_s), array])
        return array

    def motion_execute(self, arm: str, points, dt_s: float, source: str, start_from_current: bool = True, blend_s: float = 1.0, wait: bool = False) -> dict:
        arm_id = self._arm_id(arm)
        source_id = self._source_id(source)
        prepared = self._prepare_points(arm_id, points, dt_s, start_from_current, blend_s)
        job = self._execute({arm_id: prepared}, dt_s, source_id)
        if wait:
            job.done.wait()
        return {"job": job.job_id, "points": len(prepared), "duration_s": len(prepared) * dt_s, "error": job.error}

    def motion_execute_many(self, trajectories: dict[str, list], dt_s: float, source: str, blend_s: float = 1.0) -> dict:
        source_id = self._source_id(source)
        prepared = {}
        for arm, points in trajectories.items():
            arm_id = self._arm_id(arm)
            prepared[arm_id] = self._prepare_points(arm_id, points, dt_s, True, blend_s)
        # Arms start together, so the approach is padded to the longest one.
        longest = max(len(p) for p in prepared.values())
        for arm_id, points in prepared.items():
            if len(points) < longest:
                prepared[arm_id] = np.vstack([points, np.tile(points[-1], (longest - len(points), 1))])
        job = self._execute(prepared, dt_s, source_id)
        return {"job": job.job_id, "points": longest, "duration_s": longest * dt_s}

    def _source_id(self, source: str) -> int:
        if source not in SOURCE_NAMES or source == "none":
            raise _err("request.bad_argument", f"unknown control source {source!r}")
        return SOURCE_NAMES[source]

    def motion_move_to(self, arm: str, pose, duration_s: float = 2.0, source: str = "teach") -> dict:
        return self.motion_execute(arm, [pose], 0.02, source, True, duration_s)

    def motion_set_target(self, arm: str, pose, source: str = "teach") -> dict:
        """Live target for a slider or an end-effector drag. Latest wins."""
        with self.lock:
            arm_id = self._arm_id(arm)
            device = self._dev()
            source_id = self._source_id(source)
            spec = self._spec_of_arm(arm_id)
            if not spec.can_execute:
                raise _err("arm.unsupported", f"{spec.label} cannot execute")
            pose = [float(v) for v in pose]
            if len(pose) != JOINTS or not np.isfinite(pose).all():
                raise _err("request.bad_argument", f"a pose needs {JOINTS} finite values")
            state = device.snapshot.arms.get(arm_id)
            mode_ok = state is not None and state.mode == Mode.HOST_TARGET and self.source == source_id
            if not mode_ok:
                self._claim(source_id)
                generation = device.next_generation()
                self._guard(lambda: device.set_mode(Mode.HOST_TARGET, 1 << arm_id, source_id, generation=generation))
                self._set_source(source_id)
            ack = device.send_points(arm_id, [pose], 20_000, self.policy_exec_seq, source_id)
            self.policy_exec_seq += 1
            if ack.status not in (AckStatus.OK, AckStatus.QUEUE_FULL):
                raise _err("control.rejected", f"target rejected: {ack.status.name}")
            return {"ok": True, "queue_free": ack.queue_free}

    def motion_home(self, arms: list[str] | None = None, duration_s: float = 3.0) -> dict:
        names = arms or [a["arm"] for a in self.status()["arms"] if a.get("bound") and a["arm"].endswith("follower")]
        if not names:
            raise _err("arm.not_bound", "no follower arm is bound")
        trajectories = {}
        for name in names:
            arm = self._arm_id(name)
            spec = self._spec_of_arm(arm)
            trajectories[name] = [list(spec.home)]
        result = self.motion_execute_many(trajectories, 0.02, "home", duration_s)
        return result

    # -------------------------------------------------------------- guide mode

    def guide_start(self, arm: str, gravity: bool = True) -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            self._claim(int(ControlSource.GUIDE))
            spec = self._spec_of_arm(arm_id)
            if not spec.can_guide:
                raise _err("arm.unsupported", f"{spec.label} has no low-impedance mode")
            device = self._dev()
            generation = device.next_generation()
            self.guide_arms[arm_id] = gravity
            self._guard(lambda: device.set_mode(Mode.GUIDE, 1 << arm_id, ControlSource.GUIDE, generation=generation))
            self._set_source(int(ControlSource.GUIDE))
            self._emit("guide_started", arm=arm, gravity=gravity)
            return {"ok": True, "gravity": gravity}

    def guide_stop(self, arm: str) -> dict:
        with self.lock:
            arm_id = self._arm_id(arm)
            device = self._dev()
            if arm_id not in self.guide_arms:
                return {"ok": True}
            self._guard(lambda: device.stop(1 << arm_id, StopKind.HOLD))
            self.guide_arms.pop(arm_id, None)
            if not self.guide_arms:
                self._set_source(int(ControlSource.NONE))
            self._emit("guide_stopped", arm=arm)
            return {"ok": True}

    # ------------------------------------------------------------- run control

    def _stop_everything(self, kind: int) -> None:
        device = self.device
        self.executor.cancel()
        self._suspend_policy("stopped")
        self.teleop_active = False
        self.teleop_pairs = []
        self.guide_arms.clear()
        self.paused = False
        if self.intervention_active:
            self._close_intervention("stopped")
        self.policy_state = "idle"
        self._terminate_policy_process()
        self._set_source(int(ControlSource.NONE))
        if device and device.is_open:
            device.next_generation()
            device.stop(0xF, kind)

    def run_stop(self, kind: str = "hold") -> dict:
        """One-button stop: clears the computer and controller queues, then holds or damps."""
        kinds = {"hold": StopKind.HOLD, "damp": StopKind.DAMP, "release": StopKind.RELEASE}
        if kind not in kinds:
            raise _err("request.bad_argument", "kind must be hold, damp or release")
        with self.lock:
            self._guard(lambda: self._stop_everything(kinds[kind]))
            self._emit("run_stopped", stop_kind=kind)
            return {"ok": True}

    def run_pause(self) -> dict:
        with self.lock:
            device = self._dev()
            if self.source == ControlSource.NONE:
                raise _err("control.idle", "nothing is running")
            if self.teleop_active:
                self._guard(lambda: device.teleop(TeleopAction.PAUSE, self._mask(self.teleop_pairs)))
            else:
                arms = [a for a in range(4) if self._binding_of_arm(a) and a in (1, 3)]
                self.executor.set_paused(True)
                if self.policy_state == "running":
                    self.policy_state = "paused"
                self._guard(lambda: device.pause(self._mask(arms), True))
            self.paused = True
            self._emit("run_paused")
            return {"ok": True}

    def run_resume(self) -> dict:
        with self.lock:
            device = self._dev()
            if not self.paused:
                return {"ok": True}
            if self.teleop_active:
                self._guard(lambda: device.teleop(TeleopAction.RESUME, self._mask(self.teleop_pairs)))
            else:
                arms = [a for a in range(4) if self._binding_of_arm(a) and a in (1, 3)]
                self._guard(lambda: device.pause(self._mask(arms), False))
                self.executor.set_paused(False)
                if self.policy_state == "paused":
                    self.policy_state = "running"
            self.paused = False
            self._emit("run_resumed")
            return {"ok": True}

    def run_reset_estop(self) -> dict:
        with self.lock:
            device = self._dev()
            device.next_generation()
            self._guard(device.reset_estop)
            self._emit("estop_reset")
            return {"ok": True}

    def run_release(self, arms: list[str] | None = None) -> dict:
        """Release torque. Callers should home first: an RS arm has no brakes."""
        with self.lock:
            if self.source != ControlSource.NONE:
                raise _err("control.busy", "stop the current run first")
            device = self._dev()
            names = arms or [a["arm"] for a in self.status()["arms"] if a.get("bound")]
            mask = self._mask([self._arm_id(n) for n in names])
            self._guard(lambda: device.torque(mask, False))
            self._emit("torque_released", arms=names)
            return {"ok": True}

    # ------------------------------------------------------------------ policy

    def policy_status(self) -> dict:
        process = self.policy_process
        return {
            "state": self.policy_state,
            "token": self.policy_token,
            "config": {k: v for k, v in (self.policy_config or {}).items() if k != "secrets"},
            "buffered": len(self.policy_buffer) if self.policy_buffer else 0,
            "process_alive": bool(process and process.poll() is None),
            "stats": self.policy_stats,
        }

    def policy_start(self, config: dict) -> dict:
        with self.lock:
            if self.policy_state in ("running", "paused"):
                raise _err("control.busy", "a policy is already running")
            self._claim(int(ControlSource.POLICY))
            device = self._dev()
            arms = [self._arm_id(a) for a in config.get("arms", [])]
            if not arms:
                raise _err("request.bad_argument", "a policy needs at least one arm")
            for arm in arms:
                spec = self._spec_of_arm(arm)
                if not spec.can_execute:
                    raise _err("arm.unsupported", f"{spec.label} cannot execute")
            inference_hz = float(config.get("inference_hz", 5.0))
            action_hz = float(config.get("action_hz", 30.0))
            if not (0 < inference_hz <= 100 and 0 < action_hz <= 500):
                raise _err("request.bad_argument", "inference and action rates are out of range")
            generation = device.next_generation()
            self._guard(lambda: device.set_mode(Mode.HOST_TARGET, self._mask(arms), ControlSource.POLICY, generation=generation))
            self.policy_config = {**config, "inference_hz": inference_hz, "action_hz": action_hz}
            self.policy_arms = arms
            self.policy_buffer = make_buffer(config.get("strategy", "stream"))
            self.policy_token += 1
            self.policy_reset_requested = True
            self.policy_state = "running"
            self.paused = False
            self.policy_stats = {"inferences": 0, "last_inference_ms": None, "actions_sent": 0, "rejected": 0, "started_at": time.time()}
            self._set_source(int(ControlSource.POLICY))
            self._spawn_policy_process()
            self._emit("policy_started", token=self.policy_token, generation=generation, backend=config.get("backend"), task=config.get("task"))
            return self.policy_status()

    def _spawn_policy_process(self) -> None:
        config = self.policy_config or {}
        if not config.get("spawn_worker", True):
            return
        env = {**os.environ, "RAC_POLICY_CONFIG": json.dumps(config)}
        self.policy_process = subprocess.Popen([sys.executable, "-m", "rac_policy.worker"], env=env)

    def _terminate_policy_process(self) -> None:
        process = self.policy_process
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        self.policy_process = None

    def _suspend_policy(self, reason: str) -> None:
        """Invalidate every result still being computed and drop what is queued here."""
        if self.policy_state in ("running", "paused"):
            self.policy_token += 1
            dropped = self.policy_buffer.clear() if self.policy_buffer else 0
            self.policy_state = "suspended"
            self._emit("policy_suspended", reason=reason, dropped=dropped, token=self.policy_token)

    def policy_stop(self) -> dict:
        with self.lock:
            self._suspend_policy("stopped")
            self.policy_state = "idle"
            self._terminate_policy_process()
            device = self.device
            if device and device.is_open and self.source in (ControlSource.POLICY, ControlSource.INTERVENTION):
                device.next_generation()
                self._guard(lambda: device.stop(self._mask(self.policy_arms), StopKind.HOLD))
            if self.source in (ControlSource.POLICY, ControlSource.INTERVENTION):
                self._set_source(int(ControlSource.NONE))
            self.intervention_active = False
            self._emit("policy_stopped")
            return self.policy_status()

    def policy_pause(self) -> dict:
        with self.lock:
            if self.policy_state != "running":
                raise _err("policy.not_running", "no running policy to pause")
            return self.run_pause()

    def policy_resume(self) -> dict:
        with self.lock:
            if self.policy_state != "paused":
                raise _err("policy.not_paused", "the policy is not paused")
            return self.run_resume()

    def policy_worker_poll(self, token: int | None = None) -> dict:
        """The worker asks what to do next. The reset flag fires once per run or handback."""
        with self.lock:
            reset = self.policy_reset_requested
            self.policy_reset_requested = False
            return {
                "state": self.policy_state,
                "token": self.policy_token,
                "reset": reset,
                "config": self.policy_config,
            }

    def policy_submit(self, chunk, token: int, observation_ts_us: int, inference_ms: float) -> dict:
        """A model delivers an action chunk. Results computed before a takeover are refused."""
        with self.lock:
            if token != self.policy_token or self.policy_state not in ("running", "paused"):
                self.policy_stats["rejected"] = self.policy_stats.get("rejected", 0) + 1
                self._emit("policy_result_discarded", token=token, current=self.policy_token)
                raise _err("policy.stale_token", "this result predates the current control state")
            actions = np.asarray(chunk, dtype=np.float32)
            width = len(self.policy_arms) * JOINTS
            if actions.ndim != 2 or actions.shape[1] != width:
                raise _err("policy.bad_shape", f"expected T x {width} actions, got {actions.shape}")
            if not np.isfinite(actions).all():
                raise _err("policy.bad_values", "action chunk contains NaN or infinity")
            config = self.policy_config or {}
            dt = 1.0 / config.get("action_hz", 30.0)
            age_steps = int(max(0, host_now_us() - observation_ts_us) / 1e6 / dt)
            self.policy_buffer.integrate(actions, max_trim=age_steps)
            self.policy_stats["inferences"] = self.policy_stats.get("inferences", 0) + 1
            self.policy_stats["last_inference_ms"] = inference_ms
            self.policy_stats["last_chunk_len"] = len(actions)
            return {"buffered": len(self.policy_buffer)}

    def _check_policy_process(self) -> None:
        """A model process that ended on its own ends the run, visibly.

        Otherwise the buffer just runs dry and the page keeps saying the model is
        running while the arm stands still.
        """
        process = self.policy_process
        if process is None or process.poll() is None:
            return
        with self.lock:
            if self.policy_process is process and self.policy_state in ("running", "paused"):
                self._emit("policy_failed", severity=2, exit_code=process.returncode)
                self.policy_stop()

    def _policy_loop(self) -> None:
        """Feed buffered actions to the controller at the configured action rate."""
        # perf_counter, because monotonic ticks at 15.6 ms on Windows with Python 3.12.
        next_tick = time.perf_counter()
        while not self._stop.is_set():
            self._check_policy_process()
            config = self.policy_config
            if self.policy_state != "running" or not config or not self.policy_buffer:
                time.sleep(0.01)
                next_tick = time.perf_counter()
                continue
            dt = 1.0 / config.get("action_hz", 30.0)
            next_tick += dt
            delay = next_tick - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.perf_counter()
            device = self.device
            if not device or not device.is_open:
                continue
            with self.lock:
                if self.policy_state != "running":
                    continue
                action = self.policy_buffer.pop()
                if action is None:
                    continue
                dt_us = int(dt * 1e6)
                try:
                    for index, arm in enumerate(self.policy_arms):
                        slice_ = action[index * JOINTS : (index + 1) * JOINTS].tolist()
                        ack = device.send_points(arm, [slice_], dt_us, self.policy_exec_seq, ControlSource.POLICY)
                        if not ack.ok and ack.status != AckStatus.QUEUE_FULL:
                            self.policy_stats["rejected"] = self.policy_stats.get("rejected", 0) + 1
                    self.policy_exec_seq += 1
                    self.policy_stats["actions_sent"] = self.policy_stats.get("actions_sent", 0) + 1
                except ControllerError as exc:
                    log.warning("policy action not sent: %s", exc)

    # ------------------------------------------------------------- intervention

    def intervention_status(self) -> dict:
        return {
            "active": self.intervention_active,
            "started_us": self.intervention_started_us if self.intervention_active else None,
            "segments": self.intervention_segments[-20:],
        }

    def _close_intervention(self, outcome: str) -> None:
        now = host_now_us()
        segment = {"start_us": self.intervention_started_us, "end_us": now, "outcome": outcome}
        self.intervention_segments.append(segment)
        self.intervention_active = False
        self._emit("intervention_end", outcome=outcome, start_us=self.intervention_started_us, end_us=now)

    def intervention_take_over(self, follow: str = "relative", speed: float = 0.5) -> dict:
        """Hand control from the model to a person.

        In order: invalidate in-flight inference, drop queued actions on both sides
        under a new generation, then either align or take a relative offset so the
        follower does not jump to wherever the leader happens to be.
        """
        with self.lock:
            if self.policy_state not in ("running", "paused"):
                raise _err("intervention.no_policy", "no policy is running to take over from")
            if follow not in ("relative", "align"):
                raise _err("request.bad_argument", "follow must be relative or align")
            self._suspend_policy("takeover")
            self.intervention_active = True
            self.intervention_started_us = host_now_us()
            self._emit("intervention_begin", follow=follow, start_us=self.intervention_started_us)
            sides = [i for i, (_, f) in PAIR_ARMS.items() if int(f) in self.policy_arms]
            self.source = int(ControlSource.INTERVENTION)
            try:
                result = self.teleop_enter(
                    sides,
                    source=int(ControlSource.INTERVENTION),
                    preempt=(int(ControlSource.POLICY), int(ControlSource.INTERVENTION)),
                )
                if follow == "align":
                    self.teleop_align(sides, speed)
                else:
                    self.teleop_start(sides, "relative")
            except ServiceError:
                # No leader to take over with: the arms hold, and the run stays suspended.
                self._close_intervention("failed")
                self._set_source(int(ControlSource.NONE))
                raise
            return {"ok": True, "follow": follow, **result}

    def intervention_confirm_aligned(self) -> dict:
        with self.lock:
            if not self.intervention_active:
                raise _err("intervention.not_active", "no intervention in progress")
            return self.teleop_start(self.teleop_pairs, "absolute")

    def intervention_hand_back(self) -> dict:
        """Return control to the model. It re-observes and starts from a clean cache."""
        with self.lock:
            if not self.intervention_active:
                raise _err("intervention.not_active", "no intervention in progress")
            device = self._dev()
            pairs = self.teleop_pairs
            self._guard(lambda: device.teleop(TeleopAction.END, self._mask(pairs)))
            for index in pairs:
                leader = int(PAIR_ARMS[index][0])
                self.guide_arms.pop(leader, None)
            self.teleop_active = False
            self.teleop_pairs = []
            generation = device.next_generation()
            self._guard(lambda: device.set_mode(Mode.HOST_TARGET, self._mask(self.policy_arms), ControlSource.POLICY, generation=generation))
            for index in pairs:
                leader = int(PAIR_ARMS[index][0])
                if self._binding_of_arm(leader):
                    self._guard(lambda a=leader: device.set_mode(Mode.IDLE, 1 << a, ControlSource.NONE, generation=generation))
            self._close_intervention("handed_back")
            if self.policy_buffer:
                self.policy_buffer.clear()
            self.policy_token += 1
            self.policy_reset_requested = True
            self.policy_state = "running"
            self.paused = False
            self._set_source(int(ControlSource.POLICY))
            self._emit("policy_resumed_after_intervention", token=self.policy_token, generation=generation)
            return {"ok": True, "token": self.policy_token, "generation": generation}

    def intervention_mark(self, label: str, note: str = "") -> dict:
        self._emit("mark", label=label, note=note)
        return {"ok": True}

    # -------------------------------------------------------------- simulation

    def sil_inject(self, op: str, arg: int = 0, arg2: int = 0, values: list[float] | None = None) -> dict:
        device = self._dev()
        info = device.snapshot.info
        if not info or not info.capabilities & 0x10:
            raise _err("device.not_simulated", "fault injection exists only in the simulator")
        try:
            op_id = SilOp[op]
        except KeyError:
            raise _err("request.bad_argument", f"unknown simulation operation {op!r}") from None
        self._guard(lambda: device.sil_inject(op_id, arg, arg2, values))
        return {"ok": True}


def describe_models() -> list[dict]:
    return [s.describe() for s in SPECS.values()]


__all__ = ["Runtime", "describe_models", "spec_of", "Path"]
