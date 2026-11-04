"""Python SDK for the controller's wire protocol.

``ControllerDevice`` is the one object that talks to the board. Only the
Controller Service should own one: everything else in the platform goes through
the service, because two owners of the link would fight over control generations.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass, field

from ._gen import Ack as WireAck
from ._gen import (
    JOINTS,
    MAX_CHUNK_POINTS,
    AckStatus,
    ArmId,
    ArmState,
    BindPort,
    CanDialect,
    ControllerStatus,
    DeviceInfo,
    Event,
    EventCode,
    Feedforward,
    Header,
    MsgType,
    PauseResume,
    PortStatus,
    SetCalibration,
    SetLimits,
    SetMode,
    SetPairing,
    SetRates,
    SilInject,
    SilOp,
    Stop,
    StopKind,
    TargetChunk,
    TeleopControl,
    TimeSyncReply,
    Torque,
)
from .clock import ClockModel, host_now_us
from .transport import Transport, open_transport
from .wire import Deframer, build_frame, make_header

log = logging.getLogger(__name__)

ACK_TIMEOUT_S = 1.0
HEARTBEAT_PERIOD_S = 0.1
TIME_SYNC_PERIOD_S = 0.25


class ControllerError(RuntimeError):
    def __init__(self, message: str, status: AckStatus | None = None) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class Ack:
    status: AckStatus
    exec_seq: int
    queue_free: int

    @property
    def ok(self) -> bool:
        return self.status == AckStatus.OK


@dataclass
class SampleStats:
    """Effective rate and missing samples for one arm, over a sliding second."""

    count: int = 0
    missing: int = 0
    invalid: int = 0
    last_seq: int | None = None
    window_start_us: int = 0
    window_count: int = 0
    rate_hz: float = 0.0

    def update(self, seq: int, valid_mask: int, now_us: int) -> None:
        self.count += 1
        if self.last_seq is not None and seq > self.last_seq + 1:
            self.missing += seq - self.last_seq - 1
        self.last_seq = seq
        if valid_mask != 0x7F:
            self.invalid += 1
        if not self.window_start_us:
            self.window_start_us = now_us
        self.window_count += 1
        elapsed = now_us - self.window_start_us
        if elapsed >= 1_000_000:
            self.rate_hz = self.window_count * 1e6 / elapsed
            self.window_start_us = now_us
            self.window_count = 0


@dataclass
class DeviceSnapshot:
    info: DeviceInfo | None = None
    ports: dict[int, PortStatus] = field(default_factory=dict)
    status: ControllerStatus | None = None
    arms: dict[int, ArmState] = field(default_factory=dict)
    arm_host_ts: dict[int, float] = field(default_factory=dict)
    generation: int = 0
    connected: bool = False
    last_rx_host_us: int = 0


class ControllerDevice:
    def __init__(self, uri: str, heartbeat: bool = True) -> None:
        self.uri = uri
        self._transport: Transport | None = None
        self._heartbeat = heartbeat
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._pending: dict[int, tuple[threading.Event, list]] = {}
        self._pending_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._deframer = Deframer()
        self._sync_sent: dict[int, int] = {}

        self.clock = ClockModel()
        self.snapshot = DeviceSnapshot()
        self.stats: dict[int, SampleStats] = defaultdict(SampleStats)
        self.generation = 0

        self.on_state: Callable[[int, ArmState, float], None] | None = None
        self.on_event: Callable[[Event, float], None] | None = None
        self.on_status: Callable[[ControllerStatus], None] | None = None
        self.on_disconnect: Callable[[str], None] | None = None

    # ------------------------------------------------------------- lifecycle

    def open(self) -> DeviceInfo:
        """Establish communication only. Nothing here can energize a motor."""
        self._transport = open_transport(self.uri)
        self._stop.clear()
        self.snapshot.connected = True
        reader = threading.Thread(target=self._read_loop, name="rac-link-rx", daemon=True)
        reader.start()
        self._threads = [reader]
        if self._heartbeat:
            beat = threading.Thread(target=self._beat_loop, name="rac-link-beat", daemon=True)
            beat.start()
            self._threads.append(beat)

        self._command(MsgType.HELLO)
        deadline = time.monotonic() + ACK_TIMEOUT_S
        while self.snapshot.info is None and time.monotonic() < deadline:
            time.sleep(0.005)
        if self.snapshot.info is None:
            self.close()
            raise ControllerError("controller did not identify itself")
        return self.snapshot.info

    def close(self) -> None:
        self._stop.set()
        self.snapshot.connected = False
        if self._transport:
            self._transport.close()
        current = threading.current_thread()
        for thread in self._threads:
            if thread is not current:
                thread.join(timeout=1.0)
        self._threads = []
        self._transport = None
        with self._pending_lock:
            for event, _ in self._pending.values():
                event.set()
            self._pending.clear()

    @property
    def is_open(self) -> bool:
        return self.snapshot.connected

    # -------------------------------------------------------------- transmit

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            return self._seq

    def _send(self, msg_type: int, body: bytes = b"", arm: int = ArmId.NONE, generation: int | None = None) -> int:
        seq = self._next_seq()
        gen = self.generation if generation is None else generation
        header = make_header(msg_type, seq, gen, host_now_us(), arm)
        frame = build_frame(header + body)
        if not self._transport:
            raise ControllerError("controller is not connected")
        with self._write_lock:
            try:
                self._transport.write(frame)
            except OSError as exc:
                self._on_link_lost(str(exc))
                raise ControllerError(f"link write failed: {exc}") from exc
        return seq

    def _command(
        self,
        msg_type: int,
        body: bytes = b"",
        arm: int = ArmId.NONE,
        generation: int | None = None,
        timeout: float = ACK_TIMEOUT_S,
    ) -> Ack:
        event = threading.Event()
        slot: list = []
        # Registered before sending: the ack can beat the return from write().
        seq_probe = self._next_seq()
        with self._pending_lock:
            self._pending[seq_probe] = (event, slot)
        gen = self.generation if generation is None else generation
        frame = build_frame(make_header(msg_type, seq_probe, gen, host_now_us(), arm) + body)
        if not self._transport:
            raise ControllerError("controller is not connected")
        try:
            with self._write_lock:
                self._transport.write(frame)
        except OSError as exc:
            with self._pending_lock:
                self._pending.pop(seq_probe, None)
            self._on_link_lost(str(exc))
            raise ControllerError(f"link write failed: {exc}") from exc

        if not event.wait(timeout):
            with self._pending_lock:
                self._pending.pop(seq_probe, None)
            raise ControllerError(f"no acknowledgement for {MsgType(msg_type).name}")
        if not slot:
            raise ControllerError("link closed while waiting for an acknowledgement")
        return slot[0]

    def _expect_ok(self, ack: Ack, what: str) -> Ack:
        if not ack.ok:
            raise ControllerError(f"{what} rejected: {ack.status.name}", ack.status)
        return ack

    # --------------------------------------------------------------- commands

    def discover(self) -> None:
        self._expect_ok(self._command(MsgType.DISCOVER), "discover")

    def bind_port(self, port: int, arm: int, model: int, dialect: int = CanDialect.RS_PRIVATE, baud: int = 1_000_000) -> None:
        body = BindPort(port=int(port), arm_id=int(arm), model=int(model), can_dialect=int(dialect), baudrate=baud).pack()
        self._expect_ok(self._command(MsgType.BIND_PORT, body), f"bind port {port}")

    def set_calibration(self, arm: int, cal: SetCalibration) -> None:
        self._expect_ok(self._command(MsgType.SET_CALIBRATION, cal.pack(), arm), "set calibration")

    def set_limits(self, arm: int, limits: SetLimits) -> None:
        self._expect_ok(self._command(MsgType.SET_LIMITS, limits.pack(), arm), "set limits")

    def set_pairing(self, pairing: SetPairing) -> None:
        self._expect_ok(self._command(MsgType.SET_PAIRING, pairing.pack()), "set pairing")

    def set_rates(self, rates: SetRates) -> None:
        self._expect_ok(self._command(MsgType.SET_RATES, rates.pack()), "set rates")

    def next_generation(self) -> int:
        self.generation = max(self.generation, self.snapshot.generation) + 1
        return self.generation

    def set_mode(self, mode: int, arm_mask: int, source: int, generation: int | None = None) -> None:
        body = SetMode(mode=int(mode), arm_mask=arm_mask, source=int(source)).pack()
        self._expect_ok(self._command(MsgType.SET_MODE, body, generation=generation), "set mode")

    def teleop(self, action: int, pair_mask: int, follow_mode: int = 0, align_speed: float = 0.5) -> None:
        body = TeleopControl(action=int(action), follow_mode=int(follow_mode), pair_mask=pair_mask, align_speed=align_speed).pack()
        self._expect_ok(self._command(MsgType.TELEOP_CONTROL, body), "teleop control")

    def stop(self, arm_mask: int = 0xF, kind: int = StopKind.HOLD) -> None:
        body = Stop(kind=int(kind), arm_mask=arm_mask).pack()
        self._expect_ok(self._command(MsgType.STOP, body), "stop")

    def pause(self, arm_mask: int, pause: bool) -> None:
        body = PauseResume(pause=1 if pause else 0, arm_mask=arm_mask).pack()
        self._expect_ok(self._command(MsgType.PAUSE_RESUME, body), "pause")

    def feedforward(self, arm: int, torque: list[float], valid_ms: int = 200) -> None:
        # Sent unacknowledged at the guide rate; a lost one just expires.
        self._send(MsgType.FEEDFORWARD, Feedforward(valid_ms=valid_ms, torque=list(torque)).pack(), arm)

    def torque(self, arm_mask: int, enable: bool) -> None:
        body = Torque(enable=1 if enable else 0, arm_mask=arm_mask).pack()
        self._expect_ok(self._command(MsgType.TORQUE, body), "torque")

    def reset_estop(self) -> None:
        self._expect_ok(self._command(MsgType.RESET_ESTOP), "reset e-stop")

    def query_status(self) -> None:
        self._send(MsgType.QUERY_STATUS)

    def sil_inject(self, op: int, arg: int = 0, arg2: int = 0, values: list[float] | None = None) -> None:
        body = SilInject(op=int(op), arg=arg, arg2=arg2, values=list(values or [0.0] * JOINTS)).pack()
        self._expect_ok(self._command(MsgType.SIL_INJECT, body), "sil inject")

    def send_points(
        self,
        arm: int,
        points: list[list[float]],
        dt_us: int,
        exec_seq: int,
        source: int,
        t_start_us: int = 0,
        generation: int | None = None,
    ) -> Ack:
        """Queue up to MAX_CHUNK_POINTS timed targets. Returns the raw ack: QUEUE_FULL is a normal answer."""
        if len(points) > MAX_CHUNK_POINTS:
            raise ValueError(f"at most {MAX_CHUNK_POINTS} points per chunk")
        head = TargetChunk(
            exec_seq=exec_seq,
            dt_us=dt_us,
            t_start_us=t_start_us,
            point_count=len(points),
            joint_count=JOINTS,
            source=int(source),
        ).pack()
        flat = [value for point in points for value in point]
        body = head + struct.pack(f"<{len(flat)}f", *flat)
        return self._command(MsgType.TARGET_CHUNK, body, arm, generation=generation)

    # --------------------------------------------------------------- receive

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._transport.read() if self._transport else b""
            except OSError as exc:
                self._on_link_lost(str(exc))
                return
            if not data:
                continue
            for payload in self._deframer.feed(data):
                self._dispatch(payload)

    def _beat_loop(self) -> None:
        next_beat = 0.0
        next_sync = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            try:
                if now >= next_sync:
                    next_sync = now + TIME_SYNC_PERIOD_S
                    t1 = host_now_us()
                    seq = self._next_seq()
                    self._sync_sent[seq] = t1
                    frame = build_frame(make_header(MsgType.TIME_SYNC, seq, self.generation, t1))
                    if self._transport:
                        with self._write_lock:
                            self._transport.write(frame)
                if now >= next_beat:
                    next_beat = now + HEARTBEAT_PERIOD_S
                    self._send(MsgType.HEARTBEAT)
            except (OSError, ControllerError) as exc:
                self._on_link_lost(str(exc))
                return
            time.sleep(0.01)

    def _on_link_lost(self, reason: str) -> None:
        if not self.snapshot.connected:
            return
        self.snapshot.connected = False
        self._stop.set()
        log.warning("controller link lost: %s", reason)
        with self._pending_lock:
            for event, _ in self._pending.values():
                event.set()
            self._pending.clear()
        if self.on_disconnect:
            self.on_disconnect(reason)

    def _dispatch(self, payload: bytes) -> None:
        now = host_now_us()
        if len(payload) < Header.SIZE:
            return
        header = Header.unpack(payload)
        body = memoryview(payload)[Header.SIZE :]
        self.snapshot.last_rx_host_us = now

        # The controller can move the generation itself (e-stop, comm loss), and
        # commands must then be issued above it to be accepted.
        if header.generation != self.snapshot.generation:
            self.snapshot.generation = header.generation
        if header.generation > self.generation:
            self.generation = header.generation

        kind = header.msg_type
        try:
            if kind == MsgType.ARM_STATE:
                state = ArmState.unpack(body)
                host_ts = self.clock.mcu_to_host(state.sample_ts_us) if self.clock.synced else float(now)
                self.snapshot.arms[header.arm_id] = state
                self.snapshot.arm_host_ts[header.arm_id] = host_ts
                self.stats[header.arm_id].update(state.sample_seq, state.valid_mask, now)
                if self.on_state:
                    self.on_state(header.arm_id, state, host_ts)
            elif kind == MsgType.ACK:
                wire = WireAck.unpack(body)
                ack = Ack(AckStatus(wire.status), wire.exec_seq, wire.queue_free)
                with self._pending_lock:
                    pending = self._pending.pop(wire.acked_seq, None)
                if pending:
                    pending[1].append(ack)
                    pending[0].set()
            elif kind == MsgType.CONTROLLER_STATUS:
                status = ControllerStatus.unpack(body)
                self.snapshot.status = status
                if self.on_status:
                    self.on_status(status)
            elif kind == MsgType.PORT_STATUS:
                port = PortStatus.unpack(body)
                self.snapshot.ports[port.port] = port
            elif kind == MsgType.DEVICE_INFO:
                self.snapshot.info = DeviceInfo.unpack(body)
            elif kind == MsgType.EVENT:
                event = Event.unpack(body)
                host_ts = self.clock.mcu_to_host(event.event_ts_us) if self.clock.synced else float(now)
                if self.on_event:
                    self.on_event(event, host_ts)
            elif kind == MsgType.TIME_SYNC_REPLY:
                reply = TimeSyncReply.unpack(body)
                self._sync_sent.pop(next((s for s, t in self._sync_sent.items() if t == reply.host_t1_us), -1), None)
                self.clock.add_exchange(reply.host_t1_us, reply.mcu_t2_us, reply.mcu_t3_us, now)
        except (struct.error, ValueError) as exc:
            log.warning("bad %s message: %s", kind, exc)


__all__ = [
    "Ack",
    "ControllerDevice",
    "ControllerError",
    "DeviceSnapshot",
    "EventCode",
    "SampleStats",
    "SilOp",
]
