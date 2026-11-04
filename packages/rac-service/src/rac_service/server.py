"""IPC server of the Controller Service. Binds loopback only."""

from __future__ import annotations

import argparse
import hmac
import logging
import queue
import signal
import socket
import threading
from collections.abc import Callable

from rac_link.ipc import DEFAULT_PORT, ServiceError, recv_message, send_message, write_endpoint

from .runtime import Runtime, describe_models
from .store import Store

log = logging.getLogger(__name__)

OUTBOX_LIMIT = 2048


class ClientConnection:
    def __init__(self, sock: socket.socket, server: Server) -> None:
        self.sock = sock
        self.server = server
        self.authed = False
        self.topics: set[str] = set()
        self.outbox: queue.Queue = queue.Queue(maxsize=OUTBOX_LIMIT)
        self.dropped = 0
        self.alive = True
        threading.Thread(target=self._writer, daemon=True).start()
        threading.Thread(target=self._reader, daemon=True).start()

    def push(self, topic: str, data: object) -> None:
        if not self.alive or topic not in self.topics:
            return
        try:
            self.outbox.put_nowait({"topic": topic, "data": data})
        except queue.Full:
            # A slow subscriber loses state samples rather than stalling the link.
            # Events are the record of what happened, so make room for them.
            self.dropped += 1
            if topic == "event":
                try:
                    self.outbox.get_nowait()
                    self.outbox.put_nowait({"topic": topic, "data": data})
                except (queue.Empty, queue.Full):
                    pass

    def _writer(self) -> None:
        try:
            while self.alive:
                message = self.outbox.get()
                if message is None:
                    return
                send_message(self.sock, message)
        except OSError:
            self.close()

    def _reader(self) -> None:
        try:
            while self.alive:
                request = recv_message(self.sock)
                # Handlers may block, for example a verification jog, so one slow
                # request must not delay this client's stop button.
                threading.Thread(target=self._handle, args=(request,), daemon=True).start()
        except (ConnectionError, OSError, ValueError):
            self.close()

    def _handle(self, request: dict) -> None:
        request_id = request.get("id")
        method = request.get("method", "")
        params = request.get("params") or {}
        try:
            if method == "auth":
                if not hmac.compare_digest(str(params.get("token", "")), self.server.token):
                    raise ServiceError("bad session token", "auth.refused")
                self.authed = True
                result = {"ok": True}
            elif not self.authed:
                raise ServiceError("authenticate first", "auth.required")
            elif method == "subscribe":
                self.topics.add(params["topic"])
                result = {"ok": True}
            else:
                handler = self.server.routes.get(method)
                if handler is None:
                    raise ServiceError(f"unknown method {method}", "request.unknown_method")
                result = handler(**params)
            self._reply({"id": request_id, "ok": True, "result": result})
        except ServiceError as exc:
            self._reply({"id": request_id, "ok": False, "error": str(exc), "code": exc.code})
        except (TypeError, KeyError, ValueError) as exc:
            self._reply({"id": request_id, "ok": False, "error": f"bad request: {exc}", "code": "request.bad_argument"})
        except Exception as exc:
            log.exception("%s failed", method)
            self._reply({"id": request_id, "ok": False, "error": str(exc), "code": "service.internal"})

    def _reply(self, message: dict) -> None:
        try:
            self.outbox.put(message, timeout=2.0)
        except queue.Full:
            self.close()

    def close(self) -> None:
        if not self.alive:
            return
        self.alive = False
        try:
            self.sock.close()
        except OSError:
            pass
        try:
            self.outbox.put_nowait(None)
        except queue.Full:
            pass
        self.server.forget(self)


class Server:
    def __init__(self, port: int = DEFAULT_PORT) -> None:
        self.clients: list[ClientConnection] = []
        self.clients_lock = threading.Lock()
        self.runtime = Runtime(Store(), self.publish)
        self.routes = self._build_routes()
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", port))
        self.listener.listen(16)
        self.port = self.listener.getsockname()[1]
        self.token = write_endpoint(self.port)

    def _build_routes(self) -> dict[str, Callable]:
        r = self.runtime
        return {
            "device.candidates": r.candidates,
            "device.connect": r.connect,
            "device.disconnect": r.disconnect,
            "device.status": r.status,
            "device.discover": r.discover,
            "device.ports": r.ports,
            "device.bind": r.bind,
            "device.set_rates": r.set_rates,
            "device.models": describe_models,
            "calibration.get": r.calibration_get,
            "calibration.list": r.calibration_list,
            "calibration.capture_zero": r.calibration_capture_zero,
            "calibration.set_direction": r.calibration_set_direction,
            "calibration.range_start": r.calibration_range_start,
            "calibration.range_stop": r.calibration_range_stop,
            "calibration.capture_gripper": r.calibration_capture_gripper,
            "calibration.save": r.calibration_save,
            "calibration.load": r.calibration_load,
            "calibration.verify_joint": r.calibration_verify_joint,
            "teleop.config": r.teleop_config,
            "teleop.configure": r.teleop_configure,
            "teleop.enter": r.teleop_enter,
            "teleop.align": r.teleop_align,
            "teleop.start": r.teleop_start,
            "teleop.pause": r.teleop_pause,
            "teleop.resume": r.teleop_resume,
            "teleop.end": r.teleop_end,
            "motion.execute": r.motion_execute,
            "motion.execute_many": r.motion_execute_many,
            "motion.move_to": r.motion_move_to,
            "motion.set_target": r.motion_set_target,
            "motion.home": r.motion_home,
            "guide.start": r.guide_start,
            "guide.stop": r.guide_stop,
            "run.stop": r.run_stop,
            "run.pause": r.run_pause,
            "run.resume": r.run_resume,
            "run.reset_estop": r.run_reset_estop,
            "run.release": r.run_release,
            "policy.start": r.policy_start,
            "policy.stop": r.policy_stop,
            "policy.pause": r.policy_pause,
            "policy.resume": r.policy_resume,
            "policy.status": r.policy_status,
            "policy.submit": r.policy_submit,
            "policy.poll": r.policy_worker_poll,
            "intervention.take_over": r.intervention_take_over,
            "intervention.confirm_aligned": r.intervention_confirm_aligned,
            "intervention.hand_back": r.intervention_hand_back,
            "intervention.mark": r.intervention_mark,
            "intervention.status": r.intervention_status,
            "sil.inject": r.sil_inject,
        }

    def publish(self, topic: str, data: object) -> None:
        with self.clients_lock:
            clients = list(self.clients)
        for client in clients:
            client.push(topic, data)

    def forget(self, client: ClientConnection) -> None:
        with self.clients_lock:
            if client in self.clients:
                self.clients.remove(client)

    def serve_forever(self) -> None:
        while True:
            try:
                sock, _ = self.listener.accept()
            except OSError:
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client = ClientConnection(sock, self)
            with self.clients_lock:
                self.clients.append(client)

    def close(self) -> None:
        self.runtime.close()
        self.listener.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Controller Service: the single owner of the controller link")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--uri", help="connect on startup, for example usb:// or sil://127.0.0.1:5760")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(level=args.log_level, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    server = Server(args.port)
    log.info("controller service on 127.0.0.1:%d", server.port)
    if args.uri:
        try:
            server.runtime.connect(args.uri)
            log.info("connected to %s", args.uri)
        except ServiceError as exc:
            log.warning("%s; waiting for a connect request", exc)

    def shutdown(*_):
        server.close()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    server.serve_forever()


if __name__ == "__main__":
    main()
