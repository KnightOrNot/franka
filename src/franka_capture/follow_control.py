"""Private local bridge from the capture web process to its session supervisor."""

from __future__ import annotations

import json
import os
import secrets
import socket
import stat
import tempfile
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path
from threading import Event, Lock, Thread
from typing import TypeVar

FOLLOW_SOCKET_ENV = "FRANKA_CAPTURE_FOLLOW_SOCKET"
_REQUESTS = {b"status\n", b"start\n", b"stop\n"}
_PHASES = {"idle", "awaiting_confirmation", "starting", "following", "stopping", "error"}
_RESPONSE_LIMIT = 4096
_T = TypeVar("_T")


def _receive_line(conn: socket.socket, limit: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    data = bytearray()
    while len(data) < limit:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("follow control message timed out")
        conn.settimeout(remaining)
        chunk = conn.recv(limit - len(data))
        if not chunk:
            break
        data.extend(chunk)
        if b"\n" in chunk:
            break
    if data.count(b"\n") != 1 or not data.endswith(b"\n"):
        raise ValueError("invalid follow control message")
    return bytes(data)


class FollowClient:
    def __init__(self, path: str | Path | None):
        self.path = os.fspath(path) if path else None

    def _request(self, action: str) -> dict:
        if self.path is None:
            if action == "status":
                return {"phase": "unmanaged", "detail": "Follow is not managed by this recorder."}
            raise RuntimeError("Follow is not managed by this recorder.")
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
                conn.settimeout(0.75)
                conn.connect(self.path)
                conn.sendall(action.encode() + b"\n")
                response = json.loads(_receive_line(conn, _RESPONSE_LIMIT, 0.75))
            if not isinstance(response, dict) or not isinstance(response.get("detail"), str):
                raise ValueError("invalid supervisor response")
            phase = response.get("phase")
            if not isinstance(phase, str) or phase not in _PHASES:
                if phase is None and response.get("ok") is False:
                    raise RuntimeError(response["detail"])
                raise ValueError("invalid supervisor phase")
            return response
        except (OSError, ValueError) as exc:
            message = f"Follow supervisor unavailable: {exc}"
            if action == "status":
                return {"phase": "error", "detail": message}
            raise RuntimeError(message) from exc

    def status(self) -> dict:
        return self._request("status")

    def start(self) -> dict:
        return self._request("start")

    def stop(self) -> dict:
        return self._request("stop")


class FollowCancelled(RuntimeError):
    pass


class FollowCleanupError(RuntimeError):
    """An owned process may still be running. Do not allow another follow chain."""


class FollowAttempt:
    def __init__(self, supervisor: FollowControl):
        self._supervisor = supervisor
        self.cancelled = Event()

    def check(self) -> None:
        if self.cancelled.is_set():
            raise FollowCancelled

    def launch(self, spawn: Callable[[], _T]) -> _T:
        # Serialize the cancellation check with the spawn decision, not the readiness wait.
        with self._supervisor._lock:
            self.check()
            if not self._supervisor._terminal_ready():
                raise FollowCancelled
            return spawn()

    def report(
        self, phase: str, detail: str, *, stage: str | None = None, log_path: Path | None = None
    ) -> None:
        with self._supervisor._lock:
            if not self.cancelled.is_set() or phase == "stopping":
                self._supervisor._set(phase, detail, stage, log_path)

    def wait(self, seconds: float) -> None:
        if self.cancelled.wait(seconds):
            raise FollowCancelled


class FollowControl:
    def __init__(
        self,
        workflow: Callable[[FollowAttempt], None],
        *,
        terminal_ready: Callable[[], bool],
    ):
        self._workflow = workflow
        self._terminal_ready = terminal_ready
        self._lock = Lock()
        self._attempt: FollowAttempt | None = None
        self._worker: Thread | None = None
        self._challenge: str | None = None
        self._expires = 0.0
        self._unsafe = False
        self._closed = False
        self._set("idle", "Follow is stopped. Use the web page to request a start.")

    def _set(
        self, phase: str, detail: str, stage: str | None = None, log_path: Path | None = None
    ) -> None:
        self._state = {"phase": phase, "detail": detail}
        if stage is not None:
            self._state["stage"] = stage
        if log_path is not None:
            self._state["log_path"] = str(log_path)

    def _expire(self) -> None:
        if self._challenge and time.monotonic() >= self._expires:
            self._challenge = None
            self._set("idle", "Terminal confirmation expired. Request a new start.")

    def status(self) -> dict:
        with self._lock:
            self._expire()
            return dict(self._state)

    def start(self) -> dict:
        with self._lock:
            self._expire()
            if self._closed:
                raise RuntimeError("The follow supervisor is closing.")
            if self._unsafe:
                raise RuntimeError(self._state["detail"])
            if self._attempt is not None or self._challenge is not None:
                raise RuntimeError("A follow request is already active.")
            if not self._terminal_ready():
                raise RuntimeError(
                    "Follow requires an on-site operator at the foreground terminal."
                )
            self._challenge = "yes " + secrets.token_hex(4)
            self._expires = time.monotonic() + 90
            self._set("awaiting_confirmation", "Confirm this request in the session terminal.")
            return dict(self._state)

    def prompt(self) -> str | None:
        with self._lock:
            self._expire()
            return self._challenge

    def confirm(self, line: str | None) -> bool:
        with self._lock:
            self._expire()
            if self._challenge is None:
                return False
            if line == "no" or not self._terminal_ready():
                self._challenge = None
                self._set("idle", "Terminal confirmation cancelled. Follow did not start.")
                return False
            if line != self._challenge:
                return False
            self._challenge = None
            attempt = self._attempt = FollowAttempt(self)
            self._set("starting", "Terminal confirmation accepted. Preparing follow.")
            self._worker = Thread(target=self._run, args=(attempt,), name="follow-supervisor")
            self._worker.start()
            return True

    def stop(self) -> dict:
        with self._lock:
            if self._challenge is not None:
                self._challenge = None
                self._set("idle", "Start request cancelled. Follow did not start.")
            if self._attempt is not None and not self._attempt.cancelled.is_set():
                self._attempt.cancelled.set()
                self._set("stopping", "Stopping the session-owned follow chain.")
            return dict(self._state)

    def _run(self, attempt: FollowAttempt) -> None:
        failure = None
        try:
            self._workflow(attempt)
        except FollowCancelled:
            pass
        except Exception as exc:
            failure = exc
        finally:
            with self._lock:
                if failure is None:
                    self._set("idle", "Follow stopped. The recorder and cameras remain available.")
                else:
                    self._unsafe = isinstance(failure, FollowCleanupError)
                    self._set("error", str(failure))
                self._attempt = None

    def __enter__(self) -> FollowControl:
        return self

    def close(self) -> None:
        with self._lock:
            self._closed = True
        self.stop()
        if self._worker is not None:
            self._worker.join()
        if self._unsafe:
            raise FollowCleanupError(self._state["detail"])

    def __exit__(self, *_exc: object) -> None:
        self.close()


def _serve(conn: socket.socket, control: FollowControl) -> None:
    try:
        request = _receive_line(conn, 16, 0.2)
        if request not in _REQUESTS:
            raise ValueError("invalid follow control request")
        handler = {b"status\n": control.status, b"start\n": control.start, b"stop\n": control.stop}
        message = handler[request]()
    except (OSError, ValueError, RuntimeError) as exc:
        message = {"ok": False, "detail": str(exc)}
    try:
        conn.settimeout(0.2)
        conn.sendall(json.dumps(message).encode() + b"\n")
    except OSError:
        pass


@contextmanager
def open_control_socket(parent: Path, control: FollowControl) -> Iterator[Path]:
    """Bind a unique, owner-only socket, adjacent to telemetry when its path fits."""
    parent = parent.resolve()
    parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = parent.stat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise PermissionError("follow control requires an owner-private telemetry directory")
    if len(os.fsencode(parent / "f-01234567/s")) >= 108:
        parent = Path("/tmp")
    with (
        tempfile.TemporaryDirectory(prefix="f-", dir=parent) as directory,
        socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server,
    ):
        path = Path(directory) / "s"
        server.bind(str(path))
        path.chmod(0o600)
        server.listen(16)
        server.settimeout(0.1)
        stopped = Event()

        def serve() -> None:
            while not stopped.is_set():
                try:
                    conn, _ = server.accept()
                except TimeoutError:
                    continue
                with conn:
                    _serve(conn, control)

        worker = Thread(target=serve, name="follow-control")
        worker.start()
        try:
            yield path
        finally:
            stopped.set()
            worker.join()
