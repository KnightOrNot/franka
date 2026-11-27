"""Coordinate robot telemetry and lossless camera writers per operator episode."""

from __future__ import annotations

import json
import os
import shutil
import threading
import time
from collections import deque
from contextlib import ExitStack, suppress
from dataclasses import dataclass
from datetime import UTC, datetime
from importlib.metadata import version
from pathlib import Path
from typing import TextIO
from uuid import uuid4

from camera_realsense_d435i.manager import CaptureManager

from .episode import DUAL_ALIGNMENT, FORMAT, FORMAT_MULTI, analyze, atomic_json, camera_directories
from .telemetry import JOINT_NAMES, MAX_OPEN_M, Cycle


class CaptureError(RuntimeError):
    """An operator request cannot proceed or an episode could not be saved."""


@dataclass
class _Active:
    root: Path
    manifest: dict
    stream: TextIO
    camera_sessions: dict[str, str]
    previous: Cycle | None = None
    count: int = 0
    error: str | None = None
    receiving: bool = True


class Collector:
    def __init__(
        self,
        root: Path,
        camera: CaptureManager | dict[str, CaptureManager],
        *,
        simulated: bool,
        min_free_mb: int = 1024,
        stale_ms: int = 350,
        max_skew_ms: int = 35,
        max_seconds: int = 300,
        control_revision: str = "unspecified",
    ):
        if (
            not 0 <= min_free_mb <= 10_000_000
            or not 1 <= max_seconds <= 3600
            or not 1 <= max_skew_ms <= stale_ms <= 5000
        ):
            raise ValueError("invalid recording disk, duration or timing configuration")
        cameras = camera if isinstance(camera, dict) else {"default": camera}
        if cameras.keys() == {"left", "right"}:
            serials = {manager.defaults.serial.strip() for manager in cameras.values()}
            if len(serials) != 2 or not all(serials):
                raise ValueError("dual capture requires distinct non-empty camera serials")
            self.cameras = {role: cameras[role] for role in ("left", "right")}
        elif cameras.keys() == {"default"}:
            self.cameras = dict(cameras)
        else:
            raise ValueError("camera roles must be default or left and right")
        self.camera = next(iter(self.cameras.values()))
        self.root = root.resolve()
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.simulated = simulated
        self.control_revision = control_revision
        self.min_free_mb = min_free_mb
        self.stale_ns, self.skew_ns = stale_ms * 1_000_000, max_skew_ms * 1_000_000
        self.max_ns = max_seconds * 1_000_000_000
        self._data = threading.RLock()
        self._operator = threading.RLock()
        self._active: _Active | None = None
        self._last_robot: Cycle | None = None
        self._robot_stamps: deque[int] = deque(maxlen=4096)
        self._last_result: dict | None = None

    def ingest(self, cycle: Cycle) -> None:
        now = time.monotonic_ns()
        with self._data:
            active = self._active
            if not 0 <= now - cycle.command_ns <= self.stale_ns:
                if active is not None and active.receiving:
                    active.error = active.error or "late or future robot telemetry"
                return
            last = self._last_robot
            if last is not None and (
                cycle.epoch != last.epoch
                or not 0 < cycle.command_ns - last.command_ns <= self.stale_ns
            ):
                self._robot_stamps.clear()
            self._robot_stamps.append(cycle.command_ns)
            self._last_robot = cycle
            if not active or not active.receiving or active.error:
                return
            if cycle.command_ns < active.manifest["start_ns"]:
                return
            prev = active.previous
            if prev is not None and (
                prev.epoch != cycle.epoch
                or cycle.seq != prev.seq + 1
                or cycle.observed_ns <= prev.observed_ns
                or cycle.command_ns <= prev.command_ns
            ):
                active.error = (
                    f"robot telemetry discontinuity (expected {prev.epoch}/{prev.seq + 1}, "
                    f"received {cycle.epoch}/{cycle.seq})"
                )
                return
            if cycle.command_ns - cycle.observed_ns > self.stale_ns:
                active.error = "robot feedback was stale before command submission"
                return
            try:
                active.stream.write(json.dumps(cycle.row(), allow_nan=False) + "\n")
            except OSError as exc:
                active.error = f"robot write failed: {exc}"
                return
            active.previous, active.count = cycle, active.count + 1

    def input_error(self, reason: str) -> None:
        with self._data:
            if self._active and self._active.receiving:
                self._active.error = self._active.error or reason

    def _disk_ok(self) -> bool:
        return shutil.disk_usage(self.root).free >= self.min_free_mb * 1_000_000

    def _camera_message(self, role: str, reason: str) -> str:
        return f"{role} camera: {reason}" if len(self.cameras) > 1 else reason

    def status(self) -> dict:
        camera_statuses = {role: manager.status() for role, manager in self.cameras.items()}
        cameras = {}
        for role, camera in camera_statuses.items():
            age = camera["frame_age_s"]
            fresh = (
                camera["state"] == "streaming"
                and age is not None
                and 0 <= age <= self.stale_ns / 1e9
            )
            cameras[role] = {**camera, "fresh": fresh}
        cam_ok = all(camera["fresh"] for camera in cameras.values())
        writer_states = {camera["recording"]["state"] for camera in cameras.values()}
        with self._data:
            now = time.monotonic_ns()
            last_robot = self._last_robot
            age = None if last_robot is None else now - last_robot.command_ns
            robot_ok = (
                age is not None
                and 0 <= age <= self.stale_ns
                and now - last_robot.observed_ns <= self.stale_ns
            )
            stamps = self._robot_stamps
            while stamps and stamps[0] < now - 1_000_000_000:
                stamps.popleft()
            robot_fps = (
                round((len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0]), 2)
                if robot_ok and len(stamps) > 1
                else None
            )
            active = self._active
            return {
                "simulated": self.simulated,
                "phase": ("recording" if active.receiving else "finishing") if active else "idle",
                "ready": (
                    not active
                    and robot_ok
                    and cam_ok
                    and self._disk_ok()
                    and not writer_states.intersection(("recording", "stopping"))
                ),
                "robot_fresh": robot_ok,
                "robot_fps": robot_fps,
                "robot_age_ms": None if age is None else round(age / 1e6, 2),
                "camera_fresh": cam_ok,
                "camera": next(iter(camera_statuses.values())),
                "cameras": cameras,
                "episode": active.manifest["episode"] if active else None,
                "robot_cycles": active.count if active else 0,
                "error": active.error
                if active
                else (
                    "camera writer did not finalize; restart capture after fixing storage"
                    if "stopping" in writer_states
                    else None
                ),
                "last": self._last_result,
            }

    def start(self, task: str) -> dict:
        if not isinstance(task, str) or not 1 <= len(task.strip()) <= 160:
            raise CaptureError("enter a task name (1–160 characters)")
        with self._operator:
            status = self.status()
            if status["phase"] != "idle":
                raise CaptureError("an episode is already active")
            if not status["ready"]:
                raise CaptureError("fresh robot telemetry, live camera and free disk are required")
            dual = len(self.cameras) > 1
            stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S")
            episode = f"{stamp}-{uuid4().hex[:12]}"
            path = self.root / f"{episode}.partial"
            path.mkdir(mode=0o700)
            manifest = {
                "format": FORMAT_MULTI if dual else FORMAT,
                "episode": episode,
                "task": task.strip(),
                "simulated": self.simulated,
                "result": "recording",
                "armed_wall_ns": time.time_ns(),
                "max_skew_ns": self.skew_ns,
                "stale_ns": self.stale_ns,
                "software_version": version("franka-capture"),
                "control_revision": self.control_revision,
                "robot_schema": {
                    "channel_names": JOINT_NAMES,
                    "arm_unit": "rad",
                    "gripper_unit": "m",
                    "nominal_max_open_m": MAX_OPEN_M,
                    "feedback_gripper": "opening fraction",
                    "requested_gripper": "closing fraction",
                    "command_timestamp": "immediately before follower API submission",
                },
            }
            if dual:
                manifest["cameras"] = list(self.cameras)
                manifest["dual_alignment"] = DUAL_ALIGNMENT
            directories = camera_directories(manifest)
            paths = {
                role: str(path / directories[role if dual else "front"]) for role in self.cameras
            }
            sessions = {role: camera["session"] for role, camera in status["cameras"].items()}
            stream = None
            try:
                atomic_json(path / "manifest.json", manifest)
                stream = (path / "robot.jsonl").open("x", encoding="utf-8")
                for role, camera in self.cameras.items():
                    try:
                        camera.start_recording(
                            {"directory": paths[role], "session": episode, "episode": episode}
                        )
                    except Exception as exc:
                        raise CaptureError(self._camera_message(role, str(exc))) from exc
                with self._data:
                    manifest.update(start_ns=time.monotonic_ns(), start_wall_ns=time.time_ns())
                    active = _Active(path, manifest, stream, sessions)
                    if dual and (failure := self._failure(active, self.status())):
                        raise CaptureError(failure)
                    self._active = active
            except Exception as exc:
                reason = f"could not start episode: {exc}"
                if stream is not None:
                    with suppress(OSError):
                        stream.close()
                for role, camera in self.cameras.items():
                    try:
                        current = camera.status()["recording"]
                        if current.get("directory") == paths[role]:
                            camera.stop_recording(complete=False, reason=reason)
                    except Exception as cleanup_error:
                        reason += f"; {self._camera_message(role, str(cleanup_error))}"
                with suppress(OSError):
                    atomic_json(
                        path / "manifest.json", {**manifest, "result": "failed", "error": reason}
                    )
                with self._data:
                    self._last_result = {"result": "failed", "path": str(path), "error": reason}
                raise CaptureError(reason) from exc
            return self.status()

    def poll(self) -> None:
        with self._operator:
            active = self._active
            if active is None:
                return
            status = self.status()
            reason = self._failure(active, status)
            if reason:
                self.finish("abort", reason=reason)

    def _failure(self, active: _Active, status: dict) -> str | None:
        if active.error:
            return active.error
        if not status["robot_fresh"]:
            return "robot telemetry is stale"
        for role, camera in status["cameras"].items():
            if (
                not camera["fresh"]
                or camera["session"] != active.camera_sessions[role]
                or camera["recording"]["state"] != "recording"
                or camera["recording"].get("episode") != active.manifest["episode"]
            ):
                return self._camera_message(
                    role,
                    camera["recording"]["error"]
                    or camera["error"]
                    or "camera recording stopped, changed session or is stale",
                )
        if not self._disk_ok():
            return "recording filesystem free space is below its limit"
        if time.monotonic_ns() - active.manifest["start_ns"] >= self.max_ns:
            return "maximum recording duration reached"
        return None

    def finish(self, operation: str, *, reason: str | None = None) -> dict:
        if operation not in ("save", "discard", "abort"):
            raise ValueError("unknown episode operation")
        with self._operator:
            with self._data:
                active = self._active
                if not active:
                    raise CaptureError("no episode is recording")
                if operation == "save":
                    reason = self._failure(active, self.status())
                active.receiving = False
                requested_end = time.monotonic_ns()
                active.manifest["requested_end_ns"] = requested_end
                active.manifest["end_ns"] = (
                    active.previous.command_ns if active.previous is not None else requested_end
                )
            save = operation == "save" and not reason
            discarded = operation == "discard"
            try:
                try:
                    active.stream.flush()
                    os.fsync(active.stream.fileno())
                finally:
                    active.stream.close()
            except OSError as exc:
                save, discarded, reason = False, False, f"robot write failed: {exc}"
            for role, camera in self.cameras.items():
                try:
                    camera_result = camera.stop_recording(complete=save, reason=reason)
                    if save and not camera_result["complete"]:
                        raise CaptureError(camera_result["error"] or "camera did not finish")
                except Exception as exc:
                    save, discarded = False, False
                    reason = reason or self._camera_message(role, str(exc))
            if save:
                try:
                    active.manifest["report"] = analyze(active.root, active.manifest).report
                except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
                    save, reason = False, str(exc)
            phase = "complete" if save else "discarded" if discarded else "failed"
            folder = active.root
            active.manifest.update(result=phase, error=reason, robot_cycles=active.count)
            try:
                destination = self.root / active.manifest["episode"]
                if save and (destination.exists() or destination.is_symlink()):
                    raise CaptureError("episode destination already exists")
                atomic_json(folder / "manifest.json", active.manifest)
                if save:
                    folder = folder.rename(destination)
            except (OSError, CaptureError) as exc:
                phase, reason = "failed", f"could not publish episode: {exc}"
                try:
                    atomic_json(
                        folder / "manifest.json",
                        {**active.manifest, "result": phase, "error": reason},
                    )
                except OSError:
                    pass  # The .partial name remains the authoritative rejection marker.
            finally:
                value = {"result": phase, "path": str(folder), "error": reason}
                with self._data:
                    self._active = None
                    self._last_result = value
            if operation == "save" and phase != "complete":
                raise CaptureError(reason or "episode failed")
            return value

    def close(self) -> None:
        with self._operator:
            try:
                if self._active:
                    self.finish("abort", reason="capture service stopped")
            finally:
                with ExitStack() as stack:
                    for camera in self.cameras.values():
                        stack.callback(camera.stop)
                    for camera in self.cameras.values():
                        if camera.status()["recording"]["state"] == "stopping":
                            stack.callback(
                                camera.stop_recording,
                                complete=False,
                                reason="capture service stopped",
                            )
