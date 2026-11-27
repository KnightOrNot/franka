"""Controller-side conversion jobs and a catalog of complete downloadable datasets."""

from __future__ import annotations

import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import threading
from datetime import UTC, datetime
from pathlib import Path
from tempfile import mkdtemp
from uuid import uuid4

from .episode import FORMAT, FORMAT_MULTI, atomic_json, camera_directories
from .recording import CaptureError, Collector

EPISODE_ID = re.compile(r"[0-9]{8}T[0-9]{6}-[0-9a-f]{12}\Z")
DATASET_ID = re.compile(r"dataset-[0-9]{8}T[0-9]{6}-[0-9a-f]{12}\Z")
MAX_EPISODES = 200


def _object(root: Path, name: str) -> dict:
    path = root / name
    if path.resolve() != path or not path.is_file():
        raise FileNotFoundError(name)
    with path.open("rb") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"invalid metadata: {name}")
    return value


class DatasetService:
    def __init__(self, collector: Collector, root: Path):
        self.collector = collector
        self.root = root.expanduser().resolve()
        raw = self.collector.root
        if self.root.is_relative_to(raw) or raw.is_relative_to(self.root):
            raise ValueError("raw and dataset directories must not overlap")
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.available = importlib.util.find_spec("lerobot") is not None
        self._lock = threading.RLock()
        self._process: subprocess.Popen | None = None
        self._work: Path | None = None
        self._closed = False
        self._job: dict = {"state": "idle", "id": None, "error": None}
        self._batch_path = raw / ".dataset-batch.json"
        self._batch = self._restore_batch()

    def _restore_batch(self) -> dict | None:
        if not self._batch_path.exists() and not self._batch_path.is_symlink():
            return None
        value = _object(self.collector.root, self._batch_path.name)
        episodes = value.get("episodes")
        if (
            set(value) != {"id", "task", "episodes", "pending", "export_id", "sealed"}
            or type(value.get("sealed")) is not bool
            or not isinstance(value["id"], str)
            or not re.fullmatch(r"[0-9a-f]{32}", value["id"])
            or not isinstance(value["task"], str)
            or not 1 <= len(value["task"]) <= 160
            or not isinstance(episodes, list)
            or len(episodes) > MAX_EPISODES
            or not all(isinstance(i, str) and EPISODE_ID.fullmatch(i) for i in episodes)
            or len(set(episodes)) != len(episodes)
            or (
                value["pending"] is not None
                and (
                    not isinstance(value["pending"], str)
                    or not EPISODE_ID.fullmatch(value["pending"])
                )
            )
            or (
                value["export_id"] is not None
                and (
                    not isinstance(value["export_id"], str)
                    or not DATASET_ID.fullmatch(value["export_id"])
                )
            )
        ):
            raise ValueError("invalid dataset batch state")
        return value

    def _store_batch(self, batch: dict | None) -> None:
        if batch is None:
            self._batch_path.unlink(missing_ok=True)
        else:
            atomic_json(self._batch_path, batch)
        self._batch = batch

    def _reconcile_batch(self) -> None:
        batch = self._batch
        if batch is None:
            return
        queued = batch["export_id"]
        if queued is not None:
            try:
                self._dataset(queued)
            except FileNotFoundError:
                running = self._process is not None and self._job["id"] == queued
                if not running:
                    self._store_batch({**batch, "export_id": None})
            else:
                self._store_batch(None)
            return
        pending = batch["pending"]
        if pending and self.collector.status()["phase"] == "idle":
            episodes = list(batch["episodes"])
            try:
                _, info = self._episode(pending)
            except (OSError, ValueError, KeyError, TypeError):
                pass  # Failed, incomplete, or deliberately discarded.
            else:
                if info["task"] == batch["task"] and pending not in episodes:
                    episodes.append(pending)
            self._store_batch({**batch, "episodes": episodes, "pending": None})

    def _episode(self, identifier: str) -> tuple[Path, dict]:
        if not isinstance(identifier, str) or not EPISODE_ID.fullmatch(identifier):
            raise ValueError("invalid raw episode ID")
        root = self.collector.root / identifier
        if root.resolve() != root or not root.is_dir():
            raise ValueError("episode is not in the saved catalog")
        manifest = _object(root, "manifest.json")
        if (
            manifest.get("format") not in (FORMAT, FORMAT_MULTI)
            or manifest.get("result") != "complete"
            or manifest.get("simulated") is not self.collector.simulated
            or manifest.get("episode") != identifier
        ):
            raise ValueError("episode is incomplete or from a different capture mode")
        roles = camera_directories(manifest)
        rates = tuple(
            _object(root, str(path / "manifest.json"))["metadata"]["configuration"]["fps"]
            for path in roles.values()
        )
        task, frames = manifest.get("task"), manifest["report"]["matched_pairs"]
        if (
            not isinstance(task, str)
            or type(frames) is not int
            or not all(type(fps) is int and fps > 0 for fps in rates)
        ):
            raise ValueError("invalid episode metadata")
        item = {
            "id": identifier,
            "task": task,
            "frames": frames,
            "fps": min(rates),
            "simulated": self.collector.simulated,
        }
        if len(roles) > 1:
            item["cameras"] = len(roles)
        return root, item

    def _dataset(self, identifier: str, parent: Path | None = None) -> tuple[Path, dict]:
        if not isinstance(identifier, str) or not DATASET_ID.fullmatch(identifier):
            raise FileNotFoundError("dataset not found")
        root = (parent or self.root) / identifier
        archive = root / "dataset.zip"
        if archive.resolve() != archive or not archive.is_file():
            raise FileNotFoundError("dataset not found")
        try:
            fields = _object(root, "catalog.json")
            if (
                fields.get("id") != identifier
                or fields.get("simulated") is not self.collector.simulated
                or fields.get("archive_bytes") != archive.stat().st_size
                or not isinstance(fields.get("task"), str)
                or not isinstance(fields.get("episodes"), list)
                or not all(
                    isinstance(e, str) and EPISODE_ID.fullmatch(e) for e in fields["episodes"]
                )
                or type(fields.get("frames")) is not int
                or type(fields.get("fps")) is not int
            ):
                raise FileNotFoundError("dataset not found")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise FileNotFoundError("dataset not found") from exc
        return archive, fields

    def _reap(self) -> None:
        if self._process is None:
            return
        code = self._process.poll()
        if code is None:
            return
        self._process = None
        work, self._work = self._work, None
        assert work is not None
        try:
            if code != 0:
                raise RuntimeError("conversion worker exited unsuccessfully")
            identifier = self._job["id"]
            self._dataset(identifier, work)
            final = self.root / identifier
            if final.exists() or final.is_symlink():
                raise FileExistsError("dataset output already exists")
            (work / identifier).rename(final)
        except (OSError, RuntimeError) as exc:
            detail = ""
            log = work / "export.log"
            try:
                with log.open("rb") as stream:
                    stream.seek(max(0, log.stat().st_size - 1000))
                    output = stream.read().decode(errors="replace").strip()
                    detail = output.splitlines()[-1][:400] if output else ""
            except OSError:
                pass
            self._job.update(state="failed", error=detail or str(exc))
        else:
            self._job.update(state="complete", error=None)
        finally:
            shutil.rmtree(work, ignore_errors=True)

    def catalog(self) -> dict:
        with self._lock:
            self._reap()
            self._reconcile_batch()
            episodes, finished = [], []
            for name in sorted(self.collector.root.iterdir(), reverse=True):
                if EPISODE_ID.fullmatch(name.name):
                    try:
                        episodes.append(self._episode(name.name)[1])
                    except (OSError, ValueError, KeyError, TypeError):
                        continue
            for name in sorted(self.root.iterdir(), reverse=True):
                if DATASET_ID.fullmatch(name.name):
                    try:
                        finished.append(self._dataset(name.name)[1])
                    except FileNotFoundError:
                        continue
            return {
                "episodes": episodes,
                "datasets": finished,
                "job": dict(self._job),
                "available": self.available,
                "batch": (
                    {**self._batch, "episodes": list(self._batch["episodes"])}
                    if self._batch
                    else None
                ),
            }

    def archive(self, identifier: str) -> Path:
        with self._lock:
            return self._dataset(identifier)[0]

    def start_recording(self, task: str) -> dict:
        with self._lock:
            if self._closed:
                raise CaptureError("capture service is shutting down")
            self._reap()
            self._reconcile_batch()
            if self._process is not None:
                raise CaptureError("wait for dataset conversion before recording")
            if not isinstance(task, str) or not 1 <= len(task.strip()) <= 160:
                raise CaptureError("enter a task name (1–160 characters)")
            task = task.strip()
            batch = self._batch
            if batch and batch["sealed"]:
                raise CaptureError("this dataset is closed; retry conversion before recording")
            if batch and batch["episodes"] and batch["task"] != task:
                raise CaptureError("finish the current dataset before changing its task name")
            if batch and len(batch["episodes"]) >= MAX_EPISODES:
                raise CaptureError("finish the current dataset: it already has 200 saved episodes")
            state = self.collector.start(task)
            ident = state.get("episode")
            if ident is not None and (
                not isinstance(ident, str) or not EPISODE_ID.fullmatch(ident)
            ):
                self.collector.finish("abort", reason="recorder returned an invalid episode ID")
                raise CaptureError("recorder returned an invalid episode ID")
            if batch is None or batch["task"] != task:
                batch = {
                    "id": uuid4().hex,
                    "task": task,
                    "episodes": [],
                    "export_id": None,
                    "sealed": False,
                }
            try:
                self._store_batch({**batch, "pending": ident})
            except OSError as exc:
                self.collector.finish("abort", reason="could not persist dataset batch")
                raise CaptureError(f"could not persist dataset batch: {exc}") from exc
            return state

    def finish_recording(self, operation: str) -> dict:
        with self._lock:
            if self._closed:
                raise CaptureError("capture service is shutting down")
            result = self.collector.finish(operation)
            self._reconcile_batch()
            return result

    def finish_dataset(self) -> dict:
        with self._lock:
            if self._closed:
                raise CaptureError("capture service is shutting down")
            self._reap()
            self._reconcile_batch()
            if self.collector.status()["phase"] != "idle":
                raise CaptureError("finish recording before ending the dataset")
            batch = self._batch
            if batch is None or not batch["episodes"]:
                raise CaptureError("at least one saved episode is required")
            return self.start(list(batch["episodes"]), _batch={**batch, "sealed": True})

    def start(self, identifiers: list[str], *, _batch: dict | None = None) -> dict:
        if (
            not isinstance(identifiers, list)
            or not 1 <= len(identifiers) <= MAX_EPISODES
            or not all(isinstance(i, str) for i in identifiers)
            or len(set(identifiers)) != len(identifiers)
        ):
            raise ValueError("select 1–200 distinct saved episodes")
        with self._lock:
            if self._closed:
                raise CaptureError("capture service is shutting down")
            self._reap()
            if not self.available:
                raise CaptureError("dataset conversion requires the export dependency extra")
            if self._process is not None:
                raise CaptureError("a dataset conversion is already running")
            if self.collector.status()["phase"] != "idle":
                raise CaptureError("finish recording before starting dataset conversion")
            checked = [self._episode(i) for i in identifiers]
            if len({info["task"] for _, info in checked}) != 1:
                raise ValueError("select episodes from the same task")
            if len({info.get("cameras", 1) for _, info in checked}) != 1:
                raise ValueError("single- and dual-camera episodes cannot share one dataset")
            stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S")
            identifier = f"dataset-{stamp}-{uuid4().hex[:12]}"
            work = Path(mkdtemp(prefix=f".{identifier}.", dir=self.root))
            output = work / identifier
            log = work / "export.log"
            env = {
                **os.environ,
                "HF_HUB_OFFLINE": "1",
                "HF_DATASETS_OFFLINE": "1",
                "OPENBLAS_NUM_THREADS": "1",
                "OMP_NUM_THREADS": "1",
                "MKL_NUM_THREADS": "1",
                "TMPDIR": str(work),
                "HF_HOME": str(work / "hf"),
                "HF_DATASETS_CACHE": str(work / "hf" / "datasets"),
            }
            cmd = [
                sys.executable,
                "-m",
                "franka_capture",
                "_export-bundle",
                str(output),
                "--min-free-bytes",
                str(self.collector.min_free_mb * 1_000_000),
                "--",
                *(str(p) for p, _ in checked),
            ]
            try:
                if _batch is not None:
                    self._store_batch({**_batch, "export_id": identifier})
                with log.open("xb") as stream:
                    self._process = subprocess.Popen(
                        cmd,
                        stdout=stream,
                        stderr=subprocess.STDOUT,
                        stdin=subprocess.DEVNULL,
                        env=env,
                        start_new_session=True,
                    )
            except OSError:
                shutil.rmtree(work, ignore_errors=True)
                if _batch is not None:
                    self._reconcile_batch()
                raise
            self._work = work
            self._job = {"id": identifier, "state": "running", "error": None}
            return dict(self._job)

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._reap()
            if self._process is not None:
                process = self._process
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                self._reap()
