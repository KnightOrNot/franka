import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from franka_capture.datasets import DatasetService
from franka_capture.recording import CaptureError

FIRST = "20261126T072229-174cb4b5eefa"
SECOND = "20261126T072230-174cb4b5eefb"


class CollectorStub:
    simulated = False
    min_free_mb = 0

    def __init__(self, raw):
        self.root = raw
        self.phase = "idle"

    def status(self):
        return {"phase": self.phase}

    def start(self, task):
        self.phase = "recording"
        return {"phase": self.phase, "task": task}


def materialize(root, identifier, *, task="pick", result="complete", simulated=False):
    directory = root / identifier
    (directory / "camera").mkdir(parents=True)
    (directory / "manifest.json").write_text(
        json.dumps(
            {
                "format": "franka_capture_v1",
                "task": task,
                "simulated": simulated,
                "episode": identifier.removesuffix(".partial"),
                "result": result,
                "report": {"matched_pairs": 12},
            }
        )
    )
    (directory / "camera/manifest.json").write_text(
        json.dumps(
            {
                "metadata": {"configuration": {"fps": 30}},
            }
        )
    )
    return directory


@pytest.fixture
def service(tmp_path, monkeypatch):
    raw = tmp_path / "raw"
    raw.mkdir()
    collector = CollectorStub(raw)
    ds = DatasetService(collector, tmp_path / "lerobot")
    monkeypatch.setattr(ds, "available", True)
    try:
        yield ds
    finally:
        ds.close()


def test_catalog_only_exposes_completed_real_episodes(service):
    materialize(service.collector.root, FIRST)
    materialize(service.collector.root, SECOND, simulated=True)
    materialize(service.collector.root, SECOND + ".partial", result="discarded")
    (service.collector.root / "20261126T072231-174cb4b5eefc").symlink_to(
        service.collector.root / FIRST, target_is_directory=True
    )
    assert service.catalog()["episodes"] == [
        {"id": FIRST, "task": "pick", "frames": 12, "fps": 30, "simulated": False}
    ]
    with pytest.raises(ValueError):
        service.start(["../" + FIRST])
    with pytest.raises(FileNotFoundError):
        service.archive("../" + FIRST)


def test_export_and_recording_exclude_each_other(service, monkeypatch):
    materialize(service.collector.root, FIRST)
    service.collector.phase = "recording"
    with pytest.raises(CaptureError, match="record"):
        service.start([FIRST])
    service.collector.phase = "idle"
    process = SimpleNamespace(returncode=None)
    process.poll = lambda: process.returncode
    monkeypatch.setattr("franka_capture.datasets.subprocess.Popen", lambda *a, **k: process)
    job = service.start([FIRST])
    with pytest.raises(CaptureError, match="conversion"):
        service.start_recording("pick")
    with pytest.raises(CaptureError, match="conversion"):
        service.start([FIRST])
    process.returncode = 1
    assert service.catalog()["job"]["state"] == "failed"
    assert service.start_recording("pick")["phase"] == "recording"
    with pytest.raises(FileNotFoundError):
        service.archive(job["id"])


def test_published_archive_must_be_owned_by_finalized_catalog(service):
    ident = "dataset-20261126T072229-174cb4b5eefa"
    p = service.root / ident
    p.mkdir()
    archive = p / "dataset.zip"
    archive.write_bytes(b"PK")
    with pytest.raises(FileNotFoundError):
        service.archive(ident)
    (p / "catalog.json").write_text(
        json.dumps(
            {
                "id": ident,
                "task": "pick",
                "episodes": [FIRST],
                "frames": 12,
                "fps": 30,
                "simulated": False,
                "archive_bytes": 2,
            }
        )
    )
    assert service.archive(ident) == archive
    archive.unlink()
    archive.symlink_to(Path("/etc/passwd"))
    with pytest.raises(FileNotFoundError):
        service.archive(ident)


def test_closing_rejects_late_requests(service):
    materialize(service.collector.root, FIRST)
    service.close()
    with pytest.raises(CaptureError, match="shutting down"):
        service.start([FIRST])
    with pytest.raises(CaptureError, match="shutting down"):
        service.start_recording("pick")
