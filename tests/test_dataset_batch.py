from types import SimpleNamespace

import pytest
from test_dataset_service import FIRST, SECOND, materialize

from franka_capture.datasets import DatasetService
from franka_capture.recording import CaptureError

THIRD = "20261126T072231-174cb4b5eefc"


class BatchCollector:
    simulated = False
    min_free_mb = 0

    def __init__(self, root):
        self.root = root
        self.episode = None
        self.ids = iter((FIRST, SECOND, THIRD))
        self.following = True
        self.task = None

    def status(self):
        return {"phase": "recording" if self.episode else "idle", "episode": self.episode}

    def start(self, task):
        self.task = task
        self.episode = next(self.ids)
        return self.status()

    def finish(self, operation):
        ident, self.episode = self.episode, None
        if operation == "save":
            path = materialize(self.root, ident, task=self.task)
            return {"result": "complete", "path": str(path)}
        return {"result": "discarded", "path": str(self.root / (ident + ".partial"))}


@pytest.fixture
def batch(tmp_path, monkeypatch):
    root = tmp_path / "raw"
    root.mkdir()
    collector = BatchCollector(root)
    service = DatasetService(collector, tmp_path / "datasets")
    monkeypatch.setattr(service, "available", True)
    return service


def test_finish_batch_starts_conversion_of_only_saved_episodes(batch, monkeypatch):
    materialize(batch.collector.root, "20251126T072229-174cb4b5eefa")  # Same task, not this batch.
    batch.start_recording("pick")
    batch.finish_recording("save")
    batch.start_recording("pick")
    batch.finish_recording("discard")
    batch.start_recording("pick")
    batch.finish_recording("save")
    process = SimpleNamespace(returncode=None)
    process.poll = lambda: process.returncode
    cmdlines = []

    def launch(argv, **kwargs):
        cmdlines.append(argv)
        return process

    monkeypatch.setattr("franka_capture.datasets.subprocess.Popen", launch)
    assert batch.finish_dataset()["state"] == "running"
    assert batch.collector.following
    assert cmdlines[0][-3:] == [
        "--",
        str(batch.collector.root / FIRST),
        str(batch.collector.root / THIRD),
    ]
    process.returncode = 1
    catalog = batch.catalog()
    assert catalog["job"]["state"] == "failed"
    assert catalog["batch"]["episodes"] == [FIRST, THIRD]
    with pytest.raises(CaptureError, match="retry conversion"):
        batch.start_recording("pick")
    assert batch.finish_dataset()["state"] == "running"
    assert cmdlines[1][-2:] == cmdlines[0][-2:]
    process.returncode = 1
    batch.close()


def test_task_name_and_active_episode_guard(batch):
    with pytest.raises(CaptureError, match="saved"):
        batch.finish_dataset()
    batch.start_recording("pick")
    with pytest.raises(CaptureError, match="finish recording"):
        batch.finish_dataset()
    batch.finish_recording("save")
    with pytest.raises(CaptureError, match="current dataset"):
        batch.start_recording("place")


def test_saved_episode_after_interruption_is_recovered_into_its_batch(batch):
    batch.start_recording("pick")
    batch.collector.finish("save")  # Raw publish completed prior to service restart/state update.
    restarted = DatasetService(batch.collector, batch.root)
    assert restarted.catalog()["batch"]["episodes"] == [FIRST]
    assert restarted.catalog()["batch"]["task"] == "pick"
