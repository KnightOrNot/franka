import errno
import json
import os
import shutil
import threading
import time
from pathlib import Path
from zipfile import ZipFile

import numpy as np
import pytest
from PIL import Image

from franka_capture import export
from franka_capture.camera import make_camera
from franka_capture.episode import analyze, read_episode
from franka_capture.recording import Collector
from franka_capture.telemetry import Cycle

_EXPORT_STACK = pytest.mark.skipif(
    os.environ.get("FRANKA_TEST_EXPORT") != "1", reason="optional LeRobot stack"
)


@pytest.fixture(scope="module")
def recordings(tmp_path_factory):
    root = tmp_path_factory.mktemp("bundle-recordings")
    camera = make_camera(root, simulate=True)
    collector = Collector(root, camera, simulated=True, min_free_mb=0)
    stop = threading.Event()

    def tick():
        sequence = 0
        while not stop.wait(0.008):
            now = time.monotonic_ns()
            collector.ingest(Cycle("a" * 32, sequence, now, now, (0.2,) * 8, (0.8,) * 8))
            sequence += 1

    camera.start({})
    source = threading.Thread(target=tick)
    source.start()
    try:
        deadline = time.monotonic() + 3
        while not collector.status()["ready"]:
            assert time.monotonic() < deadline
            time.sleep(0.01)
        episodes = []
        for _ in range(2):
            collector.start("simulated pick and place")
            time.sleep(0.45)
            episodes.append(Path(collector.finish("save")["path"]))
        return episodes
    finally:
        stop.set()
        source.join()
        collector.close()


def test_rejects_no_or_duplicate_sources(recordings, tmp_path):
    output = tmp_path / "bundle"
    with pytest.raises(ValueError, match="at least one"):
        export.build_bundle([], output)
    with pytest.raises(ValueError, match="duplicate"):
        export.build_bundle([recordings[0], recordings[0]], output)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("change", ["task", "mode", "profile", "incomplete"])
def test_rejects_invalid_batch_before_publishing(change, recordings, tmp_path):
    source = shutil.copytree(recordings[1], tmp_path / recordings[1].name)
    manifest_file = source / "manifest.json"
    manifest = json.loads(manifest_file.read_text())
    expected = change
    if change == "task":
        manifest["task"] = "other task"
    elif change == "mode":
        manifest["simulated"] = False
    elif change == "profile":
        camera_file = source / "camera" / "manifest.json"
        camera = json.loads(camera_file.read_text())
        camera["metadata"]["configuration"]["fps"] = 15
        camera_file.write_text(json.dumps(camera))
        manifest["report"] = analyze(source, manifest).report
    else:
        manifest["result"] = "discarded"
        expected = "complete"
    manifest_file.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match=expected):
        export.build_bundle([recordings[0], source], tmp_path / "bundle")
    assert list(tmp_path.iterdir()) == [source]


def test_does_not_stage_or_publish_when_space_is_below_reserve(recordings, tmp_path):
    reserve = shutil.disk_usage(tmp_path).free + 1
    with pytest.raises(OSError) as exc:
        export.build_bundle(recordings, tmp_path / "bundle", min_free_bytes=reserve)
    assert exc.value.errno == errno.ENOSPC
    assert not list(tmp_path.iterdir())


def test_refuses_output_inside_raw_or_at_existing_destination(recordings, tmp_path):
    inside = recordings[1] / "new-dir" / "bundle"
    with pytest.raises(ValueError, match="inside.*raw"):
        export.build_bundle(recordings, inside)
    assert not inside.parent.exists()
    destination = tmp_path / "bundle"
    destination.mkdir()
    sentinel = destination / "keep"
    sentinel.write_text("original")
    with pytest.raises(FileExistsError):
        export.build_bundle(recordings, destination)
    assert sentinel.read_text() == "original"


@_EXPORT_STACK
def test_archive_reads_as_multiple_episodes_and_preserves_native_depth(recordings, tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    original_reports = [read_episode(source).report for source in recordings]
    bundle = tmp_path / "bundle"
    catalog = export.build_bundle(recordings, bundle)
    archive = bundle / "dataset.zip"
    assert catalog == json.loads((bundle / "catalog.json").read_text())
    assert catalog == {
        "id": "bundle",
        "task": "simulated pick and place",
        "episodes": [source.name for source in recordings],
        "frames": catalog["frames"],
        "fps": 30,
        "simulated": True,
        "archive_bytes": archive.stat().st_size,
    }
    extracted = tmp_path / "unzipped"
    with ZipFile(archive) as zipped:
        assert zipped.testzip() is None
        assert all(name.startswith("dataset/") for name in zipped.namelist())
        zipped.extractall(extracted)
    dataset_root = extracted / "dataset"
    dataset = LeRobotDataset("local/franka", root=dataset_root, video_backend="pyav")
    assert dataset.meta.info.codebase_version == "v3.0"
    assert dataset.num_episodes == 2
    assert len(dataset) == catalog["frames"] >= 4
    assert set(dataset.hf_dataset.unique("episode_index")) == {0, 1}
    sidecar = [
        json.loads(line)
        for line in (dataset_root / "franka_alignment.jsonl").read_text().splitlines()
    ]
    assert len(sidecar) == len(dataset)
    provenance = json.loads((dataset_root / "franka_source.json").read_text())
    for index, source in enumerate(recordings):
        source_episode = read_episode(source)
        source_metadata = provenance["episodes"][index]
        assert source_metadata["episode_index"] == index
        assert source_metadata["source_episode"] == source.name
        assert source_metadata["source_sha256"] == original_reports[index]["content_sha256"]
        assert (
            source_metadata["camera"]["depth_to_color"] == source_episode.camera["depth_to_color"]
        )
        assert source_metadata["camera"]["depth_scale_m_per_unit"] == 0.001
        rows = [row for row in sidecar if row["episode_index"] == index]
        assert len(rows) >= 2
        assert [row["frame_index"] for row in rows] == list(range(len(rows)))
        for row in rows:
            depth = dataset_root / row["depth_image"]
            source_depth = source / row["source_depth_image"]
            assert depth.read_bytes() == source_depth.read_bytes()
            assert np.asarray(Image.open(depth)).dtype == np.uint16
        item = dataset[rows[-1]["dataset_index"]]
        assert item["episode_index"].item() == index
        assert item["frame_index"].item() == len(rows) - 1
        assert item["observation.state"].tolist() == pytest.approx([0.2] * 7 + [0.018])
        assert item["action"].tolist() == pytest.approx([0.8] * 7 + [0.018])
        assert tuple(item["observation.images.front"].shape) == (3, 48, 64)
        assert read_episode(source).report == original_reports[index]


@_EXPORT_STACK
def test_zip_failure_never_publishes_or_modifies_raw(recordings, tmp_path, monkeypatch):
    original = (recordings[0] / "manifest.json").read_bytes()
    output = tmp_path / "bundle"

    def fail_write(*args, **kwargs):
        raise OSError(errno.ENOSPC, "injected archive write failure")

    monkeypatch.setattr(ZipFile, "write", fail_write)
    with pytest.raises(OSError, match="injected archive write failure"):
        export.build_bundle(recordings[:1], output, fps=15)
    assert not list(tmp_path.iterdir())
    assert (recordings[0] / "manifest.json").read_bytes() == original
