import json
import threading
import time
from pathlib import Path
from uuid import uuid4

import numpy as np
import pytest
from PIL import Image

from franka_capture.camera import make_camera
from franka_capture.episode import read_episode, select_frames
from franka_capture.recording import CaptureError, Collector
from franka_capture.telemetry import Cycle


@pytest.fixture
def capture(tmp_path):
    camera = make_camera(tmp_path, simulate=True)
    collector = Collector(tmp_path, camera, simulated=True, min_free_mb=0)
    epoch = uuid4().hex
    stop = threading.Event()

    def tick():
        seq = 0
        while not stop.wait(0.009):
            t = time.monotonic_ns()
            collector.ingest(Cycle(epoch, seq, t, t, (0.2,) * 8, (0.8,) * 8))
            seq += 1

    source = threading.Thread(target=tick)
    camera.start({})
    source.start()
    deadline = time.monotonic() + 3
    try:
        while not collector.status()["ready"]:
            assert time.monotonic() < deadline, collector.status()
            time.sleep(0.01)
        yield collector, stop
    finally:
        stop.set()
        source.join()
        collector.close()


def test_save_pair_and_read_lossless_depth(capture):
    collector, _ = capture
    collector.start("pick")
    time.sleep(0.35)
    result = collector.finish("save")
    folder = Path(result["path"])
    episode = read_episode(folder)
    samples = select_frames(episode, 30)
    assert len(samples) >= 4
    assert episode.manifest["task"] == "pick"
    sample = samples[0]
    assert sample.cycle.action == pytest.approx((0.8,) * 7 + (0.018,))
    assert sample.cycle.state == pytest.approx((0.2,) * 7 + (0.018,))
    depth = np.asarray(Image.open(folder / "camera" / sample.frame.depth))
    assert depth.dtype == np.uint16
    assert depth[0, 0] == 0
    assert 1000 <= depth[0, 1] < 1100
    assert not list(folder.parent.glob("*.partial"))


def test_discard_retains_audit_without_publishing(capture):
    collector, _ = capture
    collector.start("discard me")
    time.sleep(0.09)
    result = collector.finish("discard")
    path = Path(result["path"])
    assert path.suffix == ".partial"
    assert json.loads((path / "manifest.json").read_text())["result"] == "discarded"
    with pytest.raises(ValueError, match="complete"):
        read_episode(path)


def test_stop_of_telemetry_fails_the_current_episode(capture):
    collector, stop = capture
    collector.start("loss")
    with pytest.raises(CaptureError, match="already"):
        collector.start("duplicate")
    time.sleep(0.15)
    stop.set()
    time.sleep(collector.stale_ns / 1e9 + 0.02)
    collector.poll()
    path = Path(collector.status()["last"]["path"])
    assert path.suffix == ".partial"
    assert collector.status()["last"]["error"] == "robot telemetry is stale"


def test_readback_rejects_replaced_frame(capture):
    collector, _ = capture
    collector.start("media integrity")
    time.sleep(0.3)
    path = Path(collector.finish("save")["path"])
    episode = read_episode(path)
    (path / "camera" / episode.frames[0].depth).write_bytes(b"broken")
    with pytest.raises(ValueError, match="media"):
        read_episode(path)


def test_common_end_is_last_accepted_command(capture):
    collector, _ = capture
    collector.start("bounded")
    time.sleep(0.25)
    episode = read_episode(Path(collector.finish("save")["path"]))
    assert episode.manifest["end_ns"] == episode.cycles[-1].command_ns
    assert episode.manifest["requested_end_ns"] >= episode.manifest["end_ns"]


def test_decoder_rejects_crc_valid_but_unreadable_png(capture):
    import struct
    import zlib

    collector, _ = capture
    collector.start("crc")
    time.sleep(0.25)
    path = Path(collector.finish("save")["path"])
    image = path / "camera/depth/000000.png"
    data = image.read_bytes()
    i = data.index(b"IDAT")
    original_len = int.from_bytes(data[i - 4 : i], "big")
    invalid = b"IDAT" + b"\x00" * 5
    image.write_bytes(
        data[: i - 4]
        + struct.pack(">I", 5)
        + invalid
        + struct.pack(">I", zlib.crc32(invalid))
        + data[i + original_len + 8 :]
    )
    with pytest.raises(ValueError, match="invalid camera media"):
        read_episode(path)


def test_camera_writer_failure_does_not_publish(capture, monkeypatch):
    collector, _ = capture
    collector.start("camera writer")
    original = Image.Image.save

    def fail_png(self, fp, format=None, **options):
        if format == "PNG":
            raise OSError("injected writer failure")
        return original(self, fp, format, **options)

    monkeypatch.setattr(Image.Image, "save", fail_png)
    deadline = time.monotonic() + 2
    while collector.status()["phase"] == "recording":
        assert time.monotonic() < deadline
        time.sleep(0.01)
        collector.poll()
    result = collector.status()["last"]
    assert result["result"] == "failed"
    assert "injected writer failure" in result["error"]
    assert Path(result["path"]).suffix == ".partial"


def test_camera_frame_loss_is_reported_instead_of_filled(capture, monkeypatch):
    collector, _ = capture
    collector.start("camera sequence")
    time.sleep(0.15)
    original = collector.camera.recorder.submit

    def skip_one(frame):
        monkeypatch.setattr(collector.camera.recorder, "submit", original)

    monkeypatch.setattr(collector.camera.recorder, "submit", skip_one)
    time.sleep(0.14)
    with pytest.raises(CaptureError, match="lost, repeated or reset"):
        collector.finish("save")
    assert Path(collector.status()["last"]["path"]).suffix == ".partial"
