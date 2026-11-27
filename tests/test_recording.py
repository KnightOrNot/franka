import json
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from uuid import uuid4

import numpy as np
import pytest
from PIL import Image

from franka_capture import camera as camera_module
from franka_capture.camera import make_camera
from franka_capture.cli import main
from franka_capture.episode import read_episode, select_frames
from franka_capture.recording import CaptureError, Collector
from franka_capture.telemetry import Cycle


@contextmanager
def running_capture(tmp_path, camera, **options):
    collector = Collector(tmp_path, camera, simulated=True, min_free_mb=0, **options)
    epoch = uuid4().hex
    stop = threading.Event()

    def tick():
        seq = 0
        while not stop.wait(0.009):
            t = time.monotonic_ns()
            collector.ingest(Cycle(epoch, seq, t, t, (0.2,) * 8, (0.8,) * 8))
            seq += 1

    source = threading.Thread(target=tick)
    for manager in collector.cameras.values():
        manager.start({})
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


@pytest.fixture
def capture(tmp_path):
    with running_capture(tmp_path, make_camera(tmp_path, simulate=True)) as running:
        yield running


@pytest.fixture
def dual_capture(tmp_path):
    cameras = {
        role: make_camera(tmp_path, simulate=True, serial=f"synthetic-{role}")
        for role in ("left", "right")
    }
    with running_capture(tmp_path, cameras) as (collector, _):
        yield collector


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


def _record_dual_demo(tmp_path, capsys):
    result = main(
        [
            "demo",
            "--dual-camera",
            "--data-root",
            str(tmp_path / "raw"),
            "--socket",
            str(tmp_path / "telemetry.sock"),
            "--min-free-mb",
            "0",
            "--seconds",
            "0.35",
        ]
    )
    output = capsys.readouterr()
    assert result == 0, output.err
    return Path(json.loads(output.out)["path"])


def test_dual_cli_saves_independent_camera_streams(tmp_path, capsys):
    path = _record_dual_demo(tmp_path, capsys)
    episode = read_episode(path)
    assert episode.manifest["format"] == "franka_capture_v2"
    assert episode.manifest["cameras"] == ["left", "right"]
    tints = set()
    for role in ("left", "right"):
        camera_path = path / "cameras" / role
        camera_manifest = json.loads((camera_path / "manifest.json").read_text())
        assert camera_manifest["metadata"]["device"]["serial_number"] == f"synthetic-{role}"
        assert json.loads((camera_path / "COMPLETE.json").read_text())["complete"] is True
        depth = np.asarray(Image.open(camera_path / "depth/000000.png"))
        assert depth.dtype == np.uint16
        assert depth[0, 0] == 0
        with Image.open(camera_path / "rgb/000000.png") as image:
            tints.add(image.getpixel((0, 0))[1])
    assert len(tints) == 2
    assert not (path / "camera").exists()


def test_dual_replay_resolves_both_stream_paths(tmp_path, capsys):
    path = _record_dual_demo(tmp_path, capsys)
    assert main(["replay", str(path)]) == 0
    first = json.loads(capsys.readouterr().out.splitlines()[0])
    assert set(first["cameras"]) == {"left", "right"}
    for role, camera in first["cameras"].items():
        assert Path(camera["rgb"]).parent == path / "cameras" / role / "rgb"
        assert Path(camera["depth"]).parent == path / "cameras" / role / "depth"
        assert Path(camera["rgb"]).is_file() and Path(camera["depth"]).is_file()


def test_dual_discovery_uses_sorted_supported_serials(tmp_path, monkeypatch):
    monkeypatch.setattr(
        camera_module,
        "devices",
        lambda: [
            {"name": "Intel RealSense D435I", "serial_number": "9876"},
            {"name": "Another RealSense", "serial_number": "0000"},
            {"name": "Intel RealSense D435i", "serial_number": "1234"},
        ],
        raising=False,
    )
    cameras = camera_module.make_cameras(tmp_path, simulate=False, dual_camera=True)
    assert {role: camera.defaults.serial for role, camera in cameras.items()} == {
        "left": "1234",
        "right": "9876",
    }
    assert cameras["left"].recorder is not cameras["right"].recorder


@pytest.mark.parametrize("serials", [[], ["1234"], ["1", "2", "3"], ["1234", "1234"], ["", "12"]])
def test_dual_discovery_refuses_missing_or_ambiguous_devices(tmp_path, monkeypatch, serials):
    monkeypatch.setattr(
        camera_module,
        "devices",
        lambda: [{"name": "Intel RealSense D435I", "serial_number": serial} for serial in serials],
        raising=False,
    )
    with pytest.raises(ValueError, match="exactly two.*D435i"):
        camera_module.make_cameras(tmp_path, simulate=False, dual_camera=True)


@pytest.mark.parametrize(
    "options",
    [
        {"dual_camera": True, "serial": "1234"},
        {"dual_camera": True, "left_serial": "1234"},
        {"dual_camera": True, "left_serial": " ", "right_serial": ""},
        {"dual_camera": True, "left_serial": "1234", "right_serial": " 1234 "},
        {"left_serial": "1234", "right_serial": "5678"},
    ],
)
def test_dual_binding_rejects_conflicting_or_incomplete_serial_flags(tmp_path, options):
    with pytest.raises(ValueError, match="serial"):
        camera_module.make_cameras(tmp_path, simulate=True, **options)


def test_explicit_dual_bindings_override_discovery_order(tmp_path):
    cameras = camera_module.make_cameras(
        tmp_path,
        simulate=False,
        dual_camera=True,
        left_serial="9876",
        right_serial="1234",
    )
    assert {role: camera.defaults.serial for role, camera in cameras.items()} == {
        "left": "9876",
        "right": "1234",
    }


def test_dual_status_tracks_right_health_and_observed_rates(dual_capture, monkeypatch):
    collector = dual_capture
    time.sleep(0.12)
    healthy = collector.status()
    assert healthy["camera_fresh"] and healthy["ready"]
    assert 60 < healthy["robot_fps"] < 120
    assert set(healthy["cameras"]) == {"left", "right"}
    for camera in healthy["cameras"].values():
        assert camera["fresh"] is True
        assert 20 < camera["observed_fps"]["color"] < 40
        assert camera["configuration"]["width"] == 64
        assert camera["configuration"]["height"] == 48
    right = collector.cameras["right"]
    status = right.status
    monkeypatch.setattr(right, "status", lambda: {**status(), "frame_age_s": 1})
    stale = collector.status()
    assert not stale["camera_fresh"] and not stale["ready"]
    assert stale["cameras"]["left"]["fresh"] and not stale["cameras"]["right"]["fresh"]
    with pytest.raises(CaptureError, match="camera"):
        collector.start("must not start on one camera")


def test_dual_start_failure_rolls_back_both_writers(dual_capture, monkeypatch):
    collector = dual_capture
    right = collector.cameras["right"]
    start = right.start_recording

    def fail_after_start(options):
        start(options)
        raise RuntimeError("injected right start failure")

    monkeypatch.setattr(right, "start_recording", fail_after_start)
    with pytest.raises(CaptureError, match="injected right start failure"):
        collector.start("atomic start")
    assert collector.status()["phase"] == "idle"
    (folder,) = collector.root.glob("*.partial")
    for role in ("left", "right"):
        complete = folder / "cameras" / role / "COMPLETE.json"
        assert json.loads(complete.read_text())["complete"] is False
    assert json.loads((folder / "manifest.json").read_text())["result"] == "failed"


def test_dual_disconnect_aborts_episode_without_publishing(dual_capture, monkeypatch):
    collector = dual_capture
    collector.start("camera disconnected")
    right = collector.cameras["right"]
    status = right.status
    monkeypatch.setattr(
        right, "status", lambda: {**status(), "state": "error", "error": "lost link"}
    )
    collector.poll()
    result = collector.status()["last"]
    assert result["result"] == "failed"
    assert "right" in result["error"] and "lost link" in result["error"]
    folder = Path(result["path"])
    assert folder.suffix == ".partial"
    for role in ("left", "right"):
        complete = folder / "cameras" / role / "COMPLETE.json"
        assert json.loads(complete.read_text())["complete"] is False


def test_dual_writer_failure_does_not_publish_or_leave_an_active_writer(dual_capture, monkeypatch):
    collector = dual_capture
    collector.start("camera writer")
    time.sleep(0.2)
    left = collector.cameras["left"]
    stop = left.stop_recording

    def fail_left(**kwargs):
        stop(complete=False, reason="injected left finalization failure")
        raise OSError("injected left finalization failure")

    monkeypatch.setattr(left, "stop_recording", fail_left)
    with pytest.raises(CaptureError, match="injected left finalization failure"):
        collector.finish("save")
    result = collector.status()["last"]
    assert result["result"] == "failed"
    assert not Path(result["path"]).with_suffix("").exists()
    assert all(not camera.recorder.recording for camera in collector.cameras.values())


def test_dual_save_never_replaces_an_existing_destination(dual_capture):
    collector = dual_capture
    status = collector.start("collision")
    destination = collector.root / status["episode"]
    destination.mkdir()
    original = destination.stat()
    time.sleep(0.25)
    with pytest.raises(CaptureError, match="already exists"):
        collector.finish("save")
    assert destination.stat().st_ino == original.st_ino
    assert not list(destination.iterdir())
    result = collector.status()["last"]
    assert result["result"] == "failed"
    assert Path(result["path"]).suffix == ".partial"
