import json
import os
from zipfile import ZipFile

import numpy as np
import pytest
from PIL import Image

from franka_capture.episode import analyze, read_episode, select_frames
from franka_capture.export import build_bundle, export_episode
from franka_capture.telemetry import Cycle

_START = 1_000_000_000
_PERIOD = 33_333_333
_EXPORT_STACK = pytest.mark.skipif(
    os.environ.get("FRANKA_TEST_EXPORT") != "1", reason="optional LeRobot stack"
)


def raw_episode(root, *, dual=True, right_offset=None, robot_period=5_000_000):
    root.mkdir()
    end = _START + 250_000_000
    cycles = (
        Cycle("a" * 32, i, stamp - 1_000_000, stamp, (0.2,) * 8, (0.8,) * 8)
        for i, stamp in enumerate(range(end % robot_period + _START, end + 1, robot_period))
    )
    (root / "robot.jsonl").write_text("".join(json.dumps(c.row()) + "\n" for c in cycles))
    manifest = {
        "format": "franka_capture_v2" if dual else "franka_capture_v1",
        "episode": root.name,
        "result": "complete",
        "task": "pick and place",
        "simulated": True,
        "start_ns": _START,
        "end_ns": end,
        "requested_end_ns": end + 5_000_000,
        "max_skew_ns": 20_000_000,
        "stale_ns": 350_000_000,
    }
    if dual:
        manifest["cameras"] = ["left", "right"]
    for role, (width, height), phase in (("left", (8, 6), 0), ("right", (10, 8), 4_000_000)):
        if not dual and role == "right":
            break
        camera = root / "cameras" / role if dual else root / "camera"
        (camera / "rgb").mkdir(parents=True)
        (camera / "depth").mkdir()
        (camera / "manifest.json").write_text(
            json.dumps(
                {
                    "format": "xcore_rgbd_frames_v1",
                    "episode": root.name,
                    "metadata": {
                        "configuration": {"width": width, "height": height, "fps": 30},
                        "device": {"serial_number": f"synthetic-{role}"},
                        "depth_scale_m_per_unit": 0.001 if role == "left" else 0.002,
                        "depth_format": "z16",
                        "depth_alignment": "native_depth",
                        "color_intrinsics": {"width": width, "height": height},
                        "depth_intrinsics": {"width": width, "height": height},
                        "depth_to_color": {"translation_m": [0.01, 0, 0]},
                    },
                }
            )
        )
        rows = []
        for seq in range(8):
            stamp = _START + 5_000_000 + seq * _PERIOD + phase
            if role == "right" and right_offset:
                stamp += right_offset(seq)
            rgb, depth = f"rgb/{seq:06d}.png", f"depth/{seq:06d}.png"
            Image.new("RGB", (width, height), (40, 20, width)).save(camera / rgb)
            Image.fromarray(np.full((height, width), 1000 + width + seq, dtype=np.uint16)).save(
                camera / depth
            )
            rows.append(
                {
                    "sequence": seq,
                    "rgb": rgb,
                    "depth": depth,
                    "received_monotonic_ns": stamp,
                    "received_wall_time_ns": stamp + 100 * _START,
                    "rgb_frame_number": 30 + seq,
                    "depth_frame_number": 30 + seq,
                    "rgb_sdk_timestamp_ms": 10_000 + seq * _PERIOD / 1e6,
                    "depth_sdk_timestamp_ms": 10_000 + seq * _PERIOD / 1e6,
                    "rgb_sdk_timestamp_domain": "hardware_clock",
                    "depth_sdk_timestamp_domain": "hardware_clock",
                }
            )
        (camera / "frames.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
        (camera / "COMPLETE.json").write_text(
            json.dumps(
                {
                    "complete": True,
                    "dropped": 0,
                    "frames": len(rows),
                    "persisted": len(rows),
                }
            )
        )
    return manifest


def publish(root, manifest):
    manifest["report"] = analyze(root, manifest).report
    (root / "manifest.json").write_text(json.dumps(manifest))


def test_aligns_both_camera_roles_on_one_causal_grid_without_reuse(tmp_path):
    root = tmp_path / "dual"
    manifest = raw_episode(root)
    publish(root, manifest)
    episode = read_episode(root)
    assert episode.report["matched_pairs"] == 8
    assert {role: len(frames) for role, frames in episode.camera_frames.items()} == {
        "left": 8,
        "right": 8,
    }
    samples = select_frames(episode, 15)
    assert len(samples) == 4
    assert len(set(sample.cycle.seq for sample in samples)) == len(samples)
    for role in ("left", "right"):
        assert [sample.frames_by_camera[role].seq for sample in samples] == [0, 2, 4, 6]
    for sample in samples:
        stamps = [frame.received_ns for frame in sample.frames_by_camera.values()]
        assert max(stamps) <= sample.cycle.command_ns
        assert max(abs(t - sample.cycle.observed_ns) for t in stamps) <= 20_000_000


@pytest.mark.parametrize("fault", ["missing", "drop", "stale", "drift", "number"])
def test_invalid_right_camera_cannot_be_saved_as_a_complete_dual_episode(tmp_path, fault):
    root = tmp_path / "broken"
    right_offset = {
        "stale": lambda seq: 60_000_000 * (seq >= 4),
        "drift": lambda seq: 3_000_000 * seq,
    }.get(fault)
    manifest = raw_episode(root, right_offset=right_offset)
    camera = root / "cameras" / "right"
    if fault == "missing":
        (camera / "manifest.json").unlink()
    elif fault == "drop":
        complete = camera / "COMPLETE.json"
        row = json.loads(complete.read_text())
        complete.write_text(json.dumps({**row, "dropped": 1}))
    elif fault == "number":
        frames = camera / "frames.jsonl"
        rows = [json.loads(line) for line in frames.read_text().splitlines()]
        for row in rows[4:]:
            row["depth_frame_number"] += 1
        frames.write_text("".join(json.dumps(row) + "\n" for row in rows))
    with pytest.raises(ValueError, match="right"):
        analyze(root, manifest)


def test_camera_pair_cannot_reuse_robot_telemetry_to_fill_a_slow_stream(tmp_path):
    root = tmp_path / "slow-robot"
    manifest = raw_episode(root, robot_period=50_000_000)
    manifest["max_skew_ns"] = 35_000_000
    with pytest.raises(ValueError, match="robot.*(skew|cycle)"):
        analyze(root, manifest)


@pytest.mark.parametrize("change", ["layout", "configuration", "serial"])
def test_rejects_incompatible_dual_batch_before_creating_output(tmp_path, change):
    first, second = tmp_path / "one", tmp_path / "two"
    publish(first, raw_episode(first))
    manifest = raw_episode(second, dual=change != "layout")
    if change != "layout":
        path = second / "cameras/right/manifest.json"
        data = json.loads(path.read_text())
        if change == "serial":
            data["metadata"]["device"]["serial_number"] = "new-right-camera"
        else:
            data["metadata"]["configuration"]["serial"] = "other-camera-binding"
        path.write_text(json.dumps(data))
    publish(second, manifest)
    output = tmp_path / "bundle"
    with pytest.raises(ValueError, match="(layout|profile)"):
        build_bundle([first, second], output)
    assert not output.exists()


@_EXPORT_STACK
@pytest.mark.parametrize("standalone", [False, True])
def test_dual_lerobot_export_has_two_videos_and_both_original_depth_streams(tmp_path, standalone):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = tmp_path / "dual"
    publish(root, raw_episode(root))
    output = tmp_path / "output"
    if standalone:
        export_episode(root, output, fps=15)
        dataset_root = output
    else:
        result = build_bundle([root], output, fps=15)
        assert result["frames"] == 4
        extracted = tmp_path / "unzipped"
        with ZipFile(output / "dataset.zip") as zipped:
            assert zipped.testzip() is None
            zipped.extractall(extracted)
        dataset_root = extracted / "dataset"
    dataset = LeRobotDataset("local/franka", root=dataset_root, video_backend="pyav")
    assert len(dataset) == 4
    assert tuple(dataset[0]["observation.images.left"].shape) == (3, 6, 8)
    assert tuple(dataset[0]["observation.images.right"].shape) == (3, 8, 10)
    provenance = json.loads((dataset_root / "franka_source.json").read_text())
    if not standalone:
        provenance = provenance["episodes"][0]
    assert provenance["cameras"]["left"]["depth_scale_m_per_unit"] == 0.001
    assert provenance["cameras"]["right"]["depth_scale_m_per_unit"] == 0.002
    rows = (dataset_root / "franka_alignment.jsonl").read_text().splitlines()
    assert len(rows) == 4
    for line in rows:
        row = json.loads(line)
        for role in ("left", "right"):
            camera = row["cameras"][role]
            original = root / camera["source_depth_image"]
            depths = dataset_root / camera["depth_image"]
            assert depths.read_bytes() == original.read_bytes()
            assert np.asarray(Image.open(depths)).dtype == np.uint16
