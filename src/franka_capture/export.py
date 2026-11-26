"""Offline LeRobot export; the RGB/Z16 raw episode remains the source of truth."""

from __future__ import annotations

import errno
import json
import shutil
from pathlib import Path
from tempfile import TemporaryDirectory
from zipfile import ZipFile

import numpy as np
from PIL import Image

from .episode import Episode, Sample, atomic_json, read_episode, select_frames
from .telemetry import JOINT_NAMES

_RGB = "observation.images.front"
_REPO_ID = "local/franka"


def _lerobot():
    try:
        from lerobot.configs.video import RGBEncoderConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError("install the export extra: uv sync --locked --extra export") from exc
    return LeRobotDataset, RGBEncoderConfig(vcodec="h264")


def _features(configuration: dict) -> dict:
    vector = {"dtype": "float32", "shape": (8,), "names": list(JOINT_NAMES)}
    return {
        "observation.state": vector,
        "action": dict(vector),
        _RGB: {
            "dtype": "video",
            "shape": (configuration["height"], configuration["width"], 3),
            "names": ["height", "width", "channels"],
        },
    }


def _unused_output(output: Path) -> None:
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"dataset output already exists: {output}")


def _add_frame(dataset, episode: Episode, sample: Sample) -> None:
    with Image.open(episode.root / "camera" / sample.frame.rgb) as image:
        rgb = np.array(image, dtype=np.uint8)
    dataset.add_frame(
        {
            "task": episode.manifest["task"],
            "observation.state": np.asarray(sample.cycle.state, dtype=np.float32),
            "action": np.asarray(sample.cycle.action, dtype=np.float32),
            _RGB: rgb,
        }
    )


def _alignment_row(sample: Sample, index: int) -> dict:
    frame, cycle = sample.frame, sample.cycle
    return {
        "frame_index": index,
        "target_ns": sample.target_ns,
        "source_frame_index": frame.seq,
        "source_epoch": cycle.epoch,
        "source_cycle_index": cycle.seq,
        "rgb_image": f"camera/{frame.rgb}",
        "depth_image": f"camera/{frame.depth}",
        "rgb_ns": frame.rgb_ns,
        "depth_ns": frame.depth_ns,
        "camera_received_ns": frame.received_ns,
        "observed_ns": cycle.observed_ns,
        "command_ns": cycle.command_ns,
    }


def export_episode(path: Path, output: Path, *, fps: int) -> dict:
    lerobot, rgb_encoder = _lerobot()
    episode = read_episode(path)
    samples = select_frames(episode, fps)
    output = output.expanduser().absolute()
    _unused_output(output)
    if output.resolve().is_relative_to(episode.root):
        raise ValueError("dataset output cannot be inside the raw episode")
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "source_episode": episode.manifest["episode"],
        "source_sha256": episode.report["content_sha256"],
        "simulated": episode.manifest["simulated"],
        "rows": len(samples),
        "fps": fps,
        "source_depth_scale_m_per_unit": episode.camera["depth_scale_m_per_unit"],
        "task": episode.manifest["task"],
    }
    with TemporaryDirectory(prefix=".franka-export-", dir=output.parent) as stage:
        dataset_path = Path(stage) / "dataset"
        dataset = lerobot.create(
            _REPO_ID,
            fps,
            _features(episode.camera["configuration"]),
            root=dataset_path,
            robot_type="franka",
            video_backend="pyav",
            rgb_encoder=rgb_encoder,
        )
        try:
            with (dataset_path / "franka_alignment.jsonl").open("x", encoding="utf-8") as stream:
                for index, sample in enumerate(samples):
                    _add_frame(dataset, episode, sample)
                    stream.write(json.dumps(_alignment_row(sample, index)) + "\n")
            dataset.save_episode(parallel_encoding=False)
        finally:
            dataset.finalize()
        if len(lerobot(_REPO_ID, root=dataset_path, video_backend="pyav")) != len(samples):
            raise RuntimeError("exported dataset has unexpected frame count")
        atomic_json(dataset_path / "franka_source.json", report)
        _unused_output(output)
        dataset_path.rename(output)
    return {**report, "path": str(output)}


def _camera_profile(camera: dict) -> tuple:
    config = camera["configuration"]
    return (
        config["width"],
        config["height"],
        config["fps"],
        camera["depth_format"],
        camera["depth_alignment"],
    )


def _check_space(path: Path, required: int) -> None:
    if shutil.disk_usage(path).free < required:
        raise OSError(errno.ENOSPC, "insufficient free space for dataset export", str(path))


def _zip_dataset(dataset_path: Path, archive: Path, reserve: int) -> None:
    def members():
        return (path for path in dataset_path.rglob("*") if path.is_file())

    # These files are already compressed. ZIP_STORED also makes the size predictable.
    _check_space(
        archive.parent, reserve + 256 + sum(path.stat().st_size + 512 for path in members())
    )
    with ZipFile(archive, mode="x") as bundle:
        for path in members():
            bundle.write(path, path.relative_to(dataset_path.parent))


def _bundle_input(path: Path) -> tuple[Path, dict, dict]:
    if path.name.endswith(".partial") or path.is_symlink():
        raise ValueError("expected a published complete episode")
    path = path.resolve(strict=True)
    manifest_path = path / "manifest.json"
    camera_path = path / "camera/manifest.json"
    if manifest_path.resolve() != manifest_path or camera_path.resolve() != camera_path:
        raise ValueError("unsafe raw episode metadata path")
    manifest = json.loads(manifest_path.read_text("utf-8"))
    if manifest.get("result") != "complete" or manifest.get("episode") != path.name:
        raise ValueError("expected a published complete episode")
    return path, manifest, json.loads(camera_path.read_text("utf-8"))["metadata"]


def _write_bundle_dataset(
    dataset_path: Path,
    episodes: list[tuple[Path, dict, dict]],
    fps: int,
    reserve: int,
) -> int:
    lerobot, rgb_encoder = _lerobot()
    config = episodes[0][2]["configuration"]
    dataset = lerobot.create(
        _REPO_ID,
        fps,
        _features(config),
        root=dataset_path,
        robot_type="franka",
        video_backend="pyav",
        rgb_encoder=rgb_encoder,
        encoder_threads=1,
    )
    sources = []
    offset = 0
    try:
        with (dataset_path / "franka_alignment.jsonl").open("x", encoding="utf-8") as stream:
            for episode_index, (path, manifest, camera) in enumerate(episodes):
                # Retain one audited raw episode at a time even when converting a large batch.
                episode = read_episode(path)
                if episode.manifest != manifest or episode.camera != camera:
                    raise ValueError("raw episode metadata changed during conversion")
                samples = select_frames(episode, fps)
                depth_dir = Path("depth") / f"episode-{episode_index:06d}"
                (dataset_path / depth_dir).mkdir(parents=True)
                for frame_index, sample in enumerate(samples):
                    row = _alignment_row(sample, frame_index)
                    raw_depth = episode.root / row["depth_image"]
                    depth_path = depth_dir / f"{frame_index:06d}.png"
                    # LeRobot writes an intermediate RGB PNG before encoding this episode.
                    _check_space(
                        dataset_path,
                        reserve + raw_depth.stat().st_size + config["width"] * config["height"] * 3,
                    )
                    _add_frame(dataset, episode, sample)
                    shutil.copyfile(raw_depth, dataset_path / depth_path)
                    row.update(
                        episode_index=episode_index,
                        dataset_index=offset + frame_index,
                        source_depth_image=row["depth_image"],
                        depth_image=depth_path.as_posix(),
                    )
                    stream.write(json.dumps(row) + "\n")
                _check_space(dataset_path, reserve)
                dataset.save_episode(parallel_encoding=False)
                sources.append(
                    {
                        "episode_index": episode_index,
                        "source_episode": episode.manifest["episode"],
                        "source_sha256": episode.report["content_sha256"],
                        "dataset_from_index": offset,
                        "dataset_to_index": offset + len(samples),
                        "camera": episode.camera,
                        "raw_manifest": episode.manifest,
                    }
                )
                offset += len(samples)
                del episode, samples
    finally:
        dataset.finalize()

    # Post-finalize item reads reload LeRobot metadata and Parquet from disk and
    # decode RGB locally, without a read-only constructor's Hub fallback.
    for source in sources:
        for index in (source["dataset_from_index"], source["dataset_to_index"] - 1):
            item = dataset[index]
            if (
                item["episode_index"].item() != source["episode_index"]
                or item["frame_index"].item() != index - source["dataset_from_index"]
                or tuple(item["observation.state"].shape) != (8,)
                or tuple(item["action"].shape) != (8,)
                or tuple(item[_RGB].shape) != (3, config["height"], config["width"])
            ):
                raise RuntimeError("unexpected LeRobot readback at an episode boundary")
    if dataset.num_episodes != len(episodes) or len(dataset.hf_dataset) != offset:
        raise RuntimeError("exported dataset has unexpected episode or frame count")
    atomic_json(dataset_path / "franka_source.json", {"fps": fps, "episodes": sources})
    return offset


def build_bundle(
    paths: list[Path], output: Path, *, fps: int | None = None, min_free_bytes: int = 0
) -> dict:
    """Export a compatible batch to an atomic directory with a downloadable LeRobot ZIP."""
    if not paths:
        raise ValueError("select at least one completed raw episode")
    if type(min_free_bytes) is not int or min_free_bytes < 0:
        raise ValueError("minimum free bytes must be a nonnegative integer")
    output = output.expanduser().absolute()
    _unused_output(output)
    destination = output.resolve()
    filesystem = next(parent for parent in destination.parents if parent.exists())
    _check_space(filesystem, min_free_bytes)

    episodes = [_bundle_input(path) for path in paths]
    ids = [path.name for path, _, _ in episodes]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate source episode")
    if any(destination.is_relative_to(path) for path, _, _ in episodes):
        raise ValueError("dataset output cannot be inside a raw episode")
    _, first_manifest, first_camera = episodes[0]
    task, simulated = first_manifest["task"], first_manifest["simulated"]
    if not isinstance(task, str) or not task.strip() or type(simulated) is not bool:
        raise ValueError("raw episode has invalid task or mode")
    profile = _camera_profile(first_camera)
    for _, manifest, camera in episodes[1:]:
        if manifest["task"] != task:
            raise ValueError("episodes must share one task")
        if manifest["simulated"] is not simulated:
            raise ValueError("episodes must share one recording mode")
        if _camera_profile(camera) != profile:
            raise ValueError("episodes must share one camera profile")
    export_fps = first_camera["configuration"]["fps"] if fps is None else fps
    output.parent.mkdir(parents=True, exist_ok=True)
    _check_space(output.parent, min_free_bytes)

    with TemporaryDirectory(prefix=".franka-export-", dir=output.parent) as stage:
        bundle = Path(stage) / "bundle"
        bundle.mkdir(mode=0o700)
        frames = _write_bundle_dataset(bundle / "dataset", episodes, export_fps, min_free_bytes)
        archive = bundle / "dataset.zip"
        _zip_dataset(bundle / "dataset", archive, min_free_bytes)
        catalog = {
            "id": output.name,
            "task": task,
            "episodes": ids,
            "frames": frames,
            "fps": export_fps,
            "simulated": simulated,
            "archive_bytes": archive.stat().st_size,
        }
        atomic_json(bundle / "catalog.json", catalog)
        _check_space(bundle, min_free_bytes)
        _unused_output(output)
        bundle.rename(output)
    return catalog
