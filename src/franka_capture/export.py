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

from .episode import (
    Episode,
    FrameRecord,
    Sample,
    atomic_json,
    camera_directories,
    camera_metadata,
    read_episode,
    select_frames,
)
from .telemetry import JOINT_NAMES

_REPO_ID = "local/franka"


def _lerobot():
    try:
        from lerobot.configs.video import RGBEncoderConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError("install the export extra: uv sync --locked --extra export") from exc
    return LeRobotDataset, RGBEncoderConfig(vcodec="h264")


def _image_key(role: str) -> str:
    return f"observation.images.{role}"


def _features(cameras: dict[str, dict]) -> dict:
    vector = {"dtype": "float32", "shape": (8,), "names": list(JOINT_NAMES)}
    return {
        "observation.state": vector,
        "action": dict(vector),
        **{
            _image_key(role): {
                "dtype": "video",
                "shape": (config["height"], config["width"], 3),
                "names": ["height", "width", "channels"],
            }
            for role, config in _configurations(cameras).items()
        },
    }


def _unused_output(output: Path) -> None:
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"dataset output already exists: {output}")


def _configurations(cameras: dict[str, dict]) -> dict[str, dict]:
    return {role: camera["configuration"] for role, camera in cameras.items()}


def _add_frame(dataset, episode: Episode, sample: Sample, directories: dict[str, Path]) -> None:
    row = {
        "task": episode.manifest["task"],
        "observation.state": np.asarray(sample.cycle.state, dtype=np.float32),
        "action": np.asarray(sample.cycle.action, dtype=np.float32),
    }
    frames = sample.frames_by_camera or {"front": sample.frame}
    for role, directory in directories.items():
        with Image.open(episode.root / directory / frames[role].rgb) as image:
            row[_image_key(role)] = np.array(image, dtype=np.uint8)
    dataset.add_frame(row)


def _frame_alignment(frame: FrameRecord, directory: Path) -> dict:
    return {
        "source_frame_index": frame.seq,
        "rgb_image": (directory / frame.rgb).as_posix(),
        "depth_image": (directory / frame.depth).as_posix(),
        "rgb_ns": frame.rgb_ns,
        "depth_ns": frame.depth_ns,
        "camera_received_ns": frame.received_ns,
    }


def _alignment_row(sample: Sample, index: int, directories: dict[str, Path]) -> dict:
    cycle = sample.cycle
    row = {
        "frame_index": index,
        "target_ns": sample.target_ns,
        "source_epoch": cycle.epoch,
        "source_cycle_index": cycle.seq,
        "observed_ns": cycle.observed_ns,
        "command_ns": cycle.command_ns,
    }
    if sample.frames_by_camera is None:
        row.update(_frame_alignment(sample.frame, directories["front"]))
    else:
        row["cameras"] = {
            role: _frame_alignment(sample.frames_by_camera[role], directory)
            for role, directory in directories.items()
        }
    return row


def _write_samples(
    dataset,
    dataset_path: Path,
    stream,
    episode: Episode,
    samples: tuple[Sample, ...],
    *,
    depth_dir: Path | None,
    reserve: int,
    episode_index: int | None = None,
    offset: int = 0,
) -> None:
    directories = camera_directories(episode.manifest)
    cameras = episode.cameras or {"front": episode.camera}
    if depth_dir is not None:
        for role in directories:
            parent = depth_dir / role if episode.cameras else depth_dir
            (dataset_path / parent).mkdir(parents=True, exist_ok=True)
    # LeRobot writes an intermediate RGB PNG for each camera before encoding an episode.
    rgb_bytes = sum(c["width"] * c["height"] * 3 for c in _configurations(cameras).values())
    for frame_index, sample in enumerate(samples):
        row = _alignment_row(sample, frame_index, directories)
        role_rows = row.get("cameras", {"front": row})
        if depth_dir is not None:
            raw_depths = {
                role: episode.root / details["depth_image"] for role, details in role_rows.items()
            }
            _check_space(
                dataset_path,
                reserve + rgb_bytes + sum(path.stat().st_size for path in raw_depths.values()),
            )
            for role, details in role_rows.items():
                parent = depth_dir / role if episode.cameras else depth_dir
                depth_path = parent / f"{frame_index:06d}.png"
                shutil.copyfile(raw_depths[role], dataset_path / depth_path)
                details.update(
                    source_depth_image=details["depth_image"], depth_image=depth_path.as_posix()
                )
        _add_frame(dataset, episode, sample, directories)
        if episode_index is not None:
            row.update(episode_index=episode_index, dataset_index=offset + frame_index)
        stream.write(json.dumps(row) + "\n")


def export_episode(path: Path, output: Path, *, fps: int) -> dict:
    episode = read_episode(path)
    samples = select_frames(episode, fps)
    lerobot, rgb_encoder = _lerobot()
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
        "task": episode.manifest["task"],
    }
    cameras = episode.cameras or {"front": episode.camera}
    if episode.cameras:
        report["cameras"] = cameras
    else:
        report["source_depth_scale_m_per_unit"] = episode.camera["depth_scale_m_per_unit"]
    with TemporaryDirectory(prefix=".franka-export-", dir=output.parent) as stage:
        dataset_path = Path(stage) / "dataset"
        dataset = lerobot.create(
            _REPO_ID,
            fps,
            _features(cameras),
            root=dataset_path,
            robot_type="franka",
            video_backend="pyav",
            rgb_encoder=rgb_encoder,
            encoder_threads=1,
        )
        try:
            with (dataset_path / "franka_alignment.jsonl").open("x", encoding="utf-8") as stream:
                _write_samples(
                    dataset,
                    dataset_path,
                    stream,
                    episode,
                    samples,
                    depth_dir=Path("depth/episode-000000") if episode.cameras else None,
                    reserve=0,
                )
            dataset.save_episode(parallel_encoding=False)
        finally:
            dataset.finalize()
        if len(lerobot(_REPO_ID, root=dataset_path, video_backend="pyav")) != len(samples):
            raise RuntimeError("exported dataset has unexpected frame count")
        atomic_json(dataset_path / "franka_source.json", report)
        _unused_output(output)
        dataset_path.rename(output)
    return {**report, "path": str(output)}


def _camera_profiles(cameras: dict[str, dict]) -> dict[str, tuple]:
    profiles = {}
    for role, camera in cameras.items():
        config = camera["configuration"]
        geometry = (config["width"], config["height"], config["fps"])
        encoding = (camera["depth_format"], camera["depth_alignment"])
        # Keep v1 profile rules as-is. Dual roles must keep their physical bindings
        # and full recorded settings, in addition to geometry and depth encoding.
        binding = () if role == "front" else (camera["device"]["serial_number"], config)
        profiles[role] = geometry + encoding + binding
    return profiles


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
    if manifest_path.resolve() != manifest_path:
        raise ValueError("unsafe raw episode metadata path")
    manifest = json.loads(manifest_path.read_text("utf-8"))
    if manifest.get("result") != "complete" or manifest.get("episode") != path.name:
        raise ValueError("expected a published complete episode")
    return path, manifest, camera_metadata(path, manifest)


def _write_bundle_dataset(
    dataset_path: Path,
    episodes: list[tuple[Path, dict, dict]],
    fps: int,
    reserve: int,
) -> int:
    lerobot, rgb_encoder = _lerobot()
    configs = _configurations(episodes[0][2])
    dataset = lerobot.create(
        _REPO_ID,
        fps,
        _features(episodes[0][2]),
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
            for episode_index, (path, manifest, cameras) in enumerate(episodes):
                # Retain one audited raw episode at a time even when converting a large batch.
                episode = read_episode(path)
                if (
                    episode.manifest != manifest
                    or (episode.cameras or {"front": episode.camera}) != cameras
                ):
                    raise ValueError("raw episode metadata changed during conversion")
                samples = select_frames(episode, fps)
                depth_dir = Path("depth") / f"episode-{episode_index:06d}"
                _write_samples(
                    dataset,
                    dataset_path,
                    stream,
                    episode,
                    samples,
                    depth_dir=depth_dir,
                    reserve=reserve,
                    episode_index=episode_index,
                    offset=offset,
                )
                _check_space(dataset_path, reserve)
                dataset.save_episode(parallel_encoding=False)
                sources.append(
                    {
                        "episode_index": episode_index,
                        "source_episode": episode.manifest["episode"],
                        "source_sha256": episode.report["content_sha256"],
                        "dataset_from_index": offset,
                        "dataset_to_index": offset + len(samples),
                        **(
                            {"cameras": episode.cameras}
                            if episode.cameras
                            else {
                                "camera": episode.camera,
                            }
                        ),
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
                or any(
                    tuple(item[_image_key(role)].shape) != (3, config["height"], config["width"])
                    for role, config in configs.items()
                )
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
    _, first_manifest, first_cameras = episodes[0]
    task, simulated = first_manifest["task"], first_manifest["simulated"]
    if not isinstance(task, str) or not task.strip() or type(simulated) is not bool:
        raise ValueError("raw episode has invalid task or mode")
    profile = _camera_profiles(first_cameras)
    for _, manifest, cameras in episodes[1:]:
        if manifest["task"] != task:
            raise ValueError("episodes must share one task")
        if manifest["simulated"] is not simulated:
            raise ValueError("episodes must share one recording mode")
        if manifest["format"] != first_manifest["format"]:
            raise ValueError("episodes must share one camera layout")
        if _camera_profiles(cameras) != profile:
            raise ValueError("episodes must share one camera profile")
    export_fps = (
        min(c["configuration"]["fps"] for c in first_cameras.values()) if fps is None else fps
    )
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
