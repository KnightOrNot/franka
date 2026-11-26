"""Validate and align published Franka episodes without hardware access."""

from __future__ import annotations

import hashlib
import io
import json
import math
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

from PIL import Image

from .telemetry import Cycle

FORMAT = "franka_capture_v1"


def atomic_json(path: Path, value: dict) -> None:
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n", "utf-8")
    temp.replace(path)


def _within(root: Path, path: Path) -> Path:
    if path.is_symlink() or path.resolve() != path or not path.is_relative_to(root):
        raise ValueError(f"unsafe episode path: {path.name}")
    if not path.is_file():
        raise ValueError(f"missing episode file: {path.name}")
    return path


@dataclass(frozen=True)
class FrameRecord:
    seq: int
    rgb: str
    depth: str
    received_ns: int
    rgb_ns: int
    depth_ns: int

    @property
    def timestamp_ns(self) -> int:
        return max(self.rgb_ns, self.depth_ns)


@dataclass(frozen=True)
class Sample:
    frame: FrameRecord
    cycle: Cycle
    target_ns: int


@dataclass(frozen=True)
class Episode:
    root: Path
    manifest: dict
    camera: dict
    frames: tuple[FrameRecord, ...]
    cycles: tuple[Cycle, ...]
    matched: tuple[Sample, ...]
    report: dict


def _load(root: Path, relative: str, *, lines: bool = False):
    path = _within(root, root / relative)
    if lines:
        with path.open(encoding="utf-8") as stream:
            return [json.loads(line) for line in stream]
    return json.loads(path.read_text("utf-8"))


def _media(root: Path, path: str, image_mode: str, size: tuple[int, int], digest) -> None:
    try:
        file = _within(root, root / "camera" / path)
        data = file.read_bytes()
        digest.update(hashlib.sha256(data).digest())
        with Image.open(io.BytesIO(data)) as image:
            if image.format != "PNG" or image.mode != image_mode or image.size != size:
                raise ValueError("unexpected PNG type or dimensions")
            image.load()
    except (OSError, ValueError, SyntaxError) as exc:
        raise ValueError(f"invalid camera media {path}: {exc}") from exc


def _integer(row: dict, key: str, minimum: int = 0) -> int:
    value = row[key]
    if type(value) is not int or value < minimum:
        raise ValueError(f"invalid integer: {key}")
    return value


def _sdk_stamp(row: dict, name: str) -> tuple[str, float]:
    domain = row[f"{name}_sdk_timestamp_domain"]
    stamp = row[f"{name}_sdk_timestamp_ms"]
    if not isinstance(domain, str) or not domain:
        raise ValueError("missing camera timestamp domain")
    if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0:
        raise ValueError("invalid camera SDK timestamp")
    return domain, stamp


def _frames(root: Path, camera: dict, digest) -> tuple[list[FrameRecord], tuple[str, str]]:
    complete = _load(root, "camera/COMPLETE.json")
    rows = _load(root, "camera/frames.jsonl", lines=True)
    if (
        not rows
        or complete.get("complete") is not True
        or complete.get("dropped") != 0
        or complete.get("frames") != len(rows)
        or complete.get("persisted") != len(rows)
    ):
        raise ValueError("camera recording is not complete")
    config = camera["configuration"]
    size = (_integer(config, "width", 1), _integer(config, "height", 1))
    frames = []
    previous_numbers = None
    previous_domains = None
    previous_times = None
    for seq, row in enumerate(rows):
        rgb, depth = f"rgb/{seq:06d}.png", f"depth/{seq:06d}.png"
        if _integer(row, "sequence") != seq or row["rgb"] != rgb or row["depth"] != depth:
            raise ValueError("invalid camera sequence or media reference")
        arrived = _integer(row, "received_monotonic_ns", 1)
        # This library samples wall and monotonic at different points while
        # preparing a frame. Use receipt for alignment and retain SDK time as raw
        # evidence without deriving a clock offset from the unpaired readings.
        _integer(row, "received_wall_time_ns", 1)
        rgb_domain, rgb_stamp = _sdk_stamp(row, "rgb")
        depth_domain, depth_stamp = _sdk_stamp(row, "depth")
        domains = (rgb_domain, depth_domain)
        numbers = tuple(_integer(row, key) for key in ("rgb_frame_number", "depth_frame_number"))
        times = (arrived, rgb_stamp, depth_stamp)
        if previous_domains is not None and domains != previous_domains:
            raise ValueError("camera timestamp domain changed")
        if previous_numbers is not None and any(
            n != p + 1 for n, p in zip(numbers, previous_numbers, strict=True)
        ):
            raise ValueError("camera frame was lost, repeated or reset")
        if previous_times is not None and any(
            a <= b for a, b in zip(times, previous_times, strict=True)
        ):
            raise ValueError("camera timestamps did not advance")
        _media(root, rgb, "RGB", size, digest)
        _media(root, depth, "I;16", size, digest)
        previous_numbers, previous_domains, previous_times = numbers, domains, times
        frames.append(FrameRecord(seq, rgb, depth, arrived, arrived, arrived))
    assert previous_domains is not None
    return frames, previous_domains


def analyze(root: Path, manifest: dict) -> Episode:
    """Validate camera files and pair whole cycles with images (same host clock)."""
    root = root.resolve(strict=True)
    if manifest.get("format") != FORMAT:
        raise ValueError("unsupported episode format")
    start, end = _integer(manifest, "start_ns", 1), _integer(manifest, "end_ns", 1)
    skew = _integer(manifest, "max_skew_ns", 1)
    stale = _integer(manifest, "stale_ns", 1)
    requested_end = _integer(manifest, "requested_end_ns", 1)
    if not start < end <= requested_end or requested_end - end > stale:
        raise ValueError("invalid recording boundaries")
    camera_manifest = _load(root, "camera/manifest.json")
    if (
        camera_manifest.get("format") != "xcore_rgbd_frames_v1"
        or camera_manifest.get("episode") != manifest["episode"]
    ):
        raise ValueError("camera manifest does not belong to this episode")
    camera = camera_manifest["metadata"]
    scale = camera["depth_scale_m_per_unit"]
    if (
        type(scale) not in (float, int)
        or not math.isfinite(scale)
        or scale <= 0
        or camera.get("depth_alignment") != "native_depth"
        or camera.get("depth_format") != "z16"
        or not all(
            isinstance(camera.get(key), dict)
            for key in ("color_intrinsics", "depth_intrinsics", "depth_to_color")
        )
    ):
        raise ValueError("missing or invalid native depth calibration")
    digest = hashlib.sha256()
    for path in (
        "robot.jsonl",
        "camera/manifest.json",
        "camera/frames.jsonl",
        "camera/COMPLETE.json",
    ):
        digest.update(hashlib.sha256(_within(root, root / path).read_bytes()).digest())
    frames, domains = _frames(root, camera, digest)
    cycles = [Cycle.from_row(row) for row in _load(root, "robot.jsonl", lines=True)]
    if len(cycles) < 2:
        raise ValueError("not enough robot telemetry")
    for i, cycle in enumerate(cycles):
        if not start <= cycle.command_ns <= end or cycle.command_ns - cycle.observed_ns > stale:
            raise ValueError("robot data falls outside the episode or was stale")
        if i:
            prev = cycles[i - 1]
            if (
                cycle.epoch != prev.epoch
                or cycle.seq != prev.seq + 1
                or cycle.observed_ns <= prev.observed_ns
                or cycle.command_ns <= prev.command_ns
            ):
                raise ValueError("robot telemetry lost sequence, time order or source epoch")
    if cycles[0].command_ns - start > stale or end != cycles[-1].command_ns:
        raise ValueError("robot telemetry did not cover the episode boundaries")
    observations = [cycle.observed_ns for cycle in cycles]
    commands = [cycle.command_ns for cycle in cycles]
    usable = [f for f in frames if start <= min(f.rgb_ns, f.depth_ns) <= f.timestamp_ns <= end]
    matched = []
    errors = []
    for frame in usable:
        anchor = frame.timestamp_ns
        if anchor < observations[0] - skew or anchor > commands[-1]:
            continue  # RGB-D and robot streams have independent episode edges.
        i = max(bisect_left(observations, anchor), bisect_left(commands, anchor))
        # Prefer the freshest nearby pre-command observation; avoid sending a target
        # as the label for an image captured (or received) after it was submitted.
        valid = [
            cycles[j] for j in (i - 1, i) if 0 <= j < len(cycles) and cycles[j].command_ns >= anchor
        ]
        if not valid:
            raise ValueError("no causal robot cycle for camera frame")
        cycle = min(valid, key=lambda c: abs(c.observed_ns - anchor))
        error = max(abs(cycle.observed_ns - frame.rgb_ns), abs(cycle.observed_ns - frame.depth_ns))
        if error > skew or cycle.observed_ns < start:
            if not matched and cycle.observed_ns < start:
                continue
            raise ValueError(f"camera/robot clock skew exceeds window at frame {frame.seq}")
        errors.append(error)
        matched.append(Sample(frame, cycle, anchor))
    if len(matched) < 2:
        raise ValueError("not enough aligned RGB-D and robot observations")
    errors.sort()
    report = {
        "robot_cycles": len(cycles),
        "camera_pairs": len(frames),
        "matched_pairs": len(matched),
        "edge_pairs_excluded": len(frames) - len(matched),
        "max_offset_ms": round(errors[-1] / 1e6, 3),
        "p95_offset_ms": round(errors[math.ceil(0.95 * len(errors)) - 1] / 1e6, 3),
        "image_clock": "host_receive",
        "camera_sdk_domains": dict(zip(("rgb", "depth"), domains, strict=True)),
        "exposure_time_uncertainty_ns": None,
        "robot_clock": "host_receive",
        "content_sha256": digest.hexdigest(),
    }
    return Episode(root, manifest, camera, tuple(frames), tuple(cycles), tuple(matched), report)


def read_episode(path: Path) -> Episode:
    if path.name.endswith(".partial") or path.is_symlink():
        raise ValueError("expected a published complete episode")
    root = path.resolve(strict=True)
    manifest = _load(root, "manifest.json")
    if manifest.get("result") != "complete" or manifest.get("episode") != root.name:
        raise ValueError("expected a published complete episode")
    result = analyze(root, manifest)
    if manifest.get("report") != result.report:
        raise ValueError("episode audit differs from the completion record")
    return result


def select_frames(episode: Episode, fps: int) -> tuple[Sample, ...]:
    camera_fps = episode.camera["configuration"]["fps"]
    if type(fps) is not int or not 1 <= fps <= camera_fps:
        raise ValueError(
            "dataset FPS must be positive and no greater than the configured camera FPS"
        )
    samples = episode.matched
    stamps = [s.frame.timestamp_ns for s in samples]
    selected = []
    previous = -1
    tolerance = min(500_000_000 // fps, episode.manifest["max_skew_ns"])
    nominal = 1_000_000_000 / camera_fps
    observed = (stamps[-1] - stamps[0]) / (len(stamps) - 1)
    # Fit the real receive cadence only for a near-nominal continuous stream.
    # A small drift otherwise accumulates into a false gap on longer episodes.
    if abs(observed - nominal) <= 0.03 * nominal and all(
        b - a <= 1.5 * nominal for a, b in zip(stamps, stamps[1:], strict=False)
    ):
        period = observed * camera_fps / fps
        ticks = (len(stamps) - 1) * fps // camera_fps + 1
    else:
        period = 1_000_000_000 / fps
        ticks = (stamps[-1] - stamps[0]) * fps // 1_000_000_000 + 1
    for tick in range(ticks):
        target = stamps[0] + round(tick * period)
        i = bisect_left(stamps, target)
        eligible = [j for j in (i - 1, i) if previous < j < len(stamps)]
        if not eligible:
            raise ValueError("uniform export grid would repeat a camera frame")
        j = min(eligible, key=lambda k: abs(stamps[k] - target))
        if abs(stamps[j] - target) > tolerance:
            raise ValueError("uniform export grid has a gap")
        selected.append(Sample(samples[j].frame, samples[j].cycle, target))
        previous = j
    if len(selected) < 2:
        raise ValueError("not enough samples on the export grid")
    return tuple(selected)
