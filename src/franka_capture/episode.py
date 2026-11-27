"""Validate and align published Franka episodes without hardware access."""

from __future__ import annotations

import hashlib
import io
import json
import math
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, replace
from pathlib import Path

from PIL import Image

from .telemetry import Cycle

FORMAT = "franka_capture_v1"
FORMAT_MULTI = "franka_capture_v2"
DUAL_ALIGNMENT = "latest_camera_causal_v1"


def camera_directories(manifest: dict) -> dict[str, Path]:
    """Return the raw camera directories, ordered by their stable roles."""
    if manifest.get("format") == FORMAT:
        return {"front": Path("camera")}
    if manifest.get("format") == FORMAT_MULTI:
        if manifest.get("cameras") != ["left", "right"]:
            raise ValueError("dual episode must declare left and right camera roles")
        return {role: Path("cameras") / role for role in ("left", "right")}
    raise ValueError("unsupported episode format")


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
    frames_by_camera: dict[str, FrameRecord] | None = None


@dataclass(frozen=True)
class Episode:
    root: Path
    manifest: dict
    camera: dict
    frames: tuple[FrameRecord, ...]
    cycles: tuple[Cycle, ...]
    matched: tuple[Sample, ...]
    report: dict
    cameras: dict[str, dict] | None = None
    camera_frames: dict[str, tuple[FrameRecord, ...]] | None = None


def _load(root: Path, relative: str | Path, *, lines: bool = False):
    path = _within(root, root / relative)
    if lines:
        with path.open(encoding="utf-8") as stream:
            return [json.loads(line) for line in stream]
    return json.loads(path.read_text("utf-8"))


def _media(root: Path, path: Path, image_mode: str, size: tuple[int, int], digest) -> None:
    try:
        file = _within(root, root / path)
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


def _frames(
    root: Path, directory: Path, camera: dict, digest
) -> tuple[list[FrameRecord], tuple[str, str]]:
    complete = _load(root, directory / "COMPLETE.json")
    rows = _load(root, directory / "frames.jsonl", lines=True)
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
        _media(root, directory / rgb, "RGB", size, digest)
        _media(root, directory / depth, "I;16", size, digest)
        previous_numbers, previous_domains, previous_times = numbers, domains, times
        frames.append(FrameRecord(seq, rgb, depth, arrived, arrived, arrived))
    assert previous_domains is not None
    return frames, previous_domains


def _camera_metadata(root: Path, directory: Path, episode: str) -> dict:
    camera_manifest = _load(root, directory / "manifest.json")
    if (
        camera_manifest.get("format") != "xcore_rgbd_frames_v1"
        or camera_manifest.get("episode") != episode
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
    return camera


def camera_metadata(root: Path, manifest: dict) -> dict[str, dict]:
    """Read and validate metadata without reading any frame media."""
    cameras = {}
    serials = set()
    dual = manifest.get("format") == FORMAT_MULTI
    for role, directory in camera_directories(manifest).items():
        try:
            camera = _camera_metadata(root, directory, manifest["episode"])
            if dual:
                serial = camera["device"]["serial_number"]
                if not isinstance(serial, str) or not serial.strip() or serial in serials:
                    raise ValueError("camera serial is missing or not distinct")
                serials.add(serial)
                _integer(camera["configuration"], "fps", 1)
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            if not dual:
                raise
            raise ValueError(f"{role} camera metadata: {exc}") from exc
        cameras[role] = camera
    return cameras


def _cycles(root: Path, start: int, end: int, stale: int, *, dual: bool) -> list[Cycle]:
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
            if dual and cycle.command_ns - prev.command_ns > stale:
                raise ValueError("robot telemetry has a stale interval")
    if cycles[0].command_ns - start > stale or end != cycles[-1].command_ns:
        raise ValueError("robot telemetry did not cover the episode boundaries")
    return cycles


def _single_samples(
    frames: list[FrameRecord], cycles: list[Cycle], start: int, end: int, skew: int
) -> tuple[list[Sample], list[int]]:
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
    return matched, errors


def _dual_stream(
    frames: list[FrameRecord], start: int, end: int, stale: int, fps: int, role: str
) -> list[FrameRecord]:
    usable = [f for f in frames if start <= f.timestamp_ns <= end]
    nominal = 1_000_000_000 / fps
    gap = min(stale, 1.8 * nominal)
    if (
        len(usable) < 2
        or usable[0].timestamp_ns - start > gap
        or end - usable[-1].timestamp_ns > gap
        or any(
            after.timestamp_ns - before.timestamp_ns > gap
            for before, after in zip(usable, usable[1:], strict=False)
        )
    ):
        raise ValueError(f"{role} camera stream is missing, stale or has a frame gap")
    # Use host receipt for matching, but reject sustained drift from the recorded profile.
    intervals = len(usable) - 1
    elapsed = usable[-1].timestamp_ns - usable[0].timestamp_ns
    if abs(elapsed - intervals * nominal) > max(0.5 * nominal, 0.03 * intervals * nominal):
        raise ValueError(f"{role} camera receive cadence drifted from its configured FPS")
    return usable


def _dual_samples(
    frames: dict[str, list[FrameRecord]],
    cameras: dict[str, dict],
    cycles: list[Cycle],
    start: int,
    end: int,
    skew: int,
    stale: int,
    *,
    latest_causal: bool,
) -> tuple[list[Sample], list[int]]:
    rates = {role: camera["configuration"]["fps"] for role, camera in cameras.items()}
    usable = {
        role: _dual_stream(stream, start, end, stale, rates[role], role)
        for role, stream in frames.items()
    }
    anchor, other = sorted(rates, key=rates.__getitem__)
    partner = usable[other]
    stamps = [frame.timestamp_ns for frame in partner]
    edge = 500_000_000 / rates[other]
    same_rate = rates[anchor] == rates[other]
    observations = [cycle.observed_ns for cycle in cycles]
    commands = [cycle.command_ns for cycle in cycles]
    matched = []
    errors = []
    previous_frame = previous_cycle = -1
    for frame in usable[anchor]:
        stamp = frame.timestamp_ns
        if stamp < stamps[0] - edge or stamp > stamps[-1] + edge:
            continue  # Independent camera starts and stops can straddle a sample boundary.
        first = previous_frame + 1
        if first == len(stamps):
            raise ValueError(f"{other} camera ran out of frames during alignment")
        if same_rate and previous_frame >= 0:
            # At equal rates preserve ordinal correspondence through arrival jitter.
            j = first
        else:
            i = bisect_left(stamps, stamp, lo=first)
            j = min(
                (k for k in (i - 1, i) if first <= k < len(stamps)),
                key=lambda k: abs(stamps[k] - stamp),
            )
        if (pair_offset := abs(stamp - stamps[j])) > skew:
            if latest_causal:
                raise ValueError(
                    f"{other} camera clock skew {pair_offset / 1e6:.1f} ms against {anchor} "
                    f"exceeds {skew / 1e6:g} ms"
                )
            raise ValueError(f"{other} camera clock skew exceeds the {anchor} camera window")
        low, high = sorted((stamp, stamps[j]))
        first = max(
            previous_cycle + 1, bisect_left(commands, high), bisect_left(observations, start)
        )
        # The camera pair already passed its own skew check. Match the robot to the
        # time both frames were available, without counting camera phase twice.
        target = high if latest_causal else (low + high) // 2
        last = bisect_right(commands, high + skew, lo=first) if latest_causal else len(cycles)
        i = bisect_left(observations, target, lo=first, hi=last)
        candidates = [k for k in (i - 1, i) if first <= k < last]
        if not candidates:
            if latest_causal:
                raise ValueError(
                    f"latest camera/robot has no unused causal command within {skew / 1e6:g} ms"
                )
            raise ValueError("no unused causal robot cycle for dual camera sample")
        k = min(candidates, key=lambda k: abs(observations[k] - target))
        error = max(observations[k] - low, high - observations[k])
        if latest_causal:
            if (robot_offset := abs(observations[k] - high)) > skew:
                raise ValueError(
                    f"latest camera/robot observation skew {robot_offset / 1e6:.1f} ms "
                    f"exceeds {skew / 1e6:g} ms"
                )
        elif error > skew:
            raise ValueError("dual camera/robot clock skew exceeds window")
        selected = {anchor: frame, other: partner[j]}
        matched.append(Sample(selected["left"], cycles[k], high, selected))
        errors.append(error)
        previous_frame, previous_cycle = j, k
    return matched, errors


def analyze(root: Path, manifest: dict) -> Episode:
    """Validate camera files and pair whole cycles with images on the host clock."""
    root = root.resolve(strict=True)
    directories = camera_directories(manifest)
    dual = manifest["format"] == FORMAT_MULTI
    policy = manifest.get("dual_alignment")
    if policy is not None and (not dual or policy != DUAL_ALIGNMENT):
        raise ValueError("unsupported episode alignment policy")
    latest_causal = policy == DUAL_ALIGNMENT
    start, end = _integer(manifest, "start_ns", 1), _integer(manifest, "end_ns", 1)
    skew = _integer(manifest, "max_skew_ns", 1)
    stale = _integer(manifest, "stale_ns", 1)
    requested_end = _integer(manifest, "requested_end_ns", 1)
    if not start < end <= requested_end or requested_end - end > stale:
        raise ValueError("invalid recording boundaries")
    cameras = camera_metadata(root, manifest)
    digest = hashlib.sha256()
    digest.update(hashlib.sha256(_within(root, root / "robot.jsonl").read_bytes()).digest())
    camera_frames, camera_domains = {}, {}
    for role, directory in directories.items():
        try:
            for file in ("manifest.json", "frames.jsonl", "COMPLETE.json"):
                path = _within(root, root / directory / file)
                digest.update(hashlib.sha256(path.read_bytes()).digest())
            camera_frames[role], domains = _frames(root, directory, cameras[role], digest)
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            if not dual:
                raise
            raise ValueError(f"{role} camera recording: {exc}") from exc
        camera_domains[role] = dict(zip(("rgb", "depth"), domains, strict=True))
    cycles = _cycles(root, start, end, stale, dual=dual)
    primary = next(iter(directories))
    frames = camera_frames[primary]
    if dual:
        matched, errors = _dual_samples(
            camera_frames, cameras, cycles, start, end, skew, stale, latest_causal=latest_causal
        )
    else:
        matched, errors = _single_samples(frames, cycles, start, end, skew)
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
        "camera_sdk_domains": camera_domains if dual else camera_domains[primary],
        "exposure_time_uncertainty_ns": None,
        "robot_clock": "host_receive",
        "content_sha256": digest.hexdigest(),
    }
    if dual:
        report["camera_pairs_by_role"] = {
            role: len(stream) for role, stream in camera_frames.items()
        }
        report["excluded_pairs_by_role"] = {
            role: len(stream) - len(matched) for role, stream in camera_frames.items()
        }
    if latest_causal:
        pair_ns, observation_ns, command_ns = [], [], []
        for sample in matched:
            stamps = tuple(frame.timestamp_ns for frame in sample.frames_by_camera.values())
            latest = max(stamps)
            pair_ns.append(latest - min(stamps))
            observation_ns.append(abs(latest - sample.cycle.observed_ns))
            command_ns.append(sample.cycle.command_ns - latest)
        report.update(
            dual_alignment=policy,
            max_camera_pair_offset_ms=round(max(pair_ns) / 1e6, 3),
            max_robot_to_latest_offset_ms=round(max(observation_ns) / 1e6, 3),
            max_command_after_latest_ms=round(max(command_ns) / 1e6, 3),
        )
    return Episode(
        root,
        manifest,
        cameras[primary],
        tuple(frames),
        tuple(cycles),
        tuple(matched),
        report,
        cameras if dual else None,
        {role: tuple(stream) for role, stream in camera_frames.items()} if dual else None,
    )


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
    cameras = getattr(episode, "cameras", None)  # Existing single-camera callers may duck-type.
    camera_fps = min(
        c["configuration"]["fps"] for c in (cameras or {"front": episode.camera}).values()
    )
    if type(fps) is not int or not 1 <= fps <= camera_fps:
        raise ValueError(
            "dataset FPS must be positive and no greater than the configured camera FPS"
        )
    samples = episode.matched
    stamps = [s.target_ns if cameras else s.frame.timestamp_ns for s in samples]
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
        sample = samples[j]
        # New episodes are sampled by the later frame of a validated pair. The
        # older frame's offset from that pair was checked during raw analysis.
        if (
            sample.frames_by_camera
            and episode.manifest.get("dual_alignment") != DUAL_ALIGNMENT
            and any(
                abs(frame.timestamp_ns - target) > episode.manifest["max_skew_ns"]
                for frame in sample.frames_by_camera.values()
            )
        ):
            raise ValueError("uniform export grid has a dual camera gap")
        selected.append(replace(sample, target_ns=target))
        previous = j
    if len(selected) < 2:
        raise ValueError("not enough samples on the export grid")
    return tuple(selected)
