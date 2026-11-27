"""Operate capture, check data offline, and export training episodes."""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from contextlib import ExitStack, contextmanager
from pathlib import Path

from .bridge import Sender
from .camera import make_cameras
from .datasets import DatasetService
from .episode import camera_directories, read_episode
from .recording import CaptureError, Collector
from .telemetry import PACKET_LIMIT, Cycle, open_receiver, socket_default
from .web import OperatorServer


@contextmanager
def _recording_lock(root: Path):
    root.mkdir(parents=True, mode=0o700, exist_ok=True)
    fd = os.open(root / ".capture.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise CaptureError("this recording root is already in use") from exc
        yield
    finally:
        os.close(fd)


def _receive(receiver, collector: Collector, stopping: threading.Event) -> None:
    while not stopping.is_set():
        try:
            packet = receiver.recv(PACKET_LIMIT + 1)
        except socket.timeout:
            continue
        except OSError as exc:
            collector.input_error(f"telemetry socket failed: {exc}")
            return
        try:
            collector.ingest(Cycle.decode(packet))
        except (ValueError, OverflowError, RecursionError) as exc:
            collector.input_error(f"invalid robot telemetry: {exc}")


def _simulate(path: Path, stopping: threading.Event) -> None:
    with Sender(str(path)) as sender:
        while not stopping.wait(0.01):
            t = time.monotonic_ns()
            value = 0.2 * math.sin(t / 1e9)
            sender.publish([value] * 7 + [0.4], [value + 0.01] * 7 + [0.5], t, t)


def _monitor(collector: Collector, stopping: threading.Event) -> None:
    while not stopping.wait(0.075):
        collector.poll()


@contextmanager
def _launch(args):
    endpoint = args.socket.expanduser().absolute()
    root = args.data_root or Path("data/sim/raw" if args.simulate else "data/raw")
    root = root.expanduser().resolve()
    with ExitStack() as stack:
        stack.enter_context(_recording_lock(root))
        receiver = stack.enter_context(open_receiver(endpoint))
        cameras = make_cameras(
            root,
            simulate=args.simulate,
            dual_camera=args.dual_camera,
            serial=args.serial,
            left_serial=args.left_serial,
            right_serial=args.right_serial,
            width=args.width,
            height=args.height,
            fps=args.camera_fps,
        )
        collector = Collector(
            root,
            cameras,
            simulated=args.simulate,
            min_free_mb=args.min_free_mb,
            stale_ms=args.stale_ms,
            max_skew_ms=args.max_skew_ms,
            max_seconds=args.max_seconds,
            control_revision=args.control_revision,
        )
        stack.callback(collector.close)
        stopping = threading.Event()
        workers = [
            threading.Thread(target=_receive, args=(receiver, collector, stopping), daemon=True),
            threading.Thread(target=_monitor, args=(collector, stopping), daemon=True),
        ]
        if args.simulate:
            workers.append(
                threading.Thread(target=_simulate, args=(endpoint, stopping), daemon=True)
            )

        def stop_workers():
            stopping.set()
            for worker in workers:
                if worker.ident is not None:
                    worker.join(timeout=2)

        stack.callback(stop_workers)
        for camera in collector.cameras.values():
            camera.start({})
        for worker in workers:
            worker.start()
        yield collector


def _start_args(parser):
    parser.add_argument(
        "--data-root", type=Path, help="raw directory (different defaults for simulation)"
    )
    parser.add_argument("--socket", type=Path, default=socket_default())
    parser.add_argument("--serial", default="", help="bind a single D435i camera")
    parser.add_argument("--dual-camera", action="store_true", help="require two D435i cameras")
    parser.add_argument("--left-serial", help="bind the left camera in dual mode")
    parser.add_argument("--right-serial", help="bind the right camera in dual mode")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--stale-ms", type=int, default=350)
    parser.add_argument("--max-skew-ms", type=int, default=35)
    parser.add_argument("--min-free-mb", type=int, default=1024)
    parser.add_argument("--max-seconds", type=int, default=300)
    parser.add_argument("--control-revision", default="unspecified")


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(prog="franka-capture")
    sub = root.add_subparsers(dest="command", required=True)
    serve = sub.add_parser("serve", help="serve local capture controls and previews")
    _start_args(serve)
    serve.add_argument("--simulate", action="store_true")
    serve.add_argument("--port", type=int, default=8078)
    serve.add_argument("--dataset-root", type=Path, help="separate derived dataset directory")
    demo = sub.add_parser("demo", help="capture a short hardware-free episode")
    _start_args(demo)
    demo.set_defaults(simulate=True)
    demo.add_argument("--seconds", type=float, default=2.0)
    check = sub.add_parser("check", help="read back and check a complete raw episode")
    check.add_argument("episode", type=Path)
    replay = sub.add_parser("replay", help="print aligned records as JSON lines; no hardware")
    replay.add_argument("episode", type=Path)
    export = sub.add_parser("export", help="export one raw episode to a new LeRobot dataset")
    export.add_argument("episode", type=Path)
    export.add_argument("output", type=Path)
    export.add_argument("--fps", type=int, default=30)
    bundle = sub.add_parser("_export-bundle")
    bundle.add_argument("output", type=Path)
    bundle.add_argument("--min-free-bytes", type=int, default=0)
    bundle.add_argument("episodes", nargs="+", type=Path)
    return root


def run(args) -> dict | None:
    if args.command == "serve":
        with _launch(args) as collector, ExitStack() as stack:
            datasets = DatasetService(
                collector, args.dataset_root or collector.root.parent / "lerobot"
            )
            stack.callback(datasets.close)
            server = stack.enter_context(OperatorServer(args.port, collector, datasets))
            print(f"http://127.0.0.1:{server.server_port}", flush=True)
            server.serve_forever(poll_interval=0.2)
        return None
    if args.command == "demo":
        if not math.isfinite(args.seconds) or not 0.2 <= args.seconds < args.max_seconds:
            raise ValueError("demo seconds must be at least 0.2 and below --max-seconds")
        with _launch(args) as collector:
            deadline = time.monotonic() + 5
            while not collector.status()["ready"]:
                if time.monotonic() >= deadline:
                    raise CaptureError(f"simulation not ready: {collector.status()}")
                time.sleep(0.01)
            collector.start("simulated pick and place")
            time.sleep(args.seconds)
            return collector.finish("save")
    if args.command == "export":
        from .export import export_episode

        return export_episode(args.episode, args.output, fps=args.fps)
    if args.command == "_export-bundle":
        os.nice(10)
        from .export import build_bundle

        return build_bundle(args.episodes, args.output, min_free_bytes=args.min_free_bytes)
    episode = read_episode(args.episode)
    if args.command == "check":
        return {
            "episode": episode.manifest["episode"],
            "simulated": episode.manifest["simulated"],
            "task": episode.manifest["task"],
            "report": episode.report,
        }
    directories = camera_directories(episode.manifest)
    for sample in episode.matched:
        row = {"state": sample.cycle.state, "action": sample.cycle.action, "seq": sample.cycle.seq}
        if sample.frames_by_camera is None:
            root = episode.root / directories["front"]
            row.update(
                timestamp_ns=sample.frame.timestamp_ns,
                rgb=str(root / sample.frame.rgb),
                depth=str(root / sample.frame.depth),
            )
        else:
            row.update(
                timestamp_ns=sample.target_ns,
                cameras={
                    role: {
                        "timestamp_ns": frame.timestamp_ns,
                        "rgb": str(episode.root / directories[role] / frame.rgb),
                        "depth": str(episode.root / directories[role] / frame.depth),
                    }
                    for role, frame in sample.frames_by_camera.items()
                },
            )
        print(json.dumps(row))
    return None


def _stop_on_signal(*_):
    raise KeyboardInterrupt


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    signal.signal(signal.SIGTERM, _stop_on_signal)
    try:
        result = run(args)
        if result is not None:
            print(json.dumps(result, ensure_ascii=False))
    except KeyboardInterrupt:
        return 130
    except (
        CaptureError,
        ValueError,
        OSError,
        RuntimeError,
        ImportError,
        KeyError,
        TypeError,
    ) as exc:
        print(f"franka-capture: {exc}", file=sys.stderr)
        return 1
    return 0
