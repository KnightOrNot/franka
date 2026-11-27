#!/usr/bin/env python3
"""Own the managed Franka Hand client and close its gRPC server after it exits."""

from __future__ import annotations

import argparse
import fcntl
import os
import signal
import stat
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from pathlib import Path
from threading import Event

HAND_EXIT_TIMEOUT = 14
UNCLEAN_EXIT = 74


@contextmanager
def exclusive_lease(path: Path):
    """The native child inherits the locked FD until its connection has closed."""
    parent = path.parent.stat()
    if parent.st_uid != os.getuid() or parent.st_mode & 0o077:
        raise PermissionError("The hand lease directory must be owner-private.")
    flags = os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC
    fd = os.open(path, flags, 0o600)
    try:
        identity = os.fstat(fd)
        if (
            identity.st_uid != os.getuid()
            or identity.st_mode & 0o077
            or not stat.S_ISREG(identity.st_mode)
        ):
            raise PermissionError("The hand lease must be an owner-private regular file.")
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Do not LOCK_UN explicitly: an inherited descriptor must retain the lease.
        yield fd
    finally:
        os.close(fd)


def run_hand(server, quiesce, launch, stopping) -> int:
    """Start server and child, block new goals on stop, await the exact child, then stop gRPC."""
    child = None
    started = False
    result = 0
    try:
        server.start()
        started = True
        if not stopping.is_set():
            child = launch()
            while not stopping.is_set():
                if (rc := child.poll()) is not None:
                    print(
                        f"Managed hand client exited ({rc}); no gripper feedback.", file=sys.stderr
                    )
                    result = 1
                    break
                time.sleep(0.1)
    except OSError as exc:
        print(f"Managed hand client could not start: {exc}", file=sys.stderr)
        result = 1
    finally:
        quiesce()
        if child is not None:
            try:
                rc = child.poll()
                if rc is None:
                    child.terminate()
                    print(
                        "Waiting for the owned hand client to release its connection.", flush=True
                    )
                    rc = child.wait(timeout=HAND_EXIT_TIMEOUT)
                if stopping.is_set() and rc != 0:
                    print(
                        f"Hand client did not confirm a clean exit (status {rc}).", file=sys.stderr
                    )
                    result = UNCLEAN_EXIT
                elif stopping.is_set() and rc == 0:
                    print("Hand client confirmed disconnect.", flush=True)
            except (OSError, subprocess.TimeoutExpired) as exc:
                # Never SIGKILL a client blocked in a hardware call. Its lease stays held.
                print(f"Hand client did not confirm disconnect: {exc}", file=sys.stderr)
                result = UNCLEAN_EXIT
        if started:
            if not server.stop(1).wait(timeout=2):
                print("The gripper gRPC server did not confirm shutdown.", file=sys.stderr)
                result = UNCLEAN_EXIT
            else:
                print("Gripper gRPC server stopped.", flush=True)
    return result


def create_server(port: int):
    import grpc
    import polymetis_pb2
    import polymetis_pb2_grpc
    from polymetis.robot_servers.gripper_server import PolymetisGripperServer

    quiet = Event()

    class ManagedService(PolymetisGripperServer):
        def GetRobotClientMetadata(self, request, context):
            if self.metadata is None:
                context.abort(grpc.StatusCode.UNAVAILABLE, "Waiting for Franka Hand initialization")
            return super().GetRobotClientMetadata(request, context)

        def GetState(self, request, context):
            if self.gripper_state_cache.timestamp.seconds <= 0:
                context.abort(grpc.StatusCode.UNAVAILABLE, "Waiting for Franka Hand state")
            return super().GetState(request, context)

        def Goto(self, request, context):
            if quiet.is_set():
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "Gripper is shutting down")
            return super().Goto(request, context)

        def ControlUpdate(self, request, context):
            if quiet.is_set():
                return polymetis_pb2.GripperCommand()  # No new goal after shutdown starts.
            return super().ControlUpdate(request, context)

    server = grpc.server(ThreadPoolExecutor(max_workers=2), options=(("grpc.so_reuseport", 0),))
    polymetis_pb2_grpc.add_GripperServerServicer_to_server(ManagedService(), server)
    if server.add_insecure_port(f"127.0.0.1:{port}") != port:
        raise OSError(f"Could not bind the gripper gRPC port {port}")
    return server, quiet.set


def hand_config(path: Path, port: int) -> dict:
    import yaml

    profile = yaml.safe_load(path.read_text("utf-8"))["gripper"]
    original = profile["executable_cfg"]
    ip = original["robot_ip"]
    if (
        profile["use_real_time"] is not False
        or original["exec"] != "franka_hand_client"
        or not isinstance(ip, str)
        or not ip.strip()
    ):
        raise ValueError("Expected the verified non-realtime Franka Hand profile")
    return {"robot_ip": ip, "control_ip": "127.0.0.1", "control_port": port}


def launch_with_config(args, config, lease, stopping):
    import yaml

    with tempfile.NamedTemporaryFile("w", prefix="franka-hand-", suffix=".yaml") as cfg:
        yaml.safe_dump(config, cfg)
        cfg.flush()
        server, quiesce = create_server(args.port)
        return run_hand(
            server,
            quiesce,
            lambda: subprocess.Popen(
                [args.binary, cfg.name], stdin=subprocess.DEVNULL, pass_fds=(lease,)
            ),
            stopping,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--lease", required=True, type=Path)
    parser.add_argument("--check", action="store_true", help="validate setup without hardware")
    args = parser.parse_args()

    class StopRequest:
        requested = False

        def is_set(self):
            return self.requested

    stopping = StopRequest()

    def on_stop(_signal, _frame):
        # Do not acquire a threading lock in a Python signal handler.
        stopping.requested = True

    signal.signal(signal.SIGTERM, on_stop)
    signal.signal(signal.SIGINT, on_stop)
    try:
        if not args.binary.is_file() or not os.access(args.binary, os.X_OK):
            raise FileNotFoundError(f"Managed hand client not installed: {args.binary}")
        if not 1 <= args.port <= 65535:
            raise ValueError("invalid gripper gRPC port")
        config = hand_config(args.config, args.port)
        if args.check or stopping.is_set():
            print("Managed hand preflight passed. No connection opened.")
            return 0
        with exclusive_lease(args.lease) as lease:
            return launch_with_config(args, config, lease, stopping)
    except BlockingIOError:
        print("A previous hand client still holds the connection lease.", file=sys.stderr)
        return 1
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        print(f"Managed hand failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
