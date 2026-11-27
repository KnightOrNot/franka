"""Run the verified root Polymetis/FR3 workflow beside the read-only recorder."""

import argparse
import ast
import fcntl
import hashlib
import http.client
import json
import math
import os
import re
import select
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import termios
import time
from collections.abc import Callable
from dataclasses import dataclass
from functools import partial
from pathlib import Path

from .follow_control import (
    FOLLOW_SOCKET_ENV,
    FollowAttempt,
    FollowCleanupError,
    FollowControl,
    open_control_socket,
)
from .telemetry import socket_default

LOOPBACK = "127.0.0.1"
SESSION_INTERRUPTS = (
    signal.SIGINT,
    signal.SIGTERM,
    signal.SIGHUP,
    signal.SIGTSTP,
    signal.SIGTTIN,
    signal.SIGTTOU,
)
ROBOT_PORT, GRIPPER_PORT, NODE_PORT = 50051, 50052, 6001
POLY = Path("polymetis/polymetis")
ROBOT = POLY / "python/scripts/launch_robot_capture.py"
GRIPPER = POLY / "python/scripts/launch_gripper.py"
NODE = Path("teleop/experiments/launch_nodes.py")
FOLLOW = Path("teleop/experiments/run_env_capture.py")
FR3 = Path("teleop/teleop/robots/fr3.py")
MAP = Path("teleop/teleop/agents/teleop_agent.py")
MODEL = POLY / "conf/robot_model/franka_panda.yaml"
CLOSURE_FLAG = "FRANKA_CAPTURE_FR3_CLOSURE_STARTUP"
SHUTDOWN_FLAG = "FRANKA_CAPTURE_OWNED_ROBOT_SHUTDOWN"
VERIFIED_SOURCES = {
    NODE: "219e5ba91d2af9737082a493bc5a781ce0c135e950d42b0cc9cae11dbe1171ac",
    FR3: "ab82f36e1b2cf971a7e73bc89be14d23e17492eeb97bad5aafe5cf8826b3c97b",
    Path(
        "teleop/teleop/env.py"
    ): "733b49a7519850e55bef380381528dce188453ba069b8e9681563e7ee6d8c8ea",
}

# Run inside the site's Polymetis environment. These RPCs only read feedback.
FEEDBACK_PROBE = """
import math
import sys
import grpc
from polymetis_pb2 import Empty
from polymetis_pb2_grpc import GripperServerStub, PolymetisControllerServerStub
with grpc.insecure_channel(sys.argv[1]) as channel:
    if sys.argv[2] == 'robot':
        stub = PolymetisControllerServerStub(channel)
        meta = stub.GetRobotClientMetadata(Empty(), timeout=0.7)
        state = stub.GetRobotState(Empty(), timeout=0.7)
        valid = (meta.dof == 7 and len(state.joint_positions) == 7
                 and all(math.isfinite(q) for q in state.joint_positions))
    else:
        stub = GripperServerStub(channel)
        stub.GetRobotClientMetadata(Empty(), timeout=0.7)
        state = stub.GetState(Empty(), timeout=0.7)
        valid = math.isfinite(state.width) and 0 <= state.width <= 0.1
    sys.exit(0 if valid and state.timestamp.seconds > 0 and state.error_code == 0 else 1)
"""


class SessionError(RuntimeError):
    pass


@dataclass(frozen=True)
class Config:
    repo: Path
    teleop: Path
    conda: Path | None
    serial: Path
    port: int
    home: tuple[float, ...]
    socket_path: Path
    polymetis_prefix: Path | None = None


def select_leader(source: str, devices: Path) -> Path:
    prefix = "/dev/serial/by-id/"
    known = set()
    for entry in ast.parse(source).body:
        if isinstance(entry, ast.Assign):
            targets = entry.targets
        elif isinstance(entry, ast.AnnAssign):
            targets = [entry.target]
        else:
            continue
        if isinstance(entry.value, ast.Dict) and any(
            isinstance(target, ast.Name) and target.id == "PORT_CONFIG_MAP" for target in targets
        ):
            known.update(
                key.value
                for key in entry.value.keys
                if isinstance(key, ast.Constant)
                and isinstance(key.value, str)
                and key.value.startswith(prefix)
            )
    matches = [path for path in devices.glob("*") if prefix + path.name in known and path.exists()]
    if len(matches) != 1:
        raise SessionError("expected exactly one connected leader from PORT_CONFIG_MAP")
    return matches[0]


def config() -> Config:
    repo = Path.cwd()
    teleop = Path(os.environ.get("FRANKA_TELEOP_ROOT") or Path.home() / "frankateleop").resolve()
    prefix_env = os.environ.get("FRANKA_POLYMETIS_PREFIX")
    prefix = Path(prefix_env).expanduser().resolve() if prefix_env else None
    conda = None
    if prefix:
        if not os.access(prefix / "bin/python", os.X_OK):
            raise SessionError(f"project Polymetis interpreter is missing: {prefix}")
    else:
        conda = Path(
            os.environ.get("FRANKA_CONDA_SH") or Path.home() / "miniconda3/etc/profile.d/conda.sh"
        ).resolve()
        if not conda.is_file():
            raise SessionError("conda.sh is missing")
    if not (teleop / GRIPPER).is_file():
        raise SessionError(f"root Polymetis checkout is missing: {teleop}")
    for path, digest in VERIFIED_SOURCES.items():
        if hashlib.sha256((teleop / path).read_bytes()).hexdigest() != digest:
            raise SessionError(f"unverified root FR3 controller source: {path}")
    for path, marker in ((ROBOT, SHUTDOWN_FLAG), (FOLLOW, CLOSURE_FLAG)):
        if marker not in (teleop / path).read_text():
            raise SessionError(
                f"required session patch is missing in {path}; see docs/deployment.md"
            )
    bridge = teleop / "teleop/experiments/franka_capture_session_bridge.py"
    if "def fr3_feedback_in_action_coordinates(" not in bridge.read_text():
        raise SessionError("FR3 startup bridge helper is missing")
    home_line = re.search(r"^rest_pose:\s*(\[[^\n]+\])\s*$", (teleop / MODEL).read_text(), re.M)
    home = ast.literal_eval(home_line[1]) if home_line else []
    if len(home) != 7 or not all(type(x) in (int, float) and math.isfinite(x) for x in home):
        raise SessionError("cannot verify seven-joint FR3 home target")
    port = int(os.environ.get("FRANKA_CAPTURE_HTTP_PORT") or 8078)
    if not 1 <= port <= 65535 or port in {50051, 50052, 50053, 50054, 6001, 6002}:
        raise SessionError("invalid or conflicting capture HTTP port")
    serial = select_leader((teleop / MAP).read_text(), Path("/dev/serial/by-id"))
    endpoint = os.environ.get("FRANKA_CAPTURE_SOCKET")
    socket_path = (Path(endpoint) if endpoint else socket_default()).expanduser().absolute()
    if len(os.fsencode(socket_path)) >= 108:
        raise SessionError("telemetry socket path is too long")
    return Config(repo, teleop, conda, serial, port, tuple(home), socket_path, prefix)


def check_ports(port: int, *, include_capture: bool = True) -> None:
    busy = []
    ports = (ROBOT_PORT, GRIPPER_PORT, 50053, 50054, NODE_PORT, 6002)
    for number in (*ports, port) if include_capture else ports:
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind((LOOPBACK, number))
            except OSError as exc:
                busy.append(f"{number} ({exc.strerror})")
    if busy:
        raise SessionError(f"occupied port(s): {', '.join(busy)}; stop existing sessions normally")


def conda_command(cfg: Config, *argv: str) -> list[str]:
    if cfg.polymetis_prefix is not None:
        if argv[0] != "python":
            raise ValueError("local Polymetis commands must use its Python interpreter")
        prefix = cfg.polymetis_prefix
        system_path = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
        return [
            "/usr/bin/env",
            "-u",
            "PYTHONPATH",
            "-u",
            "PYTHONHOME",
            "-u",
            "LD_LIBRARY_PATH",
            "-u",
            "LD_PRELOAD",
            "-u",
            "LD_AUDIT",
            "PYTHONNOUSERSITE=1",
            "MKL_INTERFACE_LAYER=LP64,GNU",
            f"CONDA_PREFIX={prefix}",
            f"PATH={prefix / 'bin'}:{system_path}",
            str(prefix / "bin/python"),
            *argv[1:],
        ]
    return [
        "bash",
        "-c",
        'source "$1" && conda activate polymetis || exit 1; shift; exec "$@"',
        "franka-conda",
        str(cfg.conda),
        *argv,
    ]


def robot_command(cfg: Config, stage: str) -> list[str]:
    if stage == "robot":
        args = [
            str(cfg.teleop / ROBOT),
            "robot_client=franka_hardware",
            "robot_model=franka_panda",
            f"ip={LOOPBACK}",
            f"port={ROBOT_PORT}",
        ]
    elif stage == "gripper":
        args = [
            str(cfg.teleop / GRIPPER),
            "gripper=franka_hand",
            f"ip={LOOPBACK}",
            f"port={GRIPPER_PORT}",
        ]
    elif stage == "node":
        args = [
            str(cfg.teleop / NODE),
            "--robot=fr3",
            f"--robot-ip={LOOPBACK}",
            f"--tele-port={NODE_PORT}",
            f"--robot-port={ROBOT_PORT}",
            f"--gripper-port={GRIPPER_PORT}",
        ]
    elif stage == "follow":
        args = [
            str(cfg.teleop / FOLLOW),
            "--agent=teleop",
            f"--hostname={LOOPBACK}",
            f"--tele-port={NODE_PORT}",
            f"--teleop-port={cfg.serial}",
        ]
    else:
        raise ValueError(f"unknown session stage: {stage}")
    return conda_command(cfg, "python", *args)


Children = list[tuple[str, subprocess.Popen]]


def spawn(
    children: Children,
    name: str,
    argv: list[str],
    logs: Path,
    cwd: Path,
    *,
    extra_env: dict[str, str] | None = None,
) -> None:
    flags = dict(os.environ)
    for key in (CLOSURE_FLAG, SHUTDOWN_FLAG, FOLLOW_SOCKET_ENV):
        flags.pop(key, None)
    flags.update(extra_env or {})
    flags["PYTHONUNBUFFERED"] = "1"
    if name == "follow":
        flags[CLOSURE_FLAG] = "1"
    if name == "robot":
        flags[SHUTDOWN_FLAG] = "1"
    log = logs / f"{name}.log"
    print(f"{name}: starting; log {log}", flush=True)
    try:
        with log.open("xb") as out:
            proc = subprocess.Popen(
                argv,
                cwd=cwd,
                env=flags,
                stdin=subprocess.DEVNULL,
                stdout=out,
                stderr=subprocess.STDOUT,
                process_group=0,
            )
        children.append((name, proc))
    except OSError as exc:
        raise SessionError(f"{name}: launch failed: {exc}; log {log}") from exc


def stop_owned(children: Children, *, logs: Path | None = None) -> bool:
    success = True
    for name, proc in reversed(children):
        where = f"; log {logs / (name + '.log')}" if logs else ""
        if proc.returncode is not None:
            # Already reaped: the PID is no longer safe to use as a group identifier.
            continue
        initial_exit = child_exit(proc)
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except OSError as exc:
            success = False
            print(
                f"{name}: could not signal own process group {proc.pid}: {exc}{where}",
                file=sys.stderr,
                flush=True,
            )
        try:
            rc = proc.wait(timeout=10)
            confirmed_stop = rc in (0, 130, -signal.SIGTERM, 128 + signal.SIGTERM)
            if not confirmed_stop and (initial_exit is None or rc == 74):
                success = False
                print(
                    f"{name}: shutdown exited with status {rc}{where}", file=sys.stderr, flush=True
                )
            elif confirmed_stop:
                print(f"{name}: stopped (exit {rc}){where}", flush=True)
        except subprocess.TimeoutExpired:
            success = False
            print(
                f"{name}: did not stop in 10 s (process group {proc.pid}){where}",
                file=sys.stderr,
                flush=True,
            )
    return success


def child_exit(proc: subprocess.Popen) -> int | None:
    # Observe without reaping the leader. Cleanup must retain its group identity.
    result = os.waitid(os.P_PID, proc.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
    if result is None:
        return None
    return result.si_status if result.si_code == os.CLD_EXITED else -result.si_status


def check_children(children: Children, *, logs: Path, subphase: str) -> None:
    for name, proc in children:
        if (rc := child_exit(proc)) is not None:
            raise SessionError(
                f"{name}: exited ({rc}) during {subphase}; log {logs / (name + '.log')}"
            )


def wait_ready(
    children: Children,
    name: str,
    seconds: float,
    ready: Callable[[], bool],
    *,
    role: str,
    logs: Path,
    attempt: FollowAttempt | None = None,
) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if attempt:
            attempt.check()
        check_children(children, logs=logs, subphase=name)
        if ready():
            if attempt:
                attempt.check()
            check_children(children, logs=logs, subphase=name)
            print(f"{role}: ready ({name}); log {logs / (role + '.log')}", flush=True)
            return
        if attempt:
            attempt.wait(0.2)
        else:
            time.sleep(0.2)
    raise SessionError(
        f"{role}: {name} timed out after {seconds:g} s; log {logs / (role + '.log')}"
    )


def tcp_ready(port: int) -> bool:
    try:
        with socket.create_connection((LOOPBACK, port), timeout=0.2):
            return True
    except OSError:
        return False


def status(port: int) -> dict:
    conn = http.client.HTTPConnection(LOOPBACK, port, timeout=0.5)
    try:
        conn.request("GET", "/api/status")
        response = conn.getresponse()
        body = json.loads(response.read(100_000)) if response.status == 200 else {}
        return body if isinstance(body, dict) else {}
    except (OSError, ValueError, http.client.HTTPException):
        return {}
    finally:
        conn.close()


def wait_auxiliary(proc: subprocess.Popen, seconds: float, attempt: FollowAttempt) -> int | None:
    """Reap a one-off probe or terminal authentication, including on browser cancellation."""
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            attempt.check()
            if (rc := proc.poll()) is not None:
                return rc
            attempt.wait(0.05)
        return None
    finally:
        if proc.returncode is None:
            proc.terminate()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1)


def probe(cfg: Config, kind: str, port: int, attempt: FollowAttempt) -> bool:
    if not tcp_ready(port):
        return False
    proc = attempt.launch(
        lambda: subprocess.Popen(
            conda_command(cfg, "python", "-c", FEEDBACK_PROBE, f"{LOOPBACK}:{port}", kind),
            cwd=cfg.teleop,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ),
    )
    return wait_auxiliary(proc, 3, attempt) == 0


def foreground_terminal() -> bool:
    try:
        return (
            sys.stdin.isatty()
            and sys.stdout.isatty()
            and os.tcgetpgrp(sys.stdin.fileno()) == os.getpgrp()
        )
    except (OSError, ValueError):
        return False


def authenticate(logs: Path, attempt: FollowAttempt) -> None:
    log = logs / "sudo.log"
    attempt.report(
        "starting",
        "Waiting for local sudo authentication (120 s timeout).",
        stage="sudo",
        log_path=log,
    )
    print(f"sudo: authenticate at this terminal if prompted; log {log}", flush=True)
    if not foreground_terminal():
        raise SessionError(f"sudo: the session terminal is no longer in the foreground; log {log}")
    try:
        with log.open("x") as output:
            output.write("Authenticating in the operator terminal.\n")
            output.flush()
            # Keep sudo's actual stdin and output attached so its prompt remains visible.
            # It stays in the foreground group. Cancellation signals only this sudo PID.
            proc = attempt.launch(lambda: subprocess.Popen(["sudo", "-v"]))
            rc = wait_auxiliary(proc, 120, attempt)
            print(f"Authentication exit status: {rc if rc is not None else 'timeout'}", file=output)
    except OSError as exc:
        raise SessionError(f"sudo: local authentication failed: {exc}; log {log}") from exc
    if rc is None:
        raise SessionError(f"sudo: local authentication timed out after 120 s; log {log}")
    if rc != 0:
        raise SessionError(f"sudo: local authentication exited ({rc}); log {log}")


def fresh_telemetry(port: int, since: float) -> bool:
    state = status(port)
    age = state.get("robot_age_ms")
    return (
        state.get("robot_fresh") is True
        and type(age) in (int, float)
        and 0 <= age < (time.monotonic() - since) * 1000
    )


def run_follow(cfg: Config, logs: Path, attempt: FollowAttempt) -> None:
    children: Children = []
    cwd = cfg.teleop / POLY / "python/polymetis"
    follow_spawn_time = 0.0
    stages = (
        ("robot", "robot feedback", 35, cwd, lambda: probe(cfg, "robot", ROBOT_PORT, attempt)),
        (
            "gripper",
            "gripper feedback",
            35,
            cwd,
            lambda: probe(cfg, "gripper", GRIPPER_PORT, attempt),
        ),
        ("node", "FR3 home and node", 90, cfg.teleop, lambda: tcp_ready(NODE_PORT)),
        (
            "follow",
            "fresh robot telemetry",
            35,
            cfg.teleop,
            lambda: fresh_telemetry(cfg.port, follow_spawn_time),
        ),
    )
    try:
        check_ports(cfg.port, include_capture=False)
        authenticate(logs, attempt)
        # Authentication may take time. Check again before starting any session-owned driver.
        check_ports(cfg.port, include_capture=False)
        for role, name, seconds, directory, ready in stages:
            log = logs / f"{role}.log"
            attempt.report(
                "starting", f"Waiting for {name} ({seconds} s timeout).", stage=role, log_path=log
            )
            if role == "follow":
                follow_spawn_time = time.monotonic()
            argv = robot_command(cfg, role)
            env = {"FRANKA_CAPTURE_SOCKET": str(cfg.socket_path)} if role == "follow" else None
            attempt.launch(partial(spawn, children, role, argv, logs, directory, extra_env=env))
            wait_ready(children, name, seconds, ready, role=role, logs=logs, attempt=attempt)
        attempt.report(
            "following",
            "Follow is running. Recording is controlled on the web page.",
            stage="follow",
            log_path=logs / "follow.log",
        )
        while True:
            attempt.check()
            check_children(children, logs=logs, subphase="following")
            attempt.wait(0.2)
    finally:
        failure = sys.exception()
        if children:
            attempt.report("stopping", "Stopping the session-owned follow chain.")
            if not stop_owned(children, logs=logs):
                raise FollowCleanupError(
                    f"An owned follow process did not stop cleanly. Do not restart follow until "
                    f"the operator has checked the controller. See logs in {logs}."
                    + (f" Original failure: {failure}" if failure and str(failure) else "")
                )


def show_target(cfg: Config) -> None:
    angles = ", ".join(f"{math.degrees(q):.1f}°" for q in cfg.home)
    print(f"Starting follow moves the FR3 to home [{angles}] and opens the gripper (90 mm).")


def operator_loop(cfg: Config, logs: Path, control: FollowControl, recorder: Children) -> None:
    previous_prompt = None
    previous_state = None
    while True:
        check_children(recorder, logs=logs, subphase="capture preview")
        state = control.status()
        if state != previous_state and state["phase"] != "awaiting_confirmation":
            log = f" Log: {state['log_path']}" if "log_path" in state else ""
            print(f"Follow {state['phase']}: {state['detail']}{log}", flush=True)
        previous_state = state
        challenge = control.prompt()
        if not foreground_terminal():
            if state["phase"] in {"awaiting_confirmation", "starting", "following"}:
                control.stop()
            time.sleep(0.1)
        elif challenge:
            fd = sys.stdin.fileno()
            if challenge != previous_prompt:
                termios.tcflush(fd, termios.TCIFLUSH)
                show_target(cfg)
                print(
                    f"Check the arm workspace, emergency stop and leader alignment. "
                    f"Type '{challenge}' to start or 'no' to cancel.",
                    flush=True,
                )
            readable, _, _ = select.select([fd], [], [], 0.1)
            if readable:
                line = os.read(fd, 4096)
                if not line:
                    control.stop()
                elif (
                    not control.confirm(line.decode(errors="replace").strip()) and control.prompt()
                ):
                    print(
                        "Confirmation not accepted. Use the current terminal code or 'no'.",
                        flush=True,
                    )
        else:
            time.sleep(0.1)
        previous_prompt = challenge


def run(cfg: Config, logs: Path) -> None:
    recorder: Children = []

    def workflow(attempt: FollowAttempt) -> None:
        follow_logs = Path(tempfile.mkdtemp(prefix="follow-", dir=logs))
        run_follow(cfg, follow_logs, attempt)

    control = FollowControl(workflow, terminal_ready=foreground_terminal)
    try:
        with open_control_socket(cfg.socket_path.parent, control) as endpoint:
            spawn(
                recorder,
                "recorder",
                [str(cfg.repo / "scripts/start-capture.sh")],
                logs,
                cfg.repo,
                extra_env={
                    FOLLOW_SOCKET_ENV: str(endpoint),
                    "FRANKA_CAPTURE_SOCKET": str(cfg.socket_path),
                },
            )
            wait_ready(
                recorder,
                "camera preview",
                40,
                lambda: status(cfg.port).get("camera_fresh"),
                role="recorder",
                logs=logs,
            )
            print(
                f"Capture page: http://{LOOPBACK}:{cfg.port}. No robot motion has started.",
                flush=True,
            )
            operator_loop(cfg, logs, control, recorder)
    finally:
        # Do not let a second Ctrl+C or disconnect interrupt reverse-order cleanup.
        handlers = {sig: signal.signal(sig, signal.SIG_IGN) for sig in SESSION_INTERRUPTS}
        try:
            try:
                control.close()
            finally:
                if not stop_owned(recorder, logs=logs):
                    raise SessionError(
                        f"recorder: did not stop normally; log {logs / 'recorder.log'}"
                    )
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


def show(cfg: Config) -> None:
    print(f"FR3 leader: {cfg.serial.name}.")
    show_target(cfg)
    print(f"Capture page: http://{LOOPBACK}:{cfg.port}.", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="read-only preflight without motion")
    args = parser.parse_args()
    try:
        cfg = config()
        show(cfg)
        check_ports(cfg.port)
        if args.check:
            print("Configuration and port checks passed. No hardware opened or services started.")
            return 0
        log_root = cfg.repo / ".run"
        log_root.mkdir(mode=0o700, exist_ok=True)
        if not stat.S_ISDIR(log_root.lstat().st_mode):
            raise SessionError("invalid session log directory")
        fd = os.open(log_root / "session.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise SessionError("another one-command session is starting") from exc
            check_ports(cfg.port)
            logs = Path(tempfile.mkdtemp(prefix="session-", dir=log_root))
            # Disconnect and job control must unwind the separately grouped children too.
            handlers = {
                sig: signal.signal(sig, signal.default_int_handler) for sig in SESSION_INTERRUPTS
            }
            try:
                run(cfg, logs)
            finally:
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)
        finally:
            os.close(fd)
    except KeyboardInterrupt:
        return 130
    except (EOFError, OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        print(f"Session failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
