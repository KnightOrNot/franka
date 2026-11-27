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
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from .telemetry import socket_default

LOOPBACK = "127.0.0.1"
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


def check_ports(port: int) -> None:
    busy = []
    for number in (ROBOT_PORT, GRIPPER_PORT, 50053, 50054, NODE_PORT, 6002, port):
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


def spawn(children, name, argv, logs: Path, cwd: Path) -> None:
    flags = dict(os.environ)
    flags.pop(CLOSURE_FLAG, None)
    flags.pop(SHUTDOWN_FLAG, None)
    flags["PYTHONUNBUFFERED"] = "1"
    if name == "follow":
        flags[CLOSURE_FLAG] = "1"
    if name == "robot":
        flags[SHUTDOWN_FLAG] = "1"
    print(f"启动 {name}：{logs / (name + '.log')}", flush=True)
    with (logs / (name + ".log")).open("xb") as out:
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


def stop_owned(children) -> bool:
    success = True
    for name, proc in reversed(children):
        running = proc.returncode is None
        if not running and name == "recorder":
            continue
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            rc = proc.wait(timeout=10)
            if running and rc not in (0, 130, -signal.SIGTERM, 128 + signal.SIGTERM):
                success = False
                print(f"{name} 退出状态 {rc}，请查看日志。", file=sys.stderr)
        except subprocess.TimeoutExpired:
            success = False
            print(f"{name} 未在 10 秒内退出（进程组 {proc.pid}），请现场检查。", file=sys.stderr)
    return success


def check_children(children) -> None:
    for name, proc in children:
        rc = proc.poll()
        if rc is not None:
            raise SessionError(f"{name} exited ({rc}); see {name}.log")


def wait_ready(children, name: str, seconds: float, ready: Callable[[], bool]) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        check_children(children)
        if ready():
            check_children(children)
            return
        time.sleep(0.2)
    raise SessionError(f"timed out waiting for {name}")


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


def probe(cfg: Config, kind: str, port: int) -> bool:
    if not tcp_ready(port):
        return False
    try:
        return (
            subprocess.run(
                conda_command(cfg, "python", "-c", FEEDBACK_PROBE, f"{LOOPBACK}:{port}", kind),
                cwd=cfg.teleop,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
                check=False,
            ).returncode
            == 0
        )
    except subprocess.TimeoutExpired:
        return False


def run(cfg: Config, logs: Path) -> None:
    children = []
    try:
        spawn(children, "recorder", [str(cfg.repo / "scripts/start-capture.sh")], logs, cfg.repo)
        wait_ready(children, "D435i frames", 40, lambda: status(cfg.port).get("camera_fresh"))
        print(f"网页 http://{LOOPBACK}:{cfg.port}，正在准备跟随。", flush=True)
        spawn(
            children,
            "robot",
            robot_command(cfg, "robot"),
            logs,
            cfg.teleop / POLY / "python/polymetis",
        )
        wait_ready(children, "robot feedback", 35, lambda: probe(cfg, "robot", ROBOT_PORT))
        spawn(
            children,
            "gripper",
            robot_command(cfg, "gripper"),
            logs,
            cfg.teleop / POLY / "python/polymetis",
        )
        wait_ready(children, "gripper feedback", 35, lambda: probe(cfg, "gripper", GRIPPER_PORT))
        spawn(children, "node", robot_command(cfg, "node"), logs, cfg.teleop)
        wait_ready(children, "FR3 home and node", 90, lambda: tcp_ready(NODE_PORT))
        spawn(children, "follow", robot_command(cfg, "follow"), logs, cfg.teleop)
        wait_ready(children, "capture-ready telemetry", 35, lambda: status(cfg.port).get("ready"))
        print("遥测和相机已就绪。打开网页开始录制，结束时先保存，再 Ctrl+C。", flush=True)
        while True:
            check_children(children)
            time.sleep(0.5)
    finally:
        # Do not let a second Ctrl+C or disconnect interrupt reverse-order cleanup.
        signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
        handlers = {sig: signal.signal(sig, signal.SIG_IGN) for sig in signals}
        try:
            if not stop_owned(children):
                raise SessionError("a session process did not stop normally")
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


def show(cfg: Config) -> None:
    angles = ", ".join(f"{math.degrees(q):.1f}°" for q in cfg.home)
    print(f"根目录 FR3 接线配置。主手：{cfg.serial.name}。")
    print(f"第 3 阶段会把从臂移到配置的 home [{angles}] 并打开夹爪（目标 90 mm）。")
    print(f"数采网页为 http://{LOOPBACK}:{cfg.port}。", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="read-only preflight without motion")
    args = parser.parse_args()
    try:
        cfg = config()
        show(cfg)
        check_ports(cfg.port)
        if args.check:
            print("配置与端口检查通过，未打开硬件或启动服务。")
            return 0
        if not (sys.stdin.isatty() and sys.stdout.isatty()):
            raise SessionError("run from an interactive terminal with an on-site operator")
        if input("确认工作区无人和障碍、急停可用、主手已对齐，输入 yes 启动：").strip() != "yes":
            print("未启动任何服务。")
            return 0
        check_ports(cfg.port)
        # The existing real-time launch uses sudo. Authenticate only at this terminal.
        subprocess.run(["sudo", "-v"], check=True)
        check_ports(cfg.port)
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
            os.environ["FRANKA_CAPTURE_SOCKET"] = str(cfg.socket_path)
            # An SSH disconnect also has to unwind the separately grouped children.
            for sig in (signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, signal.default_int_handler)
            run(cfg, logs)
        finally:
            os.close(fd)
    except KeyboardInterrupt:
        return 130
    except (EOFError, OSError, SessionError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"未完成启动或运行：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
