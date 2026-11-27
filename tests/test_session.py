import sys
import time
from pathlib import Path
from subprocess import TimeoutExpired
from threading import Event
from types import SimpleNamespace

import pytest

from franka_capture import session
from franka_capture.follow_control import FOLLOW_SOCKET_ENV, FollowClient, FollowControl


def test_leader_selection_requires_one_matching_live_device(tmp_path):
    name = "usb-known-leader"
    devices = tmp_path / "by-id"
    devices.mkdir()
    (devices / name).symlink_to("/dev/null")
    (devices / "unknown").symlink_to("/dev/null")
    (devices / "usb-broken").symlink_to("/does/not/exist")
    config = f"PORT_CONFIG_MAP = {{'/dev/serial/by-id/{name}': config()}}"
    assert session.select_leader(config, devices) == devices / name
    with pytest.raises(session.SessionError, match="exactly one"):
        session.select_leader("PORT_CONFIG_MAP = {}", devices)
    (devices / "usb-second").symlink_to("/dev/null")
    config = config.replace("config()}", "config(), '/dev/serial/by-id/usb-second': config()}")
    with pytest.raises(session.SessionError, match="exactly one"):
        session.select_leader(config, devices)


class Child:
    def __init__(self, pid, *, stuck=False, exit_code=0):
        self.pid = pid
        self.returncode = None
        self.stuck = stuck
        self.exit_code = exit_code

    def poll(self):
        return None

    def wait(self, timeout=None):
        if self.stuck:
            raise TimeoutExpired(["fake"], timeout)
        return self.exit_code


def test_shutdown_signals_only_session_groups_in_reverse_order(monkeypatch):
    events = []
    monkeypatch.setattr(session.os, "killpg", lambda pid, sig: events.append((pid, sig)))
    monkeypatch.setattr(session, "child_exit", lambda proc: None)
    children = [
        ("recorder", Child(33)),
        ("robot", Child(34, stuck=True)),
        ("follower", Child(35)),
    ]
    assert not session.stop_owned(children)
    assert events == [
        (35, session.signal.SIGTERM),
        (34, session.signal.SIGTERM),
        (33, session.signal.SIGTERM),
    ]


def test_recorder_readiness_failure_does_not_start_follow(monkeypatch, tmp_path):
    config = session.Config(
        tmp_path, tmp_path, tmp_path, Path("known"), 8078, (0.0,) * 7, tmp_path / "t.sock"
    )
    started = []
    stopped = []

    def spawn(children, name, *args, **kwargs):
        started.append(name)
        children.append((name, Child(35)))

    def failed(*args, **kwargs):
        raise session.SessionError("camera unavailable")

    monkeypatch.setattr(session, "acquire_system_permission", lambda: False)
    monkeypatch.setattr(session, "spawn", spawn)
    monkeypatch.setattr(session, "wait_ready", failed)
    monkeypatch.setattr(
        session, "stop_owned", lambda children, **_: stopped.extend(children) or True
    )
    with pytest.raises(session.SessionError, match="camera unavailable"):
        session.run(config, tmp_path)
    assert started == ["recorder"]
    assert [name for name, _ in stopped] == started


def test_failed_robot_feedback_never_runs_gripper_or_follow(monkeypatch, tmp_path):
    config = SimpleNamespace(repo=tmp_path, teleop=tmp_path, port=8078)
    started = []
    stopped = []
    finished = Event()

    def spawn(children, name, *args, **kwargs):
        started.append(name)
        children.append((name, Child(35)))

    def ready(children, name, *args, **kwargs):
        if name == "robot feedback":
            raise session.SessionError("no robot feedback")

    monkeypatch.setattr(session, "spawn", spawn)
    monkeypatch.setattr(session, "wait_ready", ready)
    monkeypatch.setattr(session, "robot_command", lambda *args: ["fake"])
    monkeypatch.setattr(session, "check_ports", lambda *args, **kwargs: None)
    monkeypatch.setattr(session, "authenticate", lambda *args: None)
    monkeypatch.setattr(
        session, "stop_owned", lambda children, **_: stopped.extend(children) or True
    )

    def workflow(attempt):
        try:
            session.run_follow(config, tmp_path, attempt)
        finally:
            finished.set()

    with FollowControl(workflow, permission_ready=lambda: True) as control:
        control.start(motion_confirmed=True)
        assert finished.wait(1)
    assert control.status()["phase"] == "error"
    assert control.status()["detail"] == "no robot feedback"
    assert started == ["robot"]
    assert [name for name, _ in stopped] == started


def test_follow_stage_is_opt_in_and_uses_root_ports(tmp_path):
    cfg = SimpleNamespace(
        repo=tmp_path,
        conda=Path("/some conda.sh"),
        polymetis_prefix=None,
        serial=Path("/dev/example"),
        teleop=tmp_path,
    )
    argv = session.robot_command(cfg, "follow")
    assert str(tmp_path / "teleop/experiments/run_env_capture.py") in argv
    assert "--teleop-port=/dev/example" in argv
    assert "--tele-port=6001" in argv
    assert "--agent=teleop" in argv
    assert not any("4" in arg and ".sh" in arg for arg in argv)


def test_preflight_does_not_start_programs(monkeypatch, tmp_path, capsys):
    cfg = session.Config(
        tmp_path, tmp_path, tmp_path, Path("known"), 8078, (0.0,) * 7, tmp_path / "telemetry.sock"
    )
    monkeypatch.setattr(session.sys, "argv", ["launcher", "--check"])
    monkeypatch.setattr(session, "config", lambda: cfg)
    monkeypatch.setattr(session, "check_ports", lambda port: None)

    def unexpected(*args, **kwargs):
        pytest.fail("read-only preflight started a program")

    monkeypatch.setattr(session.subprocess, "run", unexpected)
    monkeypatch.setattr(session, "run", unexpected)
    assert session.main() == 0
    assert "90 mm" in capsys.readouterr().out


def test_job_control_interrupt_does_not_suspend_supervisor(monkeypatch, tmp_path):
    cfg = session.Config(
        tmp_path, tmp_path, tmp_path, Path("known"), 8078, (0.0,) * 7, tmp_path / "telemetry.sock"
    )
    monkeypatch.setattr(session.sys, "argv", ["launcher"])
    monkeypatch.setattr(session, "config", lambda: cfg)
    monkeypatch.setattr(session, "check_ports", lambda _port: None)
    old_handler = session.signal.getsignal(session.signal.SIGTSTP)

    def interrupted(*_):
        assert (
            session.signal.getsignal(session.signal.SIGTSTP) is session.signal.default_int_handler
        )
        session.signal.raise_signal(session.signal.SIGTSTP)
        pytest.fail("terminal job control did not interrupt the session")

    monkeypatch.setattr(session, "run", interrupted)
    assert session.main() == 130
    assert session.signal.getsignal(session.signal.SIGTSTP) is old_handler


def test_full_sequence_waits_for_feedback_before_next_process(monkeypatch, tmp_path):
    cfg = session.Config(
        tmp_path, tmp_path, tmp_path, Path("known"), 8078, (0.0,) * 7, tmp_path / "telemetry.sock"
    )
    following = Event()
    (tmp_path / session.POLY / "python/polymetis").mkdir(parents=True)
    events = []
    spawn = session.spawn
    child_envs = {}

    def tracked_spawn(children, name, *args, extra_env=None):
        child_envs[name] = extra_env or {}
        spawn(children, name, *args, extra_env=extra_env)

    monkeypatch.setattr(session, "spawn", tracked_spawn)
    monkeypatch.setattr(session, "check_ports", lambda *args, **kwargs: None)
    monkeypatch.setattr(session, "authenticate", lambda *args: events.append("sudo"))
    monkeypatch.setattr(session, "check_children", lambda *args, **kwargs: following.set())

    def command(config, name):
        return [sys.executable, "-c", "import time; time.sleep(60)", name]

    def ready(children, name, *_args, **_kwargs):
        events.append((children[-1][0], name))

    monkeypatch.setattr(session, "robot_command", command)
    monkeypatch.setattr(session, "wait_ready", ready)
    with FollowControl(
        lambda attempt: session.run_follow(cfg, tmp_path, attempt), permission_ready=lambda: True
    ) as control:
        assert control.start(motion_confirmed=True)["phase"] == "starting"
        assert following.wait(1)
        assert control.status()["phase"] == "following"
        control.stop()
    assert control.status()["phase"] == "idle"
    assert child_envs["follow"] == {"FRANKA_CAPTURE_SOCKET": str(cfg.socket_path)}
    assert all(FOLLOW_SOCKET_ENV not in env for env in child_envs.values())
    assert events == [
        "sudo",
        ("robot", "robot feedback"),
        ("gripper", "gripper feedback"),
        ("node", "FR3 home and node"),
        ("follow", "fresh robot telemetry"),
    ]
    assert {file.name for file in tmp_path.glob("*.log")} == {
        "robot.log",
        "gripper.log",
        "node.log",
        "follow.log",
    }


def test_stop_during_real_readiness_cleans_robot_but_leaves_recorder(monkeypatch, tmp_path):
    cfg = session.Config(
        tmp_path, tmp_path, tmp_path, Path("known"), 8078, (0.0,) * 7, tmp_path / "telemetry.sock"
    )
    (tmp_path / session.POLY / "python/polymetis").mkdir(parents=True)
    logs = tmp_path / "logs"
    logs.mkdir()
    controller_probed = Event()
    processes = {}
    argv = [sys.executable, "-c", "import time; time.sleep(60)"]
    spawn = session.spawn
    client = None

    def synthetic_spawn(children, name, _argv, *args, extra_env=None):
        nonlocal client
        if name == "recorder":
            client = FollowClient(extra_env[FOLLOW_SOCKET_ENV])
        spawn(children, name, argv, *args, extra_env=extra_env)
        processes[name] = children[-1][1]

    def not_yet_ready(*_):
        controller_probed.set()
        return False

    def operate(_cfg, _logs, control, _recorder, _permission):
        assert client.status()["phase"] == "idle"
        assert set(processes) == {"recorder"}
        assert client.start(motion_confirmed=True)["phase"] == "starting"
        assert controller_probed.wait(2)
        assert client.stop()["phase"] in {"stopping", "idle"}
        deadline = time.monotonic() + 2
        while client.status()["phase"] == "stopping" and time.monotonic() < deadline:
            time.sleep(0.01)
        assert client.status()["phase"] == "idle"
        assert set(processes) == {"recorder", "robot"}
        assert processes["robot"].returncode == -session.signal.SIGTERM
        assert session.child_exit(processes["recorder"]) is None
        raise KeyboardInterrupt

    monkeypatch.setattr(session, "foreground_terminal", lambda: False)
    monkeypatch.setattr(session, "acquire_system_permission", lambda: True)
    monkeypatch.setattr(session, "spawn", synthetic_spawn)
    monkeypatch.setattr(session, "status", lambda _: {"camera_fresh": True})
    monkeypatch.setattr(session, "operator_loop", operate)
    monkeypatch.setattr(session, "check_ports", lambda *args, **kwargs: None)
    monkeypatch.setattr(session, "authenticate", lambda *args: None)
    monkeypatch.setattr(session, "robot_command", lambda *args: argv)
    monkeypatch.setattr(session, "probe", not_yet_ready)
    with pytest.raises(KeyboardInterrupt):
        session.run(cfg, logs)
    assert processes["recorder"].returncode == -session.signal.SIGTERM


def test_failed_readiness_reports_role_subphase_timeout_and_log(tmp_path):
    with pytest.raises(session.SessionError, match=r"robot.*feedback.*0 s.*robot[.]log"):
        session.wait_ready([], "robot feedback", 0, lambda: False, role="robot", logs=tmp_path)


def test_supervisor_detects_owned_cleanup_failure(monkeypatch, capsys):
    monkeypatch.setattr(session.os, "killpg", lambda pid, sig: None)
    monkeypatch.setattr(session, "child_exit", lambda proc: None)
    assert not session.stop_owned([("robot", Child(34, exit_code=74))])
    assert "74" in capsys.readouterr().err


def test_project_local_prefix_launch_ignores_legacy_conda_and_injected_paths(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", "/outside/conda/bin")
    monkeypatch.setenv("PYTHONPATH", "/outside/teleop")
    cfg = SimpleNamespace(
        repo=tmp_path,
        conda=Path("/missing/conda.sh"),
        polymetis_prefix=tmp_path / "local conda",
    )
    cmd = session.conda_command(cfg, "python", "-c", "pass")
    assert cmd[0] == "/usr/bin/env"
    assert ["-u", "PYTHONPATH"] == cmd[cmd.index("PYTHONPATH") - 1 : cmd.index("PYTHONPATH") + 1]
    assert "PYTHONNOUSERSITE=1" in cmd
    assert "MKL_INTERFACE_LAYER=LP64,GNU" in cmd
    assert f"CONDA_PREFIX={cfg.polymetis_prefix}" in cmd
    assert cmd[-3:] == [str(cfg.polymetis_prefix / "bin/python"), "-c", "pass"]
    assert not any("/outside" in arg or "/missing" in arg for arg in cmd)


def test_acquire_system_permission_prompts_at_launch_once(monkeypatch):
    checks = iter((False, True))
    commands = []
    monkeypatch.setattr(session, "sudo_ready", lambda: next(checks))
    monkeypatch.setattr(session, "foreground_terminal", lambda: True)
    monkeypatch.setattr(
        session.subprocess,
        "run",
        lambda cmd, **_: commands.append(cmd) or SimpleNamespace(returncode=0),
    )
    assert session.acquire_system_permission() is True
    assert commands == [["/usr/bin/sudo", "-v"]]


def test_noninteractive_permission_check_cannot_read_password(monkeypatch):
    calls = []
    monkeypatch.setattr(
        session.subprocess,
        "run",
        lambda cmd, **kwargs: calls.append((cmd, kwargs)) or SimpleNamespace(returncode=1),
    )
    assert session.sudo_ready() is False
    cmd, kwargs = calls[0]
    assert cmd == ["/usr/bin/sudo", "-n", "-v"]
    assert kwargs["stdin"] == session.subprocess.DEVNULL
    assert kwargs["timeout"] > 0
