from pathlib import Path
from subprocess import TimeoutExpired
from types import SimpleNamespace

import pytest

from franka_capture import session


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


def test_failure_before_first_hardware_stage_stops_only_recorder(monkeypatch, tmp_path):
    config = SimpleNamespace(repo=tmp_path, teleop=tmp_path, port=8078)
    started = []
    stopped = []

    def spawn(children, name, *args, **kwargs):
        started.append(name)
        children.append((name, Child(35)))

    def failed(*args, **kwargs):
        raise session.SessionError("camera unavailable")

    monkeypatch.setattr(session, "spawn", spawn)
    monkeypatch.setattr(session, "wait_ready", failed)
    monkeypatch.setattr(session, "stop_owned", lambda children: stopped.extend(children) or True)
    with pytest.raises(session.SessionError, match="camera unavailable"):
        session.run(config, tmp_path)
    assert started == ["recorder"]
    assert [name for name, _ in stopped] == started


def test_failed_robot_feedback_never_runs_gripper_or_follow(monkeypatch, tmp_path):
    config = SimpleNamespace(repo=tmp_path, teleop=tmp_path, port=8078)
    started = []
    stopped = []

    def spawn(children, name, *args, **kwargs):
        started.append(name)
        children.append((name, Child(35)))

    def ready(children, name, *args, **kwargs):
        if name == "robot feedback":
            raise session.SessionError("no robot feedback")

    monkeypatch.setattr(session, "spawn", spawn)
    monkeypatch.setattr(session, "wait_ready", ready)
    monkeypatch.setattr(session, "robot_command", lambda *args: ["fake"])
    monkeypatch.setattr(session, "probe", lambda *args: None)
    monkeypatch.setattr(session, "stop_owned", lambda children: stopped.extend(children) or True)
    with pytest.raises(session.SessionError, match="no robot feedback"):
        session.run(config, tmp_path)
    assert started == ["recorder", "robot"]
    assert [name for name, _ in stopped] == started


def test_follow_stage_is_opt_in_and_uses_root_ports(tmp_path):
    cfg = SimpleNamespace(
        conda=Path("/some conda.sh"), serial=Path("/dev/example"), teleop=tmp_path
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


def test_cancelling_confirmation_starts_no_children(monkeypatch, tmp_path):
    cfg = session.Config(
        tmp_path, tmp_path, tmp_path, Path("known"), 8078, (0.0,) * 7, tmp_path / "telemetry.sock"
    )
    monkeypatch.setattr(session.sys, "argv", ["launcher"])
    monkeypatch.setattr(session.sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr(session.sys.stdout, "isatty", lambda: True)
    monkeypatch.setattr(session, "config", lambda: cfg)
    monkeypatch.setattr(session, "check_ports", lambda port: None)
    monkeypatch.setattr("builtins.input", lambda prompt: "no")

    def unexpected(*args, **kwargs):
        pytest.fail("cancellation started a program")

    monkeypatch.setattr(session.subprocess, "run", unexpected)
    monkeypatch.setattr(session, "run", unexpected)
    assert session.main() == 0


def test_full_sequence_waits_for_feedback_before_next_process(monkeypatch, tmp_path):
    cfg = session.Config(
        tmp_path, tmp_path, tmp_path, Path("known"), 8078, (0.0,) * 7, tmp_path / "telemetry.sock"
    )
    events = []

    def spawn(children, name, *args):
        events.append(name)
        children.append((name, Child(len(children) + 1)))

    monkeypatch.setattr(session, "spawn", spawn)
    monkeypatch.setattr(
        session, "wait_ready", lambda children, name, seconds, ready: events.append(name)
    )
    monkeypatch.setattr(session, "stop_owned", lambda children: events.append("stopped") or True)

    def interrupt(children):
        raise KeyboardInterrupt

    monkeypatch.setattr(session, "check_children", interrupt)
    with pytest.raises(KeyboardInterrupt):
        session.run(cfg, tmp_path)
    assert events == [
        "recorder",
        "D435i frames",
        "robot",
        "robot feedback",
        "gripper",
        "gripper feedback",
        "node",
        "FR3 home and node",
        "follow",
        "capture-ready telemetry",
        "stopped",
    ]


def test_supervisor_detects_owned_cleanup_failure(monkeypatch, capsys):
    monkeypatch.setattr(session.os, "killpg", lambda pid, sig: None)
    assert not session.stop_owned([("robot", Child(34, exit_code=74))])
    assert "74" in capsys.readouterr().err
