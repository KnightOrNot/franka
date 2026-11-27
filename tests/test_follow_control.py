import socket
import stat
from threading import Event

import pytest

from franka_capture.follow_control import FollowClient, FollowControl, open_control_socket


def test_standalone_client_cannot_start_or_stop_motion():
    client = FollowClient(None)
    assert client.status()["phase"] == "unmanaged"
    for action in (lambda: client.start(motion_confirmed=True), client.stop):
        with pytest.raises(RuntimeError, match="not managed"):
            action()


def test_web_confirmation_starts_follow_without_terminal_input(tmp_path):
    started = Event()

    def follow(attempt):
        started.set()
        attempt.cancelled.wait()

    with (
        FollowControl(follow, permission_ready=lambda: True, home_degrees=(0.0,) * 7) as control,
        open_control_socket(tmp_path, control) as path,
    ):
        client = FollowClient(path)
        assert client.status()["phase"] == "idle"
        assert client.status()["home_degrees"] == [0.0] * 7
        assert client.status()["gripper_open_mm"] == 90
        assert client.status()["start_enabled"] is True
        for confirmation in (False, 1):
            with pytest.raises(RuntimeError, match="motion confirmation"):
                client.start(motion_confirmed=confirmation)
        assert not started.is_set()
        assert client.start(motion_confirmed=True)["phase"] == "starting"
        assert started.wait(1)
        assert client.status()["start_enabled"] is False
        with pytest.raises(RuntimeError, match="already"):
            client.start(motion_confirmed=True)
        assert client.stop()["phase"] in {"stopping", "idle"}


def test_stop_during_startup_cancels_without_a_second_confirmation(tmp_path):
    starting, stopped = Event(), Event()

    def follow(attempt):
        attempt.report("starting", "Waiting for node.", stage="node")
        starting.set()
        if attempt.cancelled.wait(1):
            stopped.set()

    with (
        FollowControl(follow, permission_ready=lambda: True) as control,
        open_control_socket(tmp_path, control) as path,
    ):
        client = FollowClient(path)
        client.start(motion_confirmed=True)
        assert starting.wait(1)
        assert client.status()["stage"] == "node"
        assert client.stop()["phase"] in {"stopping", "idle"}
        assert stopped.wait(1)


def test_missing_system_permission_cannot_start(tmp_path):
    with (
        FollowControl(lambda _: pytest.fail("started"), permission_ready=lambda: False) as control,
        open_control_socket(tmp_path, control) as path,
    ):
        client = FollowClient(path)
        with pytest.raises(RuntimeError, match="authorization"):
            client.start(motion_confirmed=True)
        state = client.status()
        assert state["phase"] == "idle" and state["start_enabled"] is False


def test_wire_endpoint_is_private_and_rejects_unsupported_requests(tmp_path):
    with (
        FollowControl(lambda _: pytest.fail("started"), permission_ready=lambda: True) as control,
        open_control_socket(tmp_path, control) as path,
    ):
        assert stat.S_IMODE(path.parent.stat().st_mode) == 0o700
        assert stat.S_IMODE(path.stat().st_mode) == 0o600
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
            conn.settimeout(1)
            conn.connect(str(path))
            conn.sendall(b"start\n")
            assert b"invalid" in conn.recv(1024)
        assert FollowClient(path).status()["phase"] == "idle"
    assert not path.exists()
