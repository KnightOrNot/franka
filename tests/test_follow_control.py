import socket
import stat
from threading import Event

import pytest

from franka_capture.follow_control import FollowClient, FollowControl, open_control_socket


def test_standalone_client_cannot_start_or_stop_motion():
    client = FollowClient(None)
    assert client.status()["phase"] == "unmanaged"
    for action in (client.start, client.stop):
        with pytest.raises(RuntimeError, match="not managed"):
            action()


def test_web_start_only_queues_a_new_terminal_challenge(tmp_path):
    started = Event()

    def follow(attempt):
        started.set()
        attempt.cancelled.wait()

    with (
        FollowControl(follow, terminal_ready=lambda: True) as control,
        open_control_socket(tmp_path, control) as path,
    ):
        client = FollowClient(path)
        assert client.status()["phase"] == "idle"
        state = client.start()
        assert state["phase"] == "awaiting_confirmation"
        first = control.prompt()
        assert first and first not in str(state)
        assert not started.is_set()
        assert client.stop()["phase"] == "idle"
        assert not control.confirm(first)

        assert client.start()["phase"] == "awaiting_confirmation"
        second = control.prompt()
        assert second and second != first
        with pytest.raises(RuntimeError, match="already"):
            client.start()
        assert not control.confirm(first)
        assert not control.confirm("yes")
        assert not started.is_set()
        assert control.confirm(second)
        assert started.wait(1)


def test_stop_during_startup_cancels_without_a_second_confirmation(tmp_path):
    starting, stopped = Event(), Event()

    def follow(attempt):
        attempt.report("starting", "Waiting for node.", stage="node")
        starting.set()
        if attempt.cancelled.wait(1):
            stopped.set()

    with (
        FollowControl(follow, terminal_ready=lambda: True) as control,
        open_control_socket(tmp_path, control) as path,
    ):
        client = FollowClient(path)
        client.start()
        assert control.confirm(control.prompt())
        assert starting.wait(1)
        assert client.status()["stage"] == "node"
        assert client.stop()["phase"] in {"stopping", "idle"}
        assert stopped.wait(1)


def test_missing_foreground_terminal_cannot_queue_start(tmp_path):
    with (
        FollowControl(lambda _: pytest.fail("started"), terminal_ready=lambda: False) as control,
        open_control_socket(tmp_path, control) as path,
    ):
        client = FollowClient(path)
        with pytest.raises(RuntimeError, match="foreground terminal"):
            client.start()
        assert client.status()["phase"] == "idle"


def test_wire_endpoint_is_private_and_rejects_unsupported_requests(tmp_path):
    with (
        FollowControl(lambda _: pytest.fail("started"), terminal_ready=lambda: True) as control,
        open_control_socket(tmp_path, control) as path,
    ):
        assert stat.S_IMODE(path.parent.stat().st_mode) == 0o700
        assert stat.S_IMODE(path.stat().st_mode) == 0o600
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
            conn.settimeout(1)
            conn.connect(str(path))
            conn.sendall(b"start extra\n")
            assert b"invalid" in conn.recv(1024)
        assert FollowClient(path).status()["phase"] == "idle"
    assert not path.exists()
