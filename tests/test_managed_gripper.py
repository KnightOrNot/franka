import subprocess
from pathlib import Path
from threading import Event
from types import SimpleNamespace

from scripts import managed_gripper


class FakeServer:
    def __init__(self, calls):
        self.calls = calls

    def start(self):
        self.calls.append("server start")

    def stop(self, grace):
        self.calls.append("server stop")
        return SimpleNamespace(wait=lambda timeout: True)


class FakeHand:
    def __init__(self, calls, *, hung=False, returncode=None):
        self.calls = calls
        self.hung = hung
        self.returncode = returncode

    def poll(self):
        return self.returncode

    def terminate(self):
        self.calls.append("request hand stop")

    def wait(self, timeout):
        self.calls.append("wait for hand")
        if self.hung:
            raise subprocess.TimeoutExpired("fake", timeout)
        self.returncode = 0
        return self.returncode


def test_web_or_terminal_stop_drains_owned_gripper_before_closing_grpc():
    calls = []
    server = FakeServer(calls)
    hand = FakeHand(calls)
    stopping = Event()

    def launch():
        calls.append("spawn hand")
        stopping.set()
        return hand

    assert (
        managed_gripper.run_hand(server, lambda: calls.append("inhibit goals"), launch, stopping)
        == 0
    )
    assert calls == [
        "server start",
        "spawn hand",
        "inhibit goals",
        "request hand stop",
        "wait for hand",
        "server stop",
    ]


def test_gripper_failure_is_not_hidden_by_a_grpc_server(capsys):
    calls = []
    hand = FakeHand(calls, returncode=1)
    assert (
        managed_gripper.run_hand(
            FakeServer(calls), lambda: calls.append("inhibit goals"), lambda: hand, Event()
        )
        != 0
    )
    assert "exited (1)" in capsys.readouterr().err
    assert calls == ["server start", "inhibit goals", "server stop"]


def test_gripper_deadline_is_not_reported_as_clean_or_force_killed(capsys):
    calls = []
    stopping = Event()

    def launch():
        stopping.set()
        return FakeHand(calls, hung=True)

    assert (
        managed_gripper.run_hand(
            FakeServer(calls), lambda: calls.append("inhibit goals"), launch, stopping
        )
        == 74
    )
    assert "did not confirm" in capsys.readouterr().err


def test_lock_does_not_release_while_only_native_child_holds_it(tmp_path):
    lock = Path(tmp_path) / "hand.lock"
    import sys

    with managed_gripper.exclusive_lease(lock) as fd:
        child = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(60)"], pass_fds=(fd,)
        )
    try:
        try:
            with managed_gripper.exclusive_lease(lock):
                raise AssertionError("native connection lease was lost")
        except BlockingIOError:
            pass
    finally:
        child.terminate()
        child.wait(timeout=3)
    with managed_gripper.exclusive_lease(lock):
        assert lock.is_file()
