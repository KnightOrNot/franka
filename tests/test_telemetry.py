import json
import socket
import time

import pytest

from franka_capture.bridge import Sender
from franka_capture.telemetry import Cycle, open_receiver


def test_real_datagram_preserves_feedback_and_action(tmp_path):
    path = tmp_path / "cy.sock"
    with open_receiver(path) as receiver, Sender(str(path)) as sender:
        now = time.monotonic_ns()
        sender.publish([0] * 7 + [0.2], [1] * 7 + [0.8], now, now + 1)
        event = Cycle.decode(receiver.recv(4096))
    assert event.seq == 0
    assert event.state == pytest.approx((0,) * 7 + (0.018,))
    assert event.action == pytest.approx((1,) * 7 + (0.018,))
    assert event.requested[-1] == 0.8
    assert event.observed_ns < event.command_ns


def test_sender_recovers_and_sequence_discloses_loss(tmp_path):
    path = tmp_path / "cy.sock"
    with Sender(str(path)) as sender:
        now = time.monotonic_ns()
        assert not sender.publish([0] * 8, [0] * 8, now, now)
        with open_receiver(path) as receiver:
            assert sender.publish([0] * 8, [0] * 8, now, now)
            assert Cycle.decode(receiver.recv(4096)).seq == 1
    assert not path.exists()


def test_invalid_packets_are_rejected(tmp_path):
    path = tmp_path / "cy.sock"
    with open_receiver(path) as receiver, Sender(str(path)) as sender:
        now = time.monotonic_ns()
        sender.publish([0] * 8, [0] * 8, now, now)
        data = json.loads(receiver.recv(4096))
        data["observed_ns"] = now + 1
        with pytest.raises(ValueError):
            Cycle.decode(json.dumps(data).encode())
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as old_socket:
        old_socket.bind(str(path))
        with pytest.raises(OSError), open_receiver(path):
            pass
        assert path.exists()


def test_full_socket_remains_nonblocking_and_leaves_a_gap(tmp_path):
    path = tmp_path / "cy.sock"
    with open_receiver(path) as receiver, Sender(str(path)) as sender:
        t = time.monotonic_ns()
        sender.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)
        outcomes = [sender.publish([0] * 8, [0] * 8, t, t) for _ in range(128)]
        missed = outcomes.count(False)
        assert 0 < missed < len(outcomes)
        packets = [Cycle.decode(receiver.recv(4096)) for _ in range(outcomes.count(True))]
        assert [event.seq for event in packets] == list(range(len(packets)))
        assert sender.publish([0] * 8, [0] * 8, t, t)
        assert Cycle.decode(receiver.recv(4096)).seq == len(outcomes)
