import math

import numpy as np
import pytest

from franka_capture import bridge


class SimulatedAlignment:
    def __init__(self, leader, *, moves=True):
        self.leader = np.array(leader, dtype=float)
        self.position = np.zeros(8)
        self.target = self.position.copy()
        self.time = 0.0
        self.moves = moves
        self.commands = []
        self.messages = []

    def sleep(self, seconds):
        if self.moves:
            # The simulated FR3 needs a few cycles to follow each command.
            self.position += (self.target - self.position) * 0.5
        self.time += seconds

    def read(self):
        return self.leader, self.position

    def send(self, command):
        self.commands.append((self.time, np.array(command)))
        self.target = self.commands[-1][1]

    def align(self):
        return bridge.align_fr3(
            self.read,
            self.send,
            clock=lambda: self.time,
            sleep=self.sleep,
            log=self.messages.append,
        )


def test_fr3_alignment_moves_gradually_and_waits_for_feedback():
    test = SimulatedAlignment([0.65, -0.60, 0.0, 0.35, 0.0, 0.0, 0.0, 0.2])
    handoff = test.align()
    np.testing.assert_allclose(handoff, test.leader)
    assert 2.6 <= test.time < 8.0
    now = 0.0
    previous = np.zeros(8)
    for sent_at, command in test.commands:
        assert sent_at - now >= 0.04999
        assert np.abs(command[:7] - previous[:7]).max() <= (sent_at - now) * 0.25 + 1e-9
        assert command[-1] == 0.0
        now, previous = sent_at, command
    assert np.abs(test.position[:7] - test.leader[:7]).max() <= 0.05
    assert "converged" in test.messages[-1]


def test_fr3_alignment_times_out_if_feedback_does_not_move():
    test = SimulatedAlignment([0.0, -0.63, 0, 0, 0, 0, 0, 0], moves=False)
    with pytest.raises(ValueError, match=r"timed out.*J2.*0[.]630 rad"):
        test.align()
    assert math.isclose(test.time, 8.0)
    assert max(abs(command[1]) for _, command in test.commands) <= 0.05 + 1e-9


@pytest.mark.parametrize(
    ("leader", "message"),
    [
        ([0, 0, 0.81, 0, 0, 0, 0, 0], r"initial.*J3.*0[.]810 rad"),
        ([0, 0, 0, 0, 0, 0, 0, 0.7], r"gripper closure.*0[.]700"),
        ([0, 0, float("nan"), 0, 0, 0, 0, 0], r"finite"),
    ],
)
def test_fr3_alignment_rejects_unsafe_start_without_sending(leader, message):
    test = SimulatedAlignment(leader)
    with pytest.raises(ValueError, match=message):
        test.align()
    assert not test.commands


def test_fr3_alignment_aborts_instead_of_chasing_a_large_leader_movement():
    test = SimulatedAlignment([0.35, 0, 0, 0, 0, 0, 0, 0])
    send = test.send

    def jump(command):
        send(command)
        test.leader[0] = 0.95

    test.send = jump
    with pytest.raises(ValueError, match=r"during alignment.*J1"):
        test.align()
    assert len(test.commands) == 1


def test_fr3_alignment_does_not_burst_after_a_slow_feedback_cycle():
    test = SimulatedAlignment([0.4, 0, 0, 0, 0, 0, 0, 0])
    send = test.send

    def delayed(command):
        send(command)
        if len(test.commands) == 1:
            test.sleep(0.20)

    test.send = delayed
    test.align()
    times = [now for now, _ in test.commands]
    assert times[1] - times[0] >= 0.20
    assert min(np.diff(times)) >= 0.04999
