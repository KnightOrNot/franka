"""Copy this standard-library module next to Franka's run_env.py (Python 3.8+)."""

import json
import math
import socket
import time
import uuid
from numbers import Real
from typing import Iterable, List


def fr3_feedback_in_action_coordinates(feedback: Iterable[float]) -> List[float]:
    """Copy the seven FR3 joints and convert opening feedback to closure for startup."""
    try:
        joints = list(feedback)
        valid = len(joints) == 8 and all(
            isinstance(joint, Real) and not isinstance(joint, bool) and math.isfinite(joint)
            for joint in joints
        )
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        raise ValueError("FR3 startup feedback must contain eight finite values")

    opening = float(joints[-1])
    if not 0.0 <= opening <= 1.0:
        raise ValueError("FR3 startup gripper opening must be between 0 and 1")
    return [float(joint) for joint in joints[:7]] + [1.0 - opening]


_ALIGN_PERIOD = 0.05
_ALIGN_JOINT_STEP = 0.25 * _ALIGN_PERIOD
_ALIGN_DURATION = 8.0
_ALIGN_START_LIMIT = 0.8
_ALIGN_SETTLED = 0.05
_ALIGN_CLOSURE_LIMIT = 0.5
_ALIGN_SETTLED_SAMPLES = 3


def _alignment_sample(read):
    leader, follower = read()
    sample = []
    for role, vector in (("leader", leader), ("follower", follower)):
        try:
            values = list(vector)
        except TypeError:
            raise ValueError(f"FR3 alignment requires eight finite {role} values") from None
        if len(values) != 8 or any(
            not isinstance(x, Real) or isinstance(x, bool) or not math.isfinite(x) for x in values
        ):
            raise ValueError(f"FR3 alignment requires eight finite {role} values")
        if not 0.0 <= values[7] <= 1.0:
            raise ValueError(f"FR3 alignment requires {role} gripper closure in [0, 1]")
        sample.append([float(x) for x in values])
    return sample


def _alignment_gap(leader, follower, limits):
    details = []
    for i in range(8):
        target, observed, limit = leader[i], follower[i], limits[i]
        if abs(target - observed) > limit:
            label, unit = (f"J{i + 1}", " rad") if i < 7 else ("gripper closure", "")
            details.append(
                f"{label}: gap {abs(target - observed):.3f}{unit}, "
                f"leader {target:+.3f}, follower {observed:+.3f} (limit {limit:.3f})"
            )
    return "; ".join(details)


def _check_alignment_gap(leader, follower, phase):
    if gap := _alignment_gap(leader, follower, [_ALIGN_START_LIMIT] * 8):
        raise ValueError(f"FR3 {phase} pose gap exceeds {_ALIGN_START_LIMIT}: {gap}")
    if gap := _alignment_gap(leader, follower, [math.inf] * 7 + [_ALIGN_CLOSURE_LIMIT]):
        raise ValueError(f"FR3 {phase} {gap}. Match leader and follower gripper openings.")


def _paced_joint_target(previous, wanted, observed, axis):
    # Limit both commanded slew and how far a setpoint can lead actual feedback.
    low = max(previous - _ALIGN_JOINT_STEP, observed - _ALIGN_SETTLED)
    high = min(previous + _ALIGN_JOINT_STEP, observed + _ALIGN_SETTLED)
    if low > high + 1e-9:
        raise ValueError(
            f"FR3 alignment: J{axis} feedback moved too far from the previous target "
            f"(feedback {observed:+.3f} rad, previous target {previous:+.3f} rad)"
        )
    return min(max(wanted, low), high)


def align_fr3(read, send, *, clock=time.monotonic, sleep=time.sleep, log=print):
    """Pace the seven arm joints, hold measured gripper closure, and verify feedback."""
    leader, follower = _alignment_sample(read)
    _check_alignment_gap(leader, follower, "initial")
    previous = follower
    started = clock()
    deadline = started + _ALIGN_DURATION
    next_send = started + _ALIGN_PERIOD
    settled = 0
    log("Paced FR3 alignment: up to 20 Hz, 0.25 rad/s per arm joint, 8 s timeout.")

    while True:
        pause = min(next_send, deadline) - clock()
        if pause > 0:
            sleep(pause)
        leader, follower = _alignment_sample(read)
        now = clock()
        _check_alignment_gap(leader, follower, "during alignment")
        gap = _alignment_gap(leader, follower, [_ALIGN_SETTLED] * 7 + [_ALIGN_CLOSURE_LIMIT])
        if now >= deadline - 1e-9:
            largest = gap or "targets did not settle across consecutive samples"
            raise ValueError(f"FR3 alignment timed out after {_ALIGN_DURATION:.0f} s: {largest}")

        near_target = all(
            abs(leader[i] - previous[i]) <= _ALIGN_JOINT_STEP + 1e-9 for i in range(7)
        )
        settled = settled + 1 if not gap and near_target else 0
        if settled == _ALIGN_SETTLED_SAMPLES:
            log(f"FR3 alignment converged in {now - started:.2f} s.")
            return leader

        command = [
            _paced_joint_target(previous[i], leader[i], follower[i], i + 1) for i in range(7)
        ] + [follower[7]]
        send(command)
        previous = command
        # Don't send catch-up bursts when a read or RPC was delayed.
        next_send = now + _ALIGN_PERIOD


class Sender:
    """Send at most one datagram per completed cycle; no retries or waiting."""

    def __init__(self, endpoint):
        self.endpoint = str(endpoint)
        self.epoch = uuid.uuid4().hex
        self.seq = 0
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def publish(self, feedback, requested, observed_ns, command_ns):
        seq, self.seq = self.seq, self.seq + 1
        try:
            packet = json.dumps(
                {
                    "v": 1,
                    "epoch": self.epoch,
                    "seq": seq,
                    "feedback": list(map(float, feedback)),
                    "requested": list(map(float, requested)),
                    "observed_ns": observed_ns,
                    "command_ns": command_ns,
                },
                separators=(",", ":"),
                allow_nan=False,
            ).encode()
            if len(packet) > 4096:
                return False
            self.socket.sendto(packet, self.endpoint)
        except (OSError, ValueError, TypeError, OverflowError):
            return False
        return True

    def close(self):
        self.socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
