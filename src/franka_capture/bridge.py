"""Copy this standard-library module next to Franka's run_env.py (Python 3.8+)."""

import json
import math
import socket
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
