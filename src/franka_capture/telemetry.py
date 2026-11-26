"""Validated control-cycle records arriving over a private Unix socket."""

from __future__ import annotations

import json
import math
import os
import re
import socket
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from pathlib import Path

PACKET_LIMIT = 4096
MAX_OPEN_M = 0.09  # fr3.py in the inspected Franka deployment
JOINT_NAMES = tuple(f"joint_{axis}" for axis in range(1, 8)) + ("gripper_opening_m",)


def socket_default() -> Path:
    xdg = os.environ.get("XDG_RUNTIME_DIR")
    base = Path(xdg) / "franka-capture" if xdg else Path(f"/tmp/franka-capture-{os.getuid()}")
    return base / "telemetry.sock"


def _vector(value: object) -> tuple[float, ...]:
    if not isinstance(value, (tuple, list)) or len(value) != 8:
        raise ValueError("expected seven joints plus one gripper value")
    if not all(type(x) in (int, float) and math.isfinite(x) for x in value):
        raise ValueError("joint values must be finite numbers")
    return tuple(map(float, value))


@dataclass(frozen=True)
class Cycle:
    epoch: str
    seq: int
    observed_ns: int
    command_ns: int
    feedback: tuple[float, ...]
    requested: tuple[float, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.epoch, str) or not re.fullmatch(r"[0-9a-f]{32}", self.epoch):
            raise ValueError("invalid source epoch")
        if type(self.seq) is not int or self.seq < 0:
            raise ValueError("invalid source sequence")
        if (
            type(self.observed_ns) is not int
            or type(self.command_ns) is not int
            or not 0 < self.observed_ns <= self.command_ns
        ):
            raise ValueError("invalid observation/command timestamps")
        object.__setattr__(self, "feedback", _vector(self.feedback))
        object.__setattr__(self, "requested", _vector(self.requested))

    @property
    def state(self) -> tuple[float, ...]:
        return self.feedback[:7] + (MAX_OPEN_M * self.feedback[-1],)

    @property
    def action(self) -> tuple[float, ...]:
        return self.requested[:7] + (MAX_OPEN_M * (1 - self.requested[-1]),)

    def row(self) -> dict:
        return {"v": 1, **asdict(self)}

    @classmethod
    def from_row(cls, row: object) -> Cycle:
        fields = {"epoch", "seq", "observed_ns", "command_ns", "feedback", "requested", "v"}
        if not isinstance(row, dict) or set(row) != fields or type(row["v"]) is not int:
            raise ValueError("invalid telemetry fields")
        if row["v"] != 1:
            raise ValueError("unsupported telemetry version")
        return cls(**{key: value for key, value in row.items() if key != "v"})

    @classmethod
    def decode(cls, packet: bytes) -> Cycle:
        if not packet or len(packet) > PACKET_LIMIT:
            raise ValueError("invalid telemetry packet size")
        return cls.from_row(json.loads(packet))


@contextmanager
def open_receiver(path: Path) -> Iterator[socket.socket]:
    if not path.is_absolute():
        raise ValueError("telemetry socket path must be absolute")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    parent = path.parent.stat()
    if parent.st_uid != os.getuid() or parent.st_mode & 0o077:
        raise PermissionError("telemetry socket directory must be owner-private")
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as receiver:
        receiver.bind(str(path))
        identity = path.stat()
        try:
            path.chmod(0o600)
            receiver.settimeout(0.1)
            yield receiver
        finally:
            if path.exists() and os.path.samestat(identity, path.lstat()):
                path.unlink()
