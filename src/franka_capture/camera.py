"""Independent RGB-D acquisition and lossless writers using the D435i library."""

from __future__ import annotations

import io
import time
from pathlib import Path

import numpy as np
from camera_realsense_d435i.camera import Configuration, Frame, devices
from camera_realsense_d435i.manager import CaptureManager
from PIL import Image


def _encode(array: np.ndarray, format_name: str) -> bytes:
    buffer = io.BytesIO()
    Image.fromarray(array).save(buffer, format_name)
    return buffer.getvalue()


class SyntheticCamera:
    """Generate repeatable pixels through the same writer as physical capture."""

    def __init__(self, configuration: Configuration):
        self.configuration = configuration
        self.seq = 0
        serial = configuration.serial
        self.tint = 130 if serial == "synthetic" else 48 + sum(serial.encode()) % 160
        self.deadline = 0.0

    def open(self) -> dict:
        config = self.configuration
        self.deadline = time.monotonic()
        intrinsics = {
            "width": config.width,
            "height": config.height,
            "fx": 50.0,
            "fy": 50.0,
            "ppx": config.width / 2,
            "ppy": config.height / 2,
            "model": "none",
            "coeffs": [0.0] * 5,
        }
        return {
            "device": {"name": "Synthetic D435i", "serial_number": config.serial},
            "configuration": config.dictionary(),
            "depth_scale_m_per_unit": 0.001,
            "color_intrinsics": intrinsics,
            "depth_intrinsics": intrinsics,
            "depth_to_color": {
                "rotation": [1, 0, 0, 0, 1, 0, 0, 0, 1],
                "translation_m": [0, 0, 0],
            },
            "depth_format": "z16",
            "depth_alignment": "native_depth",
        }

    def read(self) -> Frame:
        config = self.configuration
        self.deadline += 1 / config.fps
        time.sleep(max(0.0, self.deadline - time.monotonic()))
        received = time.monotonic_ns()
        self.seq += 1
        raw = np.full((config.height, config.width), 1000 + self.seq % 100, np.uint16)
        raw[0, 0] = 0
        color = np.zeros((config.height, config.width, 3), np.uint8)
        color[:, :, 0] = self.seq % 256
        color[:, :, 1] = self.tint
        device = {
            "frame_number": self.seq,
            "sdk_timestamp_ms": received / 1e6,
            "sdk_timestamp_domain": "hardware_clock",
        }
        meta = {
            "color": device,
            "depth": device,
            "received_monotonic_ns": received,
            "received_wall_time_ns": time.time_ns(),
        }
        return Frame(
            _encode(color, "JPEG"),
            _encode((raw // 16).astype(np.uint8), "JPEG"),
            b"",
            meta,
            color,
            raw,
        )

    def close(self) -> None:
        pass


def make_camera(
    root: Path,
    *,
    simulate: bool,
    serial: str = "",
    width: int | None = None,
    height: int | None = None,
    fps: int = 30,
) -> CaptureManager:
    config = Configuration(
        width=width if width is not None else (64 if simulate else 640),
        height=height if height is not None else (48 if simulate else 480),
        fps=fps,
        serial=(serial or "synthetic") if simulate else serial,
        align_depth_to_color=False,
    )
    options = {"record_root": str(root), "record_queue": 32}
    if simulate:
        return CaptureManager(config, factory=SyntheticCamera, **options)
    return CaptureManager(config, **options)


def make_cameras(
    root: Path,
    *,
    simulate: bool,
    dual_camera: bool = False,
    serial: str = "",
    left_serial: str | None = None,
    right_serial: str | None = None,
    width: int | None = None,
    height: int | None = None,
    fps: int = 30,
) -> dict[str, CaptureManager]:
    """Resolve role bindings before starting any camera worker."""
    explicit = left_serial is not None or right_serial is not None
    if not dual_camera:
        if explicit:
            raise ValueError("left and right serial bindings require --dual-camera")
        bindings = {"default": "" if simulate else serial}
    else:
        if serial.strip():
            raise ValueError("--serial cannot be combined with --dual-camera")
        if explicit:
            left_serial = (left_serial or "").strip()
            right_serial = (right_serial or "").strip()
            if not left_serial or not right_serial or left_serial == right_serial:
                raise ValueError("provide distinct non-empty left and right camera serials")
        elif simulate:
            left_serial, right_serial = "synthetic-left", "synthetic-right"
        else:
            detected = [
                device.get("serial_number", "").strip()
                for device in devices()
                if "D435I" in device.get("name", "").upper()
            ]
            if len(detected) != 2 or not all(detected) or detected[0] == detected[1]:
                raise ValueError(
                    "dual auto binding requires exactly two D435i cameras with distinct serials"
                )
            left_serial, right_serial = sorted(detected)
        bindings = {"left": left_serial, "right": right_serial}
    return {
        role: make_camera(
            root, simulate=simulate, serial=binding, width=width, height=height, fps=fps
        )
        for role, binding in bindings.items()
    }
