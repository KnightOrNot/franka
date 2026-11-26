import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture
def checkout(tmp_path):
    root = tmp_path / "checkout with spaces"
    (root / "scripts").mkdir(parents=True)
    (root / "config").mkdir()
    for name in ("start-capture.sh", "install-capture.sh"):
        source = REPO / "scripts" / name
        if source.exists():
            shutil.copy2(source, root / "scripts" / name)
    return root


def fake_program(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('#!/bin/bash\nprintf "%s\\n" "$PWD" "${UV_PROJECT_ENVIRONMENT:-}" "$@"\n')
    path.chmod(0o700)


def run(checkout, tmp_path, script, *args, **settings):
    path = checkout / "scripts" / script
    assert path.is_file(), f"launcher missing: {script}"
    return subprocess.run(
        [str(path), *args],
        cwd=tmp_path,
        env={"HOME": str(tmp_path), "PATH": os.defpath, **settings},
        capture_output=True,
        text=True,
        check=False,
    )


def options(result, checkout):
    assert result.returncode == 0, result.stderr
    cwd, _, command, *argv = result.stdout.splitlines()
    assert cwd == str(checkout)
    assert command == "serve"
    return {key: value for key, _, value in (arg.partition("=") for arg in argv)}


def test_launch_reads_checkout_site_settings(checkout, tmp_path):
    fake_program(checkout / ".venv/bin/franka-capture")
    (checkout / "config/site.env").write_text(
        "export FRANKA_CAPTURE_SOCKET=/private/socket\n"
        "FRANKA_CAPTURE_CAMERA_SERIAL=12345678\n"
        "FRANKA_CAPTURE_WIDTH=848\nFRANKA_CAPTURE_HEIGHT=480\nFRANKA_CAPTURE_FPS=15\n"
        "FRANKA_CAPTURE_DATA_ROOT='/archive/session raw'\n"
        "FRANKA_CAPTURE_DATASET_ROOT='/archive/session datasets'\n"
        "FRANKA_CAPTURE_HTTP_PORT=8008\nFRANKA_CAPTURE_CONTROL_REVISION=checked\n"
        "FRANKA_CAPTURE_STALE_MS=500\nFRANKA_CAPTURE_MAX_SKEW_MS=60\n"
        "FRANKA_CAPTURE_MIN_FREE_MB=2048\nFRANKA_CAPTURE_MAX_SECONDS=360\n"
    )
    assert options(run(checkout, tmp_path, "start-capture.sh"), checkout) == {
        "--socket": "/private/socket",
        "--serial": "12345678",
        "--width": "848",
        "--height": "480",
        "--camera-fps": "15",
        "--data-root": "/archive/session raw",
        "--dataset-root": "/archive/session datasets",
        "--port": "8008",
        "--control-revision": "checked",
        "--stale-ms": "500",
        "--max-skew-ms": "60",
        "--min-free-mb": "2048",
        "--max-seconds": "360",
    }


def test_launch_uses_explicit_config_relative_to_invocation(checkout, tmp_path):
    fake_program(checkout / ".venv/bin/franka-capture")
    (checkout / "config/site.env").write_text("FRANKA_CAPTURE_HTTP_PORT=8008\n")
    relative = "different site.env"
    (tmp_path / relative).write_text("FRANKA_CAPTURE_HTTP_PORT=8009\n")
    result = run(checkout, tmp_path, "start-capture.sh", FRANKA_CAPTURE_CONFIG=relative)
    assert options(result, checkout) == {"--port": "8009"}


def test_simulation_ignores_hardware_site_settings(checkout, tmp_path):
    fake_program(checkout / ".venv/bin/franka-capture")
    (checkout / "config/site.env").write_text("exit 99\n")
    result = run(
        checkout,
        tmp_path,
        "start-capture.sh",
        "--simulate",
        FRANKA_CAPTURE_CONFIG="missing site.env",
        FRANKA_CAPTURE_SOCKET="/robot/telemetry.sock",
        FRANKA_CAPTURE_DATA_ROOT="/robot/raw",
        FRANKA_CAPTURE_DATASET_ROOT="/robot/lerobot",
        FRANKA_CAPTURE_HTTP_PORT="8078",
        XDG_RUNTIME_DIR=str(tmp_path / "runtime"),
    )
    assert options(result, checkout) == {
        "--simulate": "",
        "--socket": str(tmp_path / "runtime/franka-capture-sim/telemetry.sock"),
        "--data-root": "data/sim/raw",
        "--dataset-root": "data/sim/lerobot",
        "--port": "8079",
    }


def test_launch_rejects_missing_explicit_config(checkout, tmp_path):
    fake_program(checkout / ".venv/bin/franka-capture")
    result = run(checkout, tmp_path, "start-capture.sh", FRANKA_CAPTURE_CONFIG="absent.env")
    assert result.returncode != 0
    assert "absent.env" in result.stderr
    assert not result.stdout


def test_launch_points_to_installer_when_environment_is_missing(checkout, tmp_path):
    result = run(checkout, tmp_path, "start-capture.sh")
    assert result.returncode != 0
    assert "scripts/install-capture.sh" in result.stderr


@pytest.mark.parametrize("override", [False, True])
def test_install_syncs_locked_operator_dependencies(checkout, tmp_path, override):
    uv = tmp_path / "bin/uv"
    fake_program(uv)
    settings = {"UV": str(uv)} if override else {"PATH": f"{uv.parent}:{os.defpath}"}
    result = run(checkout, tmp_path, "install-capture.sh", **settings)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        str(checkout),
        str(checkout / ".venv"),
        "sync",
        "--locked",
        "--no-dev",
        "--extra",
        "camera",
        "--extra",
        "export",
    ]
