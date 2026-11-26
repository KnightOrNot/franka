import json
import os
import subprocess
import sys
import threading
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import pytest

from franka_capture.camera import make_camera
from franka_capture.recording import Collector
from franka_capture.web import OperatorServer


def test_operator_is_local_and_only_controls_recording(tmp_path):
    camera = make_camera(tmp_path, simulate=True)
    collector = Collector(tmp_path, camera, simulated=True, min_free_mb=0)
    with OperatorServer(0, collector) as server:
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        urlopen = build_opener(ProxyHandler({})).open
        try:
            with urlopen(url, timeout=2) as response:
                assert "Franka" in response.read().decode()
            with urlopen(url + "/api/status", timeout=2) as response:
                assert json.load(response)["ready"] is False
            request = Request(
                url + "/api/record",
                b'{"task":"x"}',
                headers={"Content-Type": "application/json", "Origin": "http://example.org"},
            )
            with pytest.raises(HTTPError) as error:
                urlopen(request, timeout=2)
            assert error.value.code == 403
            request = Request(
                url + "/api/move",
                b"{}",
                headers={"Content-Type": "application/json"},
            )
            with pytest.raises(HTTPError) as error:
                urlopen(request, timeout=2)
            assert error.value.code == 404
        finally:
            server.shutdown()
            thread.join()


def test_demo_and_offline_check_are_runnable(tmp_path):
    with subprocess.Popen(
        [
            sys.executable,
            "-m",
            "franka_capture",
            "demo",
            "--data-root",
            str(tmp_path / "raw"),
            "--socket",
            str(tmp_path / "cy.sock"),
            "--min-free-mb",
            "0",
            "--seconds",
            "0.35",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ) as proc:
        stdout, stderr = proc.communicate(timeout=10)
        assert proc.returncode == 0, stderr.decode()
    result = json.loads(stdout)
    path = Path(result["path"])
    assert path.is_dir()
    checked = subprocess.run(
        [sys.executable, "-m", "franka_capture", "check", str(path)],
        capture_output=True,
        text=True,
        check=True,
    )
    assert json.loads(checked.stdout)["report"]["matched_pairs"] >= 2


@pytest.mark.skipif(os.environ.get("FRANKA_TEST_EXPORT") != "1", reason="optional LeRobot stack")
def test_real_lerobot_readback(tmp_path):
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    from franka_capture.export import export_episode

    made = subprocess.run(
        [
            sys.executable,
            "-m",
            "franka_capture",
            "demo",
            "--data-root",
            str(tmp_path / "raw"),
            "--socket",
            str(tmp_path / "cy.sock"),
            "--min-free-mb",
            "0",
            "--seconds",
            "0.45",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    original = Path(json.loads(made.stdout)["path"])
    dataset_root = tmp_path / "lerobot"
    report = export_episode(original, dataset_root, fps=15)
    dataset = LeRobotDataset("local/franka", root=dataset_root)
    assert len(dataset) == report["rows"] >= 3
    item = dataset[0]
    assert tuple(item["observation.state"].shape) == (8,)
    assert tuple(item["action"].shape) == (8,)
    assert tuple(item["observation.images.front"].shape) == (3, 48, 64)
    assert 0 <= item["observation.state"][-1].item() <= 0.09
    assert np.isfinite(item["action"].numpy()).all()
    sidecar = [
        json.loads(line)
        for line in (dataset_root / "franka_alignment.jsonl").read_text().splitlines()
    ]
    assert len(sidecar) == len(dataset)
    assert {row["source_epoch"] for row in sidecar} == {sidecar[0]["source_epoch"]}


def test_operator_records_from_live_simulation(tmp_path):
    import time

    from franka_capture.cli import _launch, parser

    args = parser().parse_args(
        [
            "serve",
            "--simulate",
            "--data-root",
            str(tmp_path / "raw"),
            "--min-free-mb",
            "0",
            "--socket",
            str(tmp_path / "cy.sock"),
        ]
    )
    with _launch(args) as collector, OperatorServer(0, collector) as server:
        thread = threading.Thread(target=server.serve_forever, args=(0.05,))
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        http = build_opener(ProxyHandler({}))

        def post(route, data):
            request = Request(
                url + "/api/" + route,
                json.dumps(data).encode(),
                headers={"Content-Type": "application/json", "Origin": url},
            )
            with http.open(request, timeout=5) as response:
                return json.load(response)

        try:
            deadline = time.monotonic() + 3
            while not collector.status()["ready"]:
                assert time.monotonic() < deadline
                time.sleep(0.01)
            with http.open(url + "/preview/color.jpg", timeout=2) as image:
                assert image.read(2) == b"\xff\xd8"
            assert post("record", {"task": "HTTP pick"})["phase"] == "recording"
            time.sleep(0.3)
            saved = post("save", {})
            assert (Path(saved["path"]) / "camera" / "COMPLETE.json").is_file()
            assert post("record", {"task": "HTTP discard"})["phase"] == "recording"
            assert Path(post("discard", {})["path"]).suffix == ".partial"
        finally:
            server.shutdown()
            thread.join()
