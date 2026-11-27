import json
from http.client import HTTPResponse
from io import BytesIO
from types import SimpleNamespace
from zipfile import ZipFile

import pytest

from franka_capture.recording import CaptureError
from franka_capture.web import OperatorHandler


class DatasetStub:
    def __init__(self, archive):
        self._archive = archive
        self.job = {"state": "idle", "id": None, "error": None}
        self.phase = "idle"

    def catalog(self):
        return {
            "episodes": [
                {"id": eid, "task": "抓取工件", "frames": 18, "fps": 15, "simulated": True}
                for eid in ("episode-one", "episode-two")
            ],
            "datasets": [
                {
                    "id": "finished-dataset",
                    "task": "抓取工件",
                    "episodes": ["episode-one"],
                    "frames": 18,
                    "fps": 15,
                    "simulated": True,
                    "archive_bytes": self._archive.stat().st_size,
                }
            ],
            "job": self.job,
            "available": True,
        }

    def start(self, ids):
        if self.phase != "idle":
            raise CaptureError("recording in progress")
        if ids != ["episode-one", "episode-two"]:
            raise ValueError("unknown or incompatible episodes")
        if self.job["state"] == "running":
            raise CaptureError("conversion in progress")
        self.job = {"state": "running", "id": "queued-dataset", "error": None}
        return self.job

    def start_recording(self, task):
        if self.job["state"] == "running":
            raise CaptureError("conversion in progress")
        self.phase = "recording"
        return {"phase": self.phase, "task": task}

    def finish_recording(self, operation):
        self.phase = "idle"
        return {"result": "complete" if operation == "save" else "discarded"}

    def archive(self, dataset_id):
        if dataset_id != "finished-dataset":
            raise FileNotFoundError(dataset_id)
        return self._archive


@pytest.fixture
def zip_path(tmp_path):
    archive = tmp_path / "archive.zip"
    with ZipFile(archive, "w") as zip_file:
        zip_file.writestr("meta/info.json", '{"codebase_version":"v3.0"}')
        zip_file.writestr("depth/frames/example.z16", bytes(320_000))
    return archive


@pytest.fixture
def http(zip_path):
    datasets = DatasetStub(zip_path)
    return SimpleNamespace(datasets=datasets, collector=datasets)


class MemoryConnection:
    def __init__(self, data):
        self.data = data
        self.response = BytesIO()

    def makefile(self, mode, *args):
        assert mode == "rb"
        return BytesIO(self.data)

    def sendall(self, data):
        self.response.write(data)

    def settimeout(self, timeout):
        pass


def request(server, method, route, body=None, **headers):
    headers.setdefault("Host", "127.0.0.1:8765")
    if isinstance(body, dict):
        body = json.dumps(body).encode()
        headers.setdefault("Content-Type", "application/json")
    if body is not None:
        headers.setdefault("Content-Length", str(len(body)))
    request_head = f"{method} {route} HTTP/1.1\r\n"
    request_head += "".join(f"{name}: {value}\r\n" for name, value in headers.items())
    wire = MemoryConnection((request_head + "\r\n").encode() + (body or b""))
    OperatorHandler(wire, ("127.0.0.1", 55555), server)
    with HTTPResponse(MemoryConnection(wire.response.getvalue()), method=method) as response:
        response.begin()
        data = response.read()
        if response.headers.get_content_type() == "application/json":
            data = json.loads(data)
        return response.status, response.headers, data


def test_catalog_and_export_job_for_multiple_episodes(http):
    status, _, catalog = request(http, "GET", "/api/datasets")
    assert status == 200
    assert catalog["available"] is True
    assert catalog["job"]["state"] == "idle"
    assert [episode["id"] for episode in catalog["episodes"]] == ["episode-one", "episode-two"]
    assert catalog["datasets"][0]["id"] == "finished-dataset"

    status, _, job = request(
        http,
        "POST",
        "/api/export",
        {"episodes": ["episode-one", "episode-two"]},
        Origin="http://127.0.0.1:8765",
    )
    assert status == 202
    assert job == {"state": "running", "id": "queued-dataset", "error": None}
    assert request(http, "GET", "/api/datasets")[2]["job"] == job


def test_recording_and_second_export_use_dataset_interlock(http):
    assert request(http, "POST", "/api/record", {"task": "试运行"})[2]["phase"] == "recording"
    body = {"episodes": ["episode-one", "episode-two"]}
    assert request(http, "POST", "/api/export", body)[0] == 409
    assert request(http, "POST", "/api/save", {})[2]["result"] == "complete"
    assert request(http, "POST", "/api/export", body)[0] == 202
    status, _, result = request(http, "POST", "/api/record", {"task": "blocked"})
    assert status == 409 and result["error"] == "conversion in progress"
    assert request(http, "POST", "/api/export", body)[0] == 409


@pytest.mark.parametrize(
    ("method", "route", "headers"),
    [
        ("POST", "/api/export", {"Origin": "http://example.org"}),
        ("GET", "/api/datasets", {"Host": "example.org"}),
        ("GET", "/api/datasets", {"Host": "localhost:not-a-port"}),
        ("GET", "/api/datasets", {"Host": "localhost:8000@127.0.0.1"}),
        ("GET", "/api/datasets/finished-dataset/download", {"Sec-Fetch-Site": "cross-site"}),
        ("GET", "/api/datasets/finished-dataset/download", {"Origin": "http://localhost"}),
    ],
)
def test_dataset_routes_reject_disallowed_hosts_and_origins(http, method, route, headers):
    assert request(http, method, route, **headers)[0] == 403


@pytest.mark.parametrize(
    "body",
    [
        {},
        {"episodes": []},
        {"episodes": "episode-one"},
        {"episodes": [["episode-one"]]},
        {"episodes": ["episode-one"], "output": "/tmp/result"},
        {"episodes": ["m" * (64 * 1024)]},
        {"episodes": ["unknown"]},
    ],
)
def test_export_rejects_invalid_or_out_of_policy_body(http, body):
    assert request(http, "POST", "/api/export", body)[0] == 400
    assert request(http, "GET", "/api/datasets")[2]["job"]["state"] == "idle"


def test_download_streams_the_finalized_zip_as_an_attachment(http, zip_path, monkeypatch):
    chunks = []
    sendall = MemoryConnection.sendall

    def measure_chunk(connection, data):
        chunks.append(len(data))
        sendall(connection, data)

    monkeypatch.setattr(MemoryConnection, "sendall", measure_chunk)
    status, headers, payload = request(http, "GET", "/api/datasets/finished-dataset/download")
    assert status == 200
    assert headers.get_content_type() == "application/zip"
    assert headers["Content-Disposition"] == 'attachment; filename="finished-dataset.zip"'
    assert int(headers["Content-Length"]) == len(payload)
    assert payload == zip_path.read_bytes()
    assert max(chunks) <= 128 * 1024


@pytest.mark.parametrize("dataset_id", ["queued-dataset", "../archive.zip", "%2e%2e%2farchive.zip"])
def test_download_rejects_unfinalized_ids_and_path_names(http, dataset_id):
    status, headers, _ = request(http, "GET", f"/api/datasets/{dataset_id}/download")
    assert status == 404
    assert headers.get_content_type() == "application/json"


def test_unconfigured_server_reports_conversion_unavailable(http):
    http.datasets = None
    status, _, catalog = request(http, "GET", "/api/datasets")
    assert status == 200
    assert catalog == {
        "available": False,
        "episodes": [],
        "datasets": [],
        "job": {"state": "idle", "id": None, "error": None},
    }
    assert request(http, "POST", "/api/export", {"episodes": ["episode-one"]})[0] == 503


def test_follow_controls_require_confirmation_and_stay_separate_from_export(http):
    events = []
    http.follow = SimpleNamespace(
        start=lambda: events.append("start") or {"phase": "awaiting_confirmation"},
        stop=lambda: events.append("stop") or {"phase": "stopping"},
    )
    http.datasets.finish_dataset = lambda: {"state": "running", "id": "batch-job"}
    assert request(http, "POST", "/api/follow/start", {"motion_confirmed": False})[0] == 400
    assert request(http, "POST", "/api/follow/start", {"motion_confirmed": 1})[0] == 400
    assert not events
    assert request(http, "POST", "/api/follow/start", {"motion_confirmed": True})[2] == {
        "phase": "awaiting_confirmation"
    }
    assert request(http, "POST", "/api/dataset/finish", {})[0] == 202
    assert events == ["start"]
    assert request(http, "POST", "/api/follow/stop", {})[2] == {"phase": "stopping"}
    assert events == ["start", "stop"]


def test_independent_camera_previews_and_rgbd_share_one_frame(http):
    frame = SimpleNamespace(color_jpeg=b"left rgb", depth_jpeg=b"left depth")
    calls = []

    def snap():
        calls.append("left")
        return frame

    http.collector.cameras = {
        "left": SimpleNamespace(frame=snap),
        "right": SimpleNamespace(
            frame=lambda: SimpleNamespace(color_jpeg=b"right rgb", depth_jpeg=b"right depth")
        ),
    }
    assert request(http, "GET", "/preview/right/depth.jpg")[2] == b"right depth"
    assert request(http, "GET", "/preview/left/rgbd.json")[2] == {
        "color": "bGVmdCByZ2I=",
        "depth": "bGVmdCBkZXB0aA==",
    }
    assert calls == ["left"]
