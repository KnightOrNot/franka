"""Exercise the installed command, real job process, and a browser ZIP download."""

import io
import json
import os
import select
import subprocess
import sys
import time
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener
from zipfile import ZipFile

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("FRANKA_TEST_EXPORT") != "1", reason="requires installed exporter dependencies"
)


def test_one_server_records_converts_and_downloads_two_episodes(tmp_path):
    args = [
        sys.executable,
        "-m",
        "franka_capture",
        "serve",
        "--simulate",
        "--port",
        "0",
        "--socket",
        str(tmp_path / "cycle.sock"),
        "--data-root",
        str(tmp_path / "raw"),
        "--dataset-root",
        str(tmp_path / "datasets"),
        "--min-free-mb",
        "0",
    ]
    client = build_opener(ProxyHandler({}))
    server = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def wait_for(predicate, *, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = predicate()
            if result:
                return result
            time.sleep(0.03)
        raise AssertionError("controller did not reach the expected state")

    def get(path):
        with client.open(url + path, timeout=6) as response:
            return json.load(response)

    def post(path, fields):
        request = Request(
            url + path,
            json.dumps(fields).encode(),
            headers={"Content-Type": "application/json", "Origin": url},
        )
        with client.open(request, timeout=8) as response:
            return json.load(response)

    try:
        assert select.select([server.stdout], [], [], 10)[0], "server did not start"
        url = server.stdout.readline().strip()
        assert url.startswith("http://127.0.0.1:")
        episodes = []
        for _ in range(2):
            wait_for(lambda: get("/api/status")["ready"])
            assert post("/api/record", {"task": "web pick"})["phase"] == "recording"
            time.sleep(0.40)
            result = post("/api/save", {})
            assert result["result"] == "complete"
            episodes.append(Path(result["path"]).name)
        assert {e["id"] for e in get("/api/datasets")["episodes"]} == set(episodes)
        job = post("/api/export", {"episodes": episodes})
        assert job["state"] == "running"
        with pytest.raises(HTTPError) as blocked:
            post("/api/record", {"task": "wait for conversion"})
        assert blocked.value.code == 409

        def finalized():
            current = get("/api/datasets")
            assert current["job"]["state"] != "failed", current
            return current if current["job"]["state"] == "complete" else None

        current = wait_for(finalized, timeout=75)
        ready = current["datasets"][0]
        assert ready["frames"] >= 4 and ready["episodes"] == episodes
        with client.open(url + f"/api/datasets/{ready['id']}/download", timeout=10) as response:
            assert response.headers["Content-Type"].split(";")[0] == "application/zip"
            archive = response.read()
        assert len(archive) == ready["archive_bytes"]
        with ZipFile(io.BytesIO(archive)) as bundle:
            assert bundle.testzip() is None
            info = json.loads(bundle.read("dataset/meta/info.json"))
            assert info["total_episodes"] == 2
            assert info["total_frames"] == ready["frames"]
            assert any(n.endswith(".png") for n in bundle.namelist())
        assert wait_for(lambda: get("/api/status")["ready"])
    finally:
        if server.poll() is None:
            server.terminate()
        try:
            server.communicate(timeout=12)
        except subprocess.TimeoutExpired:
            server.kill()
            server.communicate(timeout=5)
