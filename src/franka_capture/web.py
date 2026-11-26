"""Loopback-only controls for recording, JPEG preview and dataset export."""

from __future__ import annotations

import json
import os
import re
import shutil
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Protocol

from .recording import CaptureError, Collector

PAGE = """<!doctype html>
<html lang="zh-CN">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Franka 数采</title>
<style>
body { font:16px system-ui,sans-serif; margin:auto; padding:1.5rem 1rem;
       max-width:62rem; color:#182026; background:#f5f7f8 }
h1 { margin:0 0 .75rem }
h2 { margin:0 0 .5rem; font-size:1.2rem }
p { line-height:1.5 }
section { padding:1rem; border:1px solid #dce2e5; border-radius:.6rem;
          margin:0 0 1rem; background:white }
.panes { display:grid; grid-template-columns:repeat(auto-fit,minmax(min(100%,240px),1fr));
         gap:1rem }
figure { margin:0; min-width:0 }
img { width:100%; aspect-ratio:4/3; background:#f0f2f4; object-fit:contain }
button, input { font:inherit; padding:.55rem .8rem; margin:.25rem .25rem .25rem 0 }
#task { box-sizing:border-box; max-width:100%; width:22rem }
button:not(:disabled), label:has(input:not(:disabled)) { cursor:pointer }
#message, #dataset-status, li { overflow-wrap:anywhere }
#message, #dataset-status { min-height:1.5em }
#episode-list, #dataset-list { padding:0; list-style:none }
#episode-list ul { padding:0; list-style:none }
#episode-list > li, #dataset-list > li { border-top:1px solid #e5eaed; padding:.6rem 0 }
#episode-list label { display:flex; align-items:start; gap:.4rem; padding:.35rem 0 }
#episode-list label:has(:disabled) { color:#5c6770 }
#episode-list input { flex:none }
.episode-id { display:block; font-size:.85em; color:#4e5d69 }
.archive { display:flex; flex-wrap:wrap; gap:.4rem 1rem; align-items:baseline }
.archive a { white-space:nowrap }
</style>
<main>
<h1>Franka 数采</h1>
<section aria-label="预览和录制">
<p id="status" role="status">正在读取设备状态…</p>
<div class="panes">
<figure><img id="color" alt="彩色画面"><figcaption>RGB</figcaption></figure>
<figure><img id="depth" alt="深度预览"><figcaption>深度预览 · 原生 Z16 另存</figcaption></figure>
</div>
<p><label for="task">任务</label>
<input id="task" maxlength="160" placeholder="例如：抓取并放置方块"></p>
<button id="record" disabled>开始录制</button>
<button id="save" disabled>保存</button>
<button id="discard" disabled>丢弃</button>
<p id="message" role="status"></p>
</section>
<section aria-labelledby="episodes-title">
<h2 id="episodes-title">已保存回合</h2>
<p>可勾选同一任务、同一采集模式的多个回合。转换期间不能录制。</p>
<p id="episode-empty" hidden>没有可转换的回合。请先录制并保存。</p>
<ul id="episode-list"></ul>
<button id="export" disabled>转换为 LeRobot</button>
<p id="dataset-status" role="status">正在读取数据目录…</p>
</section>
<section aria-labelledby="datasets-title">
<h2 id="datasets-title">已转换数据集</h2>
<p id="dataset-empty" hidden>还没有可下载的数据集。</p>
<ul id="dataset-list"></ul>
</section>
</main>
<script>
const el = id => document.getElementById(id);
const endpoints = ["record", "save", "discard", "export"];
const outcomes = {complete:"已保存", discarded:"已丢弃", failed:"录制失败"};
const selected = new Set(), options = new Map();
let capture = null, catalog = null, action = "", updating = false, revision = 0;
let captureError = "", exportError = "", previewSession = "";
let episodeKey = "", datasetKey = "";

function text(id, value) {
  if (el(id).textContent !== value) el(id).textContent = value;
}
async function api(path, body) {
  const settings = {cache:"no-store"};
  if (body !== undefined) {
    Object.assign(settings, {
      method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body)
    });
  }
  const response = await fetch("/api/" + path, settings);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || "请求失败");
  return result;
}
function updateControls() {
  const recording = capture?.phase === "recording";
  const idle = capture?.phase === "idle";
  const exporting = !catalog || catalog.job.state === "running";
  el("record").disabled = !!action || !capture?.ready || exporting;
  el("save").disabled = el("discard").disabled = !!action || !recording;
  const selectionBlocked = !!action || !idle || exporting || !catalog?.available;
  el("export").disabled = selectionBlocked || !selected.size;
  const first = options.get(selected.values().next().value)?.episode;
  for (const [id, {input, episode}] of options) {
    input.checked = selected.has(id);
    input.disabled = selectionBlocked || !!first && (
      episode.task !== first.task || episode.simulated !== first.simulated
    );
  }
}
function clearImage(img) {
  const old = img.src;
  img.removeAttribute("src");
  if (old.startsWith("blob:")) URL.revokeObjectURL(old);
}
function renderCapture() {
  const s = capture;
  if (!s) {
    text("status", "连接中断");
  } else {
    const phase = {idle:"空闲", recording:"正在录制", finishing:"正在保存或检查"}[s.phase];
    text("status", (s.simulated ? "模拟 · " : "") + phase +
      "　遥测 " + (s.robot_fresh ? "在线" : "未就绪") +
      "　相机 " + (s.camera_fresh ? "在线" : "未就绪") +
      (s.phase === "recording" ? "　关节周期 " + s.robot_cycles : ""));
  }
  const last = s?.last;
  text("message", captureError || (action === "save" ? "正在检查并保存…" : "") ||
    s?.error || (last ? outcomes[last.result] + " · " + last.path +
      (last.error ? " · " + last.error : "") : "") || (!s?.camera_fresh && s?.camera.error) || "");
  const session = s?.camera_fresh ? s.camera.session : "";
  if (previewSession !== session) {
    for (const id of ["color", "depth"]) clearImage(el(id));
    previewSession = session;
  }
}
function entryDetails(item) {
  return item.frames + " 帧 · " + item.fps + " FPS" + (item.simulated ? " · 模拟" : "");
}
function renderEpisodes(episodes) {
  el("episode-list").replaceChildren();
  el("episode-empty").hidden = episodes.length > 0;
  const groups = new Map();
  options.clear();
  for (const episode of episodes) {
    let group = groups.get(episode.task);
    if (!group) {
      const entry = document.createElement("li"), title = document.createElement("strong");
      group = document.createElement("ul");
      title.textContent = episode.task;
      entry.append(title, group);
      el("episode-list").append(entry);
      groups.set(episode.task, group);
    }
    const row = document.createElement("li"), label = document.createElement("label");
    const input = document.createElement("input"), detail = document.createElement("span");
    const id = document.createElement("span");
    input.type = "checkbox";
    id.className = "episode-id";
    id.textContent = episode.id;
    detail.append(entryDetails(episode), id);
    options.set(episode.id, {input, episode});
    input.onchange = () => {
      if (input.checked) selected.add(episode.id);
      else selected.delete(episode.id);
      updateControls();
    };
    label.append(input, detail);
    row.append(label);
    group.append(row);
  }
  for (const id of selected) if (!options.has(id)) selected.delete(id);
}
function renderArchives(datasets) {
  el("dataset-list").replaceChildren();
  el("dataset-empty").hidden = datasets.length > 0;
  for (const dataset of datasets) {
    const row = document.createElement("li"), info = document.createElement("span");
    const title = document.createElement("strong"), link = document.createElement("a");
    const id = document.createElement("span");
    row.className = "archive";
    title.textContent = dataset.task;
    id.className = "episode-id";
    id.textContent = dataset.id;
    info.append(title, " · " + dataset.episodes.length + " 回合 · " + entryDetails(dataset), id);
    link.href = "/api/datasets/" + encodeURIComponent(dataset.id) + "/download";
    link.textContent = "下载 ZIP";
    link.setAttribute("aria-label", "下载 ZIP · " + dataset.task + " · " + dataset.id);
    row.append(info, link);
    el("dataset-list").append(row);
  }
}
function renderCatalog() {
  let status = "数据目录暂不可用";
  if (catalog) {
    const job = catalog.job;
    if (job.state === "running") status = "正在转换及打包 ZIP…";
    else if (job.state === "failed") status = "转换失败" + (job.error ? " · " + job.error : "");
    else if (job.state === "complete") status = "转换完成，可下载 ZIP。";
    else status = catalog.available ? "" : "当前环境未启用转换功能。";
    const episodes = JSON.stringify(catalog.episodes), datasets = JSON.stringify(catalog.datasets);
    if (episodes !== episodeKey) {
      renderEpisodes(catalog.episodes);
      episodeKey = episodes;
    }
    if (datasets !== datasetKey) {
      renderArchives(catalog.datasets);
      datasetKey = datasets;
    }
  }
  text("dataset-status", exportError || (action === "export" ? "正在启动转换…" : status));
}
async function update() {
  if (updating || action) return;
  updating = true;
  const requestRevision = revision;
  try {
    const [s, d] = await Promise.allSettled([api("status"), api("datasets")]);
    if (requestRevision !== revision) return;
    capture = s.status === "fulfilled" ? s.value : null;
    catalog = d.status === "fulfilled" ? d.value : null;
    renderCapture();
    renderCatalog();
    updateControls();
  } finally { updating = false; }
}
async function change(id) {
  if (action || el(id).disabled) return;
  if (id === "discard" && !confirm("丢弃当前回合？该回合不会导出。")) return;
  if (id === "record" && !el("task").value.trim()) {
    captureError = "请输入任务名";
    renderCapture();
    el("task").focus();
    return;
  }
  action = id;
  revision++;
  if (id === "export") exportError = "";
  else captureError = "";
  updateControls();
  renderCapture();
  renderCatalog();
  try {
    const body = id === "record" ? {task:el("task").value} :
      id === "export" ? {episodes:[...selected]} : {};
    const result = await api(id, body);
    if (id === "export") {
      catalog.job = result;
      selected.clear();
    } else {
      capture = null;
    }
  } catch (e) {
    if (id === "export") exportError = e.message;
    else captureError = e.message;
  } finally {
    action = "";
    renderCatalog();
    updateControls();
    await update();
  }
}
for (const id of endpoints) el(id).onclick = () => change(id);
for (const id of ["color", "depth"]) {
  let fetching = false;
  setInterval(async () => {
    const session = previewSession;
    if (!session || document.hidden || fetching) return;
    fetching = true;
    try {
      const response = await fetch("/preview/" + id + ".jpg", {cache:"no-store"});
      if (!response.ok || session !== previewSession) return;
      const blob = await response.blob();
      if (session !== previewSession) return;
      const img = el(id), old = img.src;
      img.src = URL.createObjectURL(blob);
      if (old.startsWith("blob:")) URL.revokeObjectURL(old);
    } catch {
      // Device status polling reports disconnections.
    } finally { fetching = false; }
  }, 150);
}
update();
setInterval(update, 750);
</script>
"""

_HOST = re.compile(r"(?:localhost|127\.0\.0\.1)(?::([0-9]{1,5}))?", re.IGNORECASE)
_DOWNLOAD = re.compile(r"/api/datasets/([A-Za-z0-9_-]{1,128})/download")
_POST_FIELDS = {
    "/api/record": {"task"},
    "/api/save": set(),
    "/api/discard": set(),
    "/api/export": {"episodes"},
}


class DatasetOperations(Protocol):
    def catalog(self) -> dict: ...
    def start(self, ids: list[str]) -> dict: ...
    def start_recording(self, task: str) -> dict: ...
    def archive(self, dataset_id: str) -> Path: ...


class OperatorServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self, port: int, collector: Collector, datasets: DatasetOperations | None = None
    ) -> None:
        self.collector = collector
        self.datasets = datasets
        super().__init__(("127.0.0.1", port), OperatorHandler)


class OperatorHandler(BaseHTTPRequestHandler):
    server: OperatorServer

    def setup(self) -> None:
        super().setup()
        self.connection.settimeout(5)

    def log_message(self, *args) -> None:
        pass

    def _allowed_origin(self) -> bool:
        hosts = self.headers.get_all("Host", [])
        if len(hosts) != 1 or not (host := _HOST.fullmatch(hosts[0])):
            return False
        if host[1] and not 1 <= int(host[1]) <= 65535:
            return False
        origins = self.headers.get_all("Origin", [])
        if origins and origins != ["http://" + hosts[0]]:
            return False
        sites = self.headers.get_all("Sec-Fetch-Site", [])
        return not sites or sites in (["same-origin"], ["none"])

    def _headers(self, code: int, content_type: str, length: int) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")

    def _respond(self, code: int, body: bytes, content_type: str) -> None:
        try:
            self._headers(code, content_type, len(body))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def _json(self, code: int, value: dict) -> None:
        self._respond(code, json.dumps(value, ensure_ascii=False).encode(), "application/json")

    def _download(self, dataset_id: str) -> None:
        datasets = self.server.datasets
        try:
            if datasets is None:
                raise FileNotFoundError
            archive = datasets.archive(dataset_id).open("rb")
        except FileNotFoundError:
            self._json(404, {"error": "dataset archive not found"})
            return
        except OSError as exc:
            self._json(500, {"error": str(exc)})
            return
        with archive:
            try:
                size = os.fstat(archive.fileno()).st_size
            except OSError as exc:
                self._json(500, {"error": str(exc)})
                return
            try:
                self._headers(200, "application/zip", size)
                self.send_header("Content-Disposition", f'attachment; filename="{dataset_id}.zip"')
                self.end_headers()
                shutil.copyfileobj(archive, self.wfile, length=64 * 1024)
            except OSError:
                # No second response after the ZIP headers have been written.
                pass

    def do_GET(self) -> None:
        if not self._allowed_origin():
            self._json(403, {"error": "same-origin local access only"})
            return
        if self.path == "/":
            self._respond(200, PAGE.encode(), "text/html; charset=utf-8")
        elif self.path == "/api/status":
            self._json(200, self.server.collector.status())
        elif self.path == "/api/datasets":
            datasets = self.server.datasets
            try:
                catalog = (
                    datasets.catalog()
                    if datasets is not None
                    else {
                        "episodes": [],
                        "datasets": [],
                        "job": {"state": "idle", "id": None, "error": None},
                        "available": False,
                    }
                )
            except OSError as exc:
                self._json(500, {"error": str(exc)})
            else:
                self._json(200, catalog)
        elif match := _DOWNLOAD.fullmatch(self.path):
            self._download(match[1])
        elif self.path in ("/preview/color.jpg", "/preview/depth.jpg"):
            try:
                frame = self.server.collector.camera.frame()
            except RuntimeError as exc:
                self._json(503, {"error": str(exc)})
            else:
                image = frame.color_jpeg if self.path == "/preview/color.jpg" else frame.depth_jpeg
                self._respond(200, image, "image/jpeg")
        else:
            self._json(404, {"error": "unknown endpoint"})

    def _read_body(self, required: set[str], max_length: int) -> dict:
        sizes = self.headers.get_all("Content-Length", [])
        if (
            len(sizes) != 1
            or not re.fullmatch(r"[0-9]{1,5}", sizes[0])
            or not 1 <= (length := int(sizes[0])) <= max_length
            or self.headers.get_all("Transfer-Encoding")
            or self.headers.get_content_type() != "application/json"
        ):
            raise ValueError("bounded application/json body required")
        raw = self.rfile.read(length)
        if len(raw) != length:
            raise ValueError("incomplete JSON body")
        body = json.loads(raw)
        if not isinstance(body, dict) or set(body) != required:
            raise ValueError("invalid request fields")
        return body

    def do_POST(self) -> None:
        if not self._allowed_origin():
            self._json(403, {"error": "same-origin local access only"})
            return
        if self.path not in _POST_FIELDS:
            self._json(404, {"error": "unknown endpoint"})
            return
        exporting = self.path == "/api/export"
        datasets = self.server.datasets
        collector = self.server.collector
        try:
            body = self._read_body(_POST_FIELDS[self.path], 32 * 1024 if exporting else 2048)
            if exporting:
                ids = body["episodes"]
                if (
                    not isinstance(ids, list)
                    or not ids
                    or not all(isinstance(eid, str) and eid for eid in ids)
                ):
                    raise ValueError("select one or more episodes")
                if datasets is None:
                    self._json(503, {"error": "dataset conversion is not configured"})
                    return
                result = datasets.start(ids)
            elif self.path == "/api/record":
                start = datasets.start_recording if datasets is not None else collector.start
                result = start(body["task"])
            else:
                result = collector.finish(self.path.rsplit("/", 1)[1])
        except (ValueError, RecursionError, TimeoutError) as exc:
            self._json(400, {"error": str(exc)})
        except CaptureError as exc:
            self._json(409, {"error": str(exc)})
        except OSError as exc:
            self._json(500, {"error": str(exc)})
        else:
            self._json(202 if exporting else 200, result)
