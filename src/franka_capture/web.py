"""Loopback-only controls for recording, JPEG preview and dataset export."""

from __future__ import annotations

import base64
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
figure { margin:0; min-width:0; border:1px solid #e4eaee; border-radius:.55rem;
         padding:.65rem }
figure[hidden], img[hidden] { display:none }
.panel-title { display:flex; align-items:center; justify-content:space-between; gap:.4rem;
               margin-bottom:.4rem; font-weight:600 }
.panel-title select { font:inherit; font-size:.88rem; max-width:55%; padding:.2rem }
.panel-images { display:grid; grid-template-columns:minmax(0,1fr) }
.panel-images.split { grid-template-columns:repeat(2,minmax(0,1fr)); gap:.2rem }
img { display:block; width:100%; aspect-ratio:4/3; background:#f0f2f4; object-fit:contain }
.panel-stats, #telemetry-stats, #follow-detail { font-size:.86rem; color:#4e5d69;
                                             min-height:1.3em; overflow-wrap:anywhere }
.follow-controls { border-bottom:1px solid #e4eaee; padding-bottom:.6rem; margin-bottom:.8rem }
.follow-controls label { display:block }
#follow-stop { color:#a42230 }
#finish-batch:not(:disabled) { background:#164d72; color:white; border-color:#164d72 }
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
<div class="follow-controls" aria-label="跟随控制">
<p id="follow-status" role="status">正在读取跟随状态…</p>
<p id="motion-target"></p>
<label><input id="motion-confirmed" type="checkbox" autocomplete="off">
我在机械臂旁，已确认工作区清空、急停可用，并核对回零和夹爪动作</label>
<button id="follow-start" disabled>启动跟随</button>
<button id="follow-stop" disabled>停止跟随</button>
<p id="follow-detail" role="status"></p>
</div>
<p id="status" role="status">正在读取设备状态…</p>
<p id="telemetry-stats" role="status"></p>
<div class="panes">
<figure data-panel="left"><figcaption class="panel-title"><span>左相机</span>
<select aria-label="左相机预览模式"><option value="color">RGB</option>
<option value="depth">深度</option><option value="rgbd">RGBD</option></select></figcaption>
<div class="panel-images"><img alt="左相机 RGB"><img alt="左相机深度" hidden></div>
<p class="panel-stats" role="status">等待相机</p></figure>
<figure data-panel="right"><figcaption class="panel-title"><span>右相机</span>
<select aria-label="右相机预览模式"><option value="color">RGB</option>
<option value="depth">深度</option><option value="rgbd">RGBD</option></select></figcaption>
<div class="panel-images"><img alt="右相机 RGB"><img alt="右相机深度" hidden></div>
<p class="panel-stats" role="status">等待相机</p></figure>
</div>
<p><label for="task">任务</label>
<input id="task" maxlength="160" placeholder="例如：抓取并放置方块"></p>
<button id="record" disabled>开始录制</button>
<button id="save" disabled>保存</button>
<button id="discard" disabled>丢弃</button>
<button id="finish-batch" disabled>结束数据集并转换</button>
<p id="batch-status" role="status"></p>
<p id="message" role="status"></p>
</section>
<section aria-labelledby="episodes-title">
<h2 id="episodes-title">已保存回合</h2>
<p>也可以手动转换历史回合。转换期间暂停录制操作，机械臂跟随不会停止。</p>
<p id="episode-empty" hidden>没有可转换的回合。请先录制并保存。</p>
<ul id="episode-list"></ul>
<button id="export" disabled>转换所选回合为 LeRobot</button>
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
const endpoints = ["record", "save", "discard", "export", "finish-batch"];
const outcomes = {complete:"已保存", discarded:"已丢弃", failed:"录制失败"};
const selected = new Set(), options = new Map();
let capture = null, catalog = null, action = "", updating = false, revision = 0;
let captureError = "", exportError = "", followError = "";
let follow = null, followAction = false;
const panels = [...document.querySelectorAll("[data-panel]")].map(panel => ({
  panel, role:panel.dataset.panel, camera:null, key:"", fetching:false,
  images:[...panel.querySelectorAll("img")], mode:panel.querySelector("select"),
  stats:panel.querySelector(".panel-stats"), name:panel.querySelector(".panel-title span")
}));
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
  const batch = catalog?.batch;
  el("record").disabled = !!action || !capture?.ready || exporting || !!batch?.sealed;
  el("save").disabled = el("discard").disabled = !!action || !recording;
  el("task").disabled = !!action || !idle || exporting || !!batch?.episodes?.length;
  text("finish-batch", batch?.sealed && !exporting ? "重试转换" : "结束数据集并转换");
  el("finish-batch").disabled = !!action || !idle || exporting ||
    !catalog?.available || !batch?.episodes?.length;
  const phase = follow?.phase;
  el("follow-start").disabled = followAction || !el("motion-confirmed").checked ||
    follow?.start_enabled !== true || !idle || !["idle", "error"].includes(phase);
  el("follow-stop").disabled = followAction ||
    !["starting", "following", "error"].includes(phase);
  const selectionBlocked = !!action || !idle || exporting || !catalog?.available;
  el("export").disabled = selectionBlocked || !selected.size;
  const first = options.get(selected.values().next().value)?.episode;
  for (const [id, {input, episode}] of options) {
    input.checked = selected.has(id);
    input.disabled = selectionBlocked || !!first && (
      episode.task !== first.task || episode.simulated !== first.simulated ||
      (episode.cameras || 1) !== (first.cameras || 1)
    );
  }
}
function clearImage(img) {
  const old = img.src;
  img.removeAttribute("src");
  if (old.startsWith("blob:")) URL.revokeObjectURL(old);
}
function renderFollow() {
  const labels = {idle:"待启动", starting:"正在启动", following:"跟随中",
    stopping:"正在停止", error:"跟随异常", unmanaged:"此入口不控制跟随"};
  text("follow-status", "跟随 · " + (labels[follow?.phase] || "连接中断"));
  text("motion-target", Array.isArray(follow?.home_degrees) ?
    "启动动作 · 回零 [" + follow.home_degrees.map(q => q + "°").join(", ") +
    "] · 打开夹爪至 " + follow.gripper_open_mm + " mm" : "");
  const noAuth = follow?.authorized === false ? "本次会话没有系统授权，无法启动跟随。" : "";
  text("follow-detail", followError || (follow ?
    [noAuth, follow.detail, follow.log_path].filter(Boolean).join(" · ") : ""));
}
function fmtRate(value) {
  return Number.isFinite(value) ? Number(value).toFixed(1) + " FPS" : "—";
}
function size(metadata, config, stream) {
  const intr = metadata?.[stream + "_intrinsics"];
  const w = intr?.width ?? config?.width, h = intr?.height ?? config?.height;
  return w && h ? w + "×" + h : "—";
}
function renderPanel(tile, camera, role) {
  tile.camera = camera;
  tile.assigned = role;
  const mode = tile.mode.value;
  const online = camera?.fresh ?? capture?.camera_fresh;
  const key = camera && online ? role + "/" + camera.session + "/" + mode : "";
  if (key !== tile.key) {
    tile.key = key;
    for (const img of tile.images) clearImage(img);
  }
  tile.images[0].hidden = mode === "depth";
  tile.images[1].hidden = mode === "color";
  tile.panel.querySelector(".panel-images").classList.toggle("split", mode === "rgbd");
  if (!online || !camera) {
    tile.stats.textContent = camera?.error || "等待相机";
    return;
  }
  const f = camera.observed_fps, m = camera.metadata, c = camera.configuration;
  tile.stats.textContent = "RGB " + size(m, c, "color") + " · " +
    fmtRate(f?.color ?? f?.rgb) + "　深度 " + size(m, c, "depth") + " · " +
    fmtRate(f?.depth) + (Number.isFinite(camera.frame_age_s) ?
      "　帧龄 " + Math.round(camera.frame_age_s * 1000) + " ms" : "");
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
  text("telemetry-stats", "遥测 · " + (s?.robot_fresh ? "在线" : "未就绪") +
    " · " + (Number.isFinite(s?.robot_fps) ? s.robot_fps.toFixed(1) + " Hz" : "— Hz") +
    (Number.isFinite(s?.robot_age_ms) ? " · 更新时延 " + s.robot_age_ms + " ms" : ""));
  renderFollow();
  const last = s?.last;
  text("message", captureError || (action === "save" ? "正在检查并保存…" : "") ||
    s?.error || (last ? outcomes[last.result] + " · " + last.path +
      (last.error ? " · " + last.error : "") : "") || (!s?.camera_fresh && s?.camera?.error) || "");
  const cameras = s?.cameras || (s?.camera ? {default:s.camera} : {});
  const single = !!cameras.default;
  panels[0].name.textContent = single ? "相机" : "左相机";
  panels[1].panel.hidden = single;
  for (const tile of panels) {
    const role = single && tile.role === "left" ? "default" : tile.role;
    renderPanel(tile, cameras[role], role);
  }
}
function entryDetails(item) {
  return item.frames + " 帧 · " + item.fps + " FPS" +
    (item.cameras === 2 ? " · 双相机" : "") + (item.simulated ? " · 模拟" : "");
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
  const batch = catalog?.batch;
  if (batch?.episodes?.length || batch?.pending) el("task").value = batch.task;
  text("batch-status", batch ? "当前数据集 · " + batch.task + " · 已保存 " +
    batch.episodes.length + " 条 episode" + (batch.export_id ? " · 正在转换，跟随不停止" :
      batch.sealed ? " · 已结束录制，等待重试转换" : "") :
    "录制并保存至少一条 episode 后，可以结束数据集并转换。");
  const submitting = action === "export" || action === "finish-batch";
  text("dataset-status", exportError || (submitting ? "正在启动转换…" : status));
}
async function update() {
  if (updating || action) return;
  updating = true;
  const requestRevision = revision;
  try {
    const [s, d] = await Promise.allSettled([api("status"), api("datasets")]);
    if (requestRevision !== revision) return;
    capture = s.status === "fulfilled" ? s.value : null;
    if (!followAction) follow = capture?.follow || null;
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
  const conversion = id === "export" || id === "finish-batch";
  if (conversion) exportError = "";
  else captureError = "";
  updateControls();
  renderCapture();
  renderCatalog();
  try {
    const body = id === "record" ? {task:el("task").value} :
      id === "export" ? {episodes:[...selected]} : {};
    const result = await api(id === "finish-batch" ? "dataset/finish" : id, body);
    if (conversion) {
      catalog.job = result;
      if (id === "export") selected.clear();
    }
  } catch (e) {
    if (conversion) exportError = e.message;
    else captureError = e.message;
  } finally {
    action = "";
    renderCatalog();
    updateControls();
    await update();
  }
}
for (const id of endpoints) el(id).onclick = () => change(id);
el("motion-confirmed").checked = false;
el("motion-confirmed").onchange = updateControls;
async function changeFollow(start) {
  const button = el(start ? "follow-start" : "follow-stop");
  if (followAction || button.disabled) return;
  const confirmed = el("motion-confirmed").checked;
  if (start) el("motion-confirmed").checked = false;
  followAction = true;
  followError = "";
  revision++;
  updateControls();
  try {
    follow = await api(start ? "follow/start" : "follow/stop",
      start ? {motion_confirmed:confirmed} : {});
  } catch (e) {
    followError = e.message;
  } finally {
    followAction = false;
    renderFollow();
    updateControls();
    await update();
  }
}
el("follow-start").onclick = () => changeFollow(true);
el("follow-stop").onclick = () => changeFollow(false);
function replaceImage(img, value) {
  const old = img.src;
  img.src = value;
  if (old.startsWith("blob:")) URL.revokeObjectURL(old);
}
for (const tile of panels) {
  tile.mode.onchange = () => renderPanel(tile, tile.camera, tile.assigned);
  setInterval(async () => {
    const key = tile.key, mode = tile.mode.value;
    if (!key || document.hidden || tile.fetching || tile.panel.hidden) return;
    tile.fetching = true;
    try {
      const path = "/preview/" + tile.assigned + "/" +
        (mode === "rgbd" ? "rgbd.json" : mode + ".jpg");
      const response = await fetch(path, {cache:"no-store"});
      if (!response.ok || key !== tile.key) return;
      if (mode === "rgbd") {
        const images = await response.json();
        if (key !== tile.key) return;
        replaceImage(tile.images[0], "data:image/jpeg;base64," + images.color);
        replaceImage(tile.images[1], "data:image/jpeg;base64," + images.depth);
      } else {
        const blob = await response.blob();
        if (key !== tile.key) return;
        replaceImage(tile.images[mode === "depth" ? 1 : 0], URL.createObjectURL(blob));
      }
    } catch {
      // Device status polling reports disconnects.
    } finally { tile.fetching = false; }
  }, 170);
}
update();
setInterval(update, 750);
</script>
"""

_HOST = re.compile(r"(?:localhost|127\.0\.0\.1)(?::([0-9]{1,5}))?", re.IGNORECASE)
_DOWNLOAD = re.compile(r"/api/datasets/([A-Za-z0-9_-]{1,128})/download")
_PREVIEW = re.compile(r"/preview/(default|left|right)/(color\.jpg|depth\.jpg|rgbd\.json)")
_POST_FIELDS = {
    "/api/record": {"task"},
    "/api/save": set(),
    "/api/discard": set(),
    "/api/export": {"episodes"},
    "/api/dataset/finish": set(),
    "/api/follow/start": {"motion_confirmed"},
    "/api/follow/stop": set(),
}


class DatasetOperations(Protocol):
    def catalog(self) -> dict: ...
    def start(self, ids: list[str]) -> dict: ...
    def start_recording(self, task: str) -> dict: ...
    def finish_recording(self, operation: str) -> dict: ...
    def finish_dataset(self) -> dict: ...
    def archive(self, dataset_id: str) -> Path: ...


class OperatorServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self, port: int, collector: Collector, datasets: DatasetOperations | None = None
    ) -> None:
        from .follow_control import FollowClient

        self.collector = collector
        self.datasets = datasets
        socket_path = os.environ.get("FRANKA_CAPTURE_FOLLOW_SOCKET")
        self.follow = FollowClient(socket_path) if socket_path else None
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
            status = self.server.collector.status()
            follow = getattr(self.server, "follow", None)
            status = {**status, "follow": follow.status() if follow else {"phase": "unmanaged"}}
            self._json(200, status)
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
        elif match := _PREVIEW.fullmatch(self.path):
            cameras = getattr(self.server.collector, "cameras", {})
            if match[1] not in cameras:
                self._json(404, {"error": "camera is not configured"})
            else:
                self._preview(cameras[match[1]], match[2])
        elif self.path in ("/preview/color.jpg", "/preview/depth.jpg"):
            self._preview(self.server.collector.camera, self.path.rsplit("/", 1)[1])
        else:
            self._json(404, {"error": "unknown endpoint"})

    def _preview(self, camera, mode: str) -> None:
        try:
            frame = camera.frame()
        except RuntimeError as exc:
            self._json(503, {"error": str(exc)})
            return
        if mode == "rgbd.json":
            self._json(
                200,
                {
                    "color": base64.b64encode(frame.color_jpeg).decode("ascii"),
                    "depth": base64.b64encode(frame.depth_jpeg).decode("ascii"),
                },
            )
        else:
            image = frame.color_jpeg if mode == "color.jpg" else frame.depth_jpeg
            self._respond(200, image, "image/jpeg")

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
            if self.path in ("/api/follow/start", "/api/follow/stop"):
                starting = self.path.endswith("/start")
                if starting and body["motion_confirmed"] is not True:
                    raise ValueError("explicit motion confirmation is required")
                follow = getattr(self.server, "follow", None)
                if not follow:
                    self._json(404, {"error": "follow control is not configured"})
                    return
                if starting and collector.status()["phase"] != "idle":
                    raise CaptureError("finish the active recording before starting follow")
                try:
                    result = (
                        follow.start(motion_confirmed=body["motion_confirmed"])
                        if starting
                        else follow.stop()
                    )
                except RuntimeError as exc:
                    raise CaptureError(str(exc)) from exc
            elif self.path == "/api/dataset/finish":
                if datasets is None:
                    self._json(503, {"error": "dataset conversion is not configured"})
                    return
                result = datasets.finish_dataset()
            elif exporting:
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
                finish = datasets.finish_recording if datasets is not None else collector.finish
                result = finish(self.path.rsplit("/", 1)[1])
        except (ValueError, RecursionError, TimeoutError) as exc:
            self._json(400, {"error": str(exc)})
        except CaptureError as exc:
            self._json(409, {"error": str(exc)})
        except OSError as exc:
            self._json(500, {"error": str(exc)})
        else:
            self._json(202 if exporting or self.path == "/api/dataset/finish" else 200, result)
