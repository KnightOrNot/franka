# Franka 数采

在工控机上为现有 Franka 主从臂跟随链路录制关节、夹爪和 RealSense D435i RGB-D。
浏览器可预览、录制和保存回合，把选中的同任务回合转换为 LeRobot v3，并下载包含
RGB、8 维状态和动作、原生 Z16 深度和标定的 ZIP。数采运行在独立的 Python 3.12
环境，只接收遥测。工控机另有一键入口，负责启动已验证的单组 FR3 跟随。

## 快速开始

需要 Linux、Git、Python 3.12 和 uv。从 GitHub 通过 HTTPS 安装，首次同步需要联网。

```bash
git clone https://github.com/KnightOrNot/franka.git
cd franka
./scripts/install-capture.sh
./scripts/start-capture.sh --simulate
```

用浏览器打开 <http://127.0.0.1:8079>。模拟不需要 USB、机械臂或桌面环境，不读取
真机站点配置，使用独立的遥测 socket 和 `data/sim/`。填写任务名并保存回合后，
在「已保存回合」勾选一个或多个同任务回合，点击「转换为 LeRobot」。待后台转换
完成后，从「已转换数据集」点击「下载 ZIP」。转换期间不能开始录制。

在工控机按照[部署说明](docs/deployment.md)准备好站点配置后，日常只需运行：

```bash
./scripts/start-session.sh
```

不需要另跑 1–4。终端会先显示从臂的回零目标和现场确认提示，再依次启动数采、跟随和
遥测。等终端提示遥测就绪，在网页开始录制。网页始终只监听工控机的
<http://127.0.0.1:8078>，本地电脑通过 SSH 端口转发访问。在网页保存后再按
`Ctrl+C` 退出整次会话。若只想查看相机或转换旧数据，用不启动跟随的
`./scripts/start-capture.sh`。

- [工控机安装、SSH 访问、现场验收和回退](docs/deployment.md)
- [原始回合、ZIP 格式和离线命令](docs/episodes.md)

开发检查用 `make verify`。`make test-export` 会实际导出并用 LeRobot 回读数据集。
