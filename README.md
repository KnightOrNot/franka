# Franka 数采

在工控机上为现有 Franka 主从臂跟随链路录制关节、夹爪和双 RealSense RGB-D。
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
真机站点配置，使用独立的遥测 socket 和 `data/sim/`。使用 `--simulate --dual-camera` 可预览两路。填写任务名，反复录制、保存或丢弃。
点击「结束数据集并转换」，后台只转换当前批次已保存的回合。待后台转换
完成后，从「已转换数据集」点击「下载 ZIP」。转换期间不能开始录制。

在工控机按照[部署说明](docs/deployment.md)准备好站点配置后，日常只需运行：

```bash
./scripts/start-session.sh
```

不需要另跑 1–4。启动命令如有需要，只在开放网页前询问一次系统 sudo 授权。
网页打开后，核对回零目标并勾选现场安全确认，再点「启动跟随」。此后无需回终端确认。
本批保存完后，点「结束数据集并转换」。
转换不会停止跟随。「停止跟随」只结束控制链，网页和预览保持可用。

网页只监听工控机的 <http://127.0.0.1:8078>，本地电脑通过 SSH 端口转发访问。
结束整次会话前先保存或丢弃当前回合，再按 `Ctrl+C` 并等夹爪连接关闭。退出时不会
主动开合夹爪或归零。只看预览或转换历史数据
可用不控制跟随的 `./scripts/start-capture.sh`。

- [工控机安装、SSH 访问、现场验收和回退](docs/deployment.md)
- [原始回合、ZIP 格式和离线命令](docs/episodes.md)

开发检查用 `make verify`。`make test-export` 会实际导出并用 LeRobot 回读数据集。
