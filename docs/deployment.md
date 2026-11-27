# 工控机部署

数采服务只接收遥测。根目录单组 FR3 部署可以用一个前台命令启动数采和跟随。
安装不会启用机械臂或开机自启。其他接线需继续沿用现场验证的启动方式。

## 1. 从 GitHub 安装

需要 Linux、Git、Python 3.12 和 uv。用运行数采的普通用户操作。升级时先在新目录
准备经审核的版本，保留现用采集仓库和环境，便于回退。下面的 `REVIEWED_CAPTURE_REF`
填写要部署的 GitHub 提交或标签。核对 `rev-parse` 输出，再安装：

```bash
capture_repo="$HOME/projects/franka-next"
git clone https://github.com/KnightOrNot/franka.git "$capture_repo"
git -C "$capture_repo" checkout --detach "$REVIEWED_CAPTURE_REF"
git -C "$capture_repo" rev-parse HEAD
cd "$capture_repo"
./scripts/install-capture.sh
```

安装器在此仓库的 `.venv` 运行 `uv sync --locked --no-dev --extra camera --extra export`，
包括固定的相机与 LeRobot 版本。如果 `uv` 未加入 `PATH`，可用
`UV="$HOME/.local/bin/uv" ./scripts/install-capture.sh` 指定已安装的可执行文件。
安装不会启动数采。先执行 `./scripts/start-capture.sh --simulate`，可在
<http://127.0.0.1:8079> 测试录制、转换和下载，而不占用真机的 socket、数据目录或相机。
从另一台电脑进行模拟验收时，按下节的方法转发 8079 端口。

## 2. 沿用站点配置并启动

首次部署时复制模板，再填入实际采集盘、桥共用的 socket、D435i 序列号和经现场验证的串流配置：

```bash
mkdir -p config
test -e config/site.env || cp scripts/site.env.example config/site.env
```

已有部署无需重建配置。新目录可通过 `FRANKA_CAPTURE_CONFIG` 指向原有 `site.env`，
或保留同样的设置在新目录的 `config/site.env`。该文件是站点维护的 Bash 环境文件，
不入 Git。建议数据目录和 socket 都用绝对路径，避免切换仓库后指向不同位置。

| 站点变量 | 作用与默认值 |
| --- | --- |
| `FRANKA_CAPTURE_SOCKET` | 和桥共用的 Unix socket，父目录由同一用户拥有且仅该用户可访问。未设置时根据用户运行目录选择 |
| `FRANKA_CAPTURE_CAMERA_SERIAL` | 单相机模式的 D435i 序列号 |
| `FRANKA_CAPTURE_DUAL_CAMERA` | 设为 `1` 启用双相机 RGB-D |
| `FRANKA_CAPTURE_LEFT_SERIAL`、`FRANKA_CAPTURE_RIGHT_SERIAL` | 双模式的左右相机序列号。两项同时指定且不相同，或同时留空以序列号排序识别恰好两个设备 |
| `FRANKA_CAPTURE_WIDTH`、`FRANKA_CAPTURE_HEIGHT`、`FRANKA_CAPTURE_FPS` | 相机模式，默认 640×480、30 FPS。以实际 USB 链路和设备 profiles 为准 |
| `FRANKA_CAPTURE_DATA_ROOT` | 原始回合目录，默认 `data/raw` |
| `FRANKA_CAPTURE_DATASET_ROOT` | 转换结果与 ZIP 的存放目录，默认在原始目录旁的 `lerobot` 目录。例如 `data/raw` 对应 `data/lerobot` |
| `FRANKA_CAPTURE_HTTP_PORT` | 环回网页端口，默认 8078 |
| `FRANKA_CAPTURE_CONTROL_REVISION` | 实际跟随版本或站点校验 ID |
| `FRANKA_CAPTURE_STALE_MS`、`FRANKA_CAPTURE_MAX_SKEW_MS` | 遥测过期和相机/遥测配对窗口，默认 350 ms 和 35 ms |
| `FRANKA_CAPTURE_MIN_FREE_MB`、`FRANKA_CAPTURE_MAX_SECONDS` | 录制磁盘空间阈值和回合时长上限，默认 1024 MB 和 300 秒 |

并排部署时，可先在启动数采的终端执行
`export FRANKA_CAPTURE_CONFIG="$HOME/projects/franka/config/site.env"` 指向原配置。
确认旧采集服务或独占 D435i 的预览已按站点流程退出，再运行：

```bash
./scripts/start-capture.sh
```

留着启动终端。程序打印实际网页 URL，始终只监听工控机的 `127.0.0.1`。在另一台电脑
的另一个终端运行以下命令，并将 `user@controller` 换成该站点的 SSH 登录目标。如更改了
站点端口，将 `-L` 末尾的 8078 改成实际端口。

```bash
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8078:127.0.0.1:8078 user@controller
```

在这台电脑的浏览器打开 <http://127.0.0.1:8078>。不需要在控制器上开桌面，也不需要
把网页暴露到局域网。通过 `start-capture.sh` 打开的页面只控制录制和转换，不控制机械臂。

## 3. 保留原有遥测桥和跟随启动流程

已有遥测的部署直接沿用现有桥，不重复应用补丁。只在首次接入时检查以下步骤。
补丁根据 2026-11-26 的 `~/frankateleop` 两个文件生成，部署前先将源文件纳入版本记录
或备份。核对 `fr3.py` 的夹爪反馈是 `width / 0.09`，命令是
`0.09 * (1 - requested[-1])`，且源是单组 FR3 跟随。

```bash
capture_repo="$PWD"
teleop_root="$HOME/frankateleop"
git -C "$teleop_root" apply --check "$capture_repo/patches/frankateleop-run_env.patch"
```

从采集仓库根目录执行上述检查。检查通过并审核差异后，在跟随程序退出期间应用：

```bash
git -C "$teleop_root" apply "$capture_repo/patches/frankateleop-run_env.patch"
install -m 0644 "$capture_repo/src/franka_capture/bridge.py" \
  "$teleop_root/teleop/experiments/franka_capture_bridge.py"
```

在运行现有第 4 步脚本的终端，读取同一份站点配置或导出相同的 `FRANKA_CAPTURE_SOCKET`。
沿用现场确认的左／右臂脚本、串口映射和原有 1–4 步启动流程，保持旧 Pygame 逐帧
pickle 录制开关关闭。不设置此环境变量时仍走原来的跟随路线，遥测不启用。
已经应用的桥不需要随数采更新，也无需重启 Polymetis 底层服务。

| 源文件 | 生成补丁时的 SHA256 |
| --- | --- |
| `teleop/experiments/run_env.py` | `820bfd30b7635c547b3b42b2bcfe6abfd17213550d861db733fe5f1c0b1bc05e` |
| `teleop/teleop/env.py` | `2d03c3595154ebe31bd1a056bb4661176fabe615efb126b9b6eb96c7e03187d2` |

补丁冲突时先比较两份源文件，不强制应用。

## 4. 根目录单组 FR3 一键启动

这个入口对应在工控机确认过的 `~/frankateleop` **根目录** 1–4 配置。
它使用现有 Polymetis `polymetis` Conda 环境、从臂标定和根目录端口
50051、50052、6001，不适用于左/右目录的其他配置。

一次性准备（从数采仓库根目录运行）：

```bash
export FRANKA_CAPTURE_CONFIG="$HOME/projects/franka/config/site.env"
./scripts/install-session.sh

base="$HOME/projects/franka"
python3 scripts/build-managed-gripper.py \
    --teleop-root "$base/third-party/frankateleop" \
    --sdk-root "$base/third-party/libfranka" \
    --polymetis-prefix "$base/.runtime/polymetis"

./scripts/start-session.sh --check
```

安装器要求第 3 节的遥测桥已经接入，校验实际的两个 Python 源文件，再生成专用的
`launch_robot_capture.py` 和当前版本目录下 `.native/follow/` 的跟随入口与桥模块。
它不会改写现有 `1*.sh` 到 `4*.sh` 或先前的 Python 入口。已安装相同内容时重复执行无影响。
非默认路径可以在 `site.env` 设置 `FRANKA_TELEOP_ROOT`、`FRANKA_CONDA_SH`；
安装时若用非默认控制树，向安装器也传入 `FRANKA_TELEOP_ROOT`。
构建命令使用项目内已有的 Polymetis CMake 构建、libfranka 和运行环境，
仅生成独立客户端 `.native/franka_hand_capture_client`，不会连接机械臂。每个新部署目录构建一次。

第一次替换手动流程时，保存当前回合并正常退出旧数采和 1–4 进程。若有端口冲突，
`--check` 会报告端口，不会杀掉现有进程。之后每次在工控机终端只运行：

```bash
./scripts/start-session.sh
```

1. 如果系统需要，在启动命令时输入一次 sudo 密码，随后网页会显示相机预览。网页
   显示从臂 home 关节角和夹爪打开目标。现场确认工作区和急停后，勾选确认并点击
   「启动跟随」，无需回到终端。网页不会接收系统密码。如果授权不可用，仍可预览，
   但网页不能启动跟随。
2. 终端依次报告从臂反馈、夹爪反馈、节点、启动对齐和主从跟随。对齐期间让主臂保持
   稳定。七个关节的指令变化上限为每轴 0.25 rad/s，最多 20 Hz，最多等待 8 秒。收到
   连续合格的反馈才进入跟随。此阶段沿用从夹爪当前开度，开度差太大时提示先匹配。原有
   超差保护保留。网页和英文 CLI 会报告超差轴号或夹爪开度、实际差值和独立日志路径。
3. 等遥测和两路相机就绪后，用同一任务名反复开始、保存或丢弃单条 episode。「结束
   数据集并转换」会关闭本批并排队转换，不会停止跟随。「停止跟随」只退出跟随链，
   网页和相机保持运行。不要把网页按钮当急停。退出整次会话前先保存或丢弃，再
   `Ctrl+C`，并等待退出完成。两种停止方式都先结束跟随命令链，让已下发的夹爪动作收尾，
   确认 SDK 连接释放后再关闭夹爪服务。退出不会主动开合、归零或解除夹持力。正常启动
   的夹爪初始化保持不变。若日志报告未确认断开，先排查 `gripper.log`，不要直接重启跟随。
4. 遇到失败查看终端上报出的日志路径。日志包含各阶段输出。要回退，退出一键会话后，
   使用原来的 1–4 和 `start-capture.sh`，不需要还原任何控制文件或迁移数据。

## 5. 现场验收与回退

1. **先不运行跟随。** 核对网页 RGB 和深度预览、实际串流帧率和 USB 链路。
   选取已知完整的原始回合，验证转换和下载，再在另一台机器打开 ZIP。只选择一个或
   多个同任务的完整回合。点击「转换为 LeRobot」后等待后台任务完成，点击「下载 ZIP」。
   同时只运行一个转换任务，转换和录制不能重叠。
2. **跟随基线。** 现场操作员确认设备和工作空间后，按既有流程启动。比较不录制和
   约 10 秒录制时的跟随周期、超时和相机掉帧，确认夹爪开闭方向、第 7 轴和配对偏差。
   每套 640×480、30 FPS RGB-D 的未压缩参考约 46 MB/s，两路约 92 MB/s。
   实际 PNG 写盘、USB 和相机帧率需按现场内容实测，
   也要为数据集和 ZIP 留足空间。
3. **检查与回读。** 用 `check` 和 `replay` 核对周期、配对时间和帧数。保存一段、丢弃
   一段，确认丢弃回合不能导出。人工核对深度非零比例和作业视野。CLI 与 ZIP 格式见
   [数据说明](episodes.md)。
4. **回退。** 在录制和转换结束后用 `Ctrl+C` 停止本次前台服务，再用保留的原仓库、
   环境和站点配置运行此前的采集启动命令。无需修改桥或跟随脚本，不移动或覆盖已保存
   的回合。若确实要撤销首次桥接，在跟随退出后用 `git apply --reverse --check` 先检查，
   再使用 `git apply --reverse` 或站点备份。保留桥模块文件不会运行它。

若磁盘错误后网页提示相机写入器未收尾，恢复空间再重启数采服务。失败或超时回合保留
为 `.partial`，不要手工改成完整回合。

## 项目内的控制环境

控制环境复制或安装到项目内后，可在站点配置中设置 `FRANKA_POLYMETIS_PREFIX`，
指向含有 `bin/python` 的环境根目录。设置后不再读取外部 `conda.sh`，
也不从调用终端继承 Python 和动态库搜索路径。配合 `FRANKA_TELEOP_ROOT` 指向
项目内的 `third-party/frankateleop`。该 Python 环境的 Polymetis 和 Teleop
包绑定、原生客户端及其动态库也必须指向项目内的实际文件。仅复制已有构建目录
不会重写 ELF 路径。未设置此前缀时仍支持原有 Conda 环境。

站点数据目录和遥测 socket 可分别通过 `FRANKA_CAPTURE_DATA_ROOT`、
`FRANKA_CAPTURE_DATASET_ROOT`、`FRANKA_CAPTURE_SOCKET` 指向项目内的专用目录。
外部代码、单独部署的运行环境和站点数据不需要进入 Git。
