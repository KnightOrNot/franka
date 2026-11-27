# 原始回合与导出

默认一个完整回合位于 `data/raw/<episode>`，也可由站点配置改用独立的采集盘。回合包含：

| 文件 | 内容 |
| --- | --- |
| `manifest.json` | 模式、任务、控制版本、单调时间边界、通道定义、审计统计和结果 |
| `robot.jsonl` | 同一控制周期的下发前状态、主手要求的动作、序号、源 epoch 和时间 |
| `camera/manifest.json` | D435i 的实际配置、设备、标定和原生深度比例 |
| `camera/frames.jsonl` | RGB / 深度各自的帧号、SDK 时间戳和时钟域及主机接收时间 |
| `camera/rgb/`、`camera/depth/` | 同一 D435i pipeline 输出的无损 RGB8 和原生 uint16 Z16 PNG |
| `camera/COMPLETE.json` | 相机库的写入结果，由总回合校验二次核对 |

双相机原始格式 `franka_capture_v2` 把同样的相机文件分别写在
`cameras/left/` 和 `cameras/right/`。两路使用同一个控制周期记录和公共录制边界。
单相机格式 `franka_capture_v1` 不变，旧回合仍能读取和转换。

Z16 值乘 `depth_scale_m_per_unit` 得米。0 表示无有效深度。原生深度仍处在深度相机的
视角，不能用彩色图的像素坐标直接读取。校验会解码像素，并核对格式、分辨率、索引和
SHA256，深度预览不会写回训练像素。

## 机器人字段

`feedback` 和 `requested` 各保留来源的 8 维原始向量。前 7 轴是弧度。
`feedback[-1]` 是开口比例，`requested[-1]` 是闭合比例。
导出的 `observation.state` 末位是 `0.09 * feedback[-1]` 米，
`action` 末位是 `0.09 * (1 - requested[-1])` 米。
动作是提交给从臂接口的目标，即使 RPC 返回也不代表已到达。原程序的速度占位和全零末端
位姿不进入数据。真正的传感器测量、速度或位姿需要在后续版本另行定义与核验。

## 时间和边界

相机和遥测必须在同一台 Linux 主机上。首版使用工控机单调时钟上的 **主机接收时间**
对齐相机和从臂反馈。相机库的单调与墙钟时间在不同处理步骤采样，不能当作精确的
设备时钟映射。保留 SDK 原始时间戳和时钟域，曝光时间不确定度记为未知。
默认 35 ms 是接收时基的配对窗口，不是硬同步误差。

相机写入开始后才开启公共录制边界。保存时冻结机器人输入，以最后一个已接收的下发
时间为公共结束边界，同时记录按下保存的时间。相机文件可以含有边界外的早到或晚到帧，
导出会排除这些帧。每张用于训练的图匹配一条完整控制周期，目标提交时间不早于
该图的接收时间。序号跳跃、重复、重启、时钟不前进、失联和文件错误使当前回合失败。
丢弃和失败回合保留 `.partial`，不会进入导出。

## 网页转换和 ZIP

点击「结束数据集并转换」会封存当前批次，仅转换这一批已经保存的 episode。丢弃、
失败或历史同名回合不加入。同一批失败可重试，原始回合不改写或删除。也可以在
「已保存回合」手动选择历史回合转换。转换与录制互斥，转换期间跟随继续运行。
在「已转换数据集」点「下载 ZIP」。

一个数据集只能包含相同采集模式和相机配置的回合。如果变更了分辨率或帧率，分开转换。
LeRobot 使用采集配置的均匀帧率。每个目标时刻选一张足够接近且未使用的图像，并保留
同一周期的状态和动作。若存在间隙，转换失败，不复制帧来补齐。

解压 ZIP 后，`dataset/` 可直接用 LeRobot v3 打开。训练的 RGB 视频是
`observation.images.front`（单相机），或 `observation.images.left` 与
`observation.images.right`（双相机）。机器人字段是 8 维 `observation.state` 和 `action`。
额外的深度和标定作为配套文件，不是 LeRobot 视频列。

| `dataset/` 中的文件 | 内容 |
| --- | --- |
| `depth/episode-<索引>/*.png` | 每条训练样本对应的原生 uint16 Z16 PNG，逐字节保留源文件，未投影到 RGB 视角 |
| `franka_alignment.jsonl` | 用 `episode_index`、`frame_index` 或 `dataset_index` 找训练样本，`depth_image` 指向数据集内的深度 PNG，并记录原始帧、控制周期和时间 |
| `franka_source.json` | 按 `episode_index` 记录来源回合、校验摘要和相机标定，包括彩色和深度内参、深度到彩色外参和 Z16 比例 |

双相机 Z16 在 `depth/episode-<索引>/left/` 与 `right/`。
双相机对齐和标定字段以 `cameras` 下的左右角色分别记录。

读取训练样本及其深度不需要访问原始回合目录。ZIP 只包含入选的深度帧，原始回合仍是
完整的采集记录。

## 离线检查或导出单回合

在已安装的采集仓库中运行：

```bash
.venv/bin/franka-capture check /path/to/episode
.venv/bin/franka-capture replay /path/to/episode
.venv/bin/franka-capture export /path/to/episode /path/to/new-dataset --fps 15
```

`replay` 只输出配对的图像路径、状态和动作，不向机械臂发命令。CLI 单回合导出需使用
新目录，不会覆盖数据。它保留原来的 RGB／状态／动作、对齐记录和来源摘要格式，原生
Z16 留在源回合。需要可独立移交的深度时，使用网页 ZIP。`--fps` 不应高于实际采集帧率。

导出会在连续、接近相机标称帧率时按实际接收节拍拟合时间网格，防止长期微小漂移被误报为缺帧。LeRobot 时间戳仍以输出帧率标记，实际采样时间保留在对齐索引中。
