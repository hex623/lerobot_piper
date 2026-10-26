# 本机 Piper 主从示教接入说明（两臂共用一路 CAN）

已克隆 `https://github.com/brucecai-2001/lerobot_piper`，使用真正包含 Piper 驱动的上游 `piper` 分支，源提交为 `06ce135cca3c583bc17a1ea2e972c6a78339736f`。本机接入以 `piper-base` 维护，首个开发基线标签为 `baseline-shared-can-2026-10-26`；这不是完整功能已验收的发布版本。

## 已完成的实机检查

- Ubuntu WSL2，独立 Python 3.11 环境 `/home/heexu/.venvs/lerobot-piper`，完整 editable 安装；PyTorch 为 CPU 版。
- `can0`，1 Mbps，candleLight 序列号 `001C00274648570A20303731`。
- 固件查询中观察到 `Piper_X_` 和 `S-V1.9-0`。共享 CAN 应答不提供唯一设备身份。
- 通过 `PiperRobot.capture_observation()` 连续读取 60 组观测：状态 7 维，六轴为 rad、夹爪为 m；`head` 和 `hand` 都是 640×480 RGB。
- D455 的 RGB 当前为 `/dev/video4`，Dabai 当前为 `/dev/video6`。本机配置使用 `/dev/v4l/by-id/` 路径，避免重插后数字变化。Dabai 必须显式使用 MJPG；默认 YUYV 在这套 USB/IP 连接上出现了绿屏和撕裂，修改后实际画面已检查。
- 观测检查期间 CAN 接口发送计数增量为 0。单独的固件/限位诊断会发送查询帧 `0x4AF` 和 `0x472`，不发送使能、复位或动作。
- 21 项连接/单位/共享示教/动作拦截回归检查通过；依赖检查无冲突，控制和训练 CLI 帮助可正常启动。H.264 编码、ffprobe 元数据读取、PyAV 和 LeRobot 解码往返检查通过（4 帧，输出形状 `[4, 3, 48, 64]`）。
- 已结束原先持续报设备断开错误的 `dual_control.py` 进程（PID 56255），避免设备重连时与新控制入口竞争。

结果在 `outputs/local-preflight-final/report.json`、`observations.jsonl`、`head.png`、`hand.png`；参数应答原始证据在 `outputs/device-query.json`。

## 当前主从示教模式

用户已确认两臂共用 `can0`，主臂可拖动、从臂正常跟随。实际拖动期间已捕获 `0x151`、`0x155/156/157` 和 `0x159`，从臂反馈位于 `0x2A1`、`0x2A5/2A6/2A7`、`0x2A8`。主臂 `0x151` 数据为 `010164ad00000000`，即 CAN 关节目标和硬件高跟随模式。

同一次限位查询收到 J2 下限 0° 和 −5° 两种回复，符合共总线有两个应答对象的现场配置。共享示教入口允许旁路采集，**不要求拆成两路 CAN**。广播配置/查询不能唯一归属到某一条臂，因此程序不会对共总线写入角色、使能、回零或策略动作。

`action` 来自实际收到的主臂关节/夹爪目标；`observation.state` 来自从臂反馈。两者分别解码为 rad/m。主臂静止时可能停止发送，已知目标按各关节对分别锁存，真实报文时间和年龄保存在 `provenance.jsonl`，不会因读取缓存而更新时间。尚未收到任何一组关节目标或夹爪目标时，会等待操作，不用从臂反馈补造动作标签。普通 CAN 报文不包含发送设备身份，来源解释以当前已确认的硬件主从配置为前提。

本地 `SharedCANPiperBus` 全程只接收，硬件主臂继续控制从臂。原单臂电脑发令入口保留独立保护，不适用于共享示教。

本地运动保护仅支持已核对限位的 S-V1.9-0 水平正装配置：默认速度 2%，上限 5%，关节单步上限 1°，夹爪默认禁止动作。停止发送不等于硬件急停；真实运动仍需要现场确认和独立急停条件。

上述速度限制只作用于单臂电脑发令入口；共总线示教不改写硬件主臂原有跟随速度。

## 开始正式示教采集

PowerShell 在项目根目录运行：

```powershell
& .\piper_scripts\record-shared.ps1 -Task '将笔帽盖到笔上' -Duration 30
```

出现等待提示后，缓慢移动主臂并操作一次夹爪，使程序收到全套目标；随后自动采集一段 30 秒回合。默认 10 Hz，可通过 `-Fps` 调整。输出为新的 `data/shared_can_日期_时间/` 文件夹，包含 LeRobot `dataset/`（Parquet 和双路 MP4）、原始动作来源 `provenance.jsonl`、图像预览和 `report.json`。不覆盖旧目录，不上传云端。每次运行采集一个回合，重复运行可得到多个独立回合目录；合并前需核对任务、键名、统计量和人工结果标签。

报文源、图像及回合结果尚需人工复核，输出默认 `training_eligible=false`。确认示教成功与数据质量后再纳入训练，不能将连接测试当作任务成功示范。

已有连接测试保存于 `outputs/shared-can-smoketest-v2/`：90 帧、1 个本地回合，动作使用同一持续监听记录中真实捕获并锁存的目标，观测为当时实时状态和双相机；CAN TX 增量为 0。该回合标为连接测试，目标报文年龄保留在来源日志中，不能用于任务训练。拖动期间的实际指令和从臂角度变化统计在 `outputs/shared-can-listen-continuous/summary.json`。

`outputs/shared-can-smoketest-v2/readback.json` 验证了 90 帧、首/中/末帧的双视频和七维状态/动作均可从 LeRobot 重新加载。30 Hz 测试中相机存在重复帧，Dabai 的实际新帧数量低于标称值；正式入口因此默认 10 Hz，并记录每路真实帧时间、独立帧数和实际采样间隔。图像和关节是主机时间对齐，不宣称硬件同步。

## 一键重新检查（PowerShell）

在项目根目录执行：

```powershell
& .\piper_scripts\connect-local.ps1
```

脚本按序列号查找指定 USB-CAN，附加 D455 和 Dabai 到 WSL，检查 CAN 身份和波特率，再运行只读观测检查。它不会自动改写机械臂角色或限位。

已共享的设备可直接附加；若更换了尚未共享的 USB 设备，需要先按 `usbipd list` 的实际 BUSID 完成 Windows USB/IP 绑定。未连接、多个候选或适配器身份不匹配会直接报告错误。

## Ubuntu 中使用

```bash
source /home/heexu/.venvs/lerobot-piper/bin/activate
cd /mnt/c/Users/heexu/Documents/Codex/2026-10-26/https-github-com-brucecai-2001-lerobot/lerobot_piper
python -m piper_scripts.preflight --profile piper_scripts/local-hardware.json --samples 60
python -m piper_scripts.query_device
python -m unittest piper_scripts.test_connection piper_scripts.test_shared_can -v
python -m piper_scripts.record_shared_can --task '将笔帽盖到笔上' --duration 30
python -m lerobot.scripts.control_robot --help
python -m lerobot.scripts.train --help
```

相机的角色名与旧项目默认 `one/two` 不同。本机配置用 `head/hand`，加载已有模型前必须核对模型期望的键名、图像变换、关节单位及归一化统计。原仓库将关节观测直接保存为毫度、动作却使用 rad；本地已统一为 rad/m，不能未经换算直接复用旧数据的统计量。

## 重建环境

`bash piper_scripts/setup-local.sh` 使用已有的 Python 3.11 重建独立环境，依赖约束在 `constraints-local.txt`，实际安装版本记录在 `requirements-installed.txt`。系统还需要 `can-utils`、`v4l-utils`、`ffmpeg`（包含 ffprobe）。原依赖名称 `pyav` 已修正为 `av`，视频编码默认使用可用的 `libx264`。

连接/断开已去掉自动回零、100% 速度和自动失能。退出读取程序保留机械臂原有使能状态，不改变保持力。

## 训练任务的分支与运行记录

采用“一项新训练任务一个分支，同一任务多次调参使用不同运行编号”的约定。基线说明在 `experiments/baseline.json`，后续协作约定在根目录 `AGENTS.md`。

新任务从公共分支开始，例如笔帽任务：

```bash
git switch piper-base
git switch -c task/pen-capping
```

以上是命名示例；尚未指定新的训练任务时，停留在 `piper-base`。`piper-base` 可随公共修复更新，固定标签 `baseline-shared-can-2026-10-26` 保留最初开发快照。要进行严格的同基线对照，可从该标签创建任务分支。

建议每个任务在 `experiments/<task-id>/` 保存任务配置和小型运行清单。同一任务的调参使用 `run-001`、`run-002` 等编号；从 `experiments/run-template.json` 复制模板并填写实际代码 SHA、数据版本/指纹、训练配置、随机种子、环境、检查点与评估结果。

大型产物继续保存在 `data/<task-id>/...`、`outputs/train/<task-id>/<run-id>/...` 等目录，Git 只记录可追溯的引用和小型结果。切换分支不会隔离这些被忽略的文件，也不会切换 Python 环境；每次运行须使用独立路径。当前模板是手工记录入口，不是已经接通的训练调度器或数据合并工具。

通用驱动/采集修复归入 `piper-base`，再更新任务分支；任务专属配置和结果留在各自分支。评估时固定任务内的数据划分、成功标准和执行参数；不同任务的 loss 或成功率不能直接当作同一个指标比较。
