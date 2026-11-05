# rebotARMCtl

机械臂采集部署平台与 STM32 同步控制器，面向 reBot Arm B601-RS 与 Star Arm 102。

> **当前状态：软件、固件和 PCB 设计已完成，但都还没有接触过真实硬件。** 固件只在 PC 上的仿真器里运行过，STM32 镜像只完成了编译和结构检查，电路板还没有打样。第一次接机械臂前，请按 [docs/bringup.md](docs/bringup.md) 一步步做。

## 没有硬件时先跑起来

需要 [uv](https://docs.astral.sh/uv/) 和 Node.js。

```bash
uv sync
cd apps/lab/frontend && npm ci && npm run build && cd ../../..
uv run rac up --sim
```

打开 <http://127.0.0.1:8000/controller>。`--sim` 用的是真实的固件核心加一个机械臂仿真器，协议和总线时序都按真实设备走。接上控制器后，把 `--sim` 去掉即可。

## 能做什么

| 需求 | 功能 | 位置 |
| --- | --- | --- |
| 1.1 设备连接 | 控制器发现和重连、端口型号识别、机械臂绑定、相机预览和保存绑定 | 控制台「设备」页 |
| 1.2 关节校准 | 按序列号保存的校准版本，零位、方向、行程范围和夹爪标定，单关节验证 | 「校准」页 |
| 1.3 主从同步 | **在 STM32 上本地闭环**，PC 的卡顿不影响跟随；PC 断开 500 ms 后全部机械臂进入保持 | `firmware/core` |
| 1.4 可视化示教 | URDF 三维视图、关节滑块和拖动、末端拖动（PC 上解 IK）、示教点、RS 拖动示教 | 「示教」页 |
| 1.5 模型部署 | LeRobot 和 StarVLA 后端，模型放在独立进程里 | `packages/rac-policy` |
| 1.6 人工干预 | 接管和交还，过期令牌和旧代次的指令一律被拒绝 | 「模型」页 |
| 1.7 真机回放 | 从任意时间点回放，先缓慢靠近起点 | 「回放」页 |
| 1.8 统一运行控制 | 所有页面共用的状态条，停止和急停始终可用 | `RunBar` |
| 2.1–2.5 数据 | LeRobot Dataset v3 录制、原始时间线、标注、导入导出合并、浏览 | `packages/rac-data` |
| 3.1–3.3 控制器 | 2 路 CAN、2 路 UART、1 路 USB，带版本的 RAC-Link 协议，Python SDK | `firmware/`、`packages/rac-link` |

基础配置是 2 台 reBot RS 从臂加 2 台 Star 主臂。原有 MakerMods Lab 的页面和流程也保留了。`controller` 类型的机器人可以在 Lab 里遥操作和录制；回放和模型部署请用控制台页面，Lab 会直接提示，因为它的缓入逻辑按角度计算，不适用于这里以弧度工作的机械臂。

## 目录

```
apps/lab/             MakerMods Lab（已内嵌）加控制器页面和路由
packages/rac-link/    协议、传输层、Python SDK
packages/rac-service/ Controller Service，USB 的唯一持有者
packages/rac-kinematics/  URDF、正逆运动学、轨迹
packages/rac-camera/  相机进程和共享内存
packages/rac-data/    录制、LeRobot 数据集、标注
packages/rac-policy/  模型进程
packages/rac-lerobot/ LeRobot 的 Robot 和 Teleoperator 代理
firmware/core/        可移植的固件核心
firmware/port/        stm32g4（真机）和 sil（仿真）
hardware/sync-controller/  硬件选型、KiCad 工程、制造文件
tools/                协议生成、固件构建
```

设计说明见 [docs/architecture.md](docs/architecture.md)，协议见 [docs/protocol.md](docs/protocol.md)，硬件见 [hardware/sync-controller/README.md](hardware/sync-controller/README.md)。

## 常用命令

```bash
uv run pytest tests                              # 全部集成测试
uv run python tools/build_firmware.py test       # 固件核心的行为测试
uv run python tools/build_firmware.py stm32      # 交叉编译（先运行 tools/fetch_firmware_deps.py）
uv run python tools/gen_protocol.py              # 改协议后重新生成 C 和 Python
uv run python hardware/sync-controller/gen/make_all.py  # 重新生成并检查整块板子
PYTHONUTF8=1 uv run python tests/e2e/run_web_e2e.py --out <目录>  # 浏览器走查（先启动平台）
```

交叉编译用的是 `ziglang`，由 uv 管理，不需要另装 ARM 工具链。硬件部分需要 KiCad 10，路径不同时设置 `KICAD_CLI` 和 `KICAD_PYTHON`。

Lab 自带的测试要在 `apps/lab` 目录下运行，并设置 `PYTHONUTF8=1`。Windows 上有 72 个测试失败，引入控制器之前就是这 72 个，主要和路径、进程管理的平台差异有关。

## 许可

本仓库代码使用 Apache-2.0，`assets/robots/rebot_b601_rs` 沿用其 CERN-OHL-W-2.0，内嵌的 Lab 保留其上游许可。复用了哪些项目、以什么方式复用、还有哪些授权需要确认，都写在 [docs/LICENSE_AUDIT.md](docs/LICENSE_AUDIT.md)。
