# 上游复用与许可审计

需求文档 4.1 节要求“RS 代码复用前确认其仓库许可”。下表是 2026-11-05 对各上游仓库的核查结果，它决定了每个项目能以什么方式进入本仓库。

| 项目 | 固定版本 | 声明的许可 | 本仓库的使用方式 |
| --- | --- | --- | --- |
| [makermods-robotics/makermodslab](https://github.com/makermods-robotics/makermodslab) | `e377149` | Apache-2.0 | **复制并修改**，位于 `apps/lab/`，保留 LICENSE 与版权行 |
| [makermods-robotics/lerobot](https://github.com/makermods-robotics/lerobot)（LeRobot fork） | `5ce2fe41` | Apache-2.0 | **依赖**，按 SHA 固定，不复制 |
| [Noietch/EVA-CLIENT](https://github.com/Noietch/EVA-CLIENT) | `926cdf4` | Apache-2.0 | **改编**动作缓冲与 StarVLA 客户端，见 `packages/rac-policy`，文件头注明来源 |
| [starVLA/starVLA](https://github.com/starVLA/starVLA) | `13d8ad4` | MIT | **独立进程服务**，只按其 WebSocket 协议对接，不复制 |
| [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) | `5265303` | CERN-OHL-W-2.0（硬件）、Apache-2.0（软件） | **复制** RS 的 URDF 与网格到 `assets/robots/rebot_b601_rs/`，附许可文本 |
| [servodevelop/Star-Arm-102](https://github.com/servodevelop/Star-Arm-102) | `24e463b` | 分组件声明，硬件文件**无明确授权** | **不复制**。关节尺寸作为事实使用，由 `tools/make_star_urdf.py` 生成简化模型，网格不进入本仓库 |
| [Yang-Ci/ReBot_Arm_DigitalTwin_RS](https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_RS) | `84c1ea7` | **未声明** | **不复制任何代码**。只参考其公开文档描述的方法与数值（DLS 阻尼范围、MIT 增益、目标影子显示），由本仓库独立实现 |
| [RobStride/RobStrideSPC](https://github.com/RobStride/RobStrideSPC) | `d8451a4` | **未声明** | **不复制代码**。按协议独立实现 `firmware/core/drv_robstride.c`，帧格式和数值范围属于协议事实 |
| [servodevelop/servo-uart-rs485-sdk](https://github.com/servodevelop/servo-uart-rs485-sdk) | `f728776` | **未声明** | **不复制代码**。按协议独立实现 `firmware/core/drv_fashionstar.c` |
| [fashionstar-uart-sdk](https://pypi.org/project/fashionstar-uart-sdk/)（PyPI） | 1.3.12 | MIT | 仅用于核对协议字段，不是依赖 |

## 需要你确认的事项

1. **ReBot_Arm_DigitalTwin_RS 的授权。** 该仓库没有 LICENSE 文件，默认保留全部权利。如果作者授权，可以进一步直接复用它的 Three.js 控件和轨迹代码；在此之前本仓库只做独立实现。
2. **Star Arm 102 的硬件文件授权。** 上游 `LICENSE.md` 明确写明“尚未建立统一授权”。如果 Fashion Star 确认可再分发，可以把真实网格放进 `assets/robots/star_arm_102/`。
3. **本仓库自身的对外许可。** 软件部分暂定 Apache-2.0，硬件部分暂定 CERN-OHL-W-2.0，与主要上游保持一致。仓库当前是私有的，公开前请再确认一次。

## 判定原则

- 通信协议的帧格式、命令号和数值范围是事实，可以据此独立实现；具体的源代码表达受版权保护，没有许可就不复制。
- 复制进来的文件保留原有版权行。改编的文件在文件头写明来源仓库、提交号和原许可。
- 依赖一律固定版本：Python 依赖由 `uv.lock` 固定，LeRobot fork 按 SHA 固定。
