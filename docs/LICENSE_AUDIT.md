# 上游复用记录

**本仓库只用于个人学习。** 仓库所有者在 2026-11-05 决定：开源资料可以直接复制，不再逐项向上游确认授权。下表记录每个项目实际用了什么。

**不要把这个仓库原样公开。** 标为“无授权”的几项没有授予再分发的权利，它们能放进来的前提是仓库保持私有。如果要公开，先按“公开前要处理”一节清掉。

| 项目 | 固定版本 | 声明的许可 | 本仓库的使用方式 |
| --- | --- | --- | --- |
| [makermods-robotics/makermodslab](https://github.com/makermods-robotics/makermodslab) | `e377149` | Apache-2.0 | **复制并修改**，位于 `apps/lab/`，保留 LICENSE 与版权行 |
| [makermods-robotics/lerobot](https://github.com/makermods-robotics/lerobot)（LeRobot fork） | `5ce2fe41` | Apache-2.0 | **依赖**，按 SHA 固定。它的 `rebot_102_leader` 也用来核对主从方向 |
| [Noietch/EVA-CLIENT](https://github.com/Noietch/EVA-CLIENT) | `926cdf4` | Apache-2.0 | **改编**动作缓冲与 StarVLA 客户端，见 `packages/rac-policy`，文件头注明来源 |
| [starVLA/starVLA](https://github.com/starVLA/starVLA) | `13d8ad4` | MIT | **独立进程服务**，按其 WebSocket 协议对接 |
| [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) | `5265303` | CERN-OHL-W-2.0（硬件）、Apache-2.0（软件） | **复制** RS 的 URDF 与网格到 `assets/robots/rebot_b601_rs/`，附许可文本 |
| [servodevelop/Star-Arm-102](https://github.com/servodevelop/Star-Arm-102) | `24e463b` | 硬件文件**无授权** | **复制** FL、LD/HD 的 URDF 与网格到 `assets/robots/star_arm_102/`，由 `tools/import_star_urdf.py` 导入。控制脚本的指令格式和夹爪比例也用上了 |
| [Yang-Ci/ReBot_Arm_DigitalTwin_RS](https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_RS) | `84c1ea7` | **无授权** | **复制**动作平滑和夹爪助力（`rac_kinematics/motion_profiles.py`）、显示滤波、重力校准报告（`docs/reference/`）和实测数据（`tests/data/`）。增益和重力系数也来自它 |
| [RobStride/RobStrideSPC](https://github.com/RobStride/RobStrideSPC) | `d8451a4` | **无授权** | 没有搬代码：它是 STM32F1 上的阻塞式调用，放不进这里的调度。用来核对 `firmware/core/drv_robstride.c` 的帧格式和数值范围，核对结果一致 |
| [servodevelop/servo-uart-rs485-sdk](https://github.com/servodevelop/servo-uart-rs485-sdk) | `f728776` | **无授权** | 同上。用来核对 `firmware/core/drv_fashionstar.c` 的帧格式，并补上了清圈数指令 |
| [fashionstar-uart-sdk](https://pypi.org/project/fashionstar-uart-sdk/)（PyPI） | 1.3.12 | MIT | 用于核对协议字段，不是依赖 |

## 公开前要处理

这些内容来自没有授权的来源，公开前要删掉或取得许可：

- `assets/robots/star_arm_102/`
- `packages/rac-kinematics/src/rac_kinematics/motion_profiles.py`
- `apps/lab/frontend/src/components/controller/RobotView3D.tsx` 里的显示滤波
- `docs/reference/gravity_calibration_rs_2026-07-17.md`
- `tests/data/rs_gravity_sweeps.json`

增益、方向表和重力系数这类数值是事实，可以保留。

## 惯例

- 复制进来的文件保留原有版权行。改编的文件在文件头写明来源仓库和提交号。
- 依赖一律固定版本：Python 依赖由 `uv.lock` 固定，LeRobot fork 按 SHA 固定。
- 本仓库自身暂定软件 Apache-2.0、硬件 CERN-OHL-W-2.0，只有公开时才有实际意义。
