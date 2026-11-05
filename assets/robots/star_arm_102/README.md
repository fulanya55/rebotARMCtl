# Star Arm 102 模型

来自 [servodevelop/Star-Arm-102](https://github.com/servodevelop/Star-Arm-102) 的 `24e463b`，由 `tools/import_star_urdf.py` 导入。上游硬件文件**没有授予再分发的权利**，这里只供个人学习使用，见 [docs/LICENSE_AUDIT.md](../../../docs/LICENSE_AUDIT.md)。

| 目录 | 机型 |
| --- | --- |
| `fl/` | 102-FL 从臂 |
| `ld_hd/` | 102-LD 和 102-HD 主臂，上游两者的模型完全相同 |

导入时改了这几处，其余和上游一致：

- 网格路径改成相对路径。
- 加了末端坐标系 `gripper_tip`，位置从手指网格算出。
- LD/HD 的 `joint5` 由 `fixed` 改为转动关节。这个舵机实际存在，也会转。
- FL 的 `joint4` 轴反向，和 LD/HD 一致。官方控制脚本把主臂原始角度原样发给 FL，两者方向必须相同。

后两处是从官方脚本和模型几何推断的，没有用实物核对。关节范围保留上游的值，它们是占位数，比实际行程小，所以 3D 视图不按它们限位。
