# RAC-Link 协议

电脑和 STM32 控制器之间的协议。**唯一的定义在 `tools/gen_protocol.py`**，由它生成 `firmware/core/rac_link_gen.h` 和 `packages/rac-link/src/rac_link/_gen.py`。`tests/test_protocol.py` 确保两边一致。

## 帧格式

```
帧     := COBS(头 || 消息体 || crc16_le) || 0x00
头     := 24 字节，小端
crc16  := CRC-16/CCITT-FALSE，覆盖头和消息体
```

COBS 让 `0x00` 只出现在帧尾，所以中途接入或数据损坏后，接收方在下一个分隔符处就能恢复同步。

| 头字段 | 类型 | 含义 |
| --- | --- | --- |
| `version` | u8 | 协议版本，不匹配的命令会被拒绝 |
| `msg_type` | u8 | 消息类型 |
| `flags` | u16 | 保留 |
| `device_id` | u16 | 由芯片 UID 算出 |
| `arm_id` | u8 | 逻辑臂：0 左主、1 左从、2 右主、3 右从，`0xFF` 表示设备级 |
| `reserved` | u8 | 保留 |
| `seq` | u32 | 命令序号，ACK 中回传 |
| `generation` | u32 | 控制代次 |
| `timestamp_us` | u64 | 发送方时钟 |

## 控制代次

这是接管流程可靠的关键。

- 每次改变控制来源，服务都让代次加一，并通过 `SET_MODE` 告知控制器。
- `TARGET_CHUNK` 的代次必须**严格等于**当前代次，否则以 `STALE_GENERATION` 拒绝。接管前算出、接管后才到的动作块不可能被执行。
- 其他会引发运动的命令，代次必须不低于当前值。
- `STOP` 不受限制，因为停止必须永远有效。`HELLO`、`QUERY_STATUS`、`DISCOVER` 也不受限制，因为刚启动的服务还不知道当前代次。
- 控制器在急停和通信中断时**自己把代次加一**，并在之后每一帧的头里报告。这样中断前排队的命令在恢复后也不会执行。

## 时间同步

`TIME_SYNC` 是 NTP 式的四时间戳交换，每 250 ms 一次。USB 往返时间有长尾，所以只用窗口内往返最短的一半样本来拟合。录制结束后，每个 episode 还会用控制器时钟做一次整体线性拟合，再对齐到相机时间线。

板上的 `SYNC_OUT`、`SYNC_IN` 硬件同步引脚已接出，但固件核心还没有驱动它们，目前的时间同步只靠这个交换。

## 一个样本

每次读取产生一条 `ARM_STATE`：

| 字段 | 含义 | 数据集用途 |
| --- | --- | --- |
| `sample_seq`、`sample_ts_us` | 样本序号和控制器时间 | 检查丢失，时间线 |
| `position`、`velocity`、`torque` | 实测反馈，弧度，夹爪 0 到 1 | `observation.state` |
| `target_seq`、`target_ts_us`、`target` | 实际下发到电机的目标 | `action` |
| `exec_seq` | 已执行到的动作序号 | 执行确认 |
| `valid_mask`、`fault_mask` | 本次哪些关节有效、哪些有故障 | 质量统计 |
| `flags` | 是否有下发目标、是否被限位等 | |

数据集的 `action` 取自这里的 `target`，而不是电脑请求的值，所以记录的是电机实际收到的目标。

## 消息

用 `uv run python tools/gen_protocol.py` 重新生成绑定。完整的枚举和结构体见 `tools/gen_protocol.py`，分组如下：

| 类别 | 消息 |
| --- | --- |
| 发现与配置 | `HELLO`、`DISCOVER`、`BIND_PORT`、`SET_CALIBRATION`、`SET_PAIRING`、`SET_RATES`、`SET_LIMITS` |
| 运动 | `SET_MODE`、`TARGET_CHUNK`、`TELEOP_CONTROL`、`FEEDFORWARD`、`TORQUE`、`PAUSE_RESUME` |
| 安全 | `STOP`、`RESET_ESTOP`、`HEARTBEAT` |
| 上行 | `ACK`、`DEVICE_INFO`、`PORT_STATUS`、`ARM_STATE`、`CONTROLLER_STATUS`、`EVENT`、`TIME_SYNC_REPLY` |
| 仅仿真 | `SIL_INJECT` |

## USB

CDC-ACM 串口。VID `0x1209`、PID `0x0001` 属于 pid.codes 的测试段，正式产品需要申请自己的 PID。
