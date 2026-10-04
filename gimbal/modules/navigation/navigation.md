# 导航通信与控制

本模块让上位机通过 **USB CDC 虚拟串口** 向云台板发送导航命令；云台板完成命令检查和控制源仲裁后，经 CAN 将底盘命令发送给底盘板。

> USB CDC 是字节传输通道；`A5`、`A6`、`A7`、`A9` 是运行在该通道上的自定义二进制固定帧。它们不是 Seasky 帧，也不使用 `cmd_id`。`PCComm` 统一占有 USB 回调，并在同一字节流中解析 Seasky 帧和导航帧。

## 数据流与职责

```text
上位机
  │ USB CDC：A5 速度命令 / A9 云台 yaw 目标
  ▼
PCComm：拼接字节流、校验并分发完整帧
  ▼
Navigation：缓存命令、判断在线状态、发送 A6/A7 反馈
  ▼
RobotCMD：仲裁控制源，完成单位转换
  │ CAN：Chassis_Ctrl_Cmd_s
  ▼
底盘板：按原有 offset_angle 逻辑完成唯一一次坐标转换，再计算轮速
```

- `pc_comm.c`：USB CDC 的唯一入口，负责分包、粘包和错误后的重新同步。
- `navigation_protocol.c`：只负责编解码、校验和及浮点数合法性检查。
- `navigation.c`：缓存命令、判断超时、调度反馈；不直接控制电机。
- `robot_cmd.c`：导航在线时写入最终云台/底盘控制命令。
- `chassis.c`：保留坐标转换、运动学与电机闭环职责。

## 坐标系与单位

| 量 | 含义 | 单位 | 坐标系 / 正方向 |
| --- | --- | --- | --- |
| `A5.vx` | 目标前向线速度 | m/s | 云台坐标系，前为正 |
| `A5.vy` | 目标左向线速度 | m/s | 云台坐标系，左为正 |
| `A5.wz` | 底盘偏航角速度 | rad/s | 共享 Z 轴正方向 |
| `A9.yaw_target` | 云台绝对 yaw 目标 | rad | 相对 IMU 建立参考时的航向；俯视逆时针为正 |
| `A6.vx/vy/wz` | 实际速度反馈 | m/s、m/s、rad/s | 底盘坐标系 |
| `A7.yaw` | 云台 yaw 反馈 | rad | 当前 `offset_angle` 对应的相对角 |

`RobotCMD` 将 A5 的 `vx/vy` 从 m/s 转为 CAN 命令需要的 mm/s，将 `wz` 从 rad/s 转为 deg/s。云台板**不旋转** `vx/vy`；底盘板用原有 `offset_angle` 只转换一次。重复转换会导致运动方向错误。

## 帧格式

所有多字节数为小端序，浮点数为 IEEE-754 `float32`。帧尾校验是前面所有字节的 16 位无符号累加和，低字节在前。

### A5：上位机 → MCU，底盘速度命令

固定 15 字节。

| 偏移 | 长度 | 字段 | 说明 |
| ---: | ---: | --- | --- |
| 0 | 1 | `0xA5` | 帧头 |
| 1 | 4 | `vx` | `float32`，m/s |
| 5 | 4 | `vy` | `float32`，m/s |
| 9 | 4 | `wz` | `float32`，rad/s |
| 13 | 2 | `checksum` | bytes `[0..12]` 的累加和，`uint16_le` |

全零命令：

```text
A5 00 00 00 00 00 00 00 00 00 00 00 00 A5 00
```

### A9：上位机 → MCU，云台 yaw 目标

固定 7 字节。

| 偏移 | 长度 | 字段 | 说明 |
| ---: | ---: | --- | --- |
| 0 | 1 | `0xA9` | 帧头 |
| 1 | 4 | `yaw_target_rad` | `float32`，相对启动参考航向的绝对目标角，rad |
| 5 | 2 | `checksum` | bytes `[0..4]` 的累加和，`uint16_le` |

云台 IMU 姿态有效后，`GimbalTask()` 仅首次记录参考角 `yaw_zero_deg`。此前收到的 A9 会丢弃，避免开机阶段的目标在初始化完成后突然执行。

### A6：MCU → 上位机，底盘速度反馈

固定 20 字节。

| 偏移 | 长度 | 字段 | 说明 |
| ---: | ---: | --- | --- |
| 0 | 1 | `0xA6` | 帧头 |
| 1 | 4 | `sample_time_us` | `uint32_le`，采样时刻，µs |
| 5 | 1 | `flags` | 有效位与时间状态 |
| 6 | 4 | `vx` | `float32`，m/s，底盘坐标系 |
| 10 | 4 | `vy` | `float32`，m/s，底盘坐标系 |
| 14 | 4 | `wz` | `float32`，rad/s |
| 18 | 2 | `checksum` | bytes `[0..17]` 的累加和，`uint16_le` |

### A7：MCU → 上位机，云台 yaw 反馈

固定 12 字节。

| 偏移 | 长度 | 字段 | 说明 |
| ---: | ---: | --- | --- |
| 0 | 1 | `0xA7` | 帧头 |
| 1 | 4 | `sample_time_us` | `uint32_le`，采样时刻，µs |
| 5 | 1 | `flags` | bit0 表示 yaw 有效 |
| 6 | 4 | `yaw` | `float32`，rad |
| 10 | 2 | `checksum` | bytes `[0..9]` 的累加和，`uint16_le` |

### `flags`

| 位 | 宏 | A6 含义 | A7 含义 |
| ---: | --- | --- | --- |
| bit0 | `NAVIGATION_FLAG_LINEAR_VALID` | `vx`、`vy` 有效 | yaw 有效 |
| bit1 | `NAVIGATION_FLAG_ANGULAR_VALID` | `wz` 有效 | 保留，必须为 0 |
| bit7 | `NAVIGATION_FLAG_TIME_RESET` | MCU 刚启动，时间基准可能重置 | 同左 |

若 A6 对应有效位未置位，MCU 会把对应速度字段发为 `0`；上位机不得把该字段作为可信测量值。

## 控制权与安全行为

1. A5 必须具有正确帧长、帧头和校验和，且三个浮点数均为有限值；否则丢弃并增加错误计数。
2. 最近一次有效 A5 的接收时间不超过 `navigation_config.rx_timeout_ms` 时才在线；默认超时为 **200 ms**。
3. A5 在线时，导航覆盖最终底盘命令的 `vx`、`vy`、`wz`，并设置 `CHASSIS_NAVIGATION`。
4. A5 超时后，导航不再接管；控制权回到本周期生成的遥控器/键鼠命令。最后一条导航速度不会无限保持。
5. A9 只要求“收到过有效帧且 IMU 参考角已建立”，当前**没有独立超时**。若需撤销 yaw 目标，应发送新目标，或在上层增加显式使能/超时仲裁。
6. `EmergencyHandler()` 在导航写入命令之后运行，急停和离线保护可覆盖导航输出。

## 默认参数与调试

| 配置项 | 默认值 | 用途 |
| --- | ---: | --- |
| `navigation_config.tx_enabled` | `1` | 允许发送 A6/A7 |
| `speed_tx_period_ms` | `20` | A6 周期，约 50 Hz |
| `turret_tx_period_ms` | `10` | A7 周期，约 100 Hz |
| `rx_timeout_ms` | `200` | A5 在线超时，ms |
| `test_mode` | `0` | 置 1 时以 `test_vx/test_vy/test_wz` 生成 A6 测试数据 |

A6、A7 同时到期时，USB CDC 单发送缓冲只能接收一帧，模块会交替优先发送两路，防止任一路长期饥饿。发送忙时不阻塞控制任务，而是记录 `navigation_debug.tx_busy_count`。

建议在 Live Watch 观察：

```text
navigation_debug.rx_frame_count
navigation_debug.rx_value_error_count
navigation_debug.last_rx_velocity
navigation_debug.yaw_target_rx_frame_count
navigation_debug.last_rx_yaw_target_rad
navigation_debug.speed_tx_frame_count
navigation_debug.turret_tx_frame_count
navigation_debug.tx_busy_count
```

## 上位机测试

脚本位于 `gimbal/tools/navigation_serial_test.py`。先运行不需要硬件的协议自检：

```powershell
py .\tools\navigation_serial_test.py --self-test
```

列出串口：

```powershell
py .\tools\navigation_serial_test.py --list
```

以 30 Hz 发送速度并显示反馈：

```powershell
py .\tools\navigation_serial_test.py --port COM7 --send --vx 0.1 --vy 0 --wz 0.5 --rate 30
```

以 30 Hz 发送 yaw 目标：

```powershell
py .\tools\navigation_serial_test.py --port COM7 --yaw-target 1.0 --rate 30
```

联调时请先低速、悬空或留足安全空间，确认帧计数、单位换算、云台偏转后的底盘运动方向、A5 停止 200 ms 后的控制权释放，以及急停功能。
