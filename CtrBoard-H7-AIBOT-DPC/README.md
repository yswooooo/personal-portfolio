# CtrBoard-H7-AIBOT-DPC 工程说明

CtrBoard-H7-AIBOT-DPC 是基于 STM32H723VGTx 的双 LD2-RS 伺服差速底盘控制工程。当前主流程为：

```text
SBUS 遥控 -> 一阶低通/死区 -> 差速解算 -> USART3 RS485 Modbus RTU -> LD2-RS M1/M2
                                      \-> USART1 VOFA+ JustFloat 17 通道监测
```

> 主开发、调试、烧录仍使用 Keil MDK-ARM；根目录 `Makefile` 用于 GCC 命令行编译验证和产物生成。

![AMR-1](Asset/DiffWheel/AMR-1.jpg)
![AMR-2](Asset/DiffWheel/AMR-2.jpg)
![AMR-3](Asset/DiffWheel/AMR-3.jpg)

---

## 1. 硬件与外设

| 用途 | 外设 | 引脚 | 参数 | 说明 |
|------|------|------|------|------|
| LD2-RS RS485 | USART3 | PD8 TX / PD9 RX / PB14 DE | 115200 8N2 | Modbus RTU 主站 |
| VOFA+ 调试 | USART1 | PA9 TX / PA10 RX | 115200 8N1 | JustFloat |
| SBUS 遥控 | UART5 | PB13 TX / PD2 RX | 100000 8E2 | DMA+IDLE，硬件反相 |
| WS4810 LED | SPI6 | CubeMX 配置 | SPI 模拟时序 | 系统状态灯 |
| 急停按键 | GPIO | CubeMX 配置 | EXTI | 1=停机 |

---

## 2. 软件架构

```text
Core/           CubeMX 生成和主循环入口
App/            应用层：电机初始化、差速解算、FSM 调度、安全策略、LED 状态
BSP/            硬件抽象：RS485、SBUS、VOFA、WS4810
Middleware/     协议/算法：低通滤波、Modbus RTU、LD2-RS 寄存器封装
Drivers/        STM32H7 HAL + CMSIS
DSP/            本地 CMSIS-DSP 源码
GCC/            GCC startup 和 ld 链接脚本
MDK-ARM/        Keil 工程
```

依赖方向：

```text
Middleware -> BSP -> App -> Core
```

`Middleware` 不依赖 HAL/BSP/APP；`BSP` 不依赖 `APP`。

---

## 3. 主循环

`Core/Src/main.c` 初始化顺序：

1. HAL、时钟、GPIO/DMA/USART/SPI 初始化。
2. 初始化 RS485 总线和 M1/M2 LD2-RS 设备句柄。
3. 等待 `APP_SYSTEM_START_DELAY_MS = 200` ms。
4. 阻塞配置驱动器：速度模式、零速、内部速度源、加减速、PI 参数。
5. 启动 UART5 SBUS DMA+IDLE。
6. 初始化非阻塞电机 FSM 和 WS4810 状态灯。
7. `while(1)` 中循环调用：

```c
app_ld2rs_task_run();
App_Vofa_UpperDisplay();
app_led_indicator_update();
```

---

## 4. 控制逻辑

### RC 映射

- `g_rc_filter.ch_rx`：线速度方向量，经过死区和低通。
- `g_rc_filter.ch_ry`：角速度方向量，经过死区和低通。
- `SWA DOWN`：强制停机。
- `SWB UP`：线速度解锁，VRA 作为线速度油门。
- `SWC UP`：角速度解锁，VRB 作为角速度油门。
- 掉线判定：200 ms 无有效 SBUS 帧。

### 差速解算

```text
左轮 rpm = (v * 1000 - omega * 328.5) * 2.5465 * APP_MOTOR_CTRL_M1_POLARITY
右轮 rpm = (v * 1000 + omega * 328.5) * 2.5465 * APP_MOTOR_CTRL_M2_POLARITY
```

任一轮超过 `APP_CHASSIS_SPEED_LIMIT_RPM = 4500` 时，两轮等比例缩放，保持转弯半径。

### 电机 FSM

```text
IDLE -> READ_REQ -> WAIT_READ -> READ_DONE -> WRITE_REQ -> WAIT_WRITE -> WRITE_DONE -> IDLE
```

- READ：读 `PrB.05` 状态和 `PrB.06` 实时速度。
- WRITE：写 `Pr3.04` 目标速度。
- BSP 物理层最多重试 3 次，APP FSM 逻辑层读/写各最多重试 3 次。
- READ 重试耗尽后会强制写零速。

### 编码器速度估算

- FSM 阶段性读取单寄存器 `PrB.24` 的位置反馈。回传值合并为 `int32_t`，它是电机侧（转子侧）的有符号 32 位累计位置值，不是减速器输出轴位置。
- 当前 M17 默认使用 17 位磁编码器：转子一圈为 `131072` 个 encoder counts（单圈编码可表示 `0..131071`）。未转动上电后，`PrB.24` 的累计值可从 0 开始。
- 仅在相邻两次有效 `PrB.24` 反馈之间估算速度。RS485 接收完成时记录 DWT 微秒时间戳，使用 `delta_counts / delta_time_us` 求计数速度；首次有效样本只用于建立基准，不输出速度。
- 编码器方向由 `APP_LD2_ENCODER_POLARITY_M1/M2` 独立修正。`int32_t` 累计位置跨越有符号边界时，差分按 32 位回绕处理，不按 17 位单圈编码范围回绕。

```text
motor_speed_rpm = delta_counts × 60 × 1000000
                  / (131072 × delta_time_us)
wheel_speed_rpm = motor_speed_rpm / 20
wheel_speed_mps = wheel_speed_rpm × 2π × 0.075 / 60
```

估算结果通过 `app_ld2rs_task_get_speed_feedback()` 提供，分别为电机侧转速（rpm）、减速后轮毂转速（rpm）和轮毂线速度（m/s）。VOFA+ 的 CH18、CH19 输出 M1、M2 的 `PrB.24` 累计位置，便于与估算结果交叉核对。

---

## 5. VOFA+ 监测

当前实际发送 **19 个 JustFloat 通道**，周期 20 ms：

| CH | 数据 |
|----|------|
| 1~4 | M1/M2 给定转速、反馈转速 |
| 5 | 急停标志 |
| 6 | RC 掉线标志 |
| 7 | SWA 状态 |
| 8~11 | M1/M2 离线总次数和离线确认 |
| 12~13 | 滤波后的 `ch_ry` / `ch_rx` |
| 14~15 | 底盘线速度 / 角速度指令 |
| 16~17 | M1/M2 闭环周期 |
| 18~19 | M1/M2 `PrB.24` 编码器累计位置值 |

---

## 6. 构建

### Keil

打开：

```text
MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx
```

Keil5 仍是正式开发、调试和上板烧录入口。

### GCC Makefile

在工程根目录执行：

```powershell
make
```

查看完整命令：

```powershell
make V=1
```

清理 GCC 构建产物：

```powershell
make clean
```

生成：

```text
build/CtrBoard-H7-AIBOT-DPC.elf
build/CtrBoard-H7-AIBOT-DPC.map
build/CtrBoard-H7-AIBOT-DPC.hex
build/CtrBoard-H7-AIBOT-DPC.bin
```

Makefile 当前已匹配工程中的 `Core/App/BSP/Middleware/Drivers` 源文件，DSP 使用 `DSP/Source` 下的非 F16 聚合 `.c` 文件，并定义 `DISABLEFLOAT16`。

---

## 7. 关键配置

| 宏 | 当前值 | 说明 |
|----|--------|------|
| `APP_LD2_MOTOR_SLAVE_ID_M1/M2` | 1 / 2 | 驱动器站号 |
| `APP_LD2_MOTOR_TIMEOUT_MS` | 100 | Modbus 超时 |
| `APP_LD2_MOTOR_OFFLINE_CONFIRM_COUNT` | 3 | 离线确认阈值 |
| `APP_MODBUS_RTU_INTERFRAME_MS` | 1 | 帧间隔 |
| `APP_CHASSIS_SPEED_LIMIT_RPM` | 4500 | 转速限幅 |
| `APP_CHASSIS_WHEEL_TRACK_MM` | 657.0 | 轮距 |
| `APP_CHASSIS_WHEEL_RADIUS_MM` | 75.0 | 轮半径 |
| `APP_CHASSIS_GEAR_RATIO` | 20.0 | 减速比 |
| `APP_LD2_ENCODER_BITS` | 17 | M17 磁编码器位数 |
| `LD2_ENCODER_COUNTS_PER_REV` | 131072.0 | 电机侧每圈编码器计数 |
| `APP_LD2_ENCODER_POLARITY_M1/M2` | 1 / 1 | 编码器速度正方向修正 |
| `BSP_RC_STICK_DEADZONE` | 150.0 | 摇杆死区 |
| `BSP_RC_LPF_TAU_MS` | 200.0 | RC 低通时间常数 |

---

## 8. 排查速查

| 问题 | 优先检查 |
|------|----------|
| 电机不转 | VOFA CH8~11、RS485 A/B、站号、波特率、SWA 是否 DOWN |
| 遥控无响应 | VOFA CH6、SBUS 接收机供电、UART5 接线、硬件反相 |
| 方向反 | `APP_MOTOR_CTRL_M1/M2_POLARITY` 或 `APP_CHASSIS_CMD_POLARITY` |
| 响应慢 | 减小 `BSP_RC_LPF_TAU_MS` |
| 中位漂移 | 增大 `BSP_RC_STICK_DEADZONE` |
| 离线紫闪 | 查看 M1/M2 离线确认通道和 RS485 链路 |

