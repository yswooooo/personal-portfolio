# CLAUDE.md

> 为新会话准备的工程全景备忘。进入此目录后先读本文件。

---

## 0. 工作约定

- 每次提问/新对话，必须先检查并调用相关 superpowers skill（如 `superpowers:brainstorming`、`superpowers:systematic-debugging` 等）。
- CubeMX 生成文件位于 `Core/`，只在 `USER CODE BEGIN/END` 区间内编辑。
- 修改 C 源码、启动文件、链接脚本或编译配置后，优先运行根目录 `make` 做 GCC 命令行验证；硬件下载和最终一致性仍以 Keil5 工程为准。
- 不要把 Keil RTE 的 `CMSIS:DSP Source` 和本工程 `DSP/Source` 同时启用，避免重复定义。

---

## 1. 工程快照

| 项 | 当前状态 |
|----|---------|
| 硬件 | 达妙 DM-MC-Board02，主控 **STM32H723VGTx** |
| 角色 | **Modbus RTU 主站**；从机为 **LD2-RS 伺服驱动器 × 2** |
| M1 / M2 站号 | 1 / 2 |
| RS485 总线 | USART3，PD8 TX / PD9 RX + PB14 硬件 DE，**115200 8N2** |
| 调试串口 | USART1，PA9/PA10，115200 8N1 |
| 遥控 | SBUS 接收机 → UART5 DMA+IDLE，100000 8E2，硬件反相 |
| 状态灯 | WS4810，SPI6 模拟时序 |
| 运行模式 | 裸机 HAL，单线程主循环，无 RTOS |
| 应用模型 | 二轮差速：线速度 v (m/s) + 角速度 omega (rad/s) → 左/右轮 rpm |
| 底盘参数 | 轮距 657 mm，轮半径 75 mm，齿轮比 20:1 |
| VOFA+ | JustFloat，20 ms 周期，**实际发送 19 通道** |

---

## 2. 目录结构

```text
Core/               CubeMX 生成代码和主循环入口
  main.c            初始化流程、while(1)、VOFA+ 17 通道发送
  usart.c           USART1/2/3 + UART5 初始化，UART 回调分发
  gpio.c            GPIO 初始化、急停标志
  dma.c, spi.c      DMA / SPI6 初始化

App/                应用层，文件统一 app_ 前缀
  app_config.h      业务参数集中配置
  app_chassis_motor_ctrl.c
                    LD2-RS 上电配置、PI 写入、差速解算、RC 业务映射
  app_encoder_speed.c
                    相邻有效编码器样本的差分速度估算
  app_ld2rs_task.c  非阻塞电机 FSM、多电机调度、安全停机、编码器样本发布、VOFA 数据填充
  app_led_indicator.c
                    WS4810 系统状态灯

BSP/                硬件抽象层，文件统一 bsp_ 前缀
  bsp_dwt.c         DWT 64 位微秒时间戳
  bsp_rs485.c       RS485 半双工非阻塞事务状态机
  bsp_rc.c          SBUS 收帧、解析、拨杆状态、一阶低通滤波
  bsp_vofa.c        VOFA+ JustFloat / FireWater 发送
  bsp_ws4810.c      WS4810 SPI 时序驱动

Middleware/         协议/算法层，不依赖 APP/BSP/HAL
  Filter/           一阶低通滤波
  Protocol/         Modbus RTU 0x03 / 0x06 + CRC16
  Motor/            LD2-RS 寄存器读写封装

Drivers/            STM32H7 HAL + CMSIS
DSP/                本地 CMSIS-DSP Include / PrivateInclude / Source
GCC/                GCC startup + GNU ld 链接脚本
MDK-ARM/            Keil 工程
Makefile            根目录 GCC 构建入口
```

---

## 3. 启动与主循环

```text
main()
  HAL_Init + SystemClock_Config
  MX_GPIO/DMA/USART3/USART1/USART2/UART5/SPI6_Init
  bsp_rs485_init(&g_rs485_bus, &huart3)
  BSP_DWT_Init()
  ld2_motor_init(M1 ID=1, M2 ID=2)
  HAL_Delay(APP_SYSTEM_START_DELAY_MS = 200)
  app_chassis_ld2rs_motor_ctrl_init(M1/M2)
  app_chassis_ld2rs_motor_ctrl_param_read_back(M1/M2)
  app_chassis_ld2rs_motor_ctrl_pi_init(M1/M2)
  bsp_rc_init(&g_rc)
  app_ld2rs_task_init()
  app_led_indicator_init()
  while (1)
    app_ld2rs_task_run()
    App_Vofa_UpperDisplay()     # APP_VOFA_JUSTFLOAT_ENABLE=1 时启用
    app_led_indicator_update()
```

`app_ld2rs_task_run()` 每轮做一件完整的业务推进：

```text
bsp_rs485_poll()
app_rc_channels_check_lost(&g_rc)
app_diff_chassis_motor_ctrl_rc_map_to_chassis(&g_rc_filter, &g_rc_chassis)
SWB/SWC 解锁 + VRA/VRB 油门缩放
app_diff_drive_compute()
FSM force_zero_speed 检查
急停 / RC 掉线 / SWA_DOWN 最高优先级清零
app_motor_fsm_scheduler_step()
```

---

## 4. RS485 与电机 FSM

### BSP 事务状态机

```text
IDLE -> TX_BUSY -> RX_WAIT -> DONE
              \-> TIMEOUT -> retry or sticky TIMEOUT
```

- TX 使用 `HAL_UART_Transmit_IT`。
- TC 中断后清 IDLE/ORE，启动 `HAL_UARTEx_ReceiveToIdle_IT`。
- RX IDLE 完成后进入 DONE。
- RX IDLE 完成时先发布接收长度与 DWT 微秒时间戳，再置 DONE；APP 据此计算编码器相邻样本时间差。
- `bsp_rs485_poll()` 在主循环中检查超时，并做最多 `BSP_RS485_MAX_RETRY = 3` 次物理重试。
- DONE/TIMEOUT/ERROR 为粘滞态，调用者必须 `bsp_rs485_ack_done()` 释放总线。

### APP 电机 FSM

```text
IDLE -> READ_REQ -> WAIT_READ -> READ_DONE -> WRITE_REQ -> WAIT_WRITE -> WRITE_DONE -> IDLE
```

- READ：0x03 读 `PrB.05` 运行状态和 `PrB.06` 未滤波实时速度。
- WRITE：0x06 写 `Pr3.04` 目标转速。
- FSM 层读/写逻辑重试次数为 `APP_LD2RS_TASK_MAX_RETRY = 3`（定义在 `app_ld2rs_task.c`）。
- READ 重试耗尽会置 `force_zero_speed=1`，下一次写入零速。
- 多电机调度器 `motor_fsm_scheduler_t` 轮询 M1/M2，实际总线访问顺序为 M1 READ、M2 READ、M1 WRITE、M2 WRITE。

### 编码器位置与速度估算

- `PrB.24` 以单寄存器读取；高、低 16 位合并为 `int32_t encoder_position_counts`。
- M17 默认编码器分辨率为 17 位，电机侧每转为 `LD2_ENCODER_COUNTS_PER_REV = 131072.0f` 个计数；`PrB.24` 是电机侧有符号 32 位累计位置反馈，不是减速后轮毂位置。
- 每次取得有效 `PrB.24` 响应后，将“计数值 + RS485 接收 DWT 时间戳”整体发布为一个样本。首个样本仅建立基准；后续相邻样本才产生有效速度。
- 差分跨越 `int32_t` 边界时按 32 位回绕处理；方向仅由 `APP_LD2_ENCODER_POLARITY_M1/M2` 修正，不能通过交换新旧采样值隐式反向。
- `app_ld2rs_task_get_speed_feedback()` 为 M1/M2 返回电机侧估算转速（rpm）、减速后轮毂转速（rpm）和轮毂线速度（m/s）。
- 换算关系：`motor_rpm = delta_counts × 60 × 1000000 / (131072 × delta_time_us)`，`wheel_rpm = motor_rpm / 20`，`wheel_mps = wheel_rpm × 2π × 0.075 / 60`。

---

## 5. RC 与底盘映射

| SBUS 通道 | 字段 | 用途 |
|-----------|------|------|
| CH1 | `g_rc.ch_ry` | 右摇杆 Y，角速度方向 |
| CH2 | `g_rc.ch_rx` | 右摇杆 X，线速度方向 |
| CH3 | `g_rc.ch_lx` | 左摇杆 X，当前保留/滤波 |
| CH4 | `g_rc.ch_ly` | 左摇杆 Y，当前保留/滤波 |
| CH5~8 | `sw_val[0..3]` / `sw_st[]` | SWA/B/C/D |
| CH9 | `vra` | 线速度油门 |
| CH10 | `vrb` | 角速度油门 |

- 摇杆归一化范围约为 `-800..+800`。
- 方形死区：`BSP_RC_STICK_DEADZONE = 150.0f`。
- 一阶低通：`BSP_RC_LPF_TAU_MS = 200.0f`，`BSP_RC_SAMPLE_TIME_MS = 10.0f`。
- 掉线：`BSP_RC_TIMEOUT_MS = 200` ms 无有效帧。
- SWA DOWN 强制停机；SWB UP 解锁线速度；SWC UP 解锁角速度。

---

## 6. VOFA+ 实际通道

`Core/Src/main.c` 中 `App_Vofa_UpperDisplay()` 当前发送 `float vofa_channels[19]`：

| CH | 数据 |
|----|------|
| 1 | `g_vofa_speed.m1_ref_speed_rpm` |
| 2 | `g_vofa_speed.m1_feedback_speed_rpm` |
| 3 | `g_vofa_speed.m2_ref_speed_rpm` |
| 4 | `g_vofa_speed.m2_feedback_speed_rpm` |
| 5 | `g_emergency_stop_flag` |
| 6 | `g_rc.lost_flag` |
| 7 | `g_rc.sw_st[eRC_SW_A].curr` |
| 8 | `g_vofa_speed.m1_offline_total_count` |
| 9 | `g_vofa_speed.m1_offline_confirmed` |
| 10 | `g_vofa_speed.m2_offline_total_count` |
| 11 | `g_vofa_speed.m2_offline_confirmed` |
| 12 | `g_rc_filter.ch_ry` |
| 13 | `g_rc_filter.ch_rx` |
| 14 | `g_rc_chassis.fLinearVel` |
| 15 | `g_rc_chassis.fAngularVel` |
| 16 | `g_vofa_speed.m1_cycle_ms` |
| 17 | `g_vofa_speed.m2_cycle_ms` |
| 18 | `g_vofa_speed.m1_encoder_position_counts` |
| 19 | `g_vofa_speed.m2_encoder_position_counts` |

`vofa_motor_info_t` 内部还保留了 `m*_speed_error_rpm`、`m*_read_rtt_ms`、`m*_write_rtt_ms` 等字段，但这些字段当前没有放进 JustFloat 发送数组。

---

## 7. 关键参数

| 宏 | 当前值 | 说明 |
|----|--------|------|
| `APP_LD2_MOTOR_SLAVE_ID_M1/M2` | 1 / 2 | LD2-RS Modbus 站号 |
| `APP_LD2_MOTOR_COUNT` | 2 | 当前参与轮询的电机数 |
| `APP_LD2_MOTOR_MAX_COUNT` | 3 | 调度器最大预留 |
| `APP_LD2_MOTOR_TIMEOUT_MS` | 100 | 单次 Modbus 事务超时 |
| `APP_LD2_MOTOR_OFFLINE_CONFIRM_COUNT` | 3 | 连续无应答离线确认阈值 |
| `APP_MODBUS_RTU_INTERFRAME_MS` | 1 | Modbus RTU 帧间隔 |
| `APP_SYSTEM_START_DELAY_MS` | 200 | 上电等待驱动器就绪 |
| `APP_CHASSIS_SPEED_LIMIT_RPM` | 4500 | 电机转速硬限幅 |
| `APP_CHASSIS_WHEEL_TRACK_MM` | 657.0 | 轮距 |
| `APP_CHASSIS_WHEEL_RADIUS_MM` | 75.0 | 轮半径 |
| `APP_CHASSIS_GEAR_RATIO` | 20.0 | 减速比 |
| `APP_LD2_ENCODER_BITS` | 17 | M17 磁编码器位数 |
| `LD2_ENCODER_COUNTS_PER_REV` | 131072.0 | 电机侧每圈编码器计数 |
| `APP_LD2_ENCODER_POLARITY_M1/M2` | 1 / 1 | 编码器速度方向修正 |
| `APP_CHASSIS_CMD_POLARITY` | -1 | 角速度方向极性 |
| `APP_MOTOR_CTRL_M1_POLARITY` | 1 | M1 方向极性 |
| `APP_MOTOR_CTRL_M2_POLARITY` | -1 | M2 方向极性 |
| `APP_MOTOR_CTRL_ACCEL_MS_PER_1000RPM` | 250 | Pr3.12 |
| `APP_MOTOR_CTRL_DECEL_MS_PER_1000RPM` | 250 | Pr3.13 |
| `APP_VOFA_JUSTFLOAT_ENABLE` | 1 | 启用 VOFA JustFloat |
| `APP_VOFA_JUSTFLOAT_PERIOD_MS` | 20 | VOFA 发送周期 |

---

## 8. 构建入口

### Keil 主开发路径

- 打开 `MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx`
- 编译器：ARMCLANG V6.24
- 烧录器：J-Link / ST-Link

### GCC Makefile 验证路径

```powershell
make
make V=1
make clean
```

当前 Makefile 适配点：

- 工具链路径：`D:/Msys2/mingw64/bin`
- 启动文件：`GCC/startup_stm32h723xx_gcc.s`
- 链接脚本：`GCC/STM32H723VGTx_FLASH.ld`
- 已列入当前 `Core/App/BSP/Middleware/Drivers` 的全部非 DSP 固件 `.c` 文件。
- DSP 使用 16 个非 F16 聚合源文件，如 `BasicMathFunctions.c`、`FastMathFunctions.c`、`TransformFunctions.c`。
- 已定义 `DISABLEFLOAT16`。

生成产物：

```text
build/CtrBoard-H7-AIBOT-DPC.elf
build/CtrBoard-H7-AIBOT-DPC.map
build/CtrBoard-H7-AIBOT-DPC.hex
build/CtrBoard-H7-AIBOT-DPC.bin
build/obj/.../*.o
build/obj/.../*.d
```

---

## 9. 快速排查

| 现象 | 优先检查 |
|------|----------|
| 电机不转 | VOFA CH8~11、RS485 A/B、站号 1/2、115200 8N2 |
| 遥控无响应 | VOFA CH6、UART5 SBUS 接线、接收机供电、反相配置 |
| 一拨杆就停 | VOFA CH7，确认 SWA 是否 DOWN |
| 底盘方向反 | `APP_MOTOR_CTRL_M1/M2_POLARITY` 或 `APP_CHASSIS_CMD_POLARITY` |
| 响应太慢 | 减小 `BSP_RC_LPF_TAU_MS` |
| 摇杆抖动或中位漂移 | 增大 `BSP_RC_STICK_DEADZONE` 或低通时间常数 |
| 通信吞吐异常 | CH16/17 闭环周期、BSP/FSM 重试计数、USART3 NVIC 优先级 |

