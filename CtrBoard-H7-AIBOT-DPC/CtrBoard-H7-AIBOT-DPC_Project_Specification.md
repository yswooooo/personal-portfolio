# CtrBoard-H7-AIBOT-DPC Project Specification

> **版本:** 2026-08-07
> **主控:** STM32H723VGTx (达妙 DM-MC-Board02)  
> **主开发环境:** Keil MDK-ARM / ARMCLANG V6.24  
> **命令行验证:** 根目录 `Makefile` + `D:/Msys2/mingw64/bin/arm-none-eabi-*`  
> **驱动器:** LD2-RS 伺服驱动器 × 2  
> **通信:** USART3 RS485 Modbus RTU 主站，115200 8N2

---

## 1. Scope

本工程实现一个双轮差速底盘控制器：

```text
SBUS 遥控输入
  -> BSP RC 解析、一阶低通、拨杆状态
  -> APP 安全判断、油门解锁、差速解算
  -> APP LD2-RS 非阻塞 FSM
  -> BSP RS485 半双工 Modbus RTU
  -> LD2-RS M1/M2

并行输出：
  USART1 -> VOFA+ JustFloat 23 通道
  SPI6   -> WS4810 系统状态灯
```

---

## 2. Hardware Configuration

| 功能 | 外设 | 引脚 | 参数 | 代码入口 |
|------|------|------|------|----------|
| LD2-RS RS485 | USART3 | PD8 TX / PD9 RX / PB14 DE | 115200 8N2 | `bsp_rs485_init()` |
| VOFA+ / 调试串口 | USART1 | PA9 TX / PA10 RX | 115200 8N1 | `bsp_vofa_just_float()` |
| 预留串口 | USART2 | CubeMX 配置 | CubeMX 配置 | `MX_USART2_UART_Init()` |
| SBUS 遥控 | UART5 | PB13 TX / PD2 RX | 100000 8E2 | `bsp_rc_init()` |
| WS4810 LED | SPI6 | CubeMX 配置 | SPI 模拟时序 | `app_led_indicator_update()` |
| 急停 | GPIO EXTI | CubeMX 配置 | 1=停机 | `g_emergency_stop_flag` |
| 调试定时 | TIM6 | 内部定时器 | 1 ms 中断 | `HAL_TIM_PeriodElapsedCallback()` |

---

## 3. Software Architecture

```text
Core/
  main.c, usart.c, gpio.c, dma.c, spi.c, tim.c, stm32h7xx_it.c
  CubeMX 生成，主循环和外设初始化入口

App/
  app_config.h
  app_chassis_motor_ctrl.c
  app_encoder_dev.c
  app_ld2rs_task.c
  app_led_indicator.c
  应用业务、差速底盘、电机 FSM、安全策略、状态灯

BSP/
  bsp_dwt.c
  bsp_rs485.c
  bsp_rc.c
  bsp_vofa.c
  bsp_ws4810.c
  硬件抽象和外设驱动

Middleware/
  Filter/low_pass_filter.c
  Protocol/modbus_rtu.c
  Motor/ld2_motor.c
  纯算法/协议/设备寄存器封装
```

依赖规则：

```text
Middleware: 不依赖 HAL/BSP/APP
BSP:        可依赖 HAL 和 Middleware，不依赖 APP
APP:        可依赖 BSP 和 Middleware
Core:       调用 BSP/APP 入口
```

---

## 4. Runtime Flow

### 4.1 Init

```text
HAL_Init()
SystemClock_Config()
MX_GPIO_Init()
MX_DMA_Init()
MX_USART3_UART_Init()
MX_USART1_UART_Init()
MX_USART2_UART_Init()
MX_UART5_Init()
MX_SPI6_Init()
MX_TIM6_Init()
BSP_DWT_Init()                         # 校验 SystemCoreClock=480 MHz
HAL_TIM_Base_Start_IT(&htim6)          # 1 ms 调试中断
bsp_rs485_init(&g_rs485_bus, &huart3)
ld2_motor_init(&g_ld2rs_dev_m1, ..., slave_id=1)
ld2_motor_init(&g_ld2rs_dev_m2, ..., slave_id=2)
app_chassis_ld2rs_motor_ctrl_init(M1/M2)
app_chassis_ld2rs_motor_ctrl_param_read_back(M1/M2)
app_chassis_ld2rs_motor_ctrl_pi_init(M1/M2)
bsp_rc_init(&g_rc)
app_ld2rs_task_init()
app_led_indicator_init()
```

`HAL_Delay(APP_SYSTEM_START_DELAY_MS)` 当前在 `main.c` 中已注释，不属于实际启动流程。TIM6 回调仅用于观察 1 ms 计数和 DWT 的 ms/us 时间戳；编码器采样时间取自 RS485 RX IDLE 完成时记录的 DWT 时间戳。

### 4.2 Main Loop

```text
while (1)
  app_ld2rs_task_run()
    bsp_rs485_poll()
    app_rc_channels_check_lost()
    app_diff_chassis_motor_ctrl_rc_map_to_chassis()
    apply SWB/SWC unlock and VRA/VRB throttle
    app_diff_drive_compute()
    apply FSM force_zero_speed
    apply emergency / RC lost / SWA_DOWN stop
    app_motor_fsm_scheduler_step()
    app_encoder_update(M1/M2)
    publish estimated motor rpm to g_vofa_speed

  App_Vofa_UpperDisplay()
  app_led_indicator_update()
```

---

## 5. RC System

### 5.1 SBUS Parser

- 帧长：25 bytes。
- 通道数：16。
- 中位：`BSP_SBUS_MID_VALUE = 992`。
- 旋钮中位：`BSP_SBUS_VAR_VALUE = 192`。
- 摇杆归一化：约 `-800..+800`。
- UART5 接收方式：`HAL_UARTEx_ReceiveToIdle_DMA()`，关闭 DMA HT 中断，依赖 IDLE 对齐一帧。
- 硬件反相：`SET_BIT(huart5.Instance->CR2, (1UL << 19U))`。

### 5.2 Channel Map

| SBUS | 代码字段 | 当前用途 |
|------|----------|----------|
| CH1 | `g_rc.ch_ry` | 角速度方向 |
| CH2 | `g_rc.ch_rx` | 线速度方向 |
| CH3 | `g_rc.ch_lx` | 保留/滤波 |
| CH4 | `g_rc.ch_ly` | 保留/滤波 |
| CH5 | `SWA` | DOWN 强制停机 |
| CH6 | `SWB` | UP 解锁线速度 |
| CH7 | `SWC` | UP 解锁角速度 |
| CH8 | `SWD` | 保留 |
| CH9 | `vra` | 线速度油门 |
| CH10 | `vrb` | 角速度油门 |

### 5.3 Filter And Safety

- 一阶低通：`low_pass_filter_update_float()`。
- 低通通道：`ch_ry/ch_rx/ch_lx/ch_ly`。
- 死区：`BSP_RC_STICK_DEADZONE = 150.0f`，当前使用 `ch_rx/ch_ry` 方形死区。
- 掉线：`BSP_RC_TIMEOUT_MS = 200` ms 无有效帧。

---

## 6. Differential Drive

参数：

| 参数 | 值 |
|------|----|
| 轮距 | 657.0 mm |
| 半轮距 | 328.5 mm |
| 轮半径 | 75.0 mm |
| 减速比 | 20.0 |
| 转速限幅 | 4500 rpm |

公式：

```text
scale = 60 * gear / (2 * pi * wheel_radius_mm)
      = 2.5465 rpm per (mm/s)

left_rpm  = (v * 1000 - omega * 328.5) * scale * APP_MOTOR_CTRL_M1_POLARITY
right_rpm = (v * 1000 + omega * 328.5) * scale * APP_MOTOR_CTRL_M2_POLARITY
```

`app_diff_drive_limit_rpm()` 做两轮等比例限幅，不单独裁剪一侧，保持转弯半径。

---

## 7. LD2-RS Motor Control

### 7.1 Init Registers

| Step | 行为 | 寄存器 |
|------|------|--------|
| 1 | 确认/设置速度模式 | `Pr0.01 = 1` |
| 2 | 写零速 | `Pr3.04 = 0` |
| 3 | 确认/设置内部速度源 | `Pr3.00 = 1` |
| 4 | 写加减速 | `Pr3.12 / Pr3.13 = 250` |
| 5 | 写速度环 PI | `Pr1.01 = 810`, `Pr1.02 = 45` |
| 6 | 读回波特率设置 | `Pr5.30` |

### 7.2 Runtime FSM

```text
IDLE
  -> READ_REQ
  -> WAIT_READ
  -> READ_DONE
  -> WRITE_REQ
  -> WAIT_WRITE
  -> WRITE_DONE
  -> IDLE
```

READ 由两个独立子事务组成：

1. 状态/速度：功能码0x03，从 `PrB.05` 开始读2个寄存器，取得运行状态与 `PrB.06` 未滤波实时转速。
2. 编码器位置：功能码0x03，从 `PrB.24` 高16位地址开始读2个寄存器，将高低字合并成有符号 `int32_t` 累计位置。

共享总线的实际轮询顺序为：

```text
M1 状态/速度 -> M2 状态/速度
-> M1 编码器 -> M2 编码器
-> M1 写速度 -> M2 写速度
```

WRITE 请求：

- 功能码：0x06。
- 寄存器：`LD2_MOTOR_REG_SPEED_TARGET` (`Pr3.04`)。
- 值：差速解算后的目标 rpm，经过限幅和安全覆盖。

重试：

| 层级 | 次数 | 说明 |
|------|------|------|
| BSP RS485 | `BSP_RS485_MAX_RETRY = 3` | 物理事务超时自动重发 |
| APP FSM | `APP_LD2RS_TASK_MAX_RETRY = 3` | CRC/站号/功能码/超时逻辑重试 |
| 离线确认 | `APP_LD2_MOTOR_OFFLINE_CONFIRM_COUNT = 3` | 连续无应答确认离线 |

状态/速度子事务的APP重试耗尽后置 `force_zero_speed`，本轮进入零速写入；编码器子事务重试耗尽只放弃本轮观测并保留旧样本，不阻断后续速度写入。安全停机期间会取消正在等待的编码器事务。

---

## 8. RS485 / Modbus

BSP 非阻塞状态：

```text
IDLE -> TX_BUSY -> RX_WAIT -> DONE
                    \------> TIMEOUT
                    \------> ERROR
```

关键约束：

- `bsp_rs485_start_tx()` 只在 IDLE 发起。
- `bsp_rs485_poll()` 每轮主循环统一调用一次。
- DONE/TIMEOUT/ERROR 必须由调用者 `bsp_rs485_ack_done()` 释放。
- Modbus 帧间隔由 `APP_MODBUS_RTU_INTERFRAME_MS = 1` ms 控制。
- `state`、`rx_len`、`rx_timestamp_us` 和UART错误标志由ISR与主循环共享，均按 `volatile` 发布。
- RX IDLE回调先写入实际长度与 `BSP_DWT_GetTickUs()`，最后把状态置为DONE，APP只对完整有效帧发布编码器样本。
- TX_BUSY和RX_WAIT都会检查 `timeout_ms`；超时后中止当前UART收发，并由 `bsp_rs485_retry_or_timeout()`执行物理重发或进入粘滞TIMEOUT。

常用寄存器：

| 地址 | 寄存器 | 说明 |
|------|--------|------|
| 0x0003 | Pr0.01 | 控制模式 |
| 0x0103 | Pr1.01 | 速度环 Kp |
| 0x0105 | Pr1.02 | 速度环 Ti |
| 0x0301 | Pr3.00 | 速度源 |
| 0x0309 | Pr3.04 | 内部速度指令 |
| 0x0319 | Pr3.12 | 加速时间 |
| 0x031B | Pr3.13 | 减速时间 |
| 0x053D | Pr5.30 | RS485 波特率 |
| 0x0B05 | PrB.05 | 运行状态 |
| 0x0B06 | PrB.06 | 未滤波实时速度 |
| 0x0B09 | PrB.09 | 滤波后实际速度（已定义，运行时未读取） |
| 0x0B1C | PrB.24 H | 电机侧累计位置高16位 |
| 0x0B1D | PrB.24 L | 电机侧累计位置低16位 |

---

## 9. Encoder Speed Estimation

实体电机为LD2-RS4810，编码器后缀M17，当前按默认17位磁编码器配置：

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `APP_LD2_ENCODER_BITS` | 17 | 允许的配置分支为17或23位 |
| `LD2_ENCODER_COUNTS_PER_REV` | 131072.0 | 电机转子每圈计数 |
| `APP_LD2_ENCODER_POLARITY_M1/M2` | 1 / 1 | 估算速度方向修正 |
| `APP_CHASSIS_GEAR_RATIO` | 20.0 | 电机:车轮 = 20:1 |
| `APP_CHASSIS_WHEEL_RADIUS_MM` | 75.0 | 车轮半径 |

`PrB.24` 是电机侧、带符号32位累计位置，不是减速器输出轴位置。每次有效编码器响应发布一个完整样本：`counts + rx_timestamp_us + sequence`。首次样本只建立基准；只有序号变化且时间差非零时才更新速度。

```text
delta_counts = int32累计位置的32位回绕差分 × encoder_polarity
motor_speed_rpm = delta_counts × 60 × 1000000
                  / (131072 × delta_time_us)
wheel_speed_rpm = motor_speed_rpm / 20
wheel_speed_mps = wheel_speed_rpm × 2π × 0.075 / 60
```

差分中间量使用 `int64_t`，32位累计位置跨越有符号边界时按 `2^32` 回绕处理，不按17位单圈范围回绕。结果可通过 `app_ld2rs_task_get_speed_feedback()`读取。

---

## 10. VOFA+ Channels

当前 `Core/Src/main.c` 实际发送 `float vofa_channels[23]`，周期20 ms：

| CH | 变量 |
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
| 20 | `g_vofa_speed.m1_encoder_estimated_speed_rpm`，与CH2对比 |
| 21 | `g_vofa_speed.m2_encoder_estimated_speed_rpm`，与CH4对比 |
| 22 | `g_vofa_speed.est_error_m1_rpm` |
| 23 | `g_vofa_speed.est_error_m2_rpm` |

CH22/CH23目前只完成字段声明和VOFA发送，应用任务尚未计算赋值，因此初始化后保持0。若要观测真实误差，需先确定误差方向（例如 `PrB.06 - DWT估算rpm`）再在速度发布函数中赋值。`vofa_motor_info_t` 中的给定-反馈误差、read RTT、write RTT仍未进入发送数组。

---

## 11. LED Indicator

`app_led_indicator_update()` 根据系统状态选择 WS4810 颜色：

| 优先级 | 状态 | 表现 |
|--------|------|------|
| 1 | 急停 | 红色 |
| 2 | 电机离线 | 紫色闪烁，离线几台闪几次 |
| 3 | SWA_DOWN | 黄色待命 |
| 4 | 正常 | 绿色 |

---

## 12. Configuration Summary

| 宏 | 当前值 |
|----|--------|
| `APP_LD2_MOTOR_SLAVE_ID_M1` | 1 |
| `APP_LD2_MOTOR_SLAVE_ID_M2` | 2 |
| `APP_LD2_MOTOR_COUNT` | 2 |
| `APP_LD2_MOTOR_MAX_COUNT` | 3 |
| `APP_LD2_MOTOR_TIMEOUT_MS` | 100 |
| `APP_LD2_MOTOR_OFFLINE_CONFIRM_COUNT` | 3 |
| `APP_MODBUS_RTU_INTERFRAME_MS` | 1 |
| `APP_MOTOR_CTRL_ACCEL_MS_PER_1000RPM` | 250 |
| `APP_MOTOR_CTRL_DECEL_MS_PER_1000RPM` | 250 |
| `APP_SYSTEM_START_DELAY_MS` | 200 |
| `APP_CHASSIS_SPEED_LIMIT_RPM` | 4500 |
| `APP_CHASSIS_WHEEL_TRACK_MM` | 657.0 |
| `APP_CHASSIS_WHEEL_RADIUS_MM` | 75.0 |
| `APP_CHASSIS_GEAR_RATIO` | 20.0 |
| `APP_LD2_ENCODER_BITS` | 17 |
| `LD2_ENCODER_COUNTS_PER_REV` | 131072.0 |
| `APP_LD2_ENCODER_POLARITY_M1/M2` | 1 / 1 |
| `APP_CHASSIS_CMD_POLARITY` | -1 |
| `APP_MOTOR_CTRL_M1_POLARITY` | 1 |
| `APP_MOTOR_CTRL_M2_POLARITY` | -1 |
| `APP_VOFA_JUSTFLOAT_ENABLE` | 1 |
| `APP_VOFA_JUSTFLOAT_PERIOD_MS` | 20 |
| `BSP_RC_TIMEOUT_MS` | 200 |
| `BSP_RC_STICK_DEADZONE` | 150.0 |
| `BSP_RC_LPF_TAU_MS` | 200.0 |

`APP_MOTOR_CTRL_COMM_TIMEOUT_MS`、`APP_MOTOR_CTRL_MAX_RETRY_COUNT`、`APP_MOTOR_CTRL_ERROR_RETRY_DELAY_MS` 当前保留在 `app_config.h`，运行时 FSM 未直接使用。

---

## 13. Build Specification

### 13.1 Keil

```text
MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx
```

- 主开发、调试、烧录入口。
- 使用本地 `DSP/` 源码目录，不依赖固定电脑上的 Keil CMSIS-DSP Pack 路径。

### 13.2 GCC Makefile

```powershell
make
make V=1
make clean
```

Makefile 当前配置：

| 项 | 值 |
|----|----|
| 工具链 | `D:/Msys2/mingw64/bin` |
| CC/AS | `arm-none-eabi-gcc.exe` |
| MCU flags | `-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard` |
| 宏 | `USE_HAL_DRIVER`, `STM32H723xx`, `USE_PWR_LDO_SUPPLY`, `DISABLEFLOAT16` |
| Startup | `GCC/startup_stm32h723xx_gcc.s` |
| Linker | `GCC/STM32H723VGTx_FLASH.ld` |
| 输出 | `build/*.elf`, `.map`, `.hex`, `.bin` |

适配检查结论：

- Makefile 中列出的文件均存在。
- `Core/App/BSP/Middleware/Drivers` 当前非 DSP 固件 `.c` 文件没有遗漏，也没有多余项。
- DSP 使用 `DSP/Source` 下 16 个非 F16 聚合源文件。
- `Core/Src/tim.c`、`App/Src/app_encoder_dev.c`、`BSP/Src/bsp_dwt.c` 已纳入构建。

---

## 14. Diagnostics

| 现象 | 观察点 |
|------|--------|
| 电机不转 | CH8~11 离线、RS485 A/B、站号、波特率、SWA |
| 遥控无响应 | CH6 掉线、SBUS 接线、UART5 反相、接收机供电 |
| 一直强制零速 | 急停、SWA DOWN、RC lost、FSM force_zero_speed |
| 方向反 | M1/M2 极性或角速度极性 |
| 响应慢 | RC 低通时间常数和驱动器加减速时间 |
| 离线紫闪 | LD2-RS 无应答或连续超时 |
| DWT估速差异大 | CH2/CH20或CH4/CH21、编码器极性、采样周期、实际减速比 |
| CH22/CH23恒为0 | 当前误差字段尚未在APP任务中计算赋值 |

