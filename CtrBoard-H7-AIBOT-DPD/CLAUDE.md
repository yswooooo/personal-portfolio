# CLAUDE.md

> 新会话进入本目录后先读本文件。当前工程身份为 `CtrBoard-H7-AIBOT-DPD`。

## 0. 工作约定

- 每次新任务先检查是否有相关 skill/workflow 可用。
- 不要覆盖用户已有未提交源码改动。
- CubeMX 生成文件只在 `USER CODE BEGIN/END` 区间内编辑。
- 修改 C 源码、启动文件、链接脚本、Makefile 或 Keil 配置后, 优先运行 `make` 做 GCC 命令行验证。

## 1. 当前工程事实

| 项 | 当前状态 |
|----|----------|
| 工程名 | `CtrBoard-H7-AIBOT-DPD` |
| 主控 | STM32H723VGTx, 达妙 DM-MC-Board02 |
| 主运行模型 | 四个 ROBSTRIDE RS00 舵向电机, PP 位置控制 |
| 保留模型 | LD2-RS/RS485 二轮差速链路仍在源码中, 但运行期任务当前未启用 |
| CAN | FDCAN1, PD0 RX / PD1 TX, 1 Mbps, Classic CAN 2.0, 29-bit extended ID |
| 裸机调度 | TIM6 1 ms 更新中断，累计 20 次触发一组 4 帧舵向 CAN |
| 示波器测试脚 | PA0 `TEST_PIN_FOR_SCOPE`，推挽输出，默认低电平 |
| ROBSTRIDE CAN ID | FL=1, RL=2, RR=3, FR=4 |
| RC | SBUS, UART5 DMA+IDLE, 100000 8E2 |
| VOFA | USART1, JustFloat, 20 ms period, 当前实际 23 floats |
| RS485 | USART3, 115200 8N2, LD2-RS 保留链路 |
| Serial Ctrl | UART7, PE7(TX)/PE8(RX), 115200 8N1, DMA+IDLE, PC 串口控制 |
| LED | WS4810 over SPI6 |

## 2. 构建入口

Keil 主工程:

```text
MDK-ARM/CtrBoard-H7-AIBOT-DPD.uvprojx
```

GCC Makefile 验证:

```powershell
make
make V=1
make clean
```

GCC 产物:

```text
build/CtrBoard-H7-AIBOT-DPD.elf
build/CtrBoard-H7-AIBOT-DPD.map
build/CtrBoard-H7-AIBOT-DPD.hex
build/CtrBoard-H7-AIBOT-DPD.bin
```

Keil 仍是正式调试和烧录入口; GCC Makefile 主要用于命令行编译/链接验证。

## 3. 关键目录

```text
Core/
  main.c                     启动顺序、主循环、VOFA 发送

App/
  app_steer_chassis.c/h      四舵轮 RC 映射、运动学、零偏、安全停机、run_direct
  app_serial_ctrl.c/h        串口控制 (UART7 → steer_chassis_run_direct)
  app_chassis_motor_ctrl.c/h LD2-RS 初始化、差速计算、RC 掉线检测
  app_ld2rs_task.c/h         保留的 LD2-RS 非阻塞 FSM
  app_led_indicator.c/h      WS4810 状态灯
  app_config.h               业务参数

BSP/
  bsp_can_robostride.c/h     FDCAN 初始化、ROBSTRIDE 发送、反馈 ISR 写回
  bsp_rc.c/h                 SBUS 解析和一阶低通滤波包装
  bsp_rs485.c/h              RS485 半双工非阻塞状态机
  bsp_serial_protocol.c/h    UART7 串口控制协议 (55AA 帧头 + CRC16)
  bsp_vofa.c/h               VOFA+ 发送
  bsp_ws4810.c/h             LED 驱动

Middleware/
  RobStride/                 RS00 协议打包/解包和反馈缓存
  Motor/                     LD2-RS 寄存器封装
  Protocol/                  Modbus RTU
  Filter/                    一阶低通滤波
```

## 4. 当前主循环

`Core/Src/main.c` 当前 `while(1)`:

```text
app_rc_channels_check_lost(&g_rc)
app_rc_channels_set_emergency_flag(&g_rc)
app_serial_ctrl_poll()
if (app_serial_ctrl_is_active()) → app_steer_chassis_run_direct()
else → app_steer_chassis_rc_control(&g_rc_filter)
App_Vofa_UpperDisplay()
app_led_indicator_update()
```

`app_ld2rs_task_run()` 当前被注释。不要把当前固件描述为运行中的二轮差速底盘。

## 4b. 串口控制链路 (UART7)

```text
UART7 IDLE ISR
  -> HAL_UARTEx_RxEventCallback()
  -> bsp_serial_protocol_feed(s_uart7_rx_buf, len)
  -> 主循环 app_serial_ctrl_poll()
     -> bsp_serial_protocol_poll(): 帧头 0x55AA + CRC16 校验
     -> on_serial_frame() 回调
     -> 构建响应帧 & HAL_UART_Transmit_DMA(&huart7)
```

控制源切换（main.c while(1)）：
- SWD==DOWN 或 RC 掉线 → `app_serial_ctrl_is_active()` 为 true
- 激活时调用 `app_steer_chassis_run_direct(vx, vy, wz)` 绕过 RC 映射
- 未激活时走标准 `app_steer_chassis_rc_control(&g_rc_filter)`
- 串口命令 500ms 超时自动归零

协议帧定义见 `BSP/Inc/bsp_serial_protocol.h`：PC→MCU 30B (6 floats), MCU→PC 38B (8 floats)。

TIM6 调度链：

```text
TIM6_DAC_IRQHandler()
  -> HAL_TIM_IRQHandler(&htim6)
  -> HAL_TIM_PeriodElapsedCallback()
  -> 每 20 个 1 ms tick 置位 s_steerwheel_chassis_task_20ms_flag
  -> app_steer_chassis_run() 先清标志，再发送 FL/RL/RR/FR 四帧 CAN
```

`app_wheel_task_run()` 位于 20 ms CAN 分支之外，必须保持主循环高频推进。PA0 验证应在消费标志时调用一次 `HAL_GPIO_TogglePin()`；不能在主循环中持续镜像 flag 电平，否则只会得到很窄的脉冲。

## 5. 舵向控制链路

```text
app_steer_chassis_rc_control(&g_rc_filter)
  -> app_steer_chassis_run()
     -> deadzone/filter channel values
     -> map RC to vx/vy/wz
     -> safety stop
     -> compute FL/RL/RR/FR wheel velocity vectors
     -> atan2f(vy_i, vx_i)
     -> fold angle to [-pi/2, +pi/2]
     -> write hub_reverse_flag and hub_target_speed_mps
     -> target = folded_angle + steer_zero_offset_rad
     -> bsp_can_robostride_send_steer_chassis_position()
```

Current RC mapping in source:

```text
ch_rx -> vx
-ch_ly -> vy
-ch_ry -> wz
```

Right-handed kinematics:

```c
vx_i = vx - wz * y_i;
vy_i = vy + wz * x_i;
angle = atan2f(vy_i, vx_i);
target = folded_angle + steer_zero_offset_rad;
```

Module geometry:

| Module | Enum | CAN ID | x | y |
|--------|------|--------|---|---|
| FL | `APP_STEER_MODULE_FL` | 1 | +0.1675 | +0.200425 |
| RL | `APP_STEER_MODULE_RL` | 2 | -0.1675 | +0.200425 |
| RR | `APP_STEER_MODULE_RR` | 3 | -0.1675 | -0.200425 |
| FR | `APP_STEER_MODULE_FR` | 4 | +0.1675 | -0.200425 |

## 6. ROBSTRIDE Notes

Startup per motor (10 steps, comm type in brackets):

```text
① run_mode = 1 (PP)                    [0x12]
② enable motor                         [0x03]
③ EPScan_time = 3 (20ms)              [0x12]
④ enable periodical report             [0x18]
⑤ vel_max = 20.0 rad/s                [0x12]
⑥ acc_set = 100.0 rad/s²              [0x12]
⑦ loc_ref = steer_zero_offset_rad     [0x12]
⑧ spd_kp = 180 / loc_kp = 150         [0x12]
⑨ zero_sta = 1 (-π~π)                [0x12]
⑩ save all params to Flash             [0x16]
```

FDCAN key params: ExtFiltersNbr=1, StdFiltersNbr=0, AutoRetransmission=ENABLE.

Protocol layer:

- `robstride_motor_build_ext_id()`
- `robstride_motor_pack_run_mode_pp()`
- `robstride_motor_pack_enable()`
- `robstride_motor_pack_enable_report()`
- `robstride_motor_pack_epscan_time()`
- `robstride_motor_pack_vel_max()`
- `robstride_motor_pack_acc_set()`
- `robstride_motor_pack_position_ref()`
- `robstride_motor_pack_zero_sta()`
- `robstride_motor_pack_spd_kp()`
- `robstride_motor_pack_loc_kp()`
- `robstride_motor_pack_save_params()`
- `robstride_motor_parse_rx_frame()`

Feedback path:

```text
HAL_FDCAN_RxFifo0Callback()
  -> robstride_motor_parse_rx_frame()
  -> robstride_motor_set_last_update_ms()
  -> robstride_motor_get_feedback()
  -> module->feedback = *fb
  -> position_new = position_raw - steer_zero_offset
```

## 7. VOFA Current Truth

`App_Vofa_UpperDisplay()` currently sends `float vofa_channels[23]`.

Payload groups:

- emergency stop, RC lost, SWA state
- four steering target angles
- four steering feedback angles
- four hub reverse flags
- four hub target speed magnitudes

Older 17-channel and 23-channel tables are historical unless the actual array in `main.c` changes.

## 8. Retained LD2-RS Chain

LD2-RS files are still valid but are not the current runtime main path:

- `app_chassis_motor_ctrl.c/h`
- `app_ld2rs_task.c/h`
- `bsp_rs485.c/h`
- `ld2_motor.c/h`
- `modbus_rtu.c/h`

Boot still initializes two LD2-RS devices and writes configuration. Runtime read/write FSM becomes active only if `app_ld2rs_task_run()` is restored in the main loop.

## 9. Common Pitfalls

- Do not document `ch_lx -> vx`; current source uses `ch_rx -> vx`.
- Do not document current VOFA as 23 channels; source sends 19 floats.
- Do not document CSP as the active ROBSTRIDE mode; source writes PP mode.
- Do not say wheel hub speed is sent; only `hub_target_speed_mps` and `hub_reverse_flag` are stored.
- `robstride_next_tx_index` exists but current runtime send implementation iterates modules directly.

## 10. Project Identity Files

Keep these names aligned:

```text
CtrBoard-H7-AIBOT-DPD.ioc
CtrBoard-H7-AIBOT-DPD_Project_Specification.md
MDK-ARM/CtrBoard-H7-AIBOT-DPD.uvprojx
MDK-ARM/CtrBoard-H7-AIBOT-DPD.uvoptx
MDK-ARM/CtrBoard-H7-AIBOT-DPD/
Makefile PROJECT := CtrBoard-H7-AIBOT-DPD
```

