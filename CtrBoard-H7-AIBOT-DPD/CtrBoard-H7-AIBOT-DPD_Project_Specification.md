# CtrBoard-H7-AIBOT-DPD Project Specification

> Version: 2026-07-20
> MCU: STM32H723VGTx on Damiao DM-MC-Board02
> Primary runtime path: SBUS -> four-wheel steering kinematics -> ROBSTRIDE RS00 steering position control + DS/RS/UM hub speed control
> Legacy retained path: LD2-RS RS485 differential-drive code, initialized at boot but not run in the current main loop

## 1. System Overview

CtrBoard-H7-AIBOT-DPD is currently a four-wheel steering chassis firmware project. The active runtime path drives four ROBSTRIDE RS00 steering motors over FDCAN1 in PP position mode and four DS/RS/UM hub motors over USART2 RS485 in speed mode. The firmware keeps the older LD2-RS/USART3 differential-drive modules in the tree, but `Core/Src/main.c` currently comments out `app_ld2rs_task_run()`, so the LD2-RS runtime FSM does not participate after initialization.

Main responsibilities:

- Receive SBUS commands through UART5 DMA + IDLE.
- Map filtered stick channels and the raw VRA/VRB knobs to `vx`, `vy`, `wz`.
- Compute four steering target angles using right-handed inverse kinematics.
- Fold steering angles into `[-pi/2, +pi/2]` and use wheel reverse flags when generating signed hub speed commands.
- Send ROBSTRIDE RS00 `loc_ref` commands over FDCAN1.
- Convert four hub linear-speed magnitudes to signed rpm, uniformly desaturate them to `±200 rpm`, and send the commands over USART2 RS485.
- Decode ROBSTRIDE feedback in the FDCAN RX interrupt and expose debug data through VOFA+.

## 2. Hardware Interfaces

| Function | Peripheral | Pins | Runtime role |
|----------|------------|------|--------------|
| ROBSTRIDE RS00 steering x4 | FDCAN1 | PD0 RX / PD1 TX | Active, 1 Mbps Classic CAN 2.0 extended frame |
| DS/RS/UM hub motor x4 | USART2 RS485 | PD5 TX / PD6 RX / PD4 DE | Active, 115200 8N1 Modbus RTU, IDs 01/02/03/04 |
| Bare-metal scheduler | TIM6 | Internal time base | 1 ms update interrupt, 20 ms steering CAN event |
| Scheduler scope probe | GPIOA | PA0 | `TEST_PIN_FOR_SCOPE`, push-pull, initial low |
| SBUS RC | UART5 | PB13 TX / PD2 RX | Active, 100000 8E2 DMA + IDLE |
| VOFA+ debug | USART1 | PA9 TX / PA10 RX | Active, JustFloat 115200 8N1 |
| LD2-RS RS485 | USART3 | PD8 TX / PD9 RX / PB14 DE | Retained, boot init still runs |
| WS4810 LED | SPI6 | CubeMX configured | Active status indicator |

## 3. Architecture

```text
Core/
  CubeMX generated entry code. main.c wires initialization and the main loop.

App/
  Application behavior: steering kinematics, hub motor polling and speed control,
  RC safety checks, LED logic, retained LD2-RS motor control and differential-drive FSM.

BSP/
  Hardware-facing drivers: FDCAN ROBSTRIDE, RS485, SBUS, VOFA, WS4810.

Middleware/
  Protocol and pure algorithm code: ROBSTRIDE packing/parsing, Modbus RTU,
  DS/RS/UM and LD2-RS register access, low-pass filtering.
```

Dependency rule:

```text
Middleware has no HAL/App dependency.
BSP depends on HAL and Middleware.
App depends on BSP and Middleware.
Core calls App and BSP entry points.
```

## 4. Runtime Flow

Boot sequence in `main()`:

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
MX_FDCAN1_Init()
MX_TIM6_Init()
bsp_rs485_init()
ld2_motor_init(M1/M2)
HAL_Delay(APP_SYSTEM_START_DELAY_MS)
app_chassis_ld2rs_motor_ctrl_init(M1/M2)
app_chassis_ld2rs_motor_ctrl_param_read_back(M1/M2)
app_chassis_ld2rs_motor_ctrl_pi_init(M1/M2)
bsp_rc_init()
app_ld2rs_task_init()
app_led_indicator_init()
bsp_can_robostride_init()
bsp_rs485_bus2_init()
ds_rs_um_motor_init(FL/RL/RR/FR, IDs 01/02/03/04)
set FL/RL/RR/FR to speed mode and enable them
app_wheel_task_init()
app_steer_chassis_init()
HAL_TIM_Base_Start_IT(&htim6)
```

TIM6 scheduling path:

```text
TIM6_DAC_IRQHandler()
  -> HAL_TIM_IRQHandler(&htim6)
  -> HAL_TIM_PeriodElapsedCallback()
     -> increment the private 1 ms software counter
     -> set s_steerwheel_chassis_task_20ms_flag every 20 counts
  -> app_steer_chassis_run() clears the flag before enqueueing four CAN frames
```

`APP_SCHEDULER_TICK_MS=1U` and `APP_STEER_TASK_PERIOD_MS=20U` are shared through `Core/Inc/main.h`. The event flag is declared `extern volatile`; the software counter remains private to `tim.c`.

Current main loop:

```text
while (1)
  app_rc_channels_check_lost(&g_rc)
  app_steer_chassis_rc_control(&g_rc_filter)
    -> app_steer_chassis_run()
       -> app_wheel_task_run(&s_robstride_steer_chassis)
  App_Vofa_UpperDisplay()
  app_led_indicator_update()
```

The steering kinematics and wheel RS485 FSM continue to run from the main loop. Only the four-frame ROBSTRIDE steering CAN burst is gated by the 20 ms event flag.

For oscilloscope validation, toggle PA0 once when consuming the event. Adjacent GPIO transitions are 20 ms apart; rising-edge to rising-edge is 40 ms because one toggle occurs per CAN burst. Treat the scheduler flag as an event, not as a GPIO level: mirroring its value in the main loop produces only a short pulse because the CAN branch clears it immediately.

`app_ld2rs_task_run()` is present but commented out.

## 5. Steering Data Model

Defined in `BSP/Inc/bsp_can_robostride.h`:

- `steer_chassis_t`: contains four modules, latest command, CAN TX timestamps, and a retained TX index field.
- `steer_module_t`: contains module index, ROBSTRIDE CAN ID, target angle, feedback, debug PP/PID parameters, software zero offset, hub target speed, and the reverse flag consumed by hub speed control.
- `robstride_motor_feedback_t`: middleware feedback cache copied into each module by the FDCAN RX callback.

Module order:

| Module | Enum | CAN ID | x (m) | y (m) |
|--------|------|--------|-------|-------|
| FL | `APP_STEER_MODULE_FL` | 1 | +0.1675 | +0.200425 |
| RL | `APP_STEER_MODULE_RL` | 2 | -0.1675 | +0.200425 |
| RR | `APP_STEER_MODULE_RR` | 3 | -0.1675 | -0.200425 |
| FR | `APP_STEER_MODULE_FR` | 4 | +0.1675 | -0.200425 |

Software zero offsets in `App/Inc/app_steer_chassis.h`:

| Macro | Value |
|-------|-------|
| `APP_STEER_FL_ZERO_OFFSET_RAD` | `0.0316486359` |
| `APP_STEER_RL_ZERO_OFFSET_RAD` | `-0.03663452` |
| `APP_STEER_RR_ZERO_OFFSET_RAD` | `0.00556182861` |
| `APP_STEER_FR_ZERO_OFFSET_RAD` | `-0.015919196854` |

## 6. RC Mapping And Kinematics

The sticks use the low-pass-filtered fields in `RC_Filter_t`. The VRA and VRB knobs currently use the unfiltered `g_rc.vra` and `g_rc.vrb` fields directly. Both knobs are clamped to `[0,1600]` and mapped linearly:

```c
translation_scale = clampf(g_rc.vra / 1600.0f, 0.0f, 1.0f);
rotation_scale    = clampf(g_rc.vrb / 1600.0f, 0.0f, 1.0f);
```

Current configured limits in `App/Inc/app_steer_chassis.h`:

| Macro | Value | Knob |
|-------|-------|------|
| `APP_STEER_CHASSIS_MAX_VX_MPS` | `1.5708 m/s` | VRA |
| `APP_STEER_CHASSIS_MAX_VY_MPS` | `1.5708 m/s` | VRA |
| `APP_STEER_CHASSIS_MAX_WZ_RAD_S` | `2.1654 rad/s` | VRB |

Current source mapping in `app_steer_chassis_run()` is equivalent to:

```c
vx = clampf(ch_rx / BSP_RC_STICK_MAX, -1.0f, 1.0f)
     * APP_STEER_CHASSIS_MAX_VX_MPS * translation_scale;

vy = clampf(-ch_ly / BSP_RC_STICK_MAX, -1.0f, 1.0f)
     * APP_STEER_CHASSIS_MAX_VY_MPS * translation_scale;

wz = clampf(-ch_ry / BSP_RC_STICK_MAX, -1.0f, 1.0f)
     * APP_STEER_CHASSIS_MAX_WZ_RAD_S * rotation_scale;
```

Right-handed vehicle frame:

```text
x+  forward
y+  left
wz+ counter-clockwise
```

Inverse kinematics:

```c
vx_i = vx - wz * y_i;
vy_i = vy + wz * x_i;
raw_angle = atan2f(vy_i, vx_i);
```

Angle folding:

```text
raw > +pi/2 -> raw - pi, hub_reverse_flag = 1
raw < -pi/2 -> raw + pi, hub_reverse_flag = 1
otherwise   -> raw,      hub_reverse_flag = 0
```

Final target:

```c
robstride_target_angle_rad = folded_angle + steer_zero_offset_rad;
```

The code also computes:

```c
hub_target_speed_mps = sqrtf(vx_i * vx_i + vy_i * vy_i);
```

### 6.1 Hub Speed Conversion And Desaturation

Current hub parameters are:

| Parameter | Value |
|-----------|-------|
| Maximum motor speed | `±200 rpm` |
| Wheel radius | `0.075 m` |
| Drive gear ratio | `1:1` |

`app_wheel_task_run()` converts each wheel's linear-speed magnitude to signed rpm. `hub_reverse_flag` supplies the kinematic direction and `APP_WHEEL_*_POLARITY` compensates for the physical mounting direction:

```text
signed_rpm = hub_speed_mps * 60 * drive_gear_ratio / (2*pi*wheel_radius)
             * reverse_sign * mounting_polarity
```

After all four target values are calculated, `app_wheel_apply_steer_chassis()` finds the largest absolute rpm. If it exceeds `200 rpm`, all four signed target values are multiplied by the same scale:

```text
speed_scale = 200 / max_abs_wheel_rpm
```

This keeps the wheel-speed relationship while guaranteeing a final maximum absolute command of `200 rpm`.

With `VRA=800`, `VRB=800`, and the indicated sticks at full travel, the current configuration produces approximately:

| Full-stick command | Fastest wheel before final limiting |
|--------------------|-------------------------------------|
| pure `vx` or pure `vy` | `100 rpm` |
| pure `wz` | `36 rpm` |
| `vx + vy` | `141 rpm` |
| `vx + wz` | `130 rpm` |
| `vx + vy + wz` | `177 rpm` |

These examples use the current `APP_STEER_CHASSIS_MAX_WZ_RAD_S=2.1654f`. VRA and VRB scale different command components; they are not a single master throttle for the final wheel-rpm limit.

### 6.2 Hub Motor Runtime

The USART2 RS485 task is active. Motor IDs and chassis positions are:

| Modbus ID | Position |
|-----------|----------|
| 01 | FL |
| 02 | RL |
| 03 | RR |
| 04 | FR |

`app_wheel_task_run()` is a non-blocking polling state machine. It repeatedly performs the following transaction sequence for the current motor before advancing to the next motor:

```text
READ D0.000 actual speed -> WRITE F0.3.018 target speed
```

## 7. Safety Behavior

`app_steer_chassis_run()` forces `vx=vy=wz=0` if any of the following is true:

- `g_emergency_stop_flag != 0`
- `g_rc.lost_flag != 0`
- `g_rc.sw_st[eRC_SW_A].curr == eRC_POS_DOWN`

The resulting hub target speeds are therefore zero. The active wheel FSM continues polling the USART2 RS485 bus and writes zero target speed to the four hub drives.

The LD2-RS runtime FSM has its own safety handling, but it is currently inactive because `app_ld2rs_task_run()` is not called in the main loop.

## 8. ROBSTRIDE CAN Protocol

Physical layer:

- FDCAN1
- Classic CAN 2.0
- 29-bit extended ID
- DLC = 8 bytes
- BRS off, CAN FD format off
- Host CAN ID: `0xFD`

Supported communication types:

| Mode | Direction | Use |
|------|-----------|-----|
| `0x02` | Motor -> host | Periodic feedback |
| `0x03` | Host -> motor | Enable motor |
| `0x04` | Host -> motor | Stop/reset motor |
| `0x11` | Host -> motor | Read single parameter |
| `0x12` | Host -> motor | Write single parameter |
| `0x16` | Host -> motor | Save parameters to Flash |
| `0x18` | Bidirectional | Configure/report feedback |

Startup sequence per motor:

```text
write run_mode = ROBSTRIDE_RUN_MODE_PP (1)
enable motor
enable 10 ms report
write vel_max = 20.0 rad/s
write acc_set = 100.0 rad/s^2
write loc_ref = steer_zero_offset_rad
write spd_kp = 180
write loc_kp = 150
```

Runtime command:

```text
mode 0x12, parameter 0x7016 loc_ref, value = target angle in rad
```

## 9. Feedback Path

`HAL_FDCAN_RxFifo0Callback()` accepts only:

- FDCAN1
- extended ID
- data frame
- DLC 8

Then it calls `robstride_motor_parse_rx_frame()`. The middleware parses:

- position raw rad/deg
- speed rad/s
- torque Nm
- temperature C
- motor ID
- fault code
- mode state

BSP copies the cached feedback into the matching module and computes software-zeroed steering angle:

```c
position_new_rad = position_raw_rad - steer_zero_offset_rad;
position_new_deg = position_new_rad * 180.0f / pi;
```

## 10. VOFA+ Current Payload

`App_Vofa_UpperDisplay()` currently allocates:

```c
float vofa_channels[21];
```

It sends 21 floats at `APP_VOFA_JUSTFLOAT_PERIOD_MS` through `bsp_vofa_just_float()`.

Current payload:

| Channels | Payload |
|----------|---------|
| CH01-CH03 | Emergency stop, RC lost flag, SWA state |
| CH04-CH07 | FL/RL/RR/FR steering target angles (deg) |
| CH08-CH11 | FL/RL/RR/FR raw steering feedback angles (deg) |
| CH12-CH15 | FL/RL/RR/FR hub reverse flags |
| CH16-CH19 | FL/RL/RR/FR hub target linear-speed magnitudes (m/s) |
| CH20 | `s_steerwheel_chassis_task_20ms_flag` |
| CH21 | `GPIOA_PIN0_FLAG` |

The current payload does not transmit the four signed, post-desaturation wheel target rpm values. Older 17-, 19-, or 23-channel tables are historical and should not be used as the current runtime truth.

## 11. Retained LD2-RS/RS485 Modules

Retained files:

- `App/Src/app_chassis_motor_ctrl.c`
- `App/Src/app_ld2rs_task.c`
- `BSP/Src/bsp_rs485.c`
- `Middleware/Motor/Src/ld2_motor.c`
- `Middleware/Protocol/Src/modbus_rtu.c`

The generic Modbus RTU and RS485 infrastructure is also used by the active USART2 wheel task; only the USART3 LD2-RS runtime task is inactive.

Boot-time LD2-RS configuration still executes:

- set speed mode
- write zero speed
- select internal speed source
- write acceleration/deceleration
- read back baud-rate parameter
- write speed-loop PI parameters

Runtime LD2-RS control is inactive unless `app_ld2rs_task_run()` is restored in `main.c`.

## 12. Build And Project Identity

Project identity is `CtrBoard-H7-AIBOT-DPD`.

Keil entry:

```text
MDK-ARM/CtrBoard-H7-AIBOT-DPD.uvprojx
```

Keil output directory:

```text
MDK-ARM/CtrBoard-H7-AIBOT-DPD/
```

CubeMX project file:

```text
CtrBoard-H7-AIBOT-DPD.ioc
```

GCC Makefile project name:

```makefile
PROJECT := CtrBoard-H7-AIBOT-DPD
```

GCC output:

```text
build/CtrBoard-H7-AIBOT-DPD.elf
build/CtrBoard-H7-AIBOT-DPD.map
build/CtrBoard-H7-AIBOT-DPD.hex
build/CtrBoard-H7-AIBOT-DPD.bin
```

## 13. Maintenance Notes

- Do not describe the current firmware as an active differential-drive chassis unless `app_ld2rs_task_run()` is restored.
- Treat USART2 DS/RS/UM hub control as part of the active four-wheel steering runtime path.
- Keep ROBSTRIDE protocol packing/parsing in Middleware and HAL/FDCAN operations in BSP.
- Keep kinematics in App.
- Keep CubeMX edits inside `USER CODE BEGIN/END` blocks.
- Update VOFA documentation from the actual `vofa_channels[]` definition, not from older tables.
- Keep the VRA/VRB mapping, speed constants, wheel parameters, and final `±200 rpm` desaturation documentation synchronized with the active source.

