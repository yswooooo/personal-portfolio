# 四轮转向底盘 V1 设计说明

## 目标

在 STM32H723 工程中实现 4 个 ROBSTRIDE RS00 舵向电机的位置控制。V1 只控制舵向角度，不控制轮毂电机速度，不增加舵向电机速度控制、力矩控制，也不向上层暴露底盘反馈 `vx/vy/vw`。

## 当前分层

- `bsp_can_robostride.c/h`：合并 FDCAN 初始化、29 位扩展帧发送、ROBSTRIDE RS00 协议打包、反馈解码、四个舵向电机结构体管理。
- `app_steer_chassis.c/h`：负责遥控映射、底盘运动学解算、写入 `steer_chassis_t.modules[]` 中的目标角度，再把整个 `steer_chassis_t *` 传给 BSP 直接发送。

CAN 和 ROBSTRIDE 协议现在统一放在 `bsp_can_robostride.c/h`。旧的通用 CAN 文件和旧的 ROBSTRIDE middleware 已从当前构建链路移除。

## 数据结构

`steer_chassis_t` 和 `steer_module_t` 定义在 `BSP/Inc/bsp_can_robostride.h`。

模块顺序固定为：

- `APP_STEER_MODULE_FL = 0`：左前轮
- `APP_STEER_MODULE_RL = 1`：左后轮
- `APP_STEER_MODULE_RR = 2`：右后轮
- `APP_STEER_MODULE_FR = 3`：右前轮

`steer_module_t` 字段：

- `index`
- `robstride_motor_id`
- `robstride_zero_angle_rad`
- `robstride_target_angle_rad`
- `robstride_feedback_angle_rad`
- `robstride_feedback_speed_rad_s`
- `robstride_last_tx_tick_ms`

`robstride_last_tx_tick_ms` 是每个舵轮电机自己的 CAN 发送打桩计时字段。每次实际发出 ROBSTRIDE 帧后，只更新该模块的 tick。

`steer_chassis_t` 额外包含：

- `target_command`
- `robstride_next_tx_index`

`robstride_next_tx_index` 用来记录下一次运行期目标角发送时优先检查的模块，避免每次都从左前轮开始。

## 初始化流程

`main.c` 中顺序为：

1. `MX_FDCAN1_Init()`
2. `bsp_can_robostride_init()`
3. `app_steer_chassis_init()`

`app_steer_chassis_init()` 内部会调用：

```c
bsp_can_robostride_init_steer_chassis(&s_robstride_steer_chassis, ...);
bsp_can_robostride_start_steer_chassis(&s_robstride_steer_chassis);
```

`bsp_can_robostride_init_steer_chassis()` 负责绑定 4 个电机 ID 和零点角度。`bsp_can_robostride_start_steer_chassis()` 负责依次对 4 个电机发送 CSP 模式、使能、默认速度限幅。

启动配置阶段允许阻塞等待帧间隔，目的是保证启动配置帧完整发出。

## 运行期发送调度

运行期入口：

```c
bsp_can_robostride_send_steer_chassis_position(&s_robstride_steer_chassis);
```

运行期发送规则：

1. 每次调用最多实际发送 1 帧。
2. 先读取 `HAL_GetTick()`。
3. 若距离最近一次任意 ROBSTRIDE CAN 发送不足 `ROBSTRIDE_CAN_TX_INTERVAL_MS`，直接返回。
4. 从 `robstride_next_tx_index` 指向的模块开始检查。
5. 若当前模块距离自己上一次发送不足 1ms，跳过它并检查下一个模块。
6. 成功发送后更新该模块的 `robstride_last_tx_tick_ms`。
7. 成功发送后把 `robstride_next_tx_index` 指向下一个模块。

正常轮询顺序为：左前、左后、右后、右前。

## 控制流程

遥控入口：

```c
app_steer_chassis_rc_control(&g_rc_filter);
```

内部流程：

1. `app_steer_chassis_rc_map_to_command()` 将遥控通道映射到 `target_command.vx_mps / vy_mps / wz_rad_s`。
2. `app_steer_chassis_task()` 解算四个舵向角。
3. 解算结果写入 `s_robstride_steer_chassis.modules[i].robstride_target_angle_rad`。
4. 调用 `bsp_can_robostride_send_steer_chassis_position()` 尝试发送一个目标角。

## 坐标系

车体坐标系：

- 正前方为 `x+`
- 左侧为 `y+`
- 逆时针旋转为 `wz+`

运动学：

```c
wheel_velocity_x_mps = vx_mps - wz_rad_s * module_y_m;
wheel_velocity_y_mps = vy_mps + wz_rad_s * module_x_m;
angle_offset_rad = atan2(wheel_velocity_y_mps, wheel_velocity_x_mps);
robstride_target_angle_rad = angle_offset_rad - robstride_zero_angle_rad;
```

`atan2` 使用 CMSIS-DSP 的 `arm_atan2_f32()`。

## ROBSTRIDE CAN 协议

ROBSTRIDE RS00 当前使用：

- Classic CAN 2.0
- 29 位扩展 ID
- DLC 8
- FDCAN Classic CAN 格式，不启用 BRS

CSP 位置控制发送流程：

1. 写 `ROBSTRIDE_PARAM_RUN_MODE = ROBSTRIDE_RUN_MODE_POSITION_CSP`
2. 发送使能命令
3. 写 `ROBSTRIDE_PARAM_LIMIT_SPD`
4. 周期写 `ROBSTRIDE_PARAM_LOC_REF`

反馈帧只写回：

- `robstride_feedback_angle_rad`
- `robstride_feedback_speed_rad_s`

## BSP 接口

公开接口全部是直接处理风格，不再向 App 层返回无用状态：

```c
void bsp_can_robostride_init(void);
void bsp_can_robostride_init_steer_chassis(steer_chassis_t *chassis,
                                           float zero_angle_fl_rad,
                                           float zero_angle_rl_rad,
                                           float zero_angle_rr_rad,
                                           float zero_angle_fr_rad);
void bsp_can_robostride_start_steer_chassis(steer_chassis_t *chassis);
void bsp_can_robostride_stop_steer_chassis(steer_chassis_t *chassis,
                                           uint8_t clear_error);
void bsp_can_robostride_send_steer_chassis_position(steer_chassis_t *chassis);
void bsp_can_robostride_update_steer_chassis_feedback(steer_chassis_t *chassis,
                                                      uint32_t extended_id,
                                                      const uint8_t data[8]);
```

## 构建集成

Makefile 和 Keil 工程只需要包含：

- `BSP/Src/bsp_can_robostride.c`
- `App/Src/app_steer_chassis.c`

## V1 暂不实现

- 舵向电机速度控制
- 舵向电机力矩控制
- 反馈力矩上报到 App
- 轮毂电机速度控制
- 根据舵角误差进行轮速缩放
- 超过 90 度时反转轮速并优化舵角
- 闭环底盘里程计
- 底盘反馈 `vx/vy/vw`
