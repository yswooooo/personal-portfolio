# 公开电机FSM与运动反馈设计

## 目标

在现有编码器差分估算中增加电机转子增量角度和减速后轮毂增量角度，并公开M1/M2的 `motor_fsm_t` 实例，供后续里程计直接读取编码器运动状态。新增一个接收 `const motor_fsm_t *` 的API，返回驱动器 `PrB.06` 未滤波实际转速。

## 类型与实例可见性

- 将 `motor_fsm_t` 的完整定义从 `App/Src/app_ld2rs_task.c` 移到 `App/Inc/app_ld2rs_task.h`。
- 将私有实例 `s_motor_fsm_m1`、`s_motor_fsm_m2` 改名并取消 `static`：

```c
motor_fsm_t g_motor_fsm_m1;
motor_fsm_t g_motor_fsm_m2;
```

- 在 `app_ld2rs_task.h` 中用 `extern` 声明两个实例。
- FSM内部辅助函数继续保持 `static`，只公开类型、实例和指定getter。

## 编码器角度增量

在 `app_encoder_speed_estimator_t` 中保留已有 `int64_t delta_counts`，新增：

```c
float delta_motor_angle_rad;
float delta_wheel_angle_rad;
```

仅在新的有效编码器样本到达且 `delta_time_us != 0` 时更新：

```c
delta_motor_angle_rad =
    delta_counts * 2.0f * PI / LD2_ENCODER_COUNTS_PER_REV;

delta_wheel_angle_rad =
    delta_motor_angle_rad / APP_CHASSIS_GEAR_RATIO;
```

`delta_motor_angle_rad` 是电机转子本次增量角度；`delta_wheel_angle_rad` 已包含20:1减速比，是轮毂本次真实增量角度。两者单位均为rad，并继承 `APP_LD2_ENCODER_POLARITY_M1/M2` 修正后的方向。

首次有效样本只建立基准，两个角度保持初始化值0；无新样本时估算函数立即返回，不重复计算。调用者通过 `speed_estimator.last_sequence` 判断同一增量是否已经消费，比较方式使用不等于，不依赖序号大小。

## 实际转速API

在 `app_ld2rs_task.h` 声明并在 `app_ld2rs_task.c` 实现：

```c
int16_t app_ld2rs_status_getter(const motor_fsm_t *motor);
```

行为：

- `motor == NULL` 时返回0。
- 有效指针返回 `(int16_t)motor->actual_speed_rpm`。
- 返回值来自 `PrB.06` 未滤波实际转速，不是DWT编码器估算转速，也不是 `status_word`。

调用示例：

```c
int16_t m1_actual_speed_rpm =
    app_ld2rs_status_getter(&g_motor_fsm_m1);
```

## 兼容性与验证

- 现有 `app_ld2rs_task_get_speed_feedback()` 保留不变。
- 所有原 `s_motor_fsm_m1/m2` 引用机械替换为 `g_motor_fsm_m1/m2`。
- 验证角度公式、正负方向、首样本行为、无新样本不重复更新、getter的M1/M2与NULL行为。
- 完成主机侧编码器测试和完整GCC固件构建。
