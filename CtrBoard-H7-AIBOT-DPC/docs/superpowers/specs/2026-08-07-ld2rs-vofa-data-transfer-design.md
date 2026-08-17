# LD2-RS VOFA 数据集中传递设计

## 目标

统一编码器模块的通用命名，并将 `motor_fsm_t` 到 `g_vofa_speed` 的观测数据转换集中到一个状态机外部函数。移除电机状态机内部对 VOFA 展示结构的直接写入，使状态机只维护通信、控制和反馈运行时。

## 编码器模块命名统一

编码器模块同时计算位置差分、角度增量和速度，不再使用仅表示速度的模块名称。完整重命名如下：

| 当前名称 | 新名称 |
|---|---|
| `App/Src/app_encoder_speed.c` | `App/Src/app_encoder_dev.c` |
| `App/Inc/app_encoder_speed.h` | `App/Inc/app_encoder_dev.h` |
| `app_encoder_speed_update()` | `app_encoder_update()` |
| `app_encoder_speed_get_delta()` | `app_encoder_get_delta()` |
| `app_encoder_speed_save_sample()` | `app_encoder_save_sample()` |
| `app_encoder_speed_estimator_t` | `app_encoder_estimator_t` |
| `motor_fsm_t.speed_estimator` | `motor_fsm_t.encoder_estimator` |

`app_encoder_sample_t` 已是通用编码器命名，保持不变。估算器内部的 `motor_speed_rpm`、`wheel_speed_rpm` 和 `speed_counts_per_s` 等字段描述具体物理量，也保持不变。

不提供旧文件、旧类型或旧函数名称的兼容别名。同步更新 App 头文件与源文件、Makefile，以及 Keil `.uvprojx`、`.uvoptx`、`.uvguix.fantasy` 中的源文件引用，确保 GNU Arm 和 Keil 工程均指向 `app_encoder_dev.c`。

## 数据所有权

`motor_fsm_t` 是每台电机运行状态的唯一数据源，状态机只更新以下自身字段：

- `target_speed_rpm`
- `actual_speed_rpm`
- `encoder_position_counts`
- `read_rtt_ms`
- `write_rtt_ms`
- `cycle_elapsed_ms`
- `offline_total_count`
- `is_offline_confirmed`
- `encoder_estimator`

状态机及其离线辅助函数不得直接读取或写入 `g_vofa_speed`。

## 集中传递函数

将现有 `app_ld2rs_vofa_encoder_speed_publish()` 重命名并扩展为：

```c
static void app_ld2rs_vofa_data_transfer(
    vofa_motor_info_t *vofa_data,
    const motor_fsm_t *motor_m1,
    const motor_fsm_t *motor_m2);
```

三个参数均传递完整结构体地址，不拆分传递单独字段。函数为 `static`，仅供 `app_ld2rs_task.c` 内部使用；参数为 NULL 时直接返回，不写入目标结构。

## 字段映射

函数每次调用完整刷新两台电机的 VOFA 数据：

| VOFA 字段 | M1 数据源 | M2 数据源 |
|---|---|---|
| `m*_ref_speed_rpm` | `motor_m1->target_speed_rpm` | `motor_m2->target_speed_rpm` |
| `m*_feedback_speed_rpm` | `(int16_t)motor_m1->actual_speed_rpm` | `(int16_t)motor_m2->actual_speed_rpm` |
| `m*_speed_error_rpm` | 目标转速减反馈转速 | 目标转速减反馈转速 |
| `m*_read_rtt_ms` | `motor_m1->read_rtt_ms` | `motor_m2->read_rtt_ms` |
| `m*_write_rtt_ms` | `motor_m1->write_rtt_ms` | `motor_m2->write_rtt_ms` |
| `m*_cycle_ms` | `motor_m1->cycle_elapsed_ms` | `motor_m2->cycle_elapsed_ms` |
| `m*_offline_total_count` | `motor_m1->offline_total_count` | `motor_m2->offline_total_count` |
| `m*_offline_confirmed` | `motor_m1->is_offline_confirmed` | `motor_m2->is_offline_confirmed` |
| `m*_encoder_position_counts` | `motor_m1->encoder_position_counts` | `motor_m2->encoder_position_counts` |
| `m*_encoder_estimated_speed_rpm` | `motor_m1->encoder_estimator.motor_speed_rpm` | `motor_m2->encoder_estimator.motor_speed_rpm` |
| `est_error_m*_rpm` | DWT 估算转速减驱动器反馈转速 | DWT 估算转速减驱动器反馈转速 |

`target_speed_rpm` 已由遥控映射、安全停机和差速限幅流程确定，用作 VOFA 目标转速。驱动器反馈保留当前有符号 16 bit 解释方式。

## 状态机清理

从 `app_motor_fsm_step()` 中删除反馈转速、读写 RTT、编码器位置、目标转速、转速误差和周期的 `g_vofa_speed` 写入。

删除 `app_motor_fsm_sync_vofa_offline()` 的声明、定义和调用；`app_motor_fsm_mark_response_ok()` 与 `app_motor_fsm_mark_no_response()` 只更新 `motor_fsm_t` 离线字段。

## 调用位置

在 `app_ld2rs_task_run()` 中完成以下步骤后调用一次集中传递函数：

1. 推进共享 RS485 电机调度器。
2. 更新 M1 编码器速度估算器。
3. 更新 M2 编码器速度估算器。

调用形式：

```c
app_ld2rs_vofa_data_transfer(
    &g_vofa_speed,
    &g_motor_fsm_m1,
    &g_motor_fsm_m2);
```

这样 VOFA 获得同一主循环时刻的两台电机快照，状态机不再依赖 VOFA 数据结构。

## 验证

- 搜索 `g_vofa_speed`，确认 `app_ld2rs_task.c` 中只有集中传递函数调用使用全局实例。
- 搜索旧函数名和 `app_motor_fsm_sync_vofa_offline`，确认不存在残留。
- 搜索 `app_encoder_speed`、`app_encoder_speed_estimator_t` 和 `speed_estimator`，确认源码与工程配置中不存在旧命名残留。
- 使用字段映射检查确认 `vofa_motor_info_t` 的全部字段均由集中传递函数赋值。
- 执行 `git diff --check`。
- 执行 `make -j4`，确认 ARM 工程编译链接成功且固件尺寸未发生非预期变化。
- 仅暂存本项涉及的源文件和必要头文件，不纳入现有中文文档移动、`.claude/` 或第 5 项单位宏改动。
