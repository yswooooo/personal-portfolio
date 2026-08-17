# App 与 Filter Doxygen 注释完善 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为全部 App 层及 Middleware Filter 的公开接口、数据类型和内部辅助函数补齐简洁、一致且能解释物理含义、调用约束与典型用法的 Doxygen 注释。

**Architecture:** 公开 API 的权威说明位于对应 `Inc` 头文件，内部 `static` 函数的说明位于 `Src` 定义之前；结构体成员和枚举值使用相邻的 `/**< ... */` 注释。Filter 头文件额外提供可复制的 `@code` 使用示例，只修改注释，不改变预处理结果、类型布局、函数签名或运行逻辑。

**Tech Stack:** C11、Doxygen 注释语法、STM32H723、GNU Arm Embedded Makefile 构建。

## Global Constraints

- 中文解释为主，代码标识符、寄存器名、状态名和单位保持英文形式。
- 指针参数必须标注 `[in]`、`[out]` 或 `[in,out]`。
- 所有非 `void` 函数必须明确返回值语义。
- 速度、角度、时间和编码器位置必须明确 `rpm`、`rad`、`m/s`、`ms`、`us` 或 `count` 单位。
- 不修改函数签名、结构体布局、枚举值、宏值、变量名、控制流或计算公式。
- 不暂存现有未跟踪的 `.claude/` 目录。
- `low_pass_filter_update_float()` 的说明必须明确无状态特性、历史输出保存责任、首次初始化方式和 `alpha` 的 `[0, 1]` 前置条件。

---

### Task 1: 完善 App 层 Doxygen 注释

**Files:**
- Modify: `App/Inc/app_led_indicator.h`
- Modify: `App/Inc/app_encoder_speed.h`
- Modify: `App/Inc/app_ld2rs_task.h`
- Modify: `App/Inc/app_chassis_motor_ctrl.h`
- Modify: `App/Src/app_led_indicator.c`
- Modify: `App/Src/app_encoder_speed.c`
- Modify: `App/Src/app_ld2rs_task.c`
- Modify: `App/Src/app_chassis_motor_ctrl.c`
- Modify: `Middleware/Filter/Inc/low_pass_filter.h`
- Modify: `Middleware/Filter/Src/low_pass_filter.c`
- Reference only: `App/Inc/app_config.h`
- Reference: `docs/superpowers/specs/2026-08-07-app-doxygen-comments-design.md`

**Interfaces:**
- Consumes: 当前公开函数签名、FSM 类型、编码器采样与速度估算类型、底盘控制类型和无状态一阶低通滤波函数。
- Produces: 不新增接口；为现有接口和内部函数提供 Doxygen 可解析说明。

- [ ] **Step 1: 记录构建和注释缺口基线**

运行：

```powershell
make -j4
rg -n "typedef enum|typedef struct|^[a-zA-Z_].*\(" App/Inc App/Src Middleware/Filter
```

期望：工程构建成功；扫描结果显示 `app_ld2rs_task.h`、`app_encoder_speed.h`、`app_led_indicator.h` 以及多个 `static` 函数缺少相邻的完整 Doxygen 注释。

- [ ] **Step 2: 完善公开头文件的类型和接口说明**

在 `App/Inc` 中按以下清单补充：

```text
app_led_indicator.h
  app_led_indicator_init
  app_led_indicator_update

app_encoder_speed.h
  app_encoder_sample_t 及 counts/timestamp_us/sequence
  app_encoder_speed_estimator_t 的状态、历史采样、差分角度和速度字段
  app_encoder_speed_update

app_ld2rs_task.h
  motor_ctrl_phase_t 的每个事务阶段
  motor_read_transaction_t 的两类读取事务
  motor_fsm_t 的帧、反馈、估算器、状态、重试、设备、时序、离线和安全字段
  motor_fsm_scheduler_t
  app_ld2rs_speed_feedback_t
  g_motor_fsm_m1 / g_motor_fsm_m2
  app_ld2rs_task_init
  app_ld2rs_task_run
  app_ld2rs_task_get_speed_feedback
  app_ld2rs_status_getter

app_chassis_motor_ctrl.h
  ld2rs_motor_ctrl_t
  motor_speed_pi_t
  motor_ctrl_param_t
  vofa_motor_info_t 的所有通道字段，包含 est_error_m1_rpm/est_error_m2_rpm
  全部 extern 实例
  app_chassis_ld2rs_motor_ctrl_param_read_back
  app_chassis_ld2rs_motor_ctrl_init
  app_chassis_ld2rs_motor_ctrl_pi_init
  app_diff_drive_compute
  app_diff_drive_limit_rpm
  app_rc_channels_check_lost
  app_diff_chassis_motor_ctrl_rc_map_to_chassis
```

统一函数模板：

```c
/**
  * @brief      一句话说明用途
  * @param[in]  input  输入对象、语义和单位
  * @param[out] output 输出对象、语义和单位
  * @return     每类返回值的具体含义
  * @note       调用周期、NULL 行为或状态机约束
  */
```

将 `app_diff_drive_compute()` 的 `linear_velocity_mps` 单位统一为 `m/s`，与函数名和实现中的乘 `1000.0f` 保持一致。

同时完善 `Middleware/Filter/Inc/low_pass_filter.h`：

```c
/**
  * @brief      更新无状态一阶低通滤波器
  * @param[in]  input       当前采样值 x[k]
  * @param[in]  last_output 上一次滤波输出 y[k-1]
  * @param[in]  alpha       滤波系数，调用者保证范围为 [0, 1]
  * @return     当前滤波输出 y[k]
  * @note       函数不保存状态，每个信号必须独立保存上一次返回值。
  */
```

头文件加入以下完整使用示例：

```c
float filtered = initial_input;
const float alpha = sample_period_s / (sample_period_s + time_constant_s);

filtered = low_pass_filter_update_float(raw_input, filtered, alpha);
```

说明第一次把 `filtered` 初始化为首个输入可避免从零启动的跳变，并说明 `alpha` 越小越平滑但响应越慢。`Middleware/Filter/Src/low_pass_filter.c` 在函数定义前使用 `@copydoc low_pass_filter_update_float`。

- [ ] **Step 3: 完善源文件内部函数说明**

为以下内部对象添加完整 Doxygen；已有正确的完整注释保留，缺少的标签补齐：

```text
app_encoder_speed.c
  app_encoder_speed_get_delta
  app_encoder_speed_save_sample
  app_encoder_speed_update（使用 @copydoc 或保留实现级补充说明）

app_ld2rs_task.c
  app_motor_fsm_step_result_t 及各枚举值
  app_encoder_sample_publish
  app_ld2rs_speed_feedback_copy
  app_ld2rs_vofa_encoder_speed_publish
  app_encoder_runtime_reset
  app_ld2rs_task_put_u16
  app_ld2rs_task_get_u16
  app_ld2rs_task_build_read_req
  app_ld2rs_task_build_write_req
  app_ld2rs_task_validate_resp
  app_motor_fsm_init
  app_motor_fsm_sync_vofa_offline
  app_motor_fsm_mark_response_ok
  app_motor_fsm_mark_no_response
  app_motor_fsm_safety_stop_active
  app_motor_fsm_handle_read_failure
  app_motor_fsm_step
  app_motor_fsm_scheduler_init
  app_motor_fsm_scheduler_advance
  app_motor_fsm_scheduler_step
  app_ld2rs_task_run
  app_ld2rs_task_get_speed_feedback
  app_ld2rs_status_getter
  app_ld2rs_task_init

app_chassis_motor_ctrl.c
  app_chassis_ld2rs_motor_ctrl_write_reg
  已有公开函数注释中的参数方向、返回语义和单位

app_led_indicator.c
  app_led_indicator_init
  app_led_indicator_update
```

成员和枚举值统一使用：

```c
uint64_t timestamp_us; /**< DWT 微秒时间戳，us */
```

不要添加复述赋值语句的逐行注释，也不要把内部状态机函数暴露到头文件。

- [ ] **Step 4: 执行注释覆盖与无行为变更审查**

运行：

```powershell
rg -n "@brief|@param\[|@return|@retval|@code|/\*\*<" App/Inc App/Src Middleware/Filter
git diff --check
git diff -- App/Inc App/Src
```

逐项确认：

```text
1. 清单中的函数、枚举和结构体都有相邻 Doxygen 注释。
2. 非 void 函数均说明返回语义。
3. 指针参数均说明方向。
4. 结构体字段说明物理意义和单位。
5. diff 中没有函数签名、表达式、控制流、初始值或结构体字段顺序变化。
6. app_config.h 未产生无关修改。
7. Filter 头文件包含初始化、周期更新、状态保存和 alpha 范围说明。
```

- [ ] **Step 5: 重新构建 ARM 工程**

运行：

```powershell
make -j4
```

期望：退出码为 `0`，生成 `build/CtrBoard-H7-AIBOT-DPC.elf`、`.hex` 和 `.bin`；若仍出现已有 ELF RWX 段警告，记录但不在本任务中处理。

- [ ] **Step 6: 创建独立提交**

仅暂存八个 App 文件和两个 Filter 文件：

```powershell
git add -- App/Inc/app_led_indicator.h App/Inc/app_encoder_speed.h App/Inc/app_ld2rs_task.h App/Inc/app_chassis_motor_ctrl.h App/Src/app_led_indicator.c App/Src/app_encoder_speed.c App/Src/app_ld2rs_task.c App/Src/app_chassis_motor_ctrl.c Middleware/Filter/Inc/low_pass_filter.h Middleware/Filter/Src/low_pass_filter.c
git diff --cached --check
git commit -m "docs: add doxygen comments to app layer"
```

期望：提交仅包含注释与单位描述修正，`.claude/` 保持未跟踪状态。
