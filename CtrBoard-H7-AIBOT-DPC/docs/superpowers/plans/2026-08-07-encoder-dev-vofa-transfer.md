# Encoder Dev Rename and VOFA Transfer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将编码器模块统一命名为 `encoder_dev`，并将两台电机 FSM 到 VOFA 的全部观测数据集中到状态机外的单一传递函数。

**Architecture:** 编码器模块继续负责位置差分、角度和速度计算，但文件、类型、API 与 FSM 字段改用通用 `encoder` 命名。`motor_fsm_t` 作为运行数据源，`app_ld2rs_vofa_data_transfer()` 在 `app_ld2rs_task_run()` 末尾一次性生成完整 `vofa_motor_info_t` 快照，FSM 内部不再直接写 VOFA。

**Tech Stack:** C11、STM32H723、GNU Arm Embedded Makefile、Keil MDK XML 工程文件。

## Global Constraints

- 不保留 `app_encoder_speed` 旧文件、旧类型或旧函数兼容别名。
- 保留 `motor_speed_rpm`、`wheel_speed_rpm`、`speed_counts_per_s` 等物理量字段名。
- `app_ld2rs_vofa_data_transfer()` 接收三个完整结构体指针，任一参数为 NULL 时直接返回。
- 状态机只更新 `motor_fsm_t`，不得直接读写 `g_vofa_speed`。
- VOFA 目标转速取 `target_speed_rpm`，反馈转速按 `(int16_t)actual_speed_rpm` 解释。
- 第 5 项单位换算宏不在本次修改范围内。
- 不暂存现有中文文档移动和 `.claude/`。

---

### Task 1: 编码器通用命名与 VOFA 集中传递

**Files:**
- Rename: `App/Inc/app_encoder_speed.h` → `App/Inc/app_encoder_dev.h`
- Rename: `App/Src/app_encoder_speed.c` → `App/Src/app_encoder_dev.c`
- Modify: `App/Inc/app_ld2rs_task.h`
- Modify: `App/Src/app_ld2rs_task.c`
- Modify: `Makefile`
- Modify: `MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx`
- Modify: `MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvoptx`
- Modify locally (gitignored): `MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvguix.fantasy`
- Modify: `CLAUDE.md`
- Modify: `CtrBoard-H7-AIBOT-DPC_Project_Specification.md`
- Reference: `docs/superpowers/specs/2026-08-07-ld2rs-vofa-data-transfer-design.md`

**Interfaces:**
- Produces: `app_encoder_sample_t`, `app_encoder_estimator_t`, `app_encoder_update(...)`。
- Produces internally: `app_ld2rs_vofa_data_transfer(vofa_motor_info_t *, const motor_fsm_t *, const motor_fsm_t *)`。
- Removes: `app_encoder_speed_estimator_t`, `app_encoder_speed_update(...)`, `speed_estimator`, `app_ld2rs_vofa_encoder_speed_publish(...)`, `app_motor_fsm_sync_vofa_offline(...)`。

- [ ] **Step 1: 运行基线构建并验证重构约束当前失败**

运行：

```powershell
make -j4
$old = rg -n "app_encoder_speed|app_encoder_speed_estimator_t|speed_estimator|app_ld2rs_vofa_encoder_speed_publish|app_motor_fsm_sync_vofa_offline" App Makefile MDK-ARM CLAUDE.md CtrBoard-H7-AIBOT-DPC_Project_Specification.md
if (-not $old) { throw "Expected old encoder/VOFA names before refactor" }
```

期望：构建成功，约束检查找到旧文件、旧类型、旧函数和 FSM 内 VOFA 同步函数，证明检查能捕获待重构状态。

- [ ] **Step 2: 重命名编码器模块文件和公开符号**

执行文件移动后更新内容：

```text
App/Inc/app_encoder_dev.h
  include guard: APP_ENCODER_DEV_H
  type: app_encoder_estimator_t
  API: app_encoder_update(...)

App/Src/app_encoder_dev.c
  include: app_encoder_dev.h
  helpers: app_encoder_get_delta(), app_encoder_save_sample()
  API: app_encoder_update(...)
```

保持函数算法、字段布局和公式不变。将 `app_ld2rs_task.h/.c` 的包含、类型与字段同步改为：

```c
#include "app_encoder_dev.h"

app_encoder_sample_t encoder_sample;
app_encoder_estimator_t encoder_estimator;
```

所有调用改为：

```c
app_encoder_update(&g_motor_fsm_m1.encoder_sample,
                   &g_motor_fsm_m1.encoder_estimator,
                   APP_LD2_ENCODER_POLARITY_M1);
```

- [ ] **Step 3: 集中实现 FSM 到 VOFA 的字段传递**

在 `app_ld2rs_task.c` 定义：

```c
static void app_ld2rs_vofa_data_transfer(
    vofa_motor_info_t *vofa_data,
    const motor_fsm_t *motor_m1,
    const motor_fsm_t *motor_m2)
{
    float feedback_m1;
    float feedback_m2;

    if ((vofa_data == NULL) || (motor_m1 == NULL) || (motor_m2 == NULL)) {
        return;
    }

    feedback_m1 = (float)((int16_t)motor_m1->actual_speed_rpm);
    feedback_m2 = (float)((int16_t)motor_m2->actual_speed_rpm);

    vofa_data->m1_ref_speed_rpm = (float)motor_m1->target_speed_rpm;
    vofa_data->m1_feedback_speed_rpm = feedback_m1;
    vofa_data->m1_speed_error_rpm =
        vofa_data->m1_ref_speed_rpm - feedback_m1;
    vofa_data->m1_read_rtt_ms = (float)motor_m1->read_rtt_ms;
    vofa_data->m1_write_rtt_ms = (float)motor_m1->write_rtt_ms;
    vofa_data->m1_cycle_ms = (float)motor_m1->cycle_elapsed_ms;

    vofa_data->m2_ref_speed_rpm = (float)motor_m2->target_speed_rpm;
    vofa_data->m2_feedback_speed_rpm = feedback_m2;
    vofa_data->m2_speed_error_rpm =
        vofa_data->m2_ref_speed_rpm - feedback_m2;
    vofa_data->m2_read_rtt_ms = (float)motor_m2->read_rtt_ms;
    vofa_data->m2_write_rtt_ms = (float)motor_m2->write_rtt_ms;
    vofa_data->m2_cycle_ms = (float)motor_m2->cycle_elapsed_ms;

    vofa_data->m1_offline_total_count =
        (float)motor_m1->offline_total_count;
    vofa_data->m1_offline_confirmed =
        (float)motor_m1->is_offline_confirmed;
    vofa_data->m2_offline_total_count =
        (float)motor_m2->offline_total_count;
    vofa_data->m2_offline_confirmed =
        (float)motor_m2->is_offline_confirmed;

    vofa_data->m1_encoder_position_counts =
        (float)motor_m1->encoder_position_counts;
    vofa_data->m2_encoder_position_counts =
        (float)motor_m2->encoder_position_counts;
    vofa_data->m1_encoder_estimated_speed_rpm =
        motor_m1->encoder_estimator.motor_speed_rpm;
    vofa_data->m2_encoder_estimated_speed_rpm =
        motor_m2->encoder_estimator.motor_speed_rpm;
    vofa_data->est_error_m1_rpm =
        vofa_data->m1_encoder_estimated_speed_rpm - feedback_m1;
    vofa_data->est_error_m2_rpm =
        vofa_data->m2_encoder_estimated_speed_rpm - feedback_m2;
}
```

其中目标、反馈和两类误差的核心计算规则为：

```c
feedback_m1 = (float)((int16_t)motor_m1->actual_speed_rpm);
feedback_m2 = (float)((int16_t)motor_m2->actual_speed_rpm);

vofa_data->m1_ref_speed_rpm = (float)motor_m1->target_speed_rpm;
vofa_data->m2_ref_speed_rpm = (float)motor_m2->target_speed_rpm;
vofa_data->m1_speed_error_rpm = vofa_data->m1_ref_speed_rpm - feedback_m1;
vofa_data->m2_speed_error_rpm = vofa_data->m2_ref_speed_rpm - feedback_m2;
vofa_data->est_error_m1_rpm =
    motor_m1->encoder_estimator.motor_speed_rpm - feedback_m1;
vofa_data->est_error_m2_rpm =
    motor_m2->encoder_estimator.motor_speed_rpm - feedback_m2;
```

其余 RTT、周期、离线、编码器位置和 DWT 估算转速按设计规范逐字段强制转换为 `float` 后复制。

- [ ] **Step 4: 移除状态机内部全部 VOFA 写入**

删除 `app_motor_fsm_sync_vofa_offline()` 的声明、定义和三处调用。删除 `app_motor_fsm_step()` 内反馈转速、读 RTT、编码器位置、目标/反馈误差、写 RTT 和周期的 `g_vofa_speed` 赋值，仅保留对应 `motor_fsm_t` 字段更新。

在 `app_ld2rs_task_run()` 两次 `app_encoder_update()` 之后调用：

```c
app_ld2rs_vofa_data_transfer(
    &g_vofa_speed,
    &g_motor_fsm_m1,
    &g_motor_fsm_m2);
```

- [ ] **Step 5: 更新构建系统、Keil 工程和当前文档**

把以下活动工程引用从 `app_encoder_speed.c` 改为 `app_encoder_dev.c`：

```text
Makefile
MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx
MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvoptx
MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvguix.fantasy
CLAUDE.md
CtrBoard-H7-AIBOT-DPC_Project_Specification.md
```

项目规范中的 API 名称同步改为 `app_encoder_update()`。历史设计和历史实施计划保留当时名称，不作追溯性重写。`.uvguix.fantasy` 是 `.gitignore` 排除的个人 GUI 配置，只更新当前本地文件，不强制纳入提交。

- [ ] **Step 6: 验证旧命名清零和 VOFA 单向数据流**

运行：

```powershell
$old = rg -n "app_encoder_speed|app_encoder_speed_estimator_t|speed_estimator|app_ld2rs_vofa_encoder_speed_publish|app_motor_fsm_sync_vofa_offline" App Makefile MDK-ARM CLAUDE.md CtrBoard-H7-AIBOT-DPC_Project_Specification.md
if ($old) { throw "Old encoder/VOFA names remain: $old" }

$globalUse = rg -n "g_vofa_speed" App/Src/app_ld2rs_task.c
if (($globalUse | Measure-Object).Count -ne 1) { throw "g_vofa_speed must be used once" }

$assignments = rg -n "vofa_data->[A-Za-z0-9_]+\s*=" App/Src/app_ld2rs_task.c
if (($assignments | Measure-Object).Count -ne 22) { throw "Expected all 22 VOFA fields" }

git diff --check
```

期望：活动源码和工程配置没有旧命名；`g_vofa_speed` 只在统一调用处出现；集中函数恰好赋值 22 个 VOFA 字段。

- [ ] **Step 7: 构建并提交**

运行：

```powershell
make -B -j4
```

期望：从干净构建目录编译并链接成功，生成 ELF/HEX/BIN。

仅暂存本任务文件并提交：

```powershell
git add -A -- App/Inc/app_encoder_speed.h App/Inc/app_encoder_dev.h App/Src/app_encoder_speed.c App/Src/app_encoder_dev.c App/Inc/app_ld2rs_task.h App/Src/app_ld2rs_task.c Makefile MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvoptx CLAUDE.md CtrBoard-H7-AIBOT-DPC_Project_Specification.md docs/superpowers/plans/2026-08-07-encoder-dev-vofa-transfer.md
git diff --cached --check
git commit -m "refactor: unify encoder module and VOFA transfer"
```
