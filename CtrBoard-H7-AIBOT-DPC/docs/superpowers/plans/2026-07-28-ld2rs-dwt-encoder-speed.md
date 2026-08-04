# LD2-RS DWT Encoder Speed Estimation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Estimate M1/M2 motor RPM, wheel RPM, and wheel linear speed from adjacent valid `PrB.24` samples and DWT microsecond timestamps.

**Architecture:** The RS485 ISR publishes receive length and timestamp before `DONE`; the existing APP FSM validates and publishes a per-motor encoder sample; a small pure `app_encoder_speed` module consumes each new sample exactly once and computes the three requested speeds. M1 and M2 own independent sample and estimator states, and no floating-point work runs in an ISR.

**Tech Stack:** C11, STM32H723 HAL, Cortex-M7 DWT `CYCCNT`, Modbus RTU, MinGW GCC host tests, GNU Arm Embedded firmware build.

## Global Constraints

- `M17` is a confirmed 17-bit magnetic encoder: `PrB.24` changes by 131072 encoder units per motor revolution.
- `Pr0.08 = 10000` is a command-pulse setting and must not be used in feedback-speed conversion.
- `APP_CHASSIS_GEAR_RATIO = 20.0f` means motor RPM : wheel RPM = 20 : 1.
- `APP_CHASSIS_WHEEL_RADIUS_MM = 75.0f`.
- Encoder polarity is independent from command polarity; core position difference is always `now_counts - last_counts`.
- Use `int64_t` for the signed encoder delta after explicit 32-bit modular wrap handling.
- Publish an encoder sample only after a valid 9-byte Modbus response for two `PrB.24` registers.
- Keep the existing RS485 FSM, Modbus scheduling order, timeout/retry/offline policy, safety behavior, VOFA transmission, and `WRITE_REQ` start-failure behavior unchanged.
- Do not add or configure a TIM; no suitable TIM task is enabled in the project.
- Preserve all pre-existing dirty-worktree changes. Stage or commit only task-owned hunks; never reset or overwrite unrelated edits.
- The pre-existing unified-read suite currently fails at the unrelated unknown-subtransaction timeout case. Record it as a baseline failure and do not fix it in this task.

---

### Task 1: Add the pure encoder-speed estimator

**Files:**
- Create: `App/Inc/app_encoder_speed.h`
- Create: `App/Src/app_encoder_speed.c`
- Modify: `App/Inc/app_config.h`
- Create: `tests/test_app_encoder_speed.c`

**Interfaces:**
- Consumes: `APP_LD2_ENCODER_BITS`, `LD2_ENCODER_COUNTS_PER_REV`, `APP_CHASSIS_GEAR_RATIO`, and `APP_CHASSIS_WHEEL_RADIUS_MM`.
- Produces: `app_encoder_sample_t`, `app_encoder_speed_estimator_t`, and `app_encoder_speed_update(...)`.

- [ ] **Step 1: Add the failing estimator tests**

Create `tests/test_app_encoder_speed.c` with a small `expect_near()` helper and tests for first-sample initialization, a normal second sample, negative motion, polarity reversal, zero time difference, duplicate sequence, and both 32-bit wrap directions.

The normal calculation fixture is:

```c
app_encoder_sample_t sample = {
    .counts = 1000,
    .timestamp_us = 100000u,
    .sequence = 1u
};
app_encoder_speed_estimator_t estimator = {0};

app_encoder_speed_update(&sample, &estimator, 1);
expect_true(estimator.speed_valid == 0u,
            "first sample must only establish the baseline");

sample.counts = 1120;
sample.timestamp_us = 110000u;
sample.sequence = 2u;
app_encoder_speed_update(&sample, &estimator, 1);

expect_true(estimator.delta_counts == 120,
            "second sample must preserve the signed encoder delta");
expect_true(estimator.delta_time_us == 10000u,
            "second sample must preserve the DWT interval");
expect_near(estimator.speed_counts_per_s, 12000.0f, 0.01f,
            "counts per second");
expect_near(estimator.motor_speed_rpm,
            12000.0f * 60.0f / 131072.0f, 0.001f,
            "motor rpm");
expect_near(estimator.wheel_speed_rpm,
            estimator.motor_speed_rpm / 20.0f, 0.001f,
            "wheel rpm");
expect_near(estimator.wheel_speed_mps,
            estimator.wheel_speed_rpm * 2.0f * 3.14159265f
                * 0.075f / 60.0f,
            0.00001f,
            "wheel linear speed");
```

For wrap tests use:

```c
/* Forward wrap: INT32_MAX -> INT32_MIN is +1 count. */
last.counts = INT32_MAX;
now.counts = INT32_MIN;

/* Reverse wrap: INT32_MIN -> INT32_MAX is -1 count. */
last.counts = INT32_MIN;
now.counts = INT32_MAX;
```

- [ ] **Step 2: Run the new test and verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IApp/Inc `
    tests/test_app_encoder_speed.c App/Src/app_encoder_speed.c `
    -o $env:TEMP\test_app_encoder_speed.exe
```

Expected: compilation fails because the new header and source do not exist.

- [ ] **Step 3: Add the restricted encoder and polarity configuration**

Append to `App/Inc/app_config.h`:

```c
#define APP_LD2_ENCODER_BITS  17U

#if APP_LD2_ENCODER_BITS == 17U
#define LD2_ENCODER_COUNTS_PER_REV  (131072.0f)
#elif APP_LD2_ENCODER_BITS == 23U
#define LD2_ENCODER_COUNTS_PER_REV  (8388608.0f)
#else
#error "APP_LD2_ENCODER_BITS must be 17U or 23U"
#endif

#define APP_LD2_ENCODER_POLARITY_M1  1
#define APP_LD2_ENCODER_POLARITY_M2  (-1)
```

Do not alias either encoder polarity macro to a command-polarity macro.

- [ ] **Step 4: Implement the minimal estimator**

Define in `App/Inc/app_encoder_speed.h`:

```c
typedef struct
{
    int32_t counts;
    uint64_t timestamp_us;
    uint32_t sequence;
} app_encoder_sample_t;

typedef struct
{
    uint8_t initialized;
    uint8_t speed_valid;
    uint8_t is_stale;
    int32_t last_counts;
    uint64_t last_timestamp_us;
    uint32_t last_sequence;
    int64_t delta_counts;
    uint64_t delta_time_us;
    float speed_counts_per_s;
    float motor_speed_rpm;
    float wheel_speed_rpm;
    float wheel_speed_mps;
} app_encoder_speed_estimator_t;

void app_encoder_speed_update(
    const app_encoder_sample_t *sample,
    app_encoder_speed_estimator_t *estimator,
    int8_t encoder_polarity);
```

Implement the portable modular delta in `App/Src/app_encoder_speed.c`:

```c
static int64_t app_encoder_speed_delta(int32_t now_counts,
                                       int32_t last_counts)
{
    uint32_t wrapped_delta;

    wrapped_delta =
        (uint32_t)now_counts - (uint32_t)last_counts;

    if (wrapped_delta <= (uint32_t)INT32_MAX) {
        return (int64_t)wrapped_delta;
    }

    return (int64_t)wrapped_delta - (1LL << 32);
}
```

`app_encoder_speed_update()` must:

1. Return on null pointers or unchanged `sequence`.
2. Consume the first sample as the baseline with `speed_valid = 0u`.
3. Compute `delta_time_us`; if zero, consume the sample, update the baseline, and keep speed invalid.
4. Compute the modular `int64_t` delta and multiply it by `encoder_polarity`.
5. Compute `counts/s`, motor RPM, wheel RPM, and wheel m/s.
6. Set `speed_valid = 1u`, clear `is_stale`, and save the current sample and sequence as the next baseline.

- [ ] **Step 5: Run the estimator tests and verify GREEN**

Run the compile command from Step 2, then:

```powershell
& $env:TEMP\test_app_encoder_speed.exe
```

Expected:

```text
PASS: encoder speed estimator
```

- [ ] **Step 6: Review task-owned changes**

Run:

```powershell
git diff --check
git diff -- App/Inc/app_config.h App/Inc/app_encoder_speed.h `
    App/Src/app_encoder_speed.c tests/test_app_encoder_speed.c
```

Expected: no whitespace errors; no use of `Pr0.08`, hidden `last-now`, or command-polarity aliases.

---

### Task 2: Add a shared DWT microsecond timestamp

**Files:**
- Modify: `BSP/Inc/bsp_dwt.h`
- Modify: `BSP/Src/bsp_dwt.c`
- Create: `tests/fakes/main.h`
- Create: `tests/test_bsp_dwt.c`

**Interfaces:**
- Consumes: the existing DWT initialization and 64-bit software cycle accumulator.
- Produces: `uint64_t BSP_DWT_GetTickUs(void)` with the same monotonic epoch as `BSP_DWT_GetTickMs()`.

- [ ] **Step 1: Add failing microsecond and wrap tests**

Create a fake `main.h` that declares the minimal `DWT`, `CoreDebug`, `SystemCoreClock`, PRIMASK, barrier, and clock-update symbols used by `bsp_dwt.c`.

Create `tests/test_bsp_dwt.c` by including `../BSP/Src/bsp_dwt.c` so the test can reset the private accumulator. Verify:

```c
s_dwt_initialized = true;
s_dwt_last_cycles = 0u;
s_dwt_total_cycles = 0u;
g_test_dwt.CYCCNT = 480u;
expect_u64(BSP_DWT_GetTickUs(), 1u, "480 cycles must equal 1 us");

s_dwt_last_cycles = UINT32_MAX - 239u;
s_dwt_total_cycles = 0u;
g_test_dwt.CYCCNT = 240u;
expect_u64(BSP_DWT_GetTickUs(), 1u,
           "unsigned cycle subtraction must survive one CYCCNT wrap");
```

Also verify that millisecond and microsecond calls share the same total-cycle state instead of double-counting elapsed cycles.

- [ ] **Step 2: Compile and verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/fakes -IBSP/Inc `
    tests/test_bsp_dwt.c -o $env:TEMP\test_bsp_dwt.exe
```

Expected: compilation fails because `BSP_DWT_GetTickUs()` is missing.

- [ ] **Step 3: Refactor one shared cycle-snapshot helper**

In `BSP/Src/bsp_dwt.c`, move the PRIMASK-protected read/update logic into:

```c
static uint64_t bsp_dwt_get_total_cycles(void);
```

Both public getters call that helper:

```c
uint64_t BSP_DWT_GetTickUs(void)
{
    return bsp_dwt_get_total_cycles() / DWT_CYCLES_PER_US;
}

uint64_t BSP_DWT_GetTickMs(void)
{
    return bsp_dwt_get_total_cycles() / DWT_CYCLES_PER_MS;
}
```

Define `DWT_CYCLES_PER_US` as `480ULL` for the already-validated 480 MHz clock. Preserve the existing `BSP_DWT_Init()` frequency check and saved-PRIMASK restoration.

- [ ] **Step 4: Run the DWT tests and verify GREEN**

Run the compile command from Step 2 and execute the result.

Expected:

```text
PASS: BSP DWT microsecond timestamp
```

---

### Task 3: Publish the RS485 receive timestamp safely

**Files:**
- Modify: `BSP/Inc/bsp_rs485.h`
- Modify: `BSP/Src/bsp_rs485.c`
- Modify: `tests/fakes/bsp_rs485.h`
- Modify: `tests/test_bsp_rs485_cancel.c`

**Interfaces:**
- Consumes: `BSP_DWT_GetTickUs()`.
- Produces: `volatile uint64_t rx_timestamp_us` published before `BSP_RS485_STATE_DONE`.

- [ ] **Step 1: Add failing BSP publication tests**

Extend `tests/test_bsp_rs485_cancel.c` with a fake DWT getter:

```c
static bsp_rs485_handle_t *s_timestamp_bus;

uint64_t BSP_DWT_GetTickUs(void)
{
    expect_true(s_timestamp_bus->rx_len == 9u,
                "RX length must be published before timestamp capture");
    expect_true(s_timestamp_bus->state == BSP_RS485_STATE_RX_WAIT,
                "DONE must be published after timestamp capture");
    return 123456u;
}
```

Add a receive-event test:

```c
bus.state = BSP_RS485_STATE_RX_WAIT;
s_timestamp_bus = &bus;
bsp_rs485_rx_event_callback(&bus, 9u);

expect_true(bus.rx_len == 9u, "RX length");
expect_true(bus.rx_timestamp_us == 123456u, "DWT timestamp");
expect_true(bus.state == BSP_RS485_STATE_DONE, "DONE publication");
```

Add GCC compile-time checks that `state`, `rx_len`, `rx_timestamp_us`,
`rx_error_code`, and `state_error_code` retain their declared volatile types.

- [ ] **Step 2: Run the BSP test and verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IBSP/Inc -Itests/fakes `
    BSP/Src/bsp_rs485.c tests/test_bsp_rs485_cancel.c `
    -o $env:TEMP\test_bsp_rs485_cancel.exe
```

Expected: compilation fails because `rx_timestamp_us` and the DWT call are missing.

- [ ] **Step 3: Add only the required volatile fields and publication order**

Change the real handle to:

```c
volatile bsp_rs485_state_t state;
volatile uint16_t rx_len;
volatile uint64_t rx_timestamp_us;
volatile uint8_t rx_error_code;
volatile uint8_t state_error_code;
```

Keep buffers and the complete handle non-volatile. Initialize and clear the timestamp with the existing RX transaction metadata.

In `bsp_rs485_rx_event_callback()` publish exactly:

```c
bus->rx_len = rx_size;
bus->rx_timestamp_us = BSP_DWT_GetTickUs();
bus->state = BSP_RS485_STATE_DONE;
```

Do not perform floating-point work or response parsing in the callback.

- [ ] **Step 4: Run the BSP tests and verify GREEN**

Compile and run the BSP test executable.

Expected:

```text
PASS: BSP RS485 cancellation and RX publication behavior
```

---

### Task 4: Publish valid samples from the APP FSM and expose three speeds

**Files:**
- Modify: `App/Inc/app_ld2rs_task.h`
- Modify: `App/Src/app_ld2rs_task.c`
- Modify: `tests/fakes/app_config.h`
- Create: `tests/test_app_ld2rs_encoder_sampling.c`

**Interfaces:**
- Consumes: validated `PrB.24`, `g_rs485_bus.rx_timestamp_us`, and `app_encoder_speed_update(...)`.
- Produces: `app_ld2rs_speed_feedback_t` and `app_ld2rs_task_get_speed_feedback(...)`.

- [ ] **Step 1: Add failing sample-publication and isolation tests**

Create a focused APP sampling test that includes `app_ld2rs_task.c`, supplies the existing HAL/RS485/RC fakes, and links `app_encoder_speed.c`.

For a valid encoder response:

```c
fsm.phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
fsm.read_transaction = APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;
g_rs485_bus.rx_len = 9u;
g_rs485_bus.rx_timestamp_us = 500000u;
fsm.rx_frame[0] = 1u;
fsm.rx_frame[1] = MODBUS_RTU_FC_READ_HOLDING;
fsm.rx_frame[2] = 4u;
fsm.rx_frame[3] = 0x00u;
fsm.rx_frame[4] = 0x02u;
fsm.rx_frame[5] = 0x00u;
fsm.rx_frame[6] = 0x00u; /* 131072 */

app_motor_fsm_step(&fsm, BSP_RS485_STATE_DONE);

expect_true(fsm.encoder_sample.counts == 131072,
            "valid PrB.24 must publish the parsed count");
expect_true(fsm.encoder_sample.timestamp_us == 500000u,
            "sample must use the matching BSP receive timestamp");
expect_true(fsm.encoder_sample.sequence == 1u,
            "valid PrB.24 must publish one new sequence");
```

Add cases proving:

- invalid CRC/length/byte count does not increment `sequence`;
- `TIMEOUT/ERROR` does not increment `sequence`;
- a second valid sample produces the requested motor RPM, wheel RPM, and wheel m/s;
- M1 and M2 sequences, timestamps, polarities, and speed states remain independent;
- an unchanged sequence is not recalculated;
- the getter rejects null output, motor number 0/3, and insufficient samples.

- [ ] **Step 2: Run the focused APP test and verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror `
    -Itests/fakes -IApp/Inc `
    tests/test_app_ld2rs_encoder_sampling.c App/Src/app_encoder_speed.c `
    -o $env:TEMP\test_app_ld2rs_encoder_sampling.exe
```

Expected: compilation fails because the sample, estimator, and getter are missing.

- [ ] **Step 3: Integrate sample publication without changing FSM transitions**

Add `encoder_sample` and `speed_estimator` to `motor_fsm_t` and clear both in
`app_motor_fsm_init()`.

Add a small structure-to-structure publication helper:

```c
static void app_encoder_sample_publish(
    app_encoder_sample_t *target,
    const app_encoder_sample_t *source)
{
    if ((target == NULL) || (source == NULL)) {
        return;
    }

    target->counts = source->counts;
    target->timestamp_us = source->timestamp_us;
    target->sequence = source->sequence;
}
```

In the valid encoder branch, construct a complete sample and pass both complete
structures by address:

```c
app_encoder_sample_t new_sample;

new_sample.counts = fsm->encoder_position_counts;
new_sample.timestamp_us = g_rs485_bus.rx_timestamp_us;
new_sample.sequence = fsm->encoder_sample.sequence + 1u;

app_encoder_sample_publish(
    &fsm->encoder_sample,
    &new_sample);
```

The helper writes `sequence` last. Do not replace it with a whole-structure
assignment because publication order matters. Do not publish in any failure,
timeout, retry-exhaustion, safety-cancellation, or old-value-preservation path.

- [ ] **Step 4: Run both estimators in main context**

At the beginning of every `app_ld2rs_task_run()`, call `BSP_DWT_GetTickUs()` once to keep the 64-bit cycle extension current across communication outages.

After the existing scheduler step, call:

```c
app_encoder_speed_update(
    &s_motor_fsm_m1.encoder_sample,
    &s_motor_fsm_m1.speed_estimator,
    APP_LD2_ENCODER_POLARITY_M1);

app_encoder_speed_update(
    &s_motor_fsm_m2.encoder_sample,
    &s_motor_fsm_m2.speed_estimator,
    APP_LD2_ENCODER_POLARITY_M2);
```

This remains in the while-loop context; no ISR gains floating-point work.

- [ ] **Step 5: Add the read-only public result**

Define in `App/Inc/app_ld2rs_task.h`:

```c
typedef struct
{
    float motor_speed_rpm;
    float wheel_speed_rpm;
    float wheel_speed_mps;
} app_ld2rs_speed_feedback_t;

bool app_ld2rs_task_get_speed_feedback(
    uint8_t motor_number,
    app_ld2rs_speed_feedback_t *feedback);
```

The implementation selects M1 for `1u`, M2 for `2u`, returns false for invalid arguments or `speed_valid == 0u`, and otherwise copies exactly the three requested outputs.

- [ ] **Step 6: Run the focused APP and estimator tests and verify GREEN**

Run both Task 1 and Task 4 executables.

Expected:

```text
PASS: encoder speed estimator
PASS: LD2RS encoder sample publication
```

- [ ] **Step 7: Re-run the pre-existing unified suite**

Run:

```powershell
powershell -ExecutionPolicy Bypass `
    -File tests/run_ld2rs_unified_read_tests.ps1
```

Expected: BSP cancellation still passes. If the previously recorded unknown-subtransaction failure remains identical, record it as pre-existing and do not modify that unrelated branch. Any new failure before that baseline point must be fixed.

---

### Task 5: Add build metadata and run full verification

**Files:**
- Modify: `tests/run_ld2rs_unified_read_tests.ps1`
- Create: `tests/run_encoder_speed_tests.ps1`
- Modify: `Makefile`
- Modify: `MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx`

**Interfaces:**
- Consumes: all new source and test files.
- Produces: repeatable host tests and firmware builds containing both `bsp_dwt.c` and `app_encoder_speed.c`.

- [ ] **Step 1: Add the focused host-test runner**

Create `tests/run_encoder_speed_tests.ps1` to compile and run:

1. `test_app_encoder_speed.exe`;
2. `test_bsp_dwt.exe`;
3. `test_bsp_rs485_cancel.exe`;
4. `test_app_ld2rs_encoder_sampling.exe`.

Every compile uses `-std=c11 -Wall -Wextra -Werror` and stops on the first nonzero exit code.

- [ ] **Step 2: Add sources to both firmware project descriptions**

Add to `Makefile`:

```make
  App/Src/app_encoder_speed.c \
  BSP/Src/bsp_dwt.c \
```

Add `app_encoder_speed.c` to the existing `Application/User/App` group in
`MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx`. `bsp_dwt.c` is already present in the Keil BSP group; do not duplicate it.

Do not edit the untracked EIDE workspace or generated AXF/HEX files.

- [ ] **Step 3: Run focused host verification**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_encoder_speed_tests.ps1
```

Expected: all four focused executables print PASS and no compiler warning appears.

- [ ] **Step 4: Run the ARM firmware build**

Run:

```powershell
make
```

Expected: `arm-none-eabi-gcc` compiles and links the project with no new warning or undefined reference, and reports flash/RAM usage.

- [ ] **Step 5: Run final static checks**

Run:

```powershell
git diff --check
rg -n "HAL_TIM|MX_TIM|PeriodElapsedCallback" App BSP Core
```

Expected: no whitespace error; no TIM was added; VOFA’s 17-channel send list and `WRITE_REQ` start-failure logic are unchanged.

- [ ] **Step 6: Inspect the complete task diff**

Review only:

```text
App/Inc/app_config.h
App/Inc/app_encoder_speed.h
App/Inc/app_ld2rs_task.h
App/Src/app_encoder_speed.c
App/Src/app_ld2rs_task.c
BSP/Inc/bsp_dwt.h
BSP/Inc/bsp_rs485.h
BSP/Src/bsp_dwt.c
BSP/Src/bsp_rs485.c
Makefile
MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx
tests/fakes/*
tests/test_app_encoder_speed.c
tests/test_bsp_dwt.c
tests/test_bsp_rs485_cancel.c
tests/test_app_ld2rs_encoder_sampling.c
tests/run_encoder_speed_tests.ps1
```

Confirm no unrelated user edit was overwritten or staged.

- [ ] **Step 7: Report verification evidence**

The final handoff must list modified files, structures, timestamp location, sample-publication location, formulas and polarity, 17-bit CPR evidence, 32-bit wrap handling, absence of TIM changes, all test/build outputs, the unchanged pre-existing baseline failure if still present, and an explicit statement that VOFA sending and `WRITE_REQ` start-failure behavior were not changed.
