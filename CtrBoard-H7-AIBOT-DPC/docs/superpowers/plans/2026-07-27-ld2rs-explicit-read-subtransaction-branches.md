# LD2RS Explicit Read Subtransaction Branches Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every `read_transaction` classification explicit and route unknown values directly to a forced-zero speed write without sending or parsing a Modbus read.

**Architecture:** Keep the existing unified `READ_REQ/WAIT_READ` top-level state machine. Add complete `STATUS_SPEED / ENCODER_POSITION / unknown` chains at request construction, response parsing, and retry-exhaustion policy boundaries; document the exact extension points beside the read-subtransaction enum and classification code.

**Tech Stack:** C11, STM32 HAL, Modbus RTU, PowerShell host-test runner, GNU Arm Embedded Toolchain.

## Global Constraints

- Work in place on the existing `feat/ecd` branch as explicitly requested.
- Preserve current status/speed and encoder success, retry, safety, offline, scheduler, and speed-write behavior.
- An unknown `read_transaction` must not construct, transmit, or parse a Modbus read.
- An unknown `read_transaction` must set `force_zero_speed = 1u` and transition directly to `APP_LD2RS_TASK_PHASE_WRITE_REQ`.
- Future read subtransactions must be added explicitly to the enum, request construction, response validation/parsing, retry-exhaustion policy, safety/offline policy, and behavior tests.

---

### Task 1: Protect the unknown-subtransaction fallback

**Files:**
- Modify: `tests/test_app_ld2rs_read_fsm.c`
- Modify: `App/Src/app_ld2rs_task.c`

**Interfaces:**
- Consumes: `app_motor_fsm_step(motor_fsm_t *, bsp_rs485_state_t)`.
- Produces: Explicit handling for all values of `motor_read_transaction_t`.

- [ ] **Step 1: Write the failing request-phase behavior test**

Add a start-call counter to the existing `bsp_rs485_start_tx()` fake, reset it in `reset_fixture()`, and add:

```c
static void test_unknown_read_transaction_forces_zero_without_starting_read(void)
{
    motor_fsm_t fsm;
    ld2rs_motor_ctrl_t motor_ctrl;
    ld2_motor_handle_t motor_dev;
    app_motor_fsm_step_result_t result;

    reset_fixture(&fsm, &motor_ctrl, &motor_dev);
    fsm.phase = APP_LD2RS_TASK_PHASE_READ_REQ;
    fsm.read_transaction = (motor_read_transaction_t)99;

    result = app_motor_fsm_step(&fsm, BSP_RS485_STATE_IDLE);

    expect_true(result == APP_MOTOR_FSM_STEP_NONE,
                "unknown read transaction must keep this motor for zero-speed write");
    expect_true(fsm.force_zero_speed == 1u,
                "unknown read transaction must force zero speed");
    expect_true(fsm.phase == APP_LD2RS_TASK_PHASE_WRITE_REQ,
                "unknown read transaction must enter WRITE_REQ directly");
    expect_true(s_start_calls == 0u,
                "unknown read transaction must not start a Modbus read");
}
```

Call the test from `main()`.

- [ ] **Step 2: Run the host state-machine tests and verify RED**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_ld2rs_unified_read_tests.ps1
```

Expected: the new test fails because the current bare `else` treats value `99` as `STATUS_SPEED` and calls `bsp_rs485_start_tx()`.

- [ ] **Step 3: Implement complete explicit classification**

In `App/Src/app_ld2rs_task.c`:

1. Add a comment beside `motor_read_transaction_t` stating that a future subtype must be added to:
   - `READ_REQ` request address/count selection;
   - `WAIT_READ` response validation/parsing and success transition;
   - `app_motor_fsm_handle_read_failure()` retry-exhaustion policy;
   - safety/offline policy checks;
   - host behavior tests.
2. Change `app_motor_fsm_handle_read_failure()` to:

```c
if (fsm->read_transaction == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED) {
    fsm->force_zero_speed = 1u;
    fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
    return released_result;
} else if (fsm->read_transaction
           == APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION) {
    fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
    return APP_MOTOR_FSM_STEP_NONE;
} else {
    fsm->force_zero_speed = 1u;
    fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
    return APP_MOTOR_FSM_STEP_NONE;
}
```

3. Change `READ_REQ` register selection to an explicit status branch, explicit encoder branch, and unknown fallback. The unknown fallback returns before `modbus_rtu_build_read_holding()` or `bsp_rs485_start_tx()`.
4. Change `WAIT_READ` success parsing to an explicit status branch, explicit encoder branch, and unknown fallback. The unknown fallback acknowledges the completed bus exactly once, sets forced zero, enters `WRITE_REQ`, and returns `APP_MOTOR_FSM_STEP_NONE`.
5. Keep timeout/offline accounting limited explicitly to `STATUS_SPEED`; unknown values use the failure helper fallback.

- [ ] **Step 4: Run the host tests and verify GREEN**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_ld2rs_unified_read_tests.ps1
```

Expected:

```text
PASS: BSP RS485 cancellation behavior
PASS: LD2RS unified read state machine behavior
```

- [ ] **Step 5: Run full firmware verification**

Run:

```powershell
make
git diff --check
```

Expected: ARM compilation and linking succeed with no new warnings; `git diff --check` prints no errors.

- [ ] **Step 6: Request an independent code review**

Ask a reviewer to inspect the diff for:

- any remaining bare `else` that implicitly means a valid read subtype;
- any unknown subtype path that can transmit or parse a read;
- double/missing RS485 acknowledgement;
- changes to existing retry, offline, safety, or write behavior.

- [ ] **Step 7: Commit**

```powershell
git add App/Src/app_ld2rs_task.c tests/test_app_ld2rs_read_fsm.c docs/superpowers/plans/2026-07-27-ld2rs-explicit-read-subtransaction-branches.md
git commit -m "refactor: make LD2RS read categories explicit"
```
