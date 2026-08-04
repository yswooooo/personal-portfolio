# LD2-RS Unified Read State Machine Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Unify status/speed and encoder reads under one top-level read phase pair, give each read sub-transaction three APP retries, and guarantee encoder observation failures cannot permanently block speed writes.

**Architecture:** `motor_ctrl_phase_t` owns the common control phases while a new `motor_read_transaction_t` selects either status/speed or encoder work inside `READ_REQ/WAIT_READ`. The existing BSP retry policy remains unchanged; a focused BSP cancel API lets the APP abandon only an in-flight encoder observation when safety stop becomes active.

**Tech Stack:** C11-compatible STM32H7 firmware, STM32 HAL UART, Modbus RTU, GNU Arm Embedded Make build, PowerShell source-contract regression test.

## Global Constraints

- Status/speed and encoder reads each receive `APP_LD2RS_TASK_MAX_RETRY == 3` APP logical attempts.
- Every APP attempt retains the BSP default of at most `BSP_RS485_MAX_RETRY == 3` physical attempts.
- Encoder exhaustion preserves the last valid encoder value, skips the remainder of that read for the current cycle, and retries next cycle.
- Encoder failures do not set forced zero speed and do not update motor offline statistics.
- Existing speed write, VOFA 17-channel transmission, differential-drive computation, and status/speed failure policy remain unchanged.
- Safety stop may cancel an in-flight encoder observation so that the zero-speed write path is not held behind observation retries.

## TDD Execution Adjustment

Plan review rejected source-text assertions because they could pass while the
state transitions were still wrong. Execution therefore uses host-side C
behavior tests that compile the real production `.c` files:

- `tests/test_bsp_rs485_cancel.c` links the real `BSP/Src/bsp_rs485.c` against
  a fake HAL and verifies UART abort, error cleanup, bus release, and terminal
  acknowledgement behavior.
- `tests/test_app_ld2rs_read_fsm.c` includes the real
  `App/Src/app_ld2rs_task.c` and drives status success, encoder start failure,
  encoder timeout exhaustion, status timeout exhaustion, and safety
  cancellation with controlled RS485 results.
- `tests/run_ld2rs_unified_read_tests.ps1` builds both tests with host GCC and
  runs them. This command supersedes the source-contract PowerShell assertions
  described in Tasks 1 and 2.

---

### Task 1: Add a safe BSP transaction-cancel boundary

**Files:**
- Create: `tests/test_ld2rs_unified_read_fsm.ps1`
- Modify: `BSP/Inc/bsp_rs485.h`
- Modify: `BSP/Src/bsp_rs485.c`

**Interfaces:**
- Consumes: `bsp_rs485_handle_t`, `HAL_UART_AbortTransmit()`, `HAL_UART_AbortReceive()`, and `bsp_rs485_ack_done()`.
- Produces: `void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus)`.

- [ ] **Step 1: Write the failing BSP contract test**

Create `tests/test_ld2rs_unified_read_fsm.ps1`:

```powershell
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$header = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $repo 'BSP\Inc\bsp_rs485.h')
$source = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $repo 'BSP\Src\bsp_rs485.c')

function Assert-Contains([string]$Text, [string]$Needle, [string]$Message) {
    if (-not $Text.Contains($Needle)) {
        throw $Message
    }
}

Assert-Contains $header `
    'void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus);' `
    'missing BSP cancel declaration'
Assert-Contains $source `
    'void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus)' `
    'missing BSP cancel implementation'
Assert-Contains $source `
    'HAL_UART_AbortTransmit(bus->uart_handle)' `
    'cancel must abort an in-flight transmit'
Assert-Contains $source `
    'HAL_UART_AbortReceive(bus->uart_handle)' `
    'cancel must abort an in-flight receive'
Assert-Contains $source `
    'bsp_rs485_ack_done(bus)' `
    'cancel must release the bus through the BSP cleanup boundary'

Write-Output 'PASS: BSP cancellation contract'
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\test_ld2rs_unified_read_fsm.ps1
```

Expected: failure containing `missing BSP cancel declaration`.

- [ ] **Step 3: Declare the cancellation interface**

Add this declaration beside `bsp_rs485_ack_done()` in `BSP/Inc/bsp_rs485.h`:

```c
/**
  * @brief Cancel the current asynchronous transaction and release the bus.
  * @note  Intended for a higher-priority safety path that owns the active transaction.
  */
void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus);
```

- [ ] **Step 4: Implement cancellation using the existing BSP cleanup path**

Add to `BSP/Src/bsp_rs485.c` immediately before `bsp_rs485_ack_done()`:

```c
void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus)
{
    if ((bus == NULL) || (bus->uart_handle == NULL)) {
        return;
    }

    if ((bus->state == BSP_RS485_STATE_TX_BUSY)
        || (bus->state == BSP_RS485_STATE_RX_WAIT)) {
        (void)HAL_UART_AbortTransmit(bus->uart_handle);
        (void)HAL_UART_AbortReceive(bus->uart_handle);
    }

    bsp_rs485_ack_done(bus);
}
```

- [ ] **Step 5: Run the contract test and firmware build**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\test_ld2rs_unified_read_fsm.ps1
make
```

Expected: contract test prints `PASS: BSP cancellation contract`; `make` exits 0.

- [ ] **Step 6: Commit the BSP boundary**

```powershell
git add tests/test_ld2rs_unified_read_fsm.ps1 BSP/Inc/bsp_rs485.h BSP/Src/bsp_rs485.c
git commit -m "fix: add safe RS485 transaction cancellation"
```

---

### Task 2: Unify APP read phases and implement per-sub-transaction retries

**Files:**
- Modify: `tests/test_ld2rs_unified_read_fsm.ps1`
- Modify: `App/Src/app_ld2rs_task.c`

**Interfaces:**
- Consumes: `bsp_rs485_cancel_transaction()`, `APP_LD2RS_TASK_MAX_RETRY`, and existing Modbus frame helpers.
- Produces: `motor_read_transaction_t`, `motor_fsm_t.read_transaction`, and common `APP_LD2RS_TASK_PHASE_READ_REQ/WAIT_READ/READ_DONE` phases.

- [ ] **Step 1: Extend the contract test and verify RED**

Append these checks before the final PASS line:

```powershell
$app = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $repo 'App\Src\app_ld2rs_task.c')

$required = @(
    'APP_LD2RS_TASK_PHASE_READ_REQ',
    'APP_LD2RS_TASK_PHASE_WAIT_READ',
    'APP_LD2RS_TASK_PHASE_READ_DONE',
    'APP_LD2RS_READ_TRANSACTION_STATUS_SPEED',
    'APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION',
    'motor_read_transaction_t read_transaction',
    'BSP_RS485_STATUS_ERR_BUSY',
    'bsp_rs485_cancel_transaction(&g_rs485_bus)'
)
foreach ($name in $required) {
    Assert-Contains $app $name "missing unified read contract: $name"
}

$obsolete = @(
    'APP_LD2RS_TASK_PHASE_READ_STATUS_SPEED_REQ',
    'APP_LD2RS_TASK_PHASE_READ_STATUS_SPEED_WAIT',
    'APP_LD2RS_TASK_PHASE_READ_ENCODER_POSITION_REQ',
    'APP_LD2RS_TASK_PHASE_READ_ENCODER_POSITION_WAIT',
    'APP_LD2RS_TASK_PHASE_READ_FEEDBACK_DONE'
)
foreach ($name in $obsolete) {
    if ($app.Contains($name)) {
        throw "obsolete split read phase remains: $name"
    }
}

if ([regex]::Matches($app, 'case APP_LD2RS_TASK_PHASE_READ_REQ:').Count -ne 1) {
    throw 'READ_REQ must have exactly one common case'
}
if ([regex]::Matches($app, 'case APP_LD2RS_TASK_PHASE_WAIT_READ:').Count -ne 1) {
    throw 'WAIT_READ must have exactly one common case'
}
```

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\test_ld2rs_unified_read_fsm.ps1
```

Expected: failure containing `missing unified read contract`.

- [ ] **Step 2: Replace split top-level read phases with common phases and a read selector**

Define:

```c
typedef enum {
    APP_LD2RS_TASK_PHASE_IDLE = 0,
    APP_LD2RS_TASK_PHASE_READ_REQ,
    APP_LD2RS_TASK_PHASE_WAIT_READ,
    APP_LD2RS_TASK_PHASE_READ_DONE,
    APP_LD2RS_TASK_PHASE_WRITE_REQ,
    APP_LD2RS_TASK_PHASE_WAIT_WRITE,
    APP_LD2RS_TASK_PHASE_WRITE_DONE,
} motor_ctrl_phase_t;

typedef enum {
    APP_LD2RS_READ_TRANSACTION_STATUS_SPEED = 0,
    APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION,
} motor_read_transaction_t;
```

Add to `motor_fsm_t`:

```c
motor_read_transaction_t read_transaction;
```

Initialize `read_transaction` to `STATUS_SPEED` both in `app_motor_fsm_init()` and at the start of every `IDLE` cycle.

- [ ] **Step 3: Implement the common READ_REQ state**

The single `READ_REQ` case must:

```c
case APP_LD2RS_TASK_PHASE_READ_REQ:
    if (bus_state != BSP_RS485_STATE_IDLE) {
        return APP_MOTOR_FSM_STEP_WAITING;
    }

    if (fsm->read_transaction == APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION) {
        /* If safety is already active, skip only the observation read. */
        if (g_emergency_stop_flag || g_rc.lost_flag
            || (g_rc.sw_st[eRC_SW_A].curr == eRC_POS_DOWN)) {
            fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
            break;
        }
        read_register = LD2_MOTOR_REG_ENCODER_POSITION_H;
    } else {
        read_register = LD2_MOTOR_REG_RUN_STATUS;
    }

    app_ld2rs_task_build_read_req(fsm->tx_frame,
                                  fsm->motor_dev->slave_id,
                                  read_register,
                                  APP_LD2RS_TASK_MODBUS_READ_QTY_TWO);
    start_status = bsp_rs485_start_tx(&g_rs485_bus,
                                      fsm->tx_frame, sizeof(fsm->tx_frame),
                                      fsm->rx_frame, sizeof(fsm->rx_frame),
                                      fsm->motor_dev->timeout_ms);
    if (start_status == BSP_RS485_STATUS_OK) {
        fsm->tx_start_tick_ms = HAL_GetTick();
        fsm->phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
        return APP_MOTOR_FSM_STEP_STARTED;
    }
    if (start_status == BSP_RS485_STATUS_ERR_BUSY) {
        return APP_MOTOR_FSM_STEP_WAITING;
    }

    /* Count non-BUSY start failures so the request state cannot livelock. */
    fsm->read_retry_count++;
    if (fsm->read_retry_count >= APP_LD2RS_TASK_MAX_RETRY) {
        if (fsm->read_transaction == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED) {
            fsm->force_zero_speed = 1u;
            fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
            return APP_MOTOR_FSM_STEP_ERROR_RELEASED;
        }
        fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
        return APP_MOTOR_FSM_STEP_NONE;
    }
    return APP_MOTOR_FSM_STEP_ERROR_RELEASED;
```

Use a local `bsp_rs485_status_t start_status` and `uint16_t read_register`; retain the existing full argument list in the actual `bsp_rs485_start_tx()` call.

- [ ] **Step 4: Implement common WAIT_READ success transitions**

In the single `WAIT_READ` case:

```c
if (fsm->read_transaction == APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION
    && safety_stop_active) {
    bsp_rs485_cancel_transaction(&g_rs485_bus);
    fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
    return APP_MOTOR_FSM_STEP_NONE;
}
```

On a valid status/speed response, preserve the existing status, speed, RTT, VOFA, and offline-success updates, then:

```c
fsm->read_retry_count = 0u;
fsm->read_transaction = APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;
fsm->phase = APP_LD2RS_TASK_PHASE_READ_REQ;
```

On a valid encoder response, preserve the existing signed 32-bit reconstruction and VOFA field update, then:

```c
fsm->read_retry_count = 0u;
fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
```

Every `DONE` response path must call `bsp_rs485_ack_done()` exactly once.

- [ ] **Step 5: Implement common read failure transitions**

For invalid response, `TIMEOUT`, and `ERROR`:

```c
if ((bus_state == BSP_RS485_STATE_TIMEOUT)
    && (fsm->read_transaction == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED)) {
    app_motor_fsm_mark_no_response(fsm);
}

fsm->read_retry_count++;
if (fsm->read_retry_count < APP_LD2RS_TASK_MAX_RETRY) {
    fsm->phase = APP_LD2RS_TASK_PHASE_READ_REQ;
    bsp_rs485_ack_done(&g_rs485_bus);
    return (bus_state == BSP_RS485_STATE_DONE)
        ? APP_MOTOR_FSM_STEP_DONE_RELEASED
        : APP_MOTOR_FSM_STEP_ERROR_RELEASED;
}

if (fsm->read_transaction == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED) {
    fsm->force_zero_speed = 1u;
    fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
    bsp_rs485_ack_done(&g_rs485_bus);
    return APP_MOTOR_FSM_STEP_ERROR_RELEASED;
}

/* Encoder exhaustion: preserve old value and keep this motor for its write. */
fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
bsp_rs485_ack_done(&g_rs485_bus);
return APP_MOTOR_FSM_STEP_NONE;
```

Do not call `app_motor_fsm_mark_no_response()` for encoder failures.

- [ ] **Step 6: Rename write phases back to the generic top-level names**

Apply this exact mapping without changing transitions:

```text
WRITE_TARGET_SPEED_REQ  -> WRITE_REQ
WRITE_TARGET_SPEED_WAIT -> WAIT_WRITE
CYCLE_DONE              -> WRITE_DONE
```

Update the scheduler terminal-state guard to accept only `WAIT_READ` and `WAIT_WRITE`. Update the file-level state-flow comment and initialize M1 with `read_transaction = STATUS_SPEED` and `phase = READ_REQ`.

- [ ] **Step 7: Run the contract test and firmware build**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\test_ld2rs_unified_read_fsm.ps1
make
```

Expected: contract test passes; firmware compiles and links successfully with no new warnings.

- [ ] **Step 8: Commit the unified APP state machine**

```powershell
git add tests/test_ld2rs_unified_read_fsm.ps1 App/Src/app_ld2rs_task.c
git commit -m "fix: unify LD2RS read transaction states"
```

---

### Task 3: Verify the full change against the approved specification

**Files:**
- Inspect: `docs/superpowers/specs/2026-07-27-ld2rs-unified-read-state-machine-design.md`
- Inspect: `App/Src/app_ld2rs_task.c`
- Inspect: `BSP/Inc/bsp_rs485.h`
- Inspect: `BSP/Src/bsp_rs485.c`
- Inspect: `tests/test_ld2rs_unified_read_fsm.ps1`

**Interfaces:**
- Consumes: all interfaces produced by Tasks 1 and 2.
- Produces: verification evidence and a reviewer-approved implementation.

- [ ] **Step 1: Run fresh automated checks**

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\test_ld2rs_unified_read_fsm.ps1
make
git diff --check HEAD~2..HEAD
```

Expected: contract test and build exit 0; diff check produces no errors.

- [ ] **Step 2: Inspect the transition graph**

Run:

```powershell
rg -n "case APP_LD2RS_TASK_PHASE_|fsm->phase = APP_LD2RS_TASK_PHASE_|read_transaction|read_retry_count" App/Src/app_ld2rs_task.c
```

Confirm:

```text
IDLE
  -> READ_REQ(STATUS_SPEED)
  -> WAIT_READ
  -> READ_REQ(ENCODER_POSITION)
  -> WAIT_READ
  -> READ_DONE
  -> WRITE_REQ
  -> WAIT_WRITE
  -> WRITE_DONE
  -> IDLE
```

Confirm status exhaustion sets forced zero; encoder exhaustion does not. Confirm each transition out of an active terminal bus state acknowledges or cancels the bus exactly once.

- [ ] **Step 3: Request independent read-only code review**

Ask the reviewer to compare the implementation against the approved design, emphasizing:

- shared top-level read phases;
- independent three-attempt APP retry budgets;
- start failure cannot livelock;
- encoder exhaustion preserves the previous value and reaches speed write;
- safety cancellation cannot leave UART callbacks operating on an idle-marked bus;
- no status/speed, write, offline, VOFA, or differential-control regression.

- [ ] **Step 4: Apply only confirmed review fixes and rerun Step 1**

If review finds an issue, reproduce it with an added contract assertion or focused test first, make the smallest correction, and rerun all commands from Step 1. If review finds no issue, do not modify the implementation.
