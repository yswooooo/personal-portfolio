# STM32H7 Numeric Limits Increment Experiment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a one-shot STM32H7 experiment that records direct maximum-value increments and standard-defined reference results in a Keil Watch-friendly global structure.

**Architecture:** A HAL-independent application module owns the result types, global volatile result object, and one-shot experiment function. `main()` invokes the function once after `HAL_Init()` from a CubeMX user-code region, while the Makefile and Keil project both include the new source.

**Tech Stack:** C11, STM32H723, ARMCLANG/Keil5, GNU Arm Embedded GCC, PowerShell host-test runners

## Global Constraints

- Preserve the existing motor-control, remote-control, safety, VOFA+, and main-loop behavior.
- Edit `Core/Src/main.c` only inside `USER CODE BEGIN/END` regions.
- Cover `int8_t`/`uint8_t`, `int16_t`/`uint16_t`, `int32_t`/`uint32_t`, `int64_t`/`uint64_t`, `float`, and `double`.
- Export one global `volatile app_numeric_limits_experiment_t g_numeric_limits_experiment`.
- Apply exactly one direct `++` to each direct-result field.
- Store signed reference values as unsigned bit patterns; never describe them as standard-defined signed results.
- Do not add `-fwrapv` or equivalent options to either target firmware build.
- Do not assert a particular direct signed result in automated tests.
- Do not add USART, VOFA+, dynamic allocation, repeated main-loop work, or `maximum + maximum` floating-point tests.
- Preserve unrelated user changes, including the deleted AXF and untracked `.claude/` directory.

---

## File Map

- Create `App/Inc/app_numeric_limits_experiment.h`: public record types, exported volatile object, and one-shot API.
- Create `App/Src/app_numeric_limits_experiment.c`: maximum initialization, direct increments, safe references, and completion flag.
- Create `tests/test_app_numeric_limits_experiment.c`: host-side behavioral test that deliberately does not contract signed direct results.
- Create `tests/run_numeric_limits_experiment_tests.ps1`: repeatable host compile-and-run command.
- Create `tests/test_numeric_limits_integration.ps1`: static integration contract for `main.c`, Makefile, and Keil project.
- Modify `Core/Src/main.c`: include the public header and call the experiment once after `HAL_Init()`.
- Modify `Makefile`: compile the new application source in the GCC firmware build.
- Modify `MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx`: compile the same source in the Keil firmware build.

---

### Task 1: Implement the HAL-independent experiment module

**Files:**
- Create: `tests/test_app_numeric_limits_experiment.c`
- Create: `tests/run_numeric_limits_experiment_tests.ps1`
- Create: `App/Inc/app_numeric_limits_experiment.h`
- Create: `App/Src/app_numeric_limits_experiment.c`

**Interfaces:**
- Consumes: C standard headers `<float.h>` and `<stdint.h>`.
- Produces: `void app_numeric_limits_experiment_run_once(void)`.
- Produces: `volatile app_numeric_limits_experiment_t g_numeric_limits_experiment`.

- [ ] **Step 1: Write the failing host test**

Create `tests/test_app_numeric_limits_experiment.c`:

```c
#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "app_numeric_limits_experiment.h"

static void expect_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    const volatile app_numeric_limits_experiment_t *result;

    app_numeric_limits_experiment_run_once();
    result = &g_numeric_limits_experiment;

    expect_true(result->completed == 1u, "experiment completion flag");

    expect_true(result->int8_case.before_signed == INT8_MAX,
                "int8 signed initial maximum");
    expect_true(result->int8_case.before_unsigned == UINT8_MAX,
                "uint8 initial maximum");
    expect_true(result->int16_case.before_signed == INT16_MAX,
                "int16 signed initial maximum");
    expect_true(result->int16_case.before_unsigned == UINT16_MAX,
                "uint16 initial maximum");
    expect_true(result->int32_case.before_signed == INT32_MAX,
                "int32 signed initial maximum");
    expect_true(result->int32_case.before_unsigned == UINT32_MAX,
                "uint32 initial maximum");
    expect_true(result->int64_case.before_signed == INT64_MAX,
                "int64 signed initial maximum");
    expect_true(result->int64_case.before_unsigned == UINT64_MAX,
                "uint64 initial maximum");

    expect_true(result->int8_case.after_unsigned_direct == 0u,
                "uint8 direct increment wraps to zero");
    expect_true(result->int16_case.after_unsigned_direct == 0u,
                "uint16 direct increment wraps to zero");
    expect_true(result->int32_case.after_unsigned_direct == 0u,
                "uint32 direct increment wraps to zero");
    expect_true(result->int64_case.after_unsigned_direct == 0u,
                "uint64 direct increment wraps to zero");

    expect_true(result->int8_case.unsigned_reference == 0u,
                "uint8 reference is zero");
    expect_true(result->int16_case.unsigned_reference == 0u,
                "uint16 reference is zero");
    expect_true(result->int32_case.unsigned_reference == 0u,
                "uint32 reference is zero");
    expect_true(result->int64_case.unsigned_reference == 0u,
                "uint64 reference is zero");

    expect_true(result->int8_case.signed_reference_bits == UINT8_C(0x80),
                "int8 next modulo bit pattern");
    expect_true(result->int16_case.signed_reference_bits == UINT16_C(0x8000),
                "int16 next modulo bit pattern");
    expect_true(result->int32_case.signed_reference_bits ==
                    UINT32_C(0x80000000),
                "int32 next modulo bit pattern");
    expect_true(result->int64_case.signed_reference_bits ==
                    UINT64_C(0x8000000000000000),
                "int64 next modulo bit pattern");

    expect_true(result->float_case.before == FLT_MAX,
                "float initial maximum");
    expect_true(result->float_case.unchanged ==
                    (uint8_t)(result->float_case.after_direct ==
                              result->float_case.before),
                "float unchanged flag matches result");
    expect_true(result->float_case.is_positive_infinity ==
                    (uint8_t)(result->float_case.after_direct > FLT_MAX),
                "float infinity flag matches result");

    expect_true(result->double_case.before == DBL_MAX,
                "double initial maximum");
    expect_true(result->double_case.unchanged ==
                    (uint8_t)(result->double_case.after_direct ==
                              result->double_case.before),
                "double unchanged flag matches result");
    expect_true(result->double_case.is_positive_infinity ==
                    (uint8_t)(result->double_case.after_direct > DBL_MAX),
                "double infinity flag matches result");

    puts("PASS: numeric limits increment experiment");
    return 0;
}
```

The absence of assertions on `after_signed_direct` is intentional.

- [ ] **Step 2: Add the repeatable test runner**

Create `tests/run_numeric_limits_experiment_tests.ps1`:

```powershell
$ErrorActionPreference = "Stop"
$testExecutable = Join-Path $env:TEMP "test_app_numeric_limits_experiment.exe"

& gcc -std=c11 -Wall -Wextra -Werror -fwrapv `
    -IApp/Inc `
    tests/test_app_numeric_limits_experiment.c `
    App/Src/app_numeric_limits_experiment.c `
    -o $testExecutable

if ($LASTEXITCODE -ne 0) {
    throw "numeric limits test compilation failed"
}

& $testExecutable

if ($LASTEXITCODE -ne 0) {
    throw "numeric limits test execution failed"
}
```

`-fwrapv` is restricted to this host test invocation so the test process can
execute the intentional signed increments without host undefined behavior. It
must not be copied into the STM32 GCC Makefile or Keil project.

- [ ] **Step 3: Run the test and verify RED**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
    -File tests/run_numeric_limits_experiment_tests.ps1
```

Expected: compilation fails because
`App/Inc/app_numeric_limits_experiment.h` and
`App/Src/app_numeric_limits_experiment.c` do not exist.

- [ ] **Step 4: Define the public result types**

Create `App/Inc/app_numeric_limits_experiment.h`:

```c
#ifndef APP_NUMERIC_LIMITS_EXPERIMENT_H
#define APP_NUMERIC_LIMITS_EXPERIMENT_H

#include <stdint.h>

typedef struct
{
    int8_t before_signed;
    int8_t after_signed_direct;
    uint8_t before_unsigned;
    uint8_t after_unsigned_direct;
    uint8_t signed_reference_bits;
    uint8_t unsigned_reference;
} app_numeric_int8_case_t;

typedef struct
{
    int16_t before_signed;
    int16_t after_signed_direct;
    uint16_t before_unsigned;
    uint16_t after_unsigned_direct;
    uint16_t signed_reference_bits;
    uint16_t unsigned_reference;
} app_numeric_int16_case_t;

typedef struct
{
    int32_t before_signed;
    int32_t after_signed_direct;
    uint32_t before_unsigned;
    uint32_t after_unsigned_direct;
    uint32_t signed_reference_bits;
    uint32_t unsigned_reference;
} app_numeric_int32_case_t;

typedef struct
{
    int64_t before_signed;
    int64_t after_signed_direct;
    uint64_t before_unsigned;
    uint64_t after_unsigned_direct;
    uint64_t signed_reference_bits;
    uint64_t unsigned_reference;
} app_numeric_int64_case_t;

typedef struct
{
    float before;
    float after_direct;
    uint8_t unchanged;
    uint8_t is_positive_infinity;
} app_numeric_float_case_t;

typedef struct
{
    double before;
    double after_direct;
    uint8_t unchanged;
    uint8_t is_positive_infinity;
} app_numeric_double_case_t;

typedef struct
{
    app_numeric_int8_case_t int8_case;
    app_numeric_int16_case_t int16_case;
    app_numeric_int32_case_t int32_case;
    app_numeric_int64_case_t int64_case;
    app_numeric_float_case_t float_case;
    app_numeric_double_case_t double_case;
    uint32_t completed;
} app_numeric_limits_experiment_t;

extern volatile app_numeric_limits_experiment_t
    g_numeric_limits_experiment;

void app_numeric_limits_experiment_run_once(void);

#endif /* APP_NUMERIC_LIMITS_EXPERIMENT_H */
```

- [ ] **Step 5: Implement the one-shot experiment**

Create `App/Src/app_numeric_limits_experiment.c`:

```c
#include "app_numeric_limits_experiment.h"

#include <float.h>
#include <stdint.h>

volatile app_numeric_limits_experiment_t g_numeric_limits_experiment;

void app_numeric_limits_experiment_run_once(void)
{
    g_numeric_limits_experiment.completed = 0u;

    g_numeric_limits_experiment.int8_case.before_signed = INT8_MAX;
    g_numeric_limits_experiment.int8_case.after_signed_direct = INT8_MAX;
    g_numeric_limits_experiment.int8_case.before_unsigned = UINT8_MAX;
    g_numeric_limits_experiment.int8_case.after_unsigned_direct = UINT8_MAX;

    g_numeric_limits_experiment.int16_case.before_signed = INT16_MAX;
    g_numeric_limits_experiment.int16_case.after_signed_direct = INT16_MAX;
    g_numeric_limits_experiment.int16_case.before_unsigned = UINT16_MAX;
    g_numeric_limits_experiment.int16_case.after_unsigned_direct = UINT16_MAX;

    g_numeric_limits_experiment.int32_case.before_signed = INT32_MAX;
    g_numeric_limits_experiment.int32_case.after_signed_direct = INT32_MAX;
    g_numeric_limits_experiment.int32_case.before_unsigned = UINT32_MAX;
    g_numeric_limits_experiment.int32_case.after_unsigned_direct = UINT32_MAX;

    g_numeric_limits_experiment.int64_case.before_signed = INT64_MAX;
    g_numeric_limits_experiment.int64_case.after_signed_direct = INT64_MAX;
    g_numeric_limits_experiment.int64_case.before_unsigned = UINT64_MAX;
    g_numeric_limits_experiment.int64_case.after_unsigned_direct = UINT64_MAX;

    g_numeric_limits_experiment.float_case.before = FLT_MAX;
    g_numeric_limits_experiment.float_case.after_direct = FLT_MAX;
    g_numeric_limits_experiment.double_case.before = DBL_MAX;
    g_numeric_limits_experiment.double_case.after_direct = DBL_MAX;

    g_numeric_limits_experiment.int8_case.after_signed_direct++;
    g_numeric_limits_experiment.int8_case.after_unsigned_direct++;
    g_numeric_limits_experiment.int16_case.after_signed_direct++;
    g_numeric_limits_experiment.int16_case.after_unsigned_direct++;
    g_numeric_limits_experiment.int32_case.after_signed_direct++;
    g_numeric_limits_experiment.int32_case.after_unsigned_direct++;
    g_numeric_limits_experiment.int64_case.after_signed_direct++;
    g_numeric_limits_experiment.int64_case.after_unsigned_direct++;
    g_numeric_limits_experiment.float_case.after_direct++;
    g_numeric_limits_experiment.double_case.after_direct++;

    g_numeric_limits_experiment.int8_case.signed_reference_bits =
        (uint8_t)((uint8_t)INT8_MAX + UINT8_C(1));
    g_numeric_limits_experiment.int8_case.unsigned_reference =
        (uint8_t)(UINT8_MAX + UINT8_C(1));
    g_numeric_limits_experiment.int16_case.signed_reference_bits =
        (uint16_t)((uint16_t)INT16_MAX + UINT16_C(1));
    g_numeric_limits_experiment.int16_case.unsigned_reference =
        (uint16_t)(UINT16_MAX + UINT16_C(1));
    g_numeric_limits_experiment.int32_case.signed_reference_bits =
        (uint32_t)INT32_MAX + UINT32_C(1);
    g_numeric_limits_experiment.int32_case.unsigned_reference =
        UINT32_MAX + UINT32_C(1);
    g_numeric_limits_experiment.int64_case.signed_reference_bits =
        (uint64_t)INT64_MAX + UINT64_C(1);
    g_numeric_limits_experiment.int64_case.unsigned_reference =
        UINT64_MAX + UINT64_C(1);

    g_numeric_limits_experiment.float_case.unchanged =
        (uint8_t)(g_numeric_limits_experiment.float_case.after_direct ==
                  g_numeric_limits_experiment.float_case.before);
    g_numeric_limits_experiment.float_case.is_positive_infinity =
        (uint8_t)(g_numeric_limits_experiment.float_case.after_direct >
                  FLT_MAX);
    g_numeric_limits_experiment.double_case.unchanged =
        (uint8_t)(g_numeric_limits_experiment.double_case.after_direct ==
                  g_numeric_limits_experiment.double_case.before);
    g_numeric_limits_experiment.double_case.is_positive_infinity =
        (uint8_t)(g_numeric_limits_experiment.double_case.after_direct >
                  DBL_MAX);

    g_numeric_limits_experiment.completed = 1u;
}
```

- [ ] **Step 6: Run the focused test and verify GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
    -File tests/run_numeric_limits_experiment_tests.ps1
```

Expected:

```text
PASS: numeric limits increment experiment
```

The compile output must contain no warning.

- [ ] **Step 7: Inspect generated host code assumptions**

Run:

```powershell
rg -n "after_signed_direct\\+\\+|after_unsigned_direct\\+\\+|after_direct\\+\\+" `
    App/Src/app_numeric_limits_experiment.c
```

Expected: ten direct increment statements: eight integer fields plus `float`
and `double`.

- [ ] **Step 8: Commit the module and focused test**

```powershell
git add -- `
    App/Inc/app_numeric_limits_experiment.h `
    App/Src/app_numeric_limits_experiment.c `
    tests/test_app_numeric_limits_experiment.c `
    tests/run_numeric_limits_experiment_tests.ps1
git commit -m "feat: add numeric limits increment experiment"
```

---

### Task 2: Integrate the experiment into both firmware builds

**Files:**
- Create: `tests/test_numeric_limits_integration.ps1`
- Modify: `Core/Src/main.c`
- Modify: `Makefile`
- Modify: `MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx`

**Interfaces:**
- Consumes: `void app_numeric_limits_experiment_run_once(void)` from Task 1.
- Produces: one startup invocation after `HAL_Init()` and before
  `SystemClock_Config()`.
- Produces: GCC and Keil project membership for
  `App/Src/app_numeric_limits_experiment.c`.

- [ ] **Step 1: Write the failing integration contract**

Create `tests/test_numeric_limits_integration.ps1`:

```powershell
$ErrorActionPreference = "Stop"

$checks = @(
    @{
        Path = "Core/Src/main.c"
        Text = '#include "app_numeric_limits_experiment.h"'
        Description = "main includes the experiment API"
    },
    @{
        Path = "Core/Src/main.c"
        Text = "app_numeric_limits_experiment_run_once();"
        Description = "main runs the experiment once"
    },
    @{
        Path = "Makefile"
        Text = "App/Src/app_numeric_limits_experiment.c"
        Description = "GCC firmware build includes the experiment"
    },
    @{
        Path = "MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx"
        Text = "<FileName>app_numeric_limits_experiment.c</FileName>"
        Description = "Keil project names the experiment source"
    },
    @{
        Path = "MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx"
        Text = "..\App\Src\app_numeric_limits_experiment.c"
        Description = "Keil project points to the experiment source"
    }
)

foreach ($check in $checks) {
    $content = Get-Content -Raw $check.Path
    if ($content.IndexOf($check.Text,
                         [System.StringComparison]::Ordinal) -lt 0) {
        throw "FAIL: $($check.Description)"
    }
}

$main = Get-Content -Raw "Core/Src/main.c"
$halIndex = $main.IndexOf("HAL_Init();",
                          [System.StringComparison]::Ordinal)
$runIndex = $main.IndexOf("app_numeric_limits_experiment_run_once();",
                          [System.StringComparison]::Ordinal)
$clockIndex = $main.IndexOf("SystemClock_Config();",
                            [System.StringComparison]::Ordinal)

if (-not ($halIndex -lt $runIndex -and $runIndex -lt $clockIndex)) {
    throw "FAIL: experiment must run after HAL_Init and before clock setup"
}

Write-Output "PASS: numeric limits firmware integration"
```

- [ ] **Step 2: Run the integration contract and verify RED**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
    -File tests/test_numeric_limits_integration.ps1
```

Expected: FAIL because `main.c`, the Makefile, and the Keil project do not yet
reference the experiment.

- [ ] **Step 3: Add the startup include and one-shot call**

Inside `Core/Src/main.c`'s existing `USER CODE BEGIN Includes` block, add:

```c
#include "app_numeric_limits_experiment.h"
```

Inside the existing `USER CODE BEGIN Init` block after `HAL_Init()`, add:

```c
  app_numeric_limits_experiment_run_once();
```

Do not change code outside CubeMX user-code regions and do not add a call to the
main loop.

- [ ] **Step 4: Add the source to the GCC firmware build**

In `Makefile`, add the source beside the other application sources:

```make
  App/Src/app_numeric_limits_experiment.c \
```

Do not change `CFLAGS`; in particular, do not add `-fwrapv`.

- [ ] **Step 5: Add the source to the Keil project**

In the `Application/User/App` group of
`MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx`, add:

```xml
<File>
  <FileName>app_numeric_limits_experiment.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\App\Src\app_numeric_limits_experiment.c</FilePath>
</File>
```

Do not change target optimization or C language settings.

- [ ] **Step 6: Run the integration contract and verify GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
    -File tests/test_numeric_limits_integration.ps1
```

Expected:

```text
PASS: numeric limits firmware integration
```

- [ ] **Step 7: Run focused and existing host regression suites**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
    -File tests/run_numeric_limits_experiment_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass `
    -File tests/run_encoder_speed_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass `
    -File tests/run_ld2rs_unified_read_tests.ps1
```

Expected: the numeric-limits suite prints its PASS line. Existing suites must
show no regression relative to their current baseline; any pre-existing
failure must be recorded rather than hidden by unrelated edits.

- [ ] **Step 8: Run the STM32H7 GCC build**

Run:

```powershell
make
```

Expected: the new source compiles, the firmware links, and
`arm-none-eabi-size` reports flash and RAM usage. Any warning associated with
the intentional direct signed increments must be reported exactly; do not
silence it by adding wrap flags.

- [ ] **Step 9: Run final static verification**

Run:

```powershell
git diff --check
rg -n "fwrapv" Makefile MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx
rg -n "app_numeric_limits_experiment_run_once" Core App
git status --short
```

Expected:

- no whitespace errors;
- no `fwrapv` target-build setting;
- exactly one call from `main.c` and one function definition;
- the pre-existing deleted AXF and untracked `.claude/` remain untouched.

- [ ] **Step 10: Commit the integration**

```powershell
git add -- `
    Core/Src/main.c `
    Makefile `
    MDK-ARM/CtrBoard-H7-AIBOT-DPC.uvprojx `
    tests/test_numeric_limits_integration.ps1
git commit -m "build: run numeric limits experiment at startup"
```

- [ ] **Step 11: Perform the Keil Watch hardware observation**

Build and flash with ARMCLANG in Keil5, run past
`app_numeric_limits_experiment_run_once()`, then add
`g_numeric_limits_experiment` to Watch.

Record:

- `completed`;
- every `after_signed_direct`;
- every `after_unsigned_direct`;
- every `signed_reference_bits`;
- `float_case.after_direct`, `float_case.unchanged`, and
  `float_case.is_positive_infinity`;
- `double_case.after_direct`, `double_case.unchanged`, and
  `double_case.is_positive_infinity`;
- Keil target optimization level and ARMCLANG version.

Expected language-level interpretation:

- every unsigned direct result and unsigned reference is zero;
- signed direct fields are compiler/target observations, not portable
  guarantees;
- under default round-to-nearest, both floating-point direct values remain at
  their maximum and set `unchanged = 1`, `is_positive_infinity = 0`.

Hardware flashing and Watch inspection require the user's connected board and
Keil session; if unavailable, report this final observation as pending rather
than claiming it was verified.
