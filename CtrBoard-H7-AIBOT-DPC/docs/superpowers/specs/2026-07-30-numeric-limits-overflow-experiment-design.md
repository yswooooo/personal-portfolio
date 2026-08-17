# STM32H7 Numeric Limits Increment Experiment Design

## Goal

Add a one-shot numeric limits experiment to the existing STM32H723 firmware
without interrupting its motor-control, remote-control, or telemetry behavior.
The experiment initializes fixed-width integer, `float`, and `double` values to
their maximum values, applies one direct `++` operation to each value, and keeps
the results in a global `volatile` structure for inspection in the Keil Watch
window.

The experiment must distinguish the compiler and target's observed behavior
from behavior guaranteed by the C language.

## Scope

The experiment covers:

- `int8_t` and `uint8_t`
- `int16_t` and `uint16_t`
- `int32_t` and `uint32_t`
- `int64_t` and `uint64_t`
- `float`
- `double`

It runs once during startup and does not print over USART, modify the main loop,
or change the existing motor and safety logic.

## C Language Semantics

The result categories are not identical for every integer width:

- Unsigned integer conversion and arithmetic are defined modulo one more than
  the maximum representable value. Each unsigned result is therefore expected
  to become zero.
- On this 32-bit target, `int8_t` and `int16_t` operands are promoted to `int`
  for the addition performed by `++`. The intermediate result is representable
  as `int`, but converting it back to the narrower signed type is
  implementation-defined when it is out of range.
- Incrementing `INT32_MAX` and `INT64_MAX` directly overflows their promoted
  signed types on this target and has undefined behavior.
- Common ARM two's-complement builds will often show the corresponding minimum
  signed value, but the experiment must not present that observation as a C
  language guarantee.
- With the STM32H7 default round-to-nearest floating-point mode,
  `FLT_MAX + 1.0f` and `DBL_MAX + 1.0` round back to their original maximum
  values. A different floating-point rounding mode can change this observation,
  so the result structure also records equality and infinity flags.

`volatile` preserves observable accesses needed for debugger inspection. It
does not make signed overflow defined and does not impose wraparound semantics.
The build will not add `-fwrapv` or an equivalent option.

## Architecture

Create an isolated application-layer module:

```text
App/Inc/app_numeric_limits_experiment.h
App/Src/app_numeric_limits_experiment.c
tests/test_app_numeric_limits_experiment.c
```

`Core/Src/main.c` includes the module header and calls
`app_numeric_limits_experiment_run_once()` from a CubeMX `USER CODE` region
after `HAL_Init()` and before the existing peripheral and business
initialization. The rest of startup and the existing `while (1)` loop remain
unchanged.

The source is added to both the root GCC `Makefile` and the Keil project so the
two supported firmware build paths remain consistent.

## Result Structure

Each integer width has a debugger-friendly record with explicit fields:

```c
typedef struct
{
    signed_type before_signed;
    signed_type after_signed_direct;
    unsigned_type before_unsigned;
    unsigned_type after_unsigned_direct;
    unsigned_type signed_reference_bits;
    unsigned_type unsigned_reference;
} app_numeric_intN_case_t;
```

The concrete records use exact-width types rather than a macro-generated or
packed representation. This makes all fields easy to expand in Keil Watch and
avoids alignment-dependent decoding.

The fields mean:

- `before_signed`: the corresponding `INTN_MAX`
- `after_signed_direct`: target/compiler observation after direct `++`
- `before_unsigned`: the corresponding `UINTN_MAX`
- `after_unsigned_direct`: target/compiler observation after direct `++`
- `signed_reference_bits`: the next modulo bit pattern computed entirely in
  the corresponding unsigned type, such as `0x80` or `0x80000000`
- `unsigned_reference`: the standard-defined unsigned result, zero

`signed_reference_bits` is deliberately unsigned. It is a reference bit
pattern, not a claim that the C standard defines the signed direct result.

The floating-point records contain:

```c
typedef struct
{
    floating_type before;
    floating_type after_direct;
    uint8_t unchanged;
    uint8_t is_positive_infinity;
} app_numeric_float_case_t;
```

The top-level result contains four integer-width records, the `float` and
`double` records, and a `uint32_t completed` marker. The exported object itself
is volatile:

```c
extern volatile app_numeric_limits_experiment_t
    g_numeric_limits_experiment;
```

`completed` is written last. A value of `1` tells the debugger that every result
field has been populated.

## Experiment Flow

`app_numeric_limits_experiment_run_once()` performs these steps:

1. Clear `completed`.
2. Initialize all direct integer fields with `INTN_MAX` and `UINTN_MAX`.
3. Initialize `float` and `double` fields with `FLT_MAX` and `DBL_MAX`.
4. Apply exactly one direct `++` to every `after_*_direct` value.
5. Compute unsigned reference results and signed reference bit patterns without
   signed overflow.
6. Record floating-point equality and positive-infinity flags.
7. Write `completed = 1`.

The function has no HAL dependency, no dynamic allocation, no I/O, and no
looping state. Calling it once has no effect on later firmware behavior.

## Debugger Use

After programming the target:

1. Run past `app_numeric_limits_experiment_run_once()`.
2. Add `g_numeric_limits_experiment` to Keil Watch.
3. Confirm `completed == 1`.
4. Compare each `after_*_direct` field with its reference field.
5. Treat signed direct results as observations tied to the exact compiler,
   optimization settings, and target build.

The global symbol remains available after startup because it has external
linkage and volatile storage.

## Testing and Verification

The host-side test follows the existing lightweight C test style. It verifies:

- maximum initial values are recorded correctly;
- every unsigned direct result is zero;
- every unsigned reference result is zero;
- signed reference bit patterns have only the most significant bit set;
- `float` and `double` result flags agree with the recorded values;
- `completed` is written after the experiment.

The test does not assert a particular value for any
`after_signed_direct` field. Such an assertion would incorrectly turn
implementation-defined or undefined observations into an API contract.

Verification consists of:

1. Demonstrating the new host test fails before the module exists.
2. Building and running the host test after implementation.
3. Running the existing host tests to catch regressions.
4. Running the root `make` command for the STM32H7 GCC build.
5. Confirming that the new module is present in the Keil project file.

Actual ARMCLANG signed results are finally observed on the STM32H723 through
Keil Watch; a host test or GCC cross-build cannot substitute for that hardware
observation.

## Non-Goals

- Defining or normalizing signed overflow behavior
- Changing compiler overflow flags
- Sending results over USART or VOFA+
- Replacing the existing application with a dedicated experiment firmware
- Repeatedly incrementing values in the main loop
- Testing `maximum + maximum` floating-point overflow
