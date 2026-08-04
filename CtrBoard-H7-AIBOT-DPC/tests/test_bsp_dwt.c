#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../BSP/Src/bsp_dwt.c"

DWT_Type g_test_dwt;
CoreDebug_Type g_test_core_debug;
uint32_t SystemCoreClock = 480000000u;

static uint32_t s_test_primask;

void SystemCoreClockUpdate(void)
{
}

uint32_t __get_PRIMASK(void)
{
    return s_test_primask;
}

void __disable_irq(void)
{
    s_test_primask = 1u;
}

void __set_PRIMASK(uint32_t primask)
{
    s_test_primask = primask;
}

static void expect_u64(uint64_t actual,
                       uint64_t expected,
                       const char *message)
{
    if (actual != expected) {
        fprintf(stderr,
                "FAIL: %s (actual=%" PRIu64 " expected=%" PRIu64 ")\n",
                message,
                actual,
                expected);
        exit(1);
    }
}

static void reset_fixture(uint32_t last_cycles)
{
    memset(&g_test_dwt, 0, sizeof(g_test_dwt));
    memset(&g_test_core_debug, 0, sizeof(g_test_core_debug));
    s_dwt_initialized = true;
    s_dwt_last_cycles = last_cycles;
    s_dwt_total_cycles = 0u;
    s_test_primask = 0u;
}

static void test_480_cycles_equal_one_microsecond(void)
{
    reset_fixture(0u);
    g_test_dwt.CYCCNT = 480u;

    expect_u64(BSP_DWT_GetTickUs(),
               1u,
               "480 cycles must equal one microsecond");
}

static void test_microsecond_time_survives_one_cycle_counter_wrap(void)
{
    reset_fixture(UINT32_MAX - 239u);
    g_test_dwt.CYCCNT = 240u;

    expect_u64(BSP_DWT_GetTickUs(),
               1u,
               "one CYCCNT wrap must preserve 480 elapsed cycles");
}

static void test_millisecond_and_microsecond_getters_share_cycles(void)
{
    reset_fixture(0u);
    g_test_dwt.CYCCNT = 480000u;

    expect_u64(BSP_DWT_GetTickMs(),
               1u,
               "480000 cycles must equal one millisecond");

    g_test_dwt.CYCCNT = 480480u;
    expect_u64(BSP_DWT_GetTickUs(),
               1001u,
               "microsecond getter must continue the shared timeline");
}

int main(void)
{
    test_480_cycles_equal_one_microsecond();
    test_microsecond_time_survives_one_cycle_counter_wrap();
    test_millisecond_and_microsecond_getters_share_cycles();
    puts("PASS: BSP DWT microsecond timestamp");
    return 0;
}
