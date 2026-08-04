#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;

typedef struct
{
    volatile uint32_t DEMCR;
} CoreDebug_Type;

extern DWT_Type g_test_dwt;
extern CoreDebug_Type g_test_core_debug;
extern uint32_t SystemCoreClock;

#define DWT                               (&g_test_dwt)
#define CoreDebug                         (&g_test_core_debug)
#define DWT_BASE                          ((uintptr_t)0x1000u)
#define DWT_CTRL_CYCCNTENA_Msk            (1u << 0)
#define CoreDebug_DEMCR_TRCENA_Msk        (1u << 24)

void SystemCoreClockUpdate(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t primask);

#define __DSB() ((void)0)
#define __ISB() ((void)0)

#endif /* TEST_MAIN_H */
