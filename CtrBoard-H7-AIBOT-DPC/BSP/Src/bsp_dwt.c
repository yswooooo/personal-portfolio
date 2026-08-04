#include "bsp_dwt.h"

/*
 * Cortex-M7 的 DWT Lock Access Register。
 */
#define DWT_LAR_ADDRESS       (DWT_BASE + 0x0FB0UL)
#define DWT_LAR_UNLOCK_KEY    (0xC5ACCE55UL)

/*
 * STM32H723VGT6 当前 Cortex-M7 内核频率：480 MHz。
 *
 * 480 MHz：
 * 1 ms = 480000 个 CPU 周期。
 */
#define DWT_CPU_FREQ_HZ       (480000000ULL)
#define DWT_CYCLES_PER_US     (480ULL)
#define DWT_CYCLES_PER_MS     (480000ULL)

/*
 * 上一次读取到的32位 CYCCNT。
 */
static uint32_t s_dwt_last_cycles = 0U;

/*
 * 软件扩展后的64位累计周期数。
 */
static uint64_t s_dwt_total_cycles = 0ULL;

/*
 * DWT是否已经初始化。
 */
static bool s_dwt_initialized = false;

bool BSP_DWT_Init(void)
{
    /*
     * 根据当前时钟树更新 SystemCoreClock。
     */
    SystemCoreClockUpdate();

    /*
     * 确认当前 Cortex-M7 确实运行在480 MHz。
     */
    if (SystemCoreClock != (uint32_t)DWT_CPU_FREQ_HZ) {
        return false;
    }

    /*
     * 开启 Cortex-M CoreSight Trace 模块。
     * DWT属于CoreSight调试体系。
     */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /*
     * 解锁 Cortex-M7 的DWT寄存器。
     */
    *((volatile uint32_t *)DWT_LAR_ADDRESS) =
        DWT_LAR_UNLOCK_KEY;

    /*
     * 先停止CYCCNT。
     */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;

    /*
     * 清零周期计数器。
     */
    DWT->CYCCNT = 0U;

    /*
     * 开启周期计数器。
     */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /*
     * 确保寄存器写入完成，并刷新指令流水线。
     */
    __DSB();
    __ISB();

    /*
     * 初始化软件累计变量。
     */
    s_dwt_last_cycles = DWT->CYCCNT;
    s_dwt_total_cycles = 0ULL;
    s_dwt_initialized = true;

    return true;
}

static uint64_t bsp_dwt_get_total_cycles(void)
{
    uint32_t primask;
    uint32_t now_cycles;
    uint32_t delta_cycles;
    uint64_t total_cycles;

    if (!s_dwt_initialized) {
        return 0ULL;
    }

    /*
     * Cortex-M7是32位内核。
     * s_dwt_total_cycles是64位变量，因此更新时暂时保护临界区。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    /*
     * 获取当前32位CPU周期计数。
     */
    now_cycles = DWT->CYCCNT;

    /*
     * 无符号减法可以正确处理一次32位回绕。
     */
    delta_cycles =
        (uint32_t)(now_cycles - s_dwt_last_cycles);

    /*
     * 将本次经过的周期累计到64位变量。
     */
    s_dwt_total_cycles += (uint64_t)delta_cycles;

    /*
     * 保存本次周期值，供下一次计算。
     */
    s_dwt_last_cycles = now_cycles;

    total_cycles = s_dwt_total_cycles;

    /*
     * 恢复进入函数前的中断状态。
     */
    __set_PRIMASK(primask);

    /*
     * 480000个周期等于1 ms。
     */
    return total_cycles;
}

uint64_t BSP_DWT_GetTickMs(void)
{
    return bsp_dwt_get_total_cycles() / DWT_CYCLES_PER_MS;
}

uint64_t BSP_DWT_GetTickUs(void)
{
    return bsp_dwt_get_total_cycles() / DWT_CYCLES_PER_US;
}
