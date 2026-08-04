#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 Cortex-M7 DWT 周期计数器。
 *
 * 必须在 SystemClock_Config() 之后调用。
 *
 * @return true  初始化成功。
 * @return false 初始化失败。
 */
bool BSP_DWT_Init(void);

/**
 * @brief 获取从 DWT 初始化开始经过的毫秒数。
 *
 * 使用方式类似 HAL_GetTick()，但返回 uint64_t。
 *
 * @return 当前毫秒时间戳。
 */
uint64_t BSP_DWT_GetTickMs(void);

#ifdef __cplusplus
}
#endif

#endif