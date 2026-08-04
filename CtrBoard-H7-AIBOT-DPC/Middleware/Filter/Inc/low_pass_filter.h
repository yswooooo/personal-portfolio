/**
  ******************************************************************************
  * @file    low_pass_filter.h
  * @brief   通用一阶低通滤波器 (Middleware 层)
  *
  * @details 离散一阶低通滤波:
  *          y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
  *          其中 alpha = Ts / (Ts + tau), Ts 为采样周期, tau 为时间常数。
  *
  *          本模块为纯算法，不依赖任何 BSP / HAL / APP 接口。
  ******************************************************************************
  */

#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  一阶低通滤波更新 (浮点)
  * @param  input       当前输入值 x[k]
  * @param  last_output 上一次输出值 y[k-1]
  * @param  alpha       滤波系数 (0~1), alpha = Ts / (Ts + tau)
  * @retval 当前输出值 y[k]
  *
  * @details 离散表达式:
  *          y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
  *
  *          alpha 越小滤波越平滑但响应越慢。
  *          典型值: Ts=10ms, tau=200ms → alpha ≈ 0.0476
  */
float low_pass_filter_update_float(float input, float last_output, float alpha);

#ifdef __cplusplus
}
#endif

#endif /* LOW_PASS_FILTER_H */
