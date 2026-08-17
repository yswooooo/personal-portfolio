/**
  ******************************************************************************
  * @file    low_pass_filter.h
  * @brief   通用一阶低通滤波器 (Middleware 层)
  *
  * @details 离散一阶低通滤波:
  *          y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
  *          其中 alpha = Ts / (Ts + tau), Ts 为采样周期, tau 为时间常数。
  *
  *          本模块为无状态纯算法，不依赖任何 BSP / HAL / APP 接口。
  *          调用者需要为每个被滤波信号独立保存上一次输出 y[k-1]。
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
  * @brief 更新无状态一阶低通滤波器。
  * @param[in] input       当前采样值 x[k]。
  * @param[in] last_output 上一次滤波输出 y[k-1]，由调用者保存。
  * @param[in] alpha       滤波系数，调用者必须保证范围为 [0, 1]。
  * @return 当前滤波输出 y[k]。
  *
  * @details 离散表达式:
  *          y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
  *
  *          alpha 越小，输出越平滑但响应越慢；alpha 越大，输出越接近当前输入。
  *          当前实现不会裁剪 alpha，超出 [0, 1] 时不保证具有低通滤波特性。
  *
  * @note 首次使用时可把历史输出初始化为首个输入值，避免从零开始造成启动跳变。
  *       每个采样周期将上次返回值作为下一次 last_output；不同信号分别保存状态。
  *
  * @code
  * float filtered = initial_input;
  * const float alpha = sample_period_s
  *                     / (sample_period_s + time_constant_s);
  *
  * // 在固定采样周期内重复执行：
  * filtered = low_pass_filter_update_float(raw_input, filtered, alpha);
  * @endcode
  */
float low_pass_filter_update_float(float input, float last_output, float alpha);

#ifdef __cplusplus
}
#endif

#endif /* LOW_PASS_FILTER_H */
