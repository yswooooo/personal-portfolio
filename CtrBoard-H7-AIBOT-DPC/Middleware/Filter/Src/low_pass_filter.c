/**
  ******************************************************************************
  * @file    low_pass_filter.c
  * @brief   通用一阶低通滤波器实现
  ******************************************************************************
  */

#include "low_pass_filter.h"

/* Exported functions --------------------------------------------------------*/

/** @copydoc low_pass_filter_update_float */
float low_pass_filter_update_float(float input, float last_output, float alpha)
{
    return alpha * input + (1.0f - alpha) * last_output;
}
