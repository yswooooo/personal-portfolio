/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* 急停标志: 1=强制停机 0=正常运行, (仅 ISR 置位/清零，不操作速度变量) */
volatile uint8_t g_emergency_stop_flag = 0u;

/**
  * @brief  GPIO EXTI 回调 — 按键乒乓切换急停标志
  *
  * @note   ISR 只做标志位翻转，速度清零由 MotorCtrl_Task 完成。
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t s_u32LastPressTick = 0u;
    uint32_t now_tick;

    if (GPIO_Pin != KEY_STOP_Pin) {
        return;
    }

    /* 200ms 消抖 */
    now_tick = HAL_GetTick();
    if ((now_tick - s_u32LastPressTick) < 200u) {
        return;
    }
    s_u32LastPressTick = now_tick;

    /* 翻转急停标志 */
    g_emergency_stop_flag = (g_emergency_stop_flag != 0u) ? 0u : 1u;
}

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, POWER_OUT1_Pin|POWER_OUT2_Pin|POWER_5V_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : POWER_OUT1_Pin POWER_OUT2_Pin POWER_5V_Pin */
  GPIO_InitStruct.Pin = POWER_OUT1_Pin|POWER_OUT2_Pin|POWER_5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_STOP_Pin */
  GPIO_InitStruct.Pin = KEY_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_STOP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
