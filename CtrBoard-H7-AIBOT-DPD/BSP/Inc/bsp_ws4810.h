/**
  ******************************************************************************
  * @file    ws4810.h
  * @brief   WS4810 (WS2812 兼容) RGB LED 驱动
  ******************************************************************************
  */

#ifndef BSP_WS4810_H
#define BSP_WS4810_H

#include <stdint.h>

void bsp_ws4810_set_rgb(uint8_t red, uint8_t green, uint8_t blue);

#endif /* BSP_WS4810_H */


