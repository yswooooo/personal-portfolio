/**
  ******************************************************************************
  * @file    ws4810.c
  * @brief   WS4810 (WS2812 兼容) SPI 模拟时序
  ******************************************************************************
  */

#include "bsp_ws4810.h"
#include "spi.h"

#define BSP_WS4810_BIT_LOW   0xC0u
#define BSP_WS4810_BIT_HIGH  0xF0u

void bsp_ws4810_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t tx_buffer[24];
    uint8_t reset_code = 0u;
    int bit_idx;

    for (bit_idx = 0; bit_idx < 8; bit_idx++) {
        tx_buffer[7  - bit_idx] = (uint8_t)(((green >> bit_idx) & 0x01u) ? BSP_WS4810_BIT_HIGH : BSP_WS4810_BIT_LOW) >> 1;
        tx_buffer[15 - bit_idx] = (uint8_t)(((red >> bit_idx) & 0x01u) ? BSP_WS4810_BIT_HIGH : BSP_WS4810_BIT_LOW) >> 1;
        tx_buffer[23 - bit_idx] = (uint8_t)(((blue >> bit_idx) & 0x01u) ? BSP_WS4810_BIT_HIGH : BSP_WS4810_BIT_LOW) >> 1;
    }

    (void)HAL_SPI_Transmit(&hspi6, &reset_code, 0u, 0xFFFFu);
    while (hspi6.State != HAL_SPI_STATE_READY) {}

    (void)HAL_SPI_Transmit(&hspi6, tx_buffer, 24u, 0xFFFFu);

    for (bit_idx = 0; bit_idx < 100; bit_idx++) {
        (void)HAL_SPI_Transmit(&hspi6, &reset_code, 1u, 0xFFFFu);
    }
}


