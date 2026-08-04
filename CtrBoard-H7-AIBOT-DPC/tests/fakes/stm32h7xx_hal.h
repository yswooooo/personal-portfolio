#ifndef TEST_STM32H7XX_HAL_H
#define TEST_STM32H7XX_HAL_H

#include <stdint.h>

typedef struct {
    uint32_t marker;
} UART_HandleTypeDef;

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart_handle,
                                       uint8_t *buffer,
                                       uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart_handle);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart_handle);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *uart_handle,
                                              uint8_t *buffer,
                                              uint16_t capacity);

#define __HAL_UART_CLEAR_IDLEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_OREFLAG(handle)  ((void)(handle))

#endif
