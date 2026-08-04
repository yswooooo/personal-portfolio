#ifndef TEST_BSP_RS485_H
#define TEST_BSP_RS485_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define BSP_RS485_MAX_RETRY 3u

typedef enum {
    BSP_RS485_STATUS_OK = 0,
    BSP_RS485_STATUS_ERR_PARAM = 1,
    BSP_RS485_STATUS_ERR_BUSY = 2,
    BSP_RS485_STATUS_ERR_TIMEOUT = 3,
    BSP_RS485_STATUS_ERR_HAL = 4,
    BSP_RS485_STATUS_ERR_RETRY = 5
} bsp_rs485_status_t;

typedef enum {
    BSP_RS485_STATE_IDLE = 0,
    BSP_RS485_STATE_TX_BUSY = 1,
    BSP_RS485_STATE_RX_WAIT = 2,
    BSP_RS485_STATE_DONE = 3,
    BSP_RS485_STATE_TIMEOUT = 4,
    BSP_RS485_STATE_ERROR = 5
} bsp_rs485_state_t;

typedef struct {
    UART_HandleTypeDef *uart_handle;
    volatile bsp_rs485_state_t state;
    const uint8_t *tx_buffer;
    uint16_t tx_len;
    uint8_t *rx_buffer;
    uint16_t rx_capacity;
    volatile uint16_t rx_len;
    volatile uint64_t rx_timestamp_us;
    uint32_t timeout_ms;
    uint32_t start_tick_ms;
    uint8_t retry_count;
    uint32_t last_done_tick_ms;
    volatile uint8_t rx_error_code;
    volatile uint8_t state_error_code;
} bsp_rs485_handle_t;

extern bsp_rs485_handle_t g_rs485_bus;

bsp_rs485_status_t bsp_rs485_start_tx(bsp_rs485_handle_t *bus,
                                      const uint8_t *tx_buffer,
                                      uint16_t tx_len,
                                      uint8_t *rx_buffer,
                                      uint16_t rx_capacity,
                                      uint32_t timeout_ms);
bsp_rs485_state_t bsp_rs485_poll(bsp_rs485_handle_t *bus);
void bsp_rs485_ack_done(bsp_rs485_handle_t *bus);
void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus);

#endif
