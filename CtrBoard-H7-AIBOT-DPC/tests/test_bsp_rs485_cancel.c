#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_rs485.h"

static uint32_t s_tick;
static unsigned int s_abort_tx_calls;
static unsigned int s_abort_rx_calls;
static bsp_rs485_handle_t *s_timestamp_bus;

_Static_assert(
    __builtin_types_compatible_p(
        __typeof__(((bsp_rs485_handle_t *)0)->state),
        volatile bsp_rs485_state_t),
    "RS485 state must be volatile");
_Static_assert(
    __builtin_types_compatible_p(
        __typeof__(((bsp_rs485_handle_t *)0)->rx_len),
        volatile uint16_t),
    "RS485 rx_len must be volatile");
_Static_assert(
    __builtin_types_compatible_p(
        __typeof__(((bsp_rs485_handle_t *)0)->rx_timestamp_us),
        volatile uint64_t),
    "RS485 rx_timestamp_us must be volatile");
_Static_assert(
    __builtin_types_compatible_p(
        __typeof__(((bsp_rs485_handle_t *)0)->rx_error_code),
        volatile uint8_t),
    "RS485 rx_error_code must be volatile");

static void expect_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

uint64_t BSP_DWT_GetTickUs(void)
{
    expect_true(s_timestamp_bus != NULL,
                "timestamp test must bind the active bus");
    expect_true(s_timestamp_bus->rx_len == 9u,
                "RX length must be published before timestamp capture");
    expect_true(s_timestamp_bus->state == BSP_RS485_STATE_RX_WAIT,
                "DONE must be published after timestamp capture");
    return 123456u;
}

uint32_t HAL_GetTick(void)
{
    return s_tick;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart_handle,
                                       uint8_t *buffer,
                                       uint16_t length)
{
    (void)uart_handle;
    (void)buffer;
    (void)length;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart_handle)
{
    (void)uart_handle;
    s_abort_tx_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart_handle)
{
    (void)uart_handle;
    s_abort_rx_calls++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *uart_handle,
                                              uint8_t *buffer,
                                              uint16_t capacity)
{
    (void)uart_handle;
    (void)buffer;
    (void)capacity;
    return HAL_OK;
}

int main(void)
{
    UART_HandleTypeDef uart;
    bsp_rs485_handle_t bus;

    memset(&uart, 0, sizeof(uart));
    memset(&bus, 0, sizeof(bus));
    bus.uart_handle = &uart;
    bus.state = BSP_RS485_STATE_TX_BUSY;
    bus.rx_error_code = 1u;
    bus.state_error_code = 1u;
    s_tick = 25u;

    bsp_rs485_cancel_transaction(&bus);

    expect_true(s_abort_tx_calls == 1u,
                "active cancellation must abort transmit once");
    expect_true(s_abort_rx_calls == 1u,
                "active cancellation must abort receive once");
    expect_true(bus.state == BSP_RS485_STATE_IDLE,
                "active cancellation must release the bus");
    expect_true(bus.rx_error_code == 0u && bus.state_error_code == 0u,
                "active cancellation must clear BSP error flags");
    expect_true(bus.last_done_tick_ms == 25u,
                "active cancellation must refresh the inter-frame timestamp");

    bus.state = BSP_RS485_STATE_DONE;
    bsp_rs485_cancel_transaction(&bus);
    expect_true(s_abort_tx_calls == 1u && s_abort_rx_calls == 1u,
                "terminal cancellation must not abort an inactive UART");
    expect_true(bus.state == BSP_RS485_STATE_IDLE,
                "terminal cancellation must acknowledge the transaction");

    bsp_rs485_cancel_transaction(NULL);

    memset(&bus, 0, sizeof(bus));
    bus.uart_handle = &uart;
    bus.state = BSP_RS485_STATE_RX_WAIT;
    s_timestamp_bus = &bus;

    bsp_rs485_rx_event_callback(&bus, 9u);

    expect_true(bus.rx_len == 9u,
                "RX event must publish the received length");
    expect_true(bus.rx_timestamp_us == 123456u,
                "RX event must publish the DWT timestamp");
    expect_true(bus.state == BSP_RS485_STATE_DONE,
                "RX event must publish DONE last");

    puts("PASS: BSP RS485 cancellation and RX publication behavior");
    return 0;
}
