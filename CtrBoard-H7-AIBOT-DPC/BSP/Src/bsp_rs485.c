/**
  ******************************************************************************
  * @file    bsp_rs485.c
  * @brief   RS485 半双工总线 — 非阻塞状态机实现
  *
  * @details 状态转换:
  *          IDLE ─StartTx→ TX_BUSY ─TC中断→ RX_WAIT ─IDLE中断→ DONE
  *          RX_WAIT ─超时→ TIMEOUT ─重试→ TX_BUSY 或 报错
  *
  *          帧间隔: StartTx 入口用 lastDoneTick 判断, 不足 5ms 返回 BUSY。
  *          超时重试: Poll 自动处理, 最多 BSP_RS485_MAX_RETRY 次。
  ******************************************************************************
  */

#include "bsp_rs485.h"
#include "app_config.h"
#include "bsp_dwt.h"

/* ------------------------------------------------------------------ */
/* Exported instance                                                    */
/* ------------------------------------------------------------------ */
bsp_rs485_handle_t g_rs485_bus;

/* ------------------------------------------------------------------ */
/* Private helpers                                                      */
/* ------------------------------------------------------------------ */

static bsp_rs485_state_t bsp_rs485_retry_or_timeout(bsp_rs485_handle_t *bus, uint32_t now_tick)
{
    HAL_StatusTypeDef hal_status;

    if ((bus == NULL) || (bus->uart_handle == NULL) || (bus->tx_buffer == NULL)
        || (bus->tx_len == 0u)) {
        if (bus != NULL) {
            bus->state = BSP_RS485_STATE_ERROR;
            bus->state_error_code = 1u;
        }
        return BSP_RS485_STATE_ERROR;
    }

    bus->retry_count++;
    if (bus->retry_count >= BSP_RS485_MAX_RETRY) {
        bus->state = BSP_RS485_STATE_TIMEOUT;
        return BSP_RS485_STATE_TIMEOUT;
    }
    /* 若未检测出重试次数满，则执行中断重试发送 */
    bus->rx_len = 0u;
    bus->rx_timestamp_us = 0u;
    bus->rx_error_code = 0u;
    bus->state_error_code = 0u;
    bus->start_tick_ms = now_tick;
    bus->state = BSP_RS485_STATE_TX_BUSY;
    // 重试发送X3
    hal_status = HAL_UART_Transmit_IT(bus->uart_handle,
                                      (uint8_t *)bus->tx_buffer,
                                      bus->tx_len);
    if (hal_status != HAL_OK) {
        bus->state = BSP_RS485_STATE_ERROR;
        bus->state_error_code = 1u;
        return BSP_RS485_STATE_ERROR;
    }
    /* 若hal_state正常，则RS485总线状态标志为正在忙于发送 */
    return BSP_RS485_STATE_TX_BUSY;
}

/* Exported functions -------------------------------------------------------- */

/**
  * @brief  初始化 RS485 总线句柄
  */
bsp_rs485_status_t bsp_rs485_init(bsp_rs485_handle_t *bus,
                                  UART_HandleTypeDef *uart_handle)
{
    if ((bus == NULL) || (uart_handle == NULL)) {
        return BSP_RS485_STATUS_ERR_PARAM;
    }

    bus->uart_handle          = uart_handle;
    bus->state          = BSP_RS485_STATE_IDLE;
    bus->tx_buffer          = NULL;
    bus->tx_len        = 0u;
    bus->rx_buffer          = NULL;
    bus->rx_capacity        = 0u;
    bus->rx_len        = 0u;
    bus->rx_timestamp_us = 0u;
    bus->timeout_ms    = 0u;
    bus->start_tick_ms    = 0u;
    bus->retry_count      = 0u;
    bus->last_done_tick_ms = 0u;
    bus->rx_error_code       = 0u;
    bus->state_error_code    = 0u;

    return BSP_RS485_STATUS_OK;
}

/**
  * @brief  发起非阻塞 RS485 事务 (立即返回)
  */
bsp_rs485_status_t bsp_rs485_start_tx(bsp_rs485_handle_t *bus,
    const uint8_t *tx_buffer, uint16_t tx_len,
    uint8_t *rx_buffer, uint16_t rx_capacity, uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    uint32_t now_tick;

    /* 参数校验 */
    if ((bus == NULL) || (bus->uart_handle == NULL)
        || (tx_buffer == NULL) || (tx_len == 0u)
        || (rx_buffer == NULL) || (rx_capacity == 0u)) {
        return BSP_RS485_STATUS_ERR_PARAM;
    }

    /* 总线状态检查 */
    if (bus->state != BSP_RS485_STATE_IDLE) {
        return BSP_RS485_STATUS_ERR_BUSY;
    }

    /* 帧间隔: Modbus 3.5 字符 ≈ 0.33ms @ 115200bps */
    now_tick = HAL_GetTick();
    if ((now_tick - bus->last_done_tick_ms) < APP_MODBUS_RTU_INTERFRAME_MS) {
        return BSP_RS485_STATUS_ERR_BUSY;   /* 不等, 下次主循环重试 */
    }

    /* 绑定缓冲区 */
    bus->tx_buffer       = tx_buffer;
    bus->tx_len     = tx_len;
    bus->rx_buffer       = rx_buffer;
    bus->rx_capacity     = rx_capacity;
    bus->rx_len     = 0u;
    bus->rx_timestamp_us = 0u;
    bus->timeout_ms = timeout_ms;
    bus->start_tick_ms = now_tick;
    bus->retry_count   = 0u;     /* 新事务复位重试计数 */
    bus->rx_error_code    = 0u;
    bus->state_error_code = 0u;

    /* 发起中断式 TX
     * 注: HAL_UART_Transmit_IT 参数未声明 const, 但不会修改缓冲区,
     *     此处强转仅为适配 HAL API 签名。
     */
    bus->state = BSP_RS485_STATE_TX_BUSY;
    hal_status = HAL_UART_Transmit_IT(bus->uart_handle,
                                      (uint8_t *)tx_buffer,
                                      tx_len);
    if (hal_status != HAL_OK) {
        bus->state = BSP_RS485_STATE_IDLE;
        bus->state_error_code = 1u;
        return BSP_RS485_STATUS_ERR_HAL;
    }

    return BSP_RS485_STATUS_OK;
}

/**
  * @brief  主循环轮询 — 检查事务是否完成或超时
  *
  * @details 仅在 BSP_RS485_STATE_RX_WAIT 状态检查超时。
  *          超时后自动重试最多 BSP_RS485_MAX_RETRY 次，
  *          重试耗尽则置 BSP_RS485_STATE_TIMEOUT。
  *          其他状态直接返回当前状态，零 CPU 阻塞。
  *
  * @param  bus 总线句柄 (不可为 NULL)
  * @retval 当前状态
  */
bsp_rs485_state_t bsp_rs485_poll(bsp_rs485_handle_t *bus)
{
    uint32_t now_tick;

    if (bus == NULL) {
        return BSP_RS485_STATE_IDLE;
    }

    now_tick = HAL_GetTick();

    /* TX_BUSY / RX_WAIT 全阶段检查超时，避免 TC/IDLE 回调丢失后锁死总线 */
    if ((bus->state == BSP_RS485_STATE_TX_BUSY) || (bus->state == BSP_RS485_STATE_RX_WAIT)) {
        if ((now_tick - bus->start_tick_ms) > bus->timeout_ms) {
            (void)HAL_UART_AbortTransmit(bus->uart_handle);
            (void)HAL_UART_AbortReceive(bus->uart_handle);
            return bsp_rs485_retry_or_timeout(bus, now_tick);
        }
    }

    return bus->state;
}

/**
  * @brief  TC 中断回调 — TX 最后一个字节已从移位寄存器发出
  *
  * @details 此时硬件 DE 已由 UART 外设自动释放 (RS485 模式)。
  *          清除 IDLE/ORE 残余标志后启动 HAL_UARTEx_ReceiveToIdle_IT,
  *          总线进入 RX_WAIT 状态等待从机应答。
  *
  * @note   必须在 HAL_UART_TxCpltCallback 中调用, 不可用于 TXE 半空中断。
  * @param  bus 总线句柄
  */
void bsp_rs485_tx_cplt_callback(bsp_rs485_handle_t *bus)
{
    HAL_StatusTypeDef hal_status;

    if (bus == NULL || bus->uart_handle == NULL) {
        return;
    }

    if (bus->state != BSP_RS485_STATE_TX_BUSY) {
        bus->state_error_code = 1u;
        return;
    }

    if ((bus->rx_buffer == NULL) || (bus->rx_capacity == 0u)) {
        bus->state = BSP_RS485_STATE_ERROR;
        bus->state_error_code = 1u;
        return;
    }

    /* 清 IDLE/ORE 残余标志, 启动 IDLE 中断接收 */
    __HAL_UART_CLEAR_IDLEFLAG(bus->uart_handle);
    __HAL_UART_CLEAR_OREFLAG(bus->uart_handle);

    hal_status = HAL_UARTEx_ReceiveToIdle_IT(bus->uart_handle,
                                             bus->rx_buffer,
                                             bus->rx_capacity);
    if (hal_status == HAL_OK) {
        bus->state = BSP_RS485_STATE_RX_WAIT;
    } else {
        bus->state = BSP_RS485_STATE_ERROR;
        bus->state_error_code = 1u;
    }
}

/**
  * @brief  IDLE 中断回调 — 总线空闲, 应答帧接收完成
  *
  * @details HAL 在检测到 RX 线路空闲 (≥1 字符时长无数据) 时触发。
  *          记录实际接收长度和完成时刻 (供帧间隔判断), 状态置 DONE。
  *          数据已在 rx_buffer 中 (由 HAL_UARTEx_ReceiveToIdle_IT 填入)。
  *
  * @param  bus    总线句柄
  * @param  rx_size 实际接收字节数
  */
void bsp_rs485_rx_event_callback(bsp_rs485_handle_t *bus, uint16_t rx_size)
{
    if (bus != NULL) {
        if (bus->state != BSP_RS485_STATE_RX_WAIT) {
            bus->state_error_code = 1u;
            return;
        }

        bus->rx_len = rx_size;
        bus->rx_timestamp_us = BSP_DWT_GetTickUs();
        bus->state = BSP_RS485_STATE_DONE;

        /* 记录完成时刻, 供帧间隔判断 */
        bus->last_done_tick_ms = HAL_GetTick();
    }
}

/**
  * @brief  UART 错误回调 — 接收期间发生 ORE/FE/NE 等硬件错误
  *
  * @details 置 rx_error_code 标志供上层诊断, 状态直接置 TIMEOUT。
  *          上层 Poll 或 MotorRuntime 应检查 rx_error_code 并记录/告警。
  *
  * @param  bus 总线句柄
  */
void bsp_rs485_error_callback(bsp_rs485_handle_t *bus)
{
    if (bus != NULL) {
        if ((bus->state != BSP_RS485_STATE_TX_BUSY)
            && (bus->state != BSP_RS485_STATE_RX_WAIT)) {
            bus->state_error_code = 1u;
            return;
        }

        bus->rx_error_code = 1u;
        bus->state    = BSP_RS485_STATE_TIMEOUT;
    }
}

/**
  * @brief  确认事务完成, 释放总线回 IDLE
  *
  * @details 替代直接写 bus->state = BSP_RS485_STATE_IDLE。
  *          统一清理 RX 错误标志, 确保总线干净地进入下次事务。
  *          调用者在处理完 DONE 数据或确认 TIMEOUT 后调用。
  *
  * @param  bus 总线句柄 (不可为 NULL)
  */
void bsp_rs485_ack_done(bsp_rs485_handle_t *bus)
{
    if (bus != NULL) {
        bus->rx_len = 0u;
        bus->rx_timestamp_us = 0u;
        bus->state = BSP_RS485_STATE_IDLE;
        bus->rx_error_code = 0u;
        bus->state_error_code = 0u;
        bus->last_done_tick_ms = HAL_GetTick();
    }
}

/**
  * @brief  取消当前异步事务并释放总线
  */
void bsp_rs485_cancel_transaction(bsp_rs485_handle_t *bus)
{
    if ((bus == NULL) || (bus->uart_handle == NULL)) {
        return;
    }

    if ((bus->state == BSP_RS485_STATE_TX_BUSY)
        || (bus->state == BSP_RS485_STATE_RX_WAIT)) {
        (void)HAL_UART_AbortTransmit(bus->uart_handle);
        (void)HAL_UART_AbortReceive(bus->uart_handle);
    }

    bsp_rs485_ack_done(bus);
}

/**
 * @brief  RS485 阻塞事务 (仅 app_chassis_ld2rs_motor_ctrl_init 使用)
 * @note   运行时禁止使用 —— 请用 bsp_rs485_start_tx + bsp_rs485_poll。
 */
bsp_rs485_status_t bsp_rs485_transmit_receive(bsp_rs485_handle_t *bus,
    const uint8_t *tx_buffer, uint16_t tx_len, uint8_t *rx_buffer,
    uint16_t rx_capacity, uint16_t *rx_len_out, uint32_t timeout_ms)
{
    bsp_rs485_status_t status;
    bsp_rs485_state_t  bus_state;
    uint32_t start_tick;
    uint32_t safety_deadline_tick;

    *rx_len_out = 0u;

    /* 发起非阻塞 TX — init 须成功, 帧间隔不足时自旋等待 */
    do {
        status = bsp_rs485_start_tx(bus, tx_buffer, tx_len, rx_buffer, rx_capacity, timeout_ms);
    } while (status == BSP_RS485_STATUS_ERR_BUSY);
    if (status != BSP_RS485_STATUS_OK) {
        return status;
    }

    /* 轮询等待完成 (仅 init 阶段, 有安全超时兜底) */
    start_tick  = HAL_GetTick();
    safety_deadline_tick = start_tick + timeout_ms + 1000u;  /* 额外 1s 兜底 */
    do {
        bus_state = bsp_rs485_poll(bus);
        if (bus_state == BSP_RS485_STATE_DONE) {
            *rx_len_out = bus->rx_len;
            bsp_rs485_ack_done(bus);
            return (*rx_len_out > 0u) ? BSP_RS485_STATUS_OK : BSP_RS485_STATUS_ERR_RETRY;
        }
        if (bus_state == BSP_RS485_STATE_TIMEOUT) {
            bsp_rs485_ack_done(bus);
            return BSP_RS485_STATUS_ERR_TIMEOUT;
        }
        if (bus_state == BSP_RS485_STATE_ERROR) {
            bsp_rs485_ack_done(bus);
            return BSP_RS485_STATUS_ERR_HAL;
        }
    } while (HAL_GetTick() < safety_deadline_tick);

    bsp_rs485_ack_done(bus);
    return BSP_RS485_STATUS_ERR_TIMEOUT;
}



