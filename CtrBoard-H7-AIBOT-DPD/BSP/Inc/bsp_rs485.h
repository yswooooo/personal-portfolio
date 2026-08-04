/**
  ******************************************************************************
  * @file    bsp_rs485.h
  * @brief   RS485 半双工总线 — 非阻塞状态机 (BSP 层)
  *
  * @details 基于 STM32H7 HAL UART + 硬件 DE + TC/IDLE 中断。
  *          TX:  HAL_UART_Transmit_IT 发起, TC 中断切 RX。
  *          RX:  IDLE 中断检测帧尾, Poll() 轮询状态, 零 CPU 死等。
  ******************************************************************************
  */

#ifndef BSP_RS485_H
#define BSP_RS485_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* Exported defines ----------------------------------------------------------*/

#define BSP_RS485_MAX_RETRY  3u       /**< 最大重试次数                  */

/* Exported types ------------------------------------------------------------*/

/** @brief BSP RS485 状态码 */
typedef enum {
    BSP_RS485_STATUS_OK      = 0,           /**< 操作成功                      */
    BSP_RS485_STATUS_ERR_PARAM = 1,         /**< 参数非法 (空指针)              */
    BSP_RS485_STATUS_ERR_BUSY = 2,          /**< 总线忙, 事务进行中              */
    BSP_RS485_STATUS_ERR_TIMEOUT = 3,       /**< 超时无应答                     */
    BSP_RS485_STATUS_ERR_HAL = 4,           /**< HAL 返回错误                   */
    BSP_RS485_STATUS_ERR_RETRY = 5          /**< 重试耗尽                       */
} bsp_rs485_status_t;

/** @brief RS485 状态码兼容宏 */
#define BSP_RS485_OK             BSP_RS485_STATUS_OK
#define BSP_RS485_ERR_PARAM      BSP_RS485_STATUS_ERR_PARAM
#define BSP_RS485_ERR_BUSY       BSP_RS485_STATUS_ERR_BUSY
#define BSP_RS485_ERR_TIMEOUT    BSP_RS485_STATUS_ERR_TIMEOUT
#define BSP_RS485_ERR_HAL        BSP_RS485_STATUS_ERR_HAL

/** @brief RS485 总线状态机 */
typedef enum {
    BSP_RS485_STATE_IDLE    = 0,               /**< 空闲, 可发起新事务              */
    BSP_RS485_STATE_TX_BUSY = 1,               /**< TX 发送中, 等 TC 中断           */
    BSP_RS485_STATE_RX_WAIT = 2,               /**< 已切 RX, 等 IDLE 中断           */
    BSP_RS485_STATE_DONE    = 3,               /**< 事务完成, 数据就绪              */
    BSP_RS485_STATE_TIMEOUT = 4,               /**< 超时, 需重试或报错              */
    BSP_RS485_STATE_ERROR   = 5                /**< HAL/状态异常, 需上层释放总线    */
} bsp_rs485_state_t;

/**
  * @brief  RS485 总线句柄 — 非阻塞
  */
typedef struct {
    UART_HandleTypeDef *uart_handle;       /**< HAL UART 句柄 (USART3)        */

    /* 状态机 */
    bsp_rs485_state_t   state;       /**< 当前状态                      */

    /* TX 缓冲区 (调用者提供, 事务完成前不可释放) */
    const uint8_t      *tx_buffer;
    uint16_t            tx_len;

    /* RX 缓冲区 (调用者提供) */
    uint8_t            *rx_buffer;
    uint16_t            rx_capacity;
    uint16_t            rx_len;     /**< IDLE 中断填入实际长度          */

    /* 超时与重试 */
    uint32_t            timeout_ms; /**< 超时时间 (ms)                 */
    uint32_t            start_tick_ms; /**< TX 发起时刻 tick              */
    uint8_t             retry_count;   /**< 当前重试次数                  */

    /* 帧间隔 */
    uint32_t            last_done_tick_ms; /**< 上一事务完成 tick, 帧间隔用  */

    /* 错误标志 (ISR 写入, Poll 读取) */
    volatile uint8_t    rx_error_code;    /**< UART 接收异常                 */
    volatile uint8_t    state_error_code; /**< 状态机/ISR 顺序异常           */
} bsp_rs485_handle_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 RS485 总线句柄
  */
bsp_rs485_status_t bsp_rs485_init(bsp_rs485_handle_t *bus,
                                  UART_HandleTypeDef *uart_handle);

/**
  * @brief  发起非阻塞 RS485 事务 (立即返回)
  * @note   tx_buffer/rx_buffer 缓冲区在事务完成前不可释放
  * @param  bus        总线句柄
  * @param  tx_buffer         发送帧 (8 字节 Modbus 请求)
  * @param  tx_len    发送长度
  * @param  rx_buffer         接收缓冲区
  * @param  rx_capacity    接收容量
  * @param  timeout_ms 超时 (ms)
  * @retval OK / BUSY / PARAM
  */
bsp_rs485_status_t bsp_rs485_start_tx(bsp_rs485_handle_t *bus,
    const uint8_t *tx_buffer, uint16_t tx_len,
    uint8_t *rx_buffer, uint16_t rx_capacity, uint32_t timeout_ms);

/**
  * @brief  主循环轮询 — 检查事务是否完成或超时
  * @retval 当前状态 (BSP_RS485_STATE_DONE → 可取数据; BSP_RS485_STATE_TIMEOUT → 自动重试)
  */
bsp_rs485_state_t bsp_rs485_poll(bsp_rs485_handle_t *bus);

/**
  * @brief  TC 中断回调 (HAL_UART_TxCpltCallback 中调用)
  */
void bsp_rs485_tx_cplt_callback(bsp_rs485_handle_t *bus);

/**
  * @brief  IDLE 中断回调 (HAL_UARTEx_RxEventCallback 中调用)
  */
void bsp_rs485_rx_event_callback(bsp_rs485_handle_t *bus, uint16_t rx_size);

/**
  * @brief  UART 错误回调 (HAL_UART_ErrorCallback 中调用)
  */
void bsp_rs485_error_callback(bsp_rs485_handle_t *bus);

/**
 * @brief  RS485 阻塞事务 (初始化阶段专用)
 * @note   内部用非阻塞 API + while 轮询, 仅用于 app_chassis_ld2rs_motor_ctrl_init
 *         运行时禁止调用, 改用 bsp_rs485_start_tx + bsp_rs485_poll。
 */
bsp_rs485_status_t bsp_rs485_transmit_receive(bsp_rs485_handle_t *bus,
    const uint8_t *tx_buffer, uint16_t tx_len, uint8_t *rx_buffer,
    uint16_t rx_capacity, uint16_t *rx_len_out, uint32_t timeout_ms);

/**
  * @brief  确认事务完成, 重置总线到 IDLE
  * @note   调用者在 DONE/TIMEOUT 后调用此函数释放总线。
  *         替代直接写 bus_state = BSP_RS485_STATE_IDLE, 封装内部清理逻辑。
  * @param  bus 总线句柄
  */
void bsp_rs485_ack_done(bsp_rs485_handle_t *bus);

extern bsp_rs485_handle_t g_rs485_bus;

#ifdef __cplusplus
}
#endif

#endif /* BSP_RS485_H */



