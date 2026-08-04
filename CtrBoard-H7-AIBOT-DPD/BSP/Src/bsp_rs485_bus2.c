/**
  ******************************************************************************
  * @file    bsp_rs485_bus2.c
  * @brief   UART2 RS485 总线封装实现
  *
  * @details 复用 bsp_rs485.c 的通用非阻塞状态机，绑定到 huart2。
  *          主循环或电机任务通过 bsp_rs485_bus2_get_handle() 获取句柄后，
  *          即可调用 bsp_rs485_start_tx() / bsp_rs485_poll() / bsp_rs485_ack_done()
  *          等 API 进行非阻塞收发。
  *
  *          USART2 硬件配置（由 CubeMX 生成）：
  *          - PD5 = USART2_TX
  *          - PD6 = USART2_RX
  *          - PD4 = USART2_DE（硬件方向控制，高电平有效）
  *          - 115200 bps，8 数据位，1 停止位，无校验
  ******************************************************************************
  */

#include "bsp_rs485_bus2.h"
#include "usart.h"

/* ------------------------------------------------------------------ */
/* Exported instance                                                  */
/* ------------------------------------------------------------------ */
bsp_rs485_handle_t g_rs485_bus2;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 UART2 RS485 总线句柄
  * @note   绑定到 huart2，并复位状态机为 IDLE。
  */
void bsp_rs485_bus2_init(void)
{
    (void)bsp_rs485_init(&g_rs485_bus2, &huart2);
}

/**
  * @brief  获取 UART2 RS485 总线句柄指针
  * @retval g_rs485_bus2 指针
  */
bsp_rs485_handle_t *bsp_rs485_bus2_get_handle(void)
{
    return &g_rs485_bus2;
}
