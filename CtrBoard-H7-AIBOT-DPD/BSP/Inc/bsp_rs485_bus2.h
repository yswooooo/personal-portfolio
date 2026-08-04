/**
  ******************************************************************************
  * @file    bsp_rs485_bus2.h
  * @brief   UART2 RS485 总线封装 (BSP 层)
  *
  * @details 基于 bsp_rs485.c 通用状态机，绑定到 USART2 (PD4/PD5/PD6 硬件 DE)。
  *          与 USART3 上的 g_rs485_bus 完全独立，专门用于四轮轮毂电机 Modbus RTU。
  *
  *          使用流程：
  *          1. CubeMX 中配置 USART2 为 Asynchronous + RS485 Hardware Flow Control。
  *          2. main.c 中调用 MX_USART2_UART_Init()。
  *          3. 调用 bsp_rs485_bus2_init() 初始化句柄。
  *          4. 通过 bsp_rs485_bus2_get_handle() 获取句柄后，即可调用非阻塞 API。
  *
  * @note    必须在 Core/Src/usart.c 的 HAL_UARTEx_RxEventCallback /
  *          HAL_UART_ErrorCallback / HAL_UART_TxCpltCallback 中把 huart2
  *          事件分发到 g_rs485_bus2，否则非阻塞状态机无法工作。
  ******************************************************************************
  */

#ifndef BSP_RS485_BUS2_H
#define BSP_RS485_BUS2_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_rs485.h"

/* Exported defines ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 UART2 RS485 总线句柄
  * @note   调用前需先执行 MX_USART2_UART_Init()，使 huart2 就绪。
  */
void bsp_rs485_bus2_init(void);

/**
  * @brief  获取 UART2 RS485 总线句柄指针
  * @retval g_rs485_bus2 指针
  */
bsp_rs485_handle_t *bsp_rs485_bus2_get_handle(void);

/* Exported variables --------------------------------------------------------*/
extern bsp_rs485_handle_t g_rs485_bus2;  /**< UART2 RS485 总线全局句柄 */

#ifdef __cplusplus
}
#endif

#endif /* BSP_RS485_BUS2_H */
