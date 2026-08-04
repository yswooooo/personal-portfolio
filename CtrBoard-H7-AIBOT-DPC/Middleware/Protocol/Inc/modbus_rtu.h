/**
  ******************************************************************************
  * @file    modbus_rtu.h
  * @brief   Modbus RTU 协议层接口 (中间件)
  *
  * @details 纯 Modbus RTU 协议实现，不关心寄存器语义。
  *          支持 0x03 读保持寄存器、0x06 写单个寄存器。
  *          CRC16 多项式 0xA001，初值 0xFFFF，LSB 先发。
  *
  *          协议层通过 bsp_rs485_transmit_receive() 与物理层通信，
  *          不直接依赖任何 STM32 HAL API。
  ******************************************************************************
  */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bsp_rs485.h"

/* Exported defines ----------------------------------------------------------*/

/** @name Modbus 功能码 */
/** @{ */
#define MODBUS_RTU_FC_READ_HOLDING       0x03u  /**< 读保持寄存器                    */
#define MODBUS_RTU_FC_WRITE_SINGLE       0x06u  /**< 写单个寄存器                    */
/** @} */

/** @name Modbus 异常码掩码 */
#define MODBUS_RTU_EXCEPTION_MASK        0x80u  /**< 功能码 bit7 为 1 表示异常应答    */

/** @name Modbus 帧长度常量 */
#define MODBUS_RTU_READ_REQ_LEN          8u     /**< 0x03 请求帧长度                  */
#define MODBUS_RTU_READ_RSP_LEN          7u     /**< 0x03 正常应答帧长度               */
#define MODBUS_RTU_READ_BYTE_COUNT       2u     /**< 0x03 应答字节数 (读1个寄存器)      */
#define MODBUS_RTU_WRITE_REQ_LEN         8u     /**< 0x06 请求帧长度                  */
#define MODBUS_RTU_WRITE_RSP_LEN         8u     /**< 0x06 正常应答帧长度 (原样回显)     */
#define MODBUS_RTU_EXCEPTION_RSP_LEN     5u     /**< 异常应答帧长度                    */
#define MODBUS_RTU_RX_BUF_MAX            32u    /**< 最大接收缓冲区 (容纳最长可能帧)     */

/** @name Modbus 从机地址范围 */
#define MODBUS_RTU_SLAVE_ID_MIN          1u     /**< 最小从机站号                     */
#define MODBUS_RTU_SLAVE_ID_MAX          247u   /**< 最大从机站号                     */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Modbus 操作状态码
  *
  * @note   统一错误码，BSP 层和 LD2RS 驱动层均使用此枚举。
  *         无需再维护多层错误码翻译。
  */
typedef enum {
    MODBUS_RTU_OK = 0,               /**< 操作成功                        */
    MODBUS_RTU_ERR_PARAM = 1,        /**< 参数非法 (空指针/站号越界等)      */
    MODBUS_RTU_ERR_TIMEOUT = 2,      /**< 总线超时无应答                   */
    MODBUS_RTU_ERR_CRC = 3,          /**< CRC 校验失败                     */
    MODBUS_RTU_ERR_SLAVE_ID = 4,     /**< 应答站号与请求不匹配              */
    MODBUS_RTU_ERR_FUNCTION = 5,     /**< 应答功能码与请求不匹配            */
    MODBUS_RTU_ERR_EXCEPTION = 6,    /**< 从机返回 Modbus 异常码            */
    MODBUS_RTU_ERR_LENGTH = 7,       /**< 应答帧长度异常                   */
    MODBUS_RTU_ERR_BUSY = 8          /**< 总线忙                          */
} modbus_rtu_status_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  计算 Modbus CRC16
  *
  * @details 多项式 0xA001，初值 0xFFFF，按字节右移，LSB 先发。
  *          标准 Modbus RTU CRC 算法。
  *
  * @param  data_buffer  数据指针
  * @param  data_len 数据长度 (字节)
  * @retval 16 位 CRC 值 (主机字节序，放入帧时需 LSB 在前)
  */
uint16_t modbus_rtu_crc16(const uint8_t *data_buffer, uint16_t data_len);

/**
  * @brief  读单个保持寄存器 (Modbus 功能码 0x03)
  *
  * @details 组装 0x03 请求帧 → 发送 → 接收应答 → 校验 → 提取寄存器值。
  *          一次完整的总线事务，在主线程中同步阻塞执行。
  *
  * @param  bus         RS485 总线句柄
  * @param  slave_id    从机站号 (1~247)
  * @param  reg_addr   寄存器地址 (Modbus 485 地址)
  * @param  value_out       读取结果 (输出, 调用者提供存储空间)
  * @param  timeout_ms 超时时间 (ms)
  * @retval MODBUS_RTU_OK            成功, value_out 有效
  * @retval MODBUS_RTU_ERR_PARAM     参数非法
  * @retval MODBUS_RTU_ERR_TIMEOUT   超时无应答
  * @retval MODBUS_RTU_ERR_CRC       CRC 校验失败
  * @retval MODBUS_RTU_ERR_SLAVE_ID  应答站号不匹配
  * @retval MODBUS_RTU_ERR_FUNCTION  应答功能码不匹配
  * @retval MODBUS_RTU_ERR_EXCEPTION 从机返回异常码
  * @retval MODBUS_RTU_ERR_LENGTH    应答长度异常
  */
modbus_rtu_status_t modbus_rtu_read_holding_reg(bsp_rs485_handle_t *bus,
                                          uint8_t slave_id,
                                          uint16_t reg_addr,
                                          uint16_t *value_out,
                                          uint32_t timeout_ms);

/**
  * @brief  写单个寄存器 (Modbus 功能码 0x06)
  *
  * @details 组装 0x06 请求帧 → 发送 → 接收应答 → CRC 校验 → 逐字节比对回显。
  *          一次完整的总线事务，在主线程中同步阻塞执行。
  *
  * @param  bus         RS485 总线句柄
  * @param  slave_id    从机站号 (1~247)
  * @param  reg_addr   寄存器地址 (Modbus 485 地址)
  * @param  value_u16     写入值
  * @param  timeout_ms 超时时间 (ms)
  * @retval MODBUS_RTU_OK            成功, 从机已原样回显
  * @retval MODBUS_RTU_ERR_PARAM     参数非法
  * @retval MODBUS_RTU_ERR_TIMEOUT   超时无应答
  * @retval MODBUS_RTU_ERR_CRC       CRC 校验失败
  * @retval MODBUS_RTU_ERR_SLAVE_ID  应答站号不匹配
  * @retval MODBUS_RTU_ERR_FUNCTION  应答帧与请求帧不一致
  * @retval MODBUS_RTU_ERR_EXCEPTION 从机返回异常码
  * @retval MODBUS_RTU_ERR_LENGTH    应答长度异常
  */
modbus_rtu_status_t modbus_rtu_write_single_reg(bsp_rs485_handle_t *bus,
                                          uint8_t slave_id,
                                          uint16_t reg_addr,
                                          uint16_t value_u16,
                                          uint32_t timeout_ms);

/**
  * @brief  将 Modbus 状态码转为可打印的错误描述字符串
  *
  * @param  status Modbus 状态码
  * @retval 字符串字面量 (如 "OK" / "TIMEOUT" / "CRC" / "EXCEPTION" 等)
  */
const char *modbus_rtu_status_str(modbus_rtu_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */


