/**
  ******************************************************************************
  * @file    modbus_rtu.c
  * @brief   Modbus RTU 协议层实现
  *
  * @details 实现 CRC16 计算、0x03 读保持寄存器、0x06 写单个寄存器。
  *          核心复用点：modbus_rtu_validate_response() 统一做 CRC + 站号 + 功能码
  *          + 异常码检测，被 Read 和 Write 函数共用，消除重复代码。
  ******************************************************************************
  */

#include "modbus_rtu.h"


/* Private defines -----------------------------------------------------------*/

/** @brief 读寄存器请求中的寄存器数量 (固定为 1) */
#define MODBUS_RTU_READ_QUANTITY_ONE    0x0001u

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  将 uint16_t 以大端序写入缓冲区 (Modbus 网络字节序)
  * @param  dst_buffer   目标缓冲区
  * @param  value_u16 16 位值
  */
static void modbus_rtu_put_u16(uint8_t *dst_buffer, uint16_t value_u16)
{
    dst_buffer[0] = (uint8_t)(value_u16 >> 8);
    dst_buffer[1] = (uint8_t)(value_u16 & 0xFFu);
}

/**
  * @brief  从缓冲区以大端序读取 uint16_t (Modbus 网络字节序)
  * @param  src_buffer 源缓冲区
  * @retval 16 位值
  */
static uint16_t modbus_rtu_get_u16(const uint8_t *src_buffer)
{
    return (uint16_t)(((uint16_t)src_buffer[0] << 8) | (uint16_t)src_buffer[1]);
}

/**
  * @brief  在帧末尾追加 CRC16 (LSB 在前)
  * @param  frame_buffer      帧缓冲区
  * @param  payload_len CRC 计算的数据长度 (不含 CRC 自身)
  */
static void modbus_rtu_append_crc(uint8_t *frame_buffer, uint16_t payload_len)
{
    uint16_t crc_value;

    crc_value = modbus_rtu_crc16(frame_buffer, payload_len);
    frame_buffer[payload_len]     = (uint8_t)(crc_value & 0xFFu);      /* CRC Low  */
    frame_buffer[payload_len + 1u] = (uint8_t)(crc_value >> 8);        /* CRC High */
}

/**
  * @brief  校验应答帧的 CRC
  * @param  frame_buffer    应答帧缓冲区
  * @param  frame_len 帧总长度 (含 CRC)
  * @retval MODBUS_RTU_OK CRC 正确 / MODBUS_RTU_ERR_CRC 校验失败
  */
static modbus_rtu_status_t modbus_rtu_check_crc(const uint8_t *frame_buffer, uint16_t frame_len)
{
    uint16_t actual_crc;
    uint16_t expected_crc;

    if ((frame_buffer == NULL) || (frame_len < 4u)) {
        return MODBUS_RTU_ERR_LENGTH;
    }

    actual_crc   = modbus_rtu_crc16(frame_buffer, (uint16_t)(frame_len - 2u));
    expected_crc = (uint16_t)(((uint16_t)frame_buffer[frame_len - 1u] << 8)
                             | (uint16_t)frame_buffer[frame_len - 2u]);

    if (actual_crc != expected_crc) {
        return MODBUS_RTU_ERR_CRC;
    }

    return MODBUS_RTU_OK;
}

/**
  * @brief  将 BSP 层状态码映射为 Modbus 状态码
  * @param  bsp_status BSP RS485 状态码
  * @retval modbus_rtu_status_t
  */
static modbus_rtu_status_t modbus_rtu_map_bsp_status(bsp_rs485_status_t bsp_status)
{
    switch (bsp_status) {
    case BSP_RS485_OK:
        return MODBUS_RTU_OK;
    case BSP_RS485_ERR_TIMEOUT:
        return MODBUS_RTU_ERR_TIMEOUT;
    case BSP_RS485_ERR_PARAM:
        return MODBUS_RTU_ERR_PARAM;
    case BSP_RS485_STATUS_ERR_RETRY:
        return MODBUS_RTU_ERR_LENGTH;
    case BSP_RS485_ERR_BUSY:
        return MODBUS_RTU_ERR_BUSY;
    default:
        return MODBUS_RTU_ERR_TIMEOUT;
    }
}

/**
  * @brief  统一校验 Modbus 应答帧 (CRC + 站号 + 功能码 + 异常码)
  *
  * @details 此函数是 Read 和 Write 的共用校验核心，消除原代码中 ~40 行重复。
  *          校验顺序：帧长 → CRC → 异常码检测 → 站号 → 功能码。
  *
  * @param  rx_buffer             应答帧缓冲区
  * @param  rx_len        应答帧长度
  * @param  slave_id       期望的从机站号
  * @param  expected_func  期望的功能码 (0x03 或 0x06)
  * @param  exception_code_out  异常码输出 (仅当返回 MODBUS_RTU_ERR_EXCEPTION 时有效, 可为 NULL)
  * @retval MODBUS_RTU_OK 校验通过 / 其他错误码
  */
static modbus_rtu_status_t modbus_rtu_validate_response(const uint8_t *rx_buffer,
                                              uint16_t rx_len,
                                              uint8_t slave_id,
                                              uint8_t expected_func,
                                              uint8_t *exception_code_out)
{
    modbus_rtu_status_t status;

    /* 1. 最小长度检查 (至少 4 字节: ID + Func + Error/Data(1) + CRC(2)) */
    if ((rx_buffer == NULL) || (rx_len < 4u)) {
        return MODBUS_RTU_ERR_LENGTH;
    }

    /* 2. CRC 校验 */
    status = modbus_rtu_check_crc(rx_buffer, rx_len);
    if (status != MODBUS_RTU_OK) {
        return status;
    }

    /* 3. 检测是否为异常应答 (功能码高位为 1) */
    if ((rx_buffer[1] & MODBUS_RTU_EXCEPTION_MASK) != 0u) {
        if ((rx_len != MODBUS_RTU_EXCEPTION_RSP_LEN) ||
            (rx_buffer[1] != (uint8_t)(expected_func | MODBUS_RTU_EXCEPTION_MASK))) {
            return MODBUS_RTU_ERR_FUNCTION;
        }

        if (rx_buffer[0] != slave_id) {
            return MODBUS_RTU_ERR_SLAVE_ID;
        }

        /* 记录异常码 */
        if (exception_code_out != NULL) {
            *exception_code_out = rx_buffer[2];
        }

        return MODBUS_RTU_ERR_EXCEPTION;
    }

    /* 4. 站号校验 */
    if (rx_buffer[0] != slave_id) {
        return MODBUS_RTU_ERR_SLAVE_ID;
    }

    /* 5. 功能码校验 */
    if (rx_buffer[1] != expected_func) {
        return MODBUS_RTU_ERR_FUNCTION;
    }

    return MODBUS_RTU_OK;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  计算 Modbus CRC16
  * @param  data_buffer  数据指针
  * @param  data_len 数据长度
  * @retval 16 位 CRC 值
  */
uint16_t modbus_rtu_crc16(const uint8_t *data_buffer, uint16_t data_len)
{
    uint16_t crc_value = 0xFFFFu;
    uint16_t byte_idx;
    uint8_t  bit_idx;

    if (data_buffer == NULL) {
        return 0u;
    }

    for (byte_idx = 0u; byte_idx < data_len; byte_idx++) {
        crc_value ^= (uint16_t)data_buffer[byte_idx];

        for (bit_idx = 0u; bit_idx < 8u; bit_idx++) {
            if ((crc_value & 0x0001u) != 0u) {
                crc_value = (uint16_t)((crc_value >> 1) ^ 0xA001u);
            } else {
                crc_value >>= 1;
            }
        }
    }

    return crc_value;
}

/**
  * @brief  读单个保持寄存器 (Modbus 功能码 0x03)
  * @param  bus         RS485 总线句柄
  * @param  slave_id    从机站号
  * @param  reg_addr   寄存器地址
  * @param  value_out       读取结果 (输出)
  * @param  timeout_ms 超时时间
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t modbus_rtu_read_holding_reg(bsp_rs485_handle_t *bus,
                                          uint8_t slave_id,
                                          uint16_t reg_addr,
                                          uint16_t *value_out,
                                          uint32_t timeout_ms)
{
    uint8_t  tx_buffer[MODBUS_RTU_READ_REQ_LEN];
    uint8_t  rx_buffer[MODBUS_RTU_RX_BUF_MAX];
    uint16_t rx_len = 0u;
    bsp_rs485_status_t bsp_status;
    modbus_rtu_status_t    status;
    uint8_t          exception_code = 0u;

    /* 参数校验 */
    if ((bus == NULL) || (value_out == NULL)) {
        return MODBUS_RTU_ERR_PARAM;
    }
    if ((slave_id < MODBUS_RTU_SLAVE_ID_MIN) || (slave_id > MODBUS_RTU_SLAVE_ID_MAX)) {
        return MODBUS_RTU_ERR_PARAM;
    }

    /* 组装 0x03 请求帧: [ID][0x03][AddrH][AddrL][QtyH=0][QtyL=1][CRCL][CRCH] */
    tx_buffer[0] = slave_id;
    tx_buffer[1] = MODBUS_RTU_FC_READ_HOLDING;
    modbus_rtu_put_u16(&tx_buffer[2], reg_addr);
    modbus_rtu_put_u16(&tx_buffer[4], MODBUS_RTU_READ_QUANTITY_ONE);
    modbus_rtu_append_crc(tx_buffer, 6u);

    /* 总线事务 */
    bsp_status = bsp_rs485_transmit_receive(bus,
                                           tx_buffer, sizeof(tx_buffer),
                                           rx_buffer, sizeof(rx_buffer),
                                           &rx_len, timeout_ms);
    status = modbus_rtu_map_bsp_status(bsp_status);
    if (status != MODBUS_RTU_OK) {
        return status;
    }

    /* 校验应答帧 */
    status = modbus_rtu_validate_response(rx_buffer, rx_len, slave_id,
                                           MODBUS_RTU_FC_READ_HOLDING, &exception_code);
    if (status != MODBUS_RTU_OK) {
        return status;
    }

    /* 校验应答长度和字节计数 */
    if (rx_len != MODBUS_RTU_READ_RSP_LEN) {
        return MODBUS_RTU_ERR_LENGTH;
    }
    if (rx_buffer[2] != MODBUS_RTU_READ_BYTE_COUNT) {
        return MODBUS_RTU_ERR_LENGTH;
    }

    /* 提取寄存器值 */
    *value_out = modbus_rtu_get_u16(&rx_buffer[3]);

    return MODBUS_RTU_OK;
}

/**
  * @brief  写单个寄存器 (Modbus 功能码 0x06)
  * @param  bus         RS485 总线句柄
  * @param  slave_id    从机站号
  * @param  reg_addr   寄存器地址
  * @param  value_u16     写入值
  * @param  timeout_ms 超时时间
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t modbus_rtu_write_single_reg(bsp_rs485_handle_t *bus,
                                          uint8_t slave_id,
                                          uint16_t reg_addr,
                                          uint16_t value_u16,
                                          uint32_t timeout_ms)
{
    uint8_t  tx_buffer[MODBUS_RTU_WRITE_REQ_LEN];
    uint8_t  rx_buffer[MODBUS_RTU_RX_BUF_MAX];
    uint16_t rx_len = 0u;
    uint16_t byte_idx;
    bsp_rs485_status_t bsp_status;
    modbus_rtu_status_t    status;
    uint8_t          exception_code = 0u;

    /* 参数校验 */
    if (bus == NULL) {
        return MODBUS_RTU_ERR_PARAM;
    }
    if ((slave_id < MODBUS_RTU_SLAVE_ID_MIN) || (slave_id > MODBUS_RTU_SLAVE_ID_MAX)) {
        return MODBUS_RTU_ERR_PARAM;
    }

    /* 组装 0x06 请求帧: [ID][0x06][AddrH][AddrL][ValH][ValL][CRCL][CRCH] */
    tx_buffer[0] = slave_id;
    tx_buffer[1] = MODBUS_RTU_FC_WRITE_SINGLE;
    modbus_rtu_put_u16(&tx_buffer[2], reg_addr);
    modbus_rtu_put_u16(&tx_buffer[4], value_u16);
    modbus_rtu_append_crc(tx_buffer, 6u);

    /* 总线事务 */
    bsp_status = bsp_rs485_transmit_receive(bus,
                                           tx_buffer, sizeof(tx_buffer),
                                           rx_buffer, sizeof(rx_buffer),
                                           &rx_len, timeout_ms);
    status = modbus_rtu_map_bsp_status(bsp_status);
    if (status != MODBUS_RTU_OK) {
        return status;
    }

    /* 校验应答帧 */
    status = modbus_rtu_validate_response(rx_buffer, rx_len, slave_id,
                                           MODBUS_RTU_FC_WRITE_SINGLE, &exception_code);
    if (status != MODBUS_RTU_OK) {
        return status;
    }

    /* 校验应答长度 */
    if (rx_len != MODBUS_RTU_WRITE_RSP_LEN) {
        return MODBUS_RTU_ERR_LENGTH;
    }

    /* 逐字节比对：应答必须与请求完全一致 */
    for (byte_idx = 0u; byte_idx < sizeof(tx_buffer); byte_idx++) {
        if (tx_buffer[byte_idx] != rx_buffer[byte_idx]) {
            return MODBUS_RTU_ERR_FUNCTION;
        }
    }

    return MODBUS_RTU_OK;
}

/**
  * @brief  将 Modbus 状态码转为可打印字符串
  * @param  status Modbus 状态码
  * @retval 字符串字面量
  */
const char *modbus_rtu_status_str(modbus_rtu_status_t status)
{
    switch (status) {
    case MODBUS_RTU_OK:
        return "OK";
    case MODBUS_RTU_ERR_PARAM:
        return "PARAM";
    case MODBUS_RTU_ERR_TIMEOUT:
        return "TIMEOUT";
    case MODBUS_RTU_ERR_CRC:
        return "CRC";
    case MODBUS_RTU_ERR_SLAVE_ID:
        return "SLAVE_ID";
    case MODBUS_RTU_ERR_FUNCTION:
        return "FUNCTION";
    case MODBUS_RTU_ERR_EXCEPTION:
        return "EXCEPTION";
    case MODBUS_RTU_ERR_LENGTH:
        return "LENGTH";
    case MODBUS_RTU_ERR_BUSY:
        return "BUSY";
    default:
        return "UNKNOWN";
    }
}


