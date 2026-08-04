/**
  ******************************************************************************
  * @file    ds_rs_um_motor.c
  * @brief   DS / RS / UM 系列低压伺服驱动器 Modbus RTU 驱动实现
  *
  * @details 封装 DS/RS/UM 系列驱动器的寄存器读写。
  *          所有通信通过 modbus_rtu.h 完成，本层负责：
  *          - 参数校验（站号、转速范围、使能值）
  *          - 调用 Modbus 协议层发送/接收
  *          - 记录诊断信息（功能码、异常码、最近接收帧）
  *
  * @note    本层函数内部调用 bsp_rs485_transmit_receive()，属于阻塞调用，
  *          仅适合初始化阶段使用。运行期高频控制请使用 app_wheel_task.c
  *          中的非阻塞 FSM，避免阻塞主循环。
  ******************************************************************************
  */

#include "ds_rs_um_motor.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Exported instances                                                 */
/* ------------------------------------------------------------------ */
ds_rs_um_motor_handle_t g_ds_rs_um_wheel_fl;
ds_rs_um_motor_handle_t g_ds_rs_um_wheel_rl;
ds_rs_um_motor_handle_t g_ds_rs_um_wheel_rr;
ds_rs_um_motor_handle_t g_ds_rs_um_wheel_fr;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  保存接收帧副本用于诊断
  * @param  dev       设备句柄
  * @param  rx_buffer 接收帧缓冲区
  * @param  rx_len    接收帧长度
  */
static void ds_rs_um_motor_save_last_rx(ds_rs_um_motor_handle_t *dev,
                                        const uint8_t *rx_buffer,
                                        uint16_t rx_len)
{
    uint16_t copy_len;
    uint16_t byte_idx;

    if ((dev == NULL) || (rx_buffer == NULL)) {
        return;
    }

    copy_len = (rx_len > sizeof(dev->last_rx_buffer))
                 ? (uint16_t)sizeof(dev->last_rx_buffer)
                 : rx_len;

    for (byte_idx = 0u; byte_idx < copy_len; byte_idx++) {
        dev->last_rx_buffer[byte_idx] = rx_buffer[byte_idx];
    }
    dev->last_rx_len = copy_len;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 DS/RS/UM 电机设备句柄
  */
void ds_rs_um_motor_init(ds_rs_um_motor_handle_t *dev,
                         bsp_rs485_handle_t *bus,
                         uint8_t slave_id,
                         uint32_t timeout_ms)
{
    if (dev == NULL) {
        return;
    }

    dev->bus                 = bus;
    dev->slave_id            = slave_id;
    dev->timeout_ms          = (timeout_ms == 0u) ? DS_RS_UM_MOTOR_DEFAULT_TIMEOUT_MS
                                                  : timeout_ms;
    dev->last_function_code  = 0u;
    dev->last_exception_code = 0u;
    dev->last_rx_len         = 0u;
}

/**
  * @brief  读单个保持寄存器 (Modbus 0x03)
  */
modbus_rtu_status_t ds_rs_um_motor_read_reg(ds_rs_um_motor_handle_t *dev,
                                            uint16_t reg_addr,
                                            uint16_t *value_out)
{
    modbus_rtu_status_t status;

    if ((dev == NULL) || (dev->bus == NULL) || (value_out == NULL)) {
        return MODBUS_RTU_ERR_PARAM;
    }

    dev->last_rx_len = 0u;

    status = modbus_rtu_read_holding_reg(dev->bus,
                                         dev->slave_id,
                                         reg_addr,
                                         value_out,
                                         dev->timeout_ms);

    if (status == MODBUS_RTU_OK) {
        dev->last_function_code  = MODBUS_RTU_FC_READ_HOLDING;
        dev->last_exception_code = 0u;
        ds_rs_um_motor_save_last_rx(dev, dev->bus->rx_buffer, dev->bus->rx_len);
    }

    return status;
}

/**
  * @brief  写单个保持寄存器 (Modbus 0x06)
  */
modbus_rtu_status_t ds_rs_um_motor_write_reg(ds_rs_um_motor_handle_t *dev,
                                             uint16_t reg_addr,
                                             uint16_t reg_value)
{
    modbus_rtu_status_t status;

    if ((dev == NULL) || (dev->bus == NULL)) {
        return MODBUS_RTU_ERR_PARAM;
    }

    dev->last_rx_len = 0u;

    status = modbus_rtu_write_single_reg(dev->bus,
                                         dev->slave_id,
                                         reg_addr,
                                         reg_value,
                                         dev->timeout_ms);

    if (status == MODBUS_RTU_OK) {
        dev->last_function_code  = MODBUS_RTU_FC_WRITE_SINGLE;
        dev->last_exception_code = 0u;
        ds_rs_um_motor_save_last_rx(dev, dev->bus->rx_buffer, dev->bus->rx_len);
    }

    return status;
}

/**
  * @brief  设置控制模式为速度模式 (F0.1.002 = 1)
  */
modbus_rtu_status_t ds_rs_um_motor_set_speed_mode(ds_rs_um_motor_handle_t *dev)
{
    return ds_rs_um_motor_write_reg(dev,
                                    DS_RS_UM_MOTOR_REG_CONTROL_MODE,
                                    DS_RS_UM_MOTOR_MODE_SPEED);
}

/**
  * @brief  电机使能/停止控制 (F0.1.000)
  */
modbus_rtu_status_t ds_rs_um_motor_set_enable(ds_rs_um_motor_handle_t *dev,
                                              uint16_t enable)
{
    if (enable > 1u) {
        return MODBUS_RTU_ERR_PARAM;
    }

    return ds_rs_um_motor_write_reg(dev,
                                    DS_RS_UM_MOTOR_REG_ENABLE,
                                    enable);
}

/**
  * @brief  设置目标转速 (F0.3.018)，含范围校验
  */
modbus_rtu_status_t ds_rs_um_motor_set_target_speed(ds_rs_um_motor_handle_t *dev,
                                                    int16_t target_speed_rpm)
{
    /* 安全校验：转速必须在允许范围内 */
    if ((target_speed_rpm < DS_RS_UM_MOTOR_SPEED_MIN_RPM) ||
        (target_speed_rpm > DS_RS_UM_MOTOR_SPEED_MAX_RPM)) {
        return MODBUS_RTU_ERR_PARAM;
    }

    return ds_rs_um_motor_write_reg(dev,
                                    DS_RS_UM_MOTOR_REG_TARGET_SPEED,
                                    (uint16_t)target_speed_rpm);
}

/**
  * @brief  读取实际转速 (D0.000)
  */
modbus_rtu_status_t ds_rs_um_motor_get_actual_speed(ds_rs_um_motor_handle_t *dev,
                                                    int16_t *speed_out)
{
    uint16_t value_u16;
    modbus_rtu_status_t status;

    if (speed_out == NULL) {
        return MODBUS_RTU_ERR_PARAM;
    }

    status = ds_rs_um_motor_read_reg(dev, DS_RS_UM_MOTOR_REG_ACTUAL_SPEED, &value_u16);
    if (status == MODBUS_RTU_OK) {
        *speed_out = (int16_t)value_u16;
    }

    return status;
}

/**
  * @brief  读取状态字 (D0.004)
  */
modbus_rtu_status_t ds_rs_um_motor_get_status_word(ds_rs_um_motor_handle_t *dev,
                                                   uint16_t *status_out)
{
    if (status_out == NULL) {
        return MODBUS_RTU_ERR_PARAM;
    }

    return ds_rs_um_motor_read_reg(dev, DS_RS_UM_MOTOR_REG_STATUS_WORD, status_out);
}
