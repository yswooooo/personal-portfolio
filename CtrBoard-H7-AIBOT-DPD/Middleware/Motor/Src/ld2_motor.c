/**
  ******************************************************************************
  * @file    ld2_motor.c
  * @brief   LD2-RS 伺服驱动器驱动实现
  *
  * @details 封装 LD2-RS 寄存器读写操作。
  *          所有函数均通过 Modbus 协议层 (modbus_rtu.h) 完成通信，
  *          本层负责：参数校验、诊断信息记录。
  ******************************************************************************
  */

#include "ld2_motor.h"
#include <string.h>

ld2_motor_handle_t g_ld2rs_dev_m1;
ld2_motor_handle_t g_ld2rs_dev_m2;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 LD2-RS 设备句柄
  * @param  dev         设备句柄指针
  * @param  bus         RS485 总线句柄
  * @param  slave_id    从机站号
  * @param  timeout_ms 通信超时 (ms)
  */
void ld2_motor_init(ld2_motor_handle_t *dev,
                    bsp_rs485_handle_t *bus,
                    uint8_t slave_id,
                    uint32_t timeout_ms)
{
    if (dev == NULL) {
        return;
    }

    dev->bus           = bus;
    dev->slave_id      = slave_id;
    dev->timeout_ms   = (timeout_ms == 0u) ? LD2_MOTOR_DEFAULT_TIMEOUT_MS
                                                : timeout_ms;
    dev->last_function_code = 0u;
    dev->last_exception_code = 0u;
    dev->last_rx_len   = 0u;
}

/**
  * @brief  读 LD2-RS 寄存器
  * @param  dev    设备句柄
  * @param  reg_addr 寄存器地址
  * @param  value_out  读取结果 (输出)
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ld2_motor_read_reg(ld2_motor_handle_t *dev,
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
    }

    return status;
}

/**
  * @brief  写 LD2-RS 寄存器
  * @param  dev    设备句柄
  * @param  reg_addr 寄存器地址
  * @param  reg_value 写入值
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ld2_motor_write_reg(ld2_motor_handle_t *dev,
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
    }

    return status;
}

/**
  * @brief  设置目标转速 (写 Pr3.04)，含范围校验
  * @param  dev   设备句柄
  * @param  target_speed_rpm 目标转速 (rpm)
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ld2_motor_set_target_speed(ld2_motor_handle_t *dev, int16_t target_speed_rpm)
{
    /* 安全校验：转速必须在允许范围内 */
    if ((target_speed_rpm < LD2_MOTOR_SPEED_MIN_RPM) || (target_speed_rpm > LD2_MOTOR_SPEED_MAX_RPM)) {
        return MODBUS_RTU_ERR_PARAM;
    }

    return ld2_motor_write_reg(dev, LD2_MOTOR_REG_SPEED_TARGET, (uint16_t)target_speed_rpm);
}

/**
  * @brief  设置内部速度源 (写 Pr3.00 = 1)
  * @param  dev 设备句柄
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ld2_motor_set_speed_source_internal(ld2_motor_handle_t *dev)
{
    return ld2_motor_write_reg(dev, LD2_MOTOR_REG_SPEED_SOURCE,
                               LD2_MOTOR_SPEED_SRC_INTERNAL);
}

/**
  * @brief  设置加减速时间 (写 Pr3.12 + Pr3.13)
  * @param  dev       设备句柄
  * @param  accel_time_ms 加速时间
  * @param  decel_time_ms 减速时间
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ld2_motor_set_accel_decel(ld2_motor_handle_t *dev,
                                        uint16_t accel_time_ms,
                                        uint16_t decel_time_ms)
{
    modbus_rtu_status_t status;

    status = ld2_motor_write_reg(dev, LD2_MOTOR_REG_SPEED_ACCEL, accel_time_ms);
    if (status != MODBUS_RTU_OK) {
        return status;
    }

    return ld2_motor_write_reg(dev, LD2_MOTOR_REG_SPEED_DECEL, decel_time_ms);
}

/**
  * @brief  硬写入 RS485 波特率 (Pr5.30), 读回验证
  * @param  dev    设备句柄
  * @param  baud_setting 波特率设定值 (LD2RS_BAUD_*)
  * @retval MODBUS_RTU_OK 写入+验证成功
  */
modbus_rtu_status_t ld2_motor_set_baud_rate(ld2_motor_handle_t *dev, uint16_t baud_setting)
{
    uint16_t reg_value;

    if (dev == NULL) {
        return MODBUS_RTU_ERR_PARAM;
    }

    if (baud_setting > LD2_MOTOR_BAUD_115200) {
        return MODBUS_RTU_ERR_PARAM;
    }

    /* 写 Pr5.30 */
    if (ld2_motor_write_reg(dev, LD2_MOTOR_REG_BAUD_RATE, baud_setting) != MODBUS_RTU_OK) {
        return MODBUS_RTU_ERR_TIMEOUT;
    }

    /* 读回验证 */
    if (ld2_motor_read_reg(dev, LD2_MOTOR_REG_BAUD_RATE, &reg_value) != MODBUS_RTU_OK) {
        return MODBUS_RTU_ERR_TIMEOUT;
    }
    if (reg_value != baud_setting) {
        return MODBUS_RTU_ERR_TIMEOUT;
    }

    return MODBUS_RTU_OK;
}


