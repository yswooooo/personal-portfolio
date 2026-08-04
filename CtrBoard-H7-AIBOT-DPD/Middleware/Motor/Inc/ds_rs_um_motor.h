/**
  ******************************************************************************
  * @file    ds_rs_um_motor.h
  * @brief   DS / RS / UM 系列低压伺服驱动器 Modbus RTU 驱动接口
  *
  * @details 依据《DS RS UM系列低压伺服驱动器通讯手册V9.0》的 Fn/Dn 寄存器映射：
  *          - A 轴参数 F0.x.y 映射到 Modbus 保持寄存器：
  *            0x2000 + 0x1000*x + 0x100*y + z
  *          - 监控数据 D0.x   映射到 Modbus 保持寄存器：
  *            0x5000 + 0x100*x + y
  *
  *          本驱动只使用 A 轴（单轴驱动器一拖一），实现：
  *          - 控制模式设置 (F0.1.002)
  *          - 电机使能/停止 (F0.1.000)
  *          - 目标速度写入 (F0.3.018)，单位 rpm，有符号 16 位
  *          - 实际速度回读 (D0.000)
  *          - 状态/错误码回读 (D0.004)
  *
  *          默认通信参数：115200 bps，8 数据位，1 停止位，无校验 (8N1)。
  *
  *          本层为中间件，不依赖 APP/BSP 具体业务，只通过 bsp_rs485_handle_t
  *          与底层总线关联，可被阻塞或非阻塞调用方复用。
  ******************************************************************************
  */

#ifndef DS_RS_UM_MOTOR_H
#define DS_RS_UM_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bsp_rs485.h"
#include "modbus_rtu.h"

/* Exported defines ----------------------------------------------------------*/

/** @name DS/RS/UM A 轴寄存器 Modbus 地址 */
/** @{ */
#define DS_RS_UM_MOTOR_REG_CONTROL_MODE     0x2102u /**< F0.1.002: 控制模式，=1 速度模式     */
#define DS_RS_UM_MOTOR_REG_ENABLE           0x2100u /**< F0.1.000: 电机使能，=1 使能，=0 停止 */
#define DS_RS_UM_MOTOR_REG_TARGET_SPEED     0x2318u /**< F0.3.018: 目标速度指令 (rpm)        */
#define DS_RS_UM_MOTOR_REG_ACTUAL_SPEED     0x5000u /**< D0.000:   实际速度反馈 (rpm)        */
#define DS_RS_UM_MOTOR_REG_STATUS_WORD      0x5004u /**< D0.004:   状态字 / 错误码            */
/** @} */

/** @name 控制模式值 */
/** @{ */
#define DS_RS_UM_MOTOR_MODE_SPEED           1u      /**< 速度模式                            */
/** @} */

/** @name 使能控制 */
/** @{ */
#define DS_RS_UM_MOTOR_ENABLE_OFF           0u      /**< 停止                                */
#define DS_RS_UM_MOTOR_ENABLE_ON            1u      /**< 使能                                */
/** @} */

/** @name 转速范围限制 */
/** @{ */
#define DS_RS_UM_MOTOR_SPEED_MIN_RPM        (-200) /**< 最小转速 (rpm)                      */
#define DS_RS_UM_MOTOR_SPEED_MAX_RPM        200    /**< 最大转速 (rpm)                      */
/** @} */

/** @name 默认参数 */
/** @{ */
#define DS_RS_UM_MOTOR_DEFAULT_TIMEOUT_MS   100u    /**< 默认 Modbus 超时 (ms)               */
/** @} */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  DS/RS/UM 伺服设备句柄
  */
typedef struct {
    bsp_rs485_handle_t *bus;            /**< RS485 总线句柄                      */
    uint8_t   slave_id;                 /**< Modbus 从机站号 (1~247)              */
    uint32_t  timeout_ms;               /**< 通信超时 (ms)                        */

    /* 诊断信息 */
    uint8_t   last_function_code;       /**< 上一次发送的功能码                   */
    uint8_t   last_exception_code;      /**< 上一次收到的异常码 (0=无异常)        */
    uint8_t   last_rx_buffer[32];       /**< 上一次接收帧的原始数据               */
    uint16_t  last_rx_len;              /**< 上一次接收帧的长度                   */
} ds_rs_um_motor_handle_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 DS/RS/UM 电机设备句柄
  * @param  dev         设备句柄指针
  * @param  bus         RS485 总线句柄
  * @param  slave_id    从机站号 (1~247)
  * @param  timeout_ms  通信超时 (ms)，0 则使用默认值 100ms
  */
void ds_rs_um_motor_init(ds_rs_um_motor_handle_t *dev,
                         bsp_rs485_handle_t *bus,
                         uint8_t slave_id,
                         uint32_t timeout_ms);

/**
  * @brief  读单个保持寄存器 (Modbus 0x03)
  * @param  dev        设备句柄
  * @param  reg_addr   寄存器地址
  * @param  value_out  读取结果（输出）
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ds_rs_um_motor_read_reg(ds_rs_um_motor_handle_t *dev,
                                            uint16_t reg_addr,
                                            uint16_t *value_out);

/**
  * @brief  写单个保持寄存器 (Modbus 0x06)
  * @param  dev        设备句柄
  * @param  reg_addr   寄存器地址
  * @param  reg_value  写入值
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ds_rs_um_motor_write_reg(ds_rs_um_motor_handle_t *dev,
                                             uint16_t reg_addr,
                                             uint16_t reg_value);

/**
  * @brief  设置控制模式为速度模式 (F0.1.002 = 1)
  * @param  dev 设备句柄
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ds_rs_um_motor_set_speed_mode(ds_rs_um_motor_handle_t *dev);

/**
  * @brief  电机使能/停止控制 (F0.1.000)
  * @param  dev     设备句柄
  * @param  enable  DS_RS_UM_MOTOR_ENABLE_ON / OFF
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ds_rs_um_motor_set_enable(ds_rs_um_motor_handle_t *dev,
                                              uint16_t enable);

/**
  * @brief  设置目标转速 (F0.3.018)，含范围校验
  * @param  dev              设备句柄
  * @param  target_speed_rpm 目标转速 (rpm)，正转/反转由符号决定
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ds_rs_um_motor_set_target_speed(ds_rs_um_motor_handle_t *dev,
                                                    int16_t target_speed_rpm);

/**
  * @brief  读取实际转速 (D0.000)
  * @param  dev       设备句柄
  * @param  speed_out 实际转速输出 (rpm)
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ds_rs_um_motor_get_actual_speed(ds_rs_um_motor_handle_t *dev,
                                                    int16_t *speed_out);

/**
  * @brief  读取状态字 (D0.004)
  * @param  dev        设备句柄
  * @param  status_out 状态字输出
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ds_rs_um_motor_get_status_word(ds_rs_um_motor_handle_t *dev,
                                                   uint16_t *status_out);

/* Exported variables --------------------------------------------------------*/
extern ds_rs_um_motor_handle_t g_ds_rs_um_wheel_fl;  /**< ID 01 左前轮 */
extern ds_rs_um_motor_handle_t g_ds_rs_um_wheel_rl;  /**< ID 02 左后轮 */
extern ds_rs_um_motor_handle_t g_ds_rs_um_wheel_rr;  /**< ID 03 右后轮 */
extern ds_rs_um_motor_handle_t g_ds_rs_um_wheel_fr;  /**< ID 04 右前轮 */

#ifdef __cplusplus
}
#endif

#endif /* DS_RS_UM_MOTOR_H */
