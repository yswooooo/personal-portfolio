/**
  ******************************************************************************
  * @file    ld2_motor.h
  * @brief   LD2-RS 伺服驱动器驱动接口 (中间件)
  *
  * @details 封装 LD2-RS 的 Modbus 寄存器地址和常用操作。
  *          所有寄存器读写通过 Modbus 协议层 (modbus_rtu.h) 完成。
  *          包含参数合法性校验（转速范围、站号范围等安全策略）。
  *
  *          LD2-RS 内部有四段速寄存器 (Pr3.04~Pr3.11)，
  *          本驱动仅使用第一段速 Pr3.04 (0x0309)，通过周期性复写实现动态调速。
  ******************************************************************************
  */

#ifndef LD2_MOTOR_H
#define LD2_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bsp_rs485.h"
#include "modbus_rtu.h"

/* Exported defines ----------------------------------------------------------*/

/** @name LD2-RS 寄存器地址 (Modbus 485 地址) */
/** @{ */
#define LD2_MOTOR_REG_CONTROL_MODE      0x0003u  /**< Pr0.01 控制模式: 0=位置 1=速度 2=转矩        */
#define LD2_MOTOR_REG_SPEED_SOURCE      0x0301u  /**< Pr3.00 速度源选择: 1=内部Pr3.04             */
#define LD2_MOTOR_REG_SPEED_TARGET      0x0309u  /**< Pr3.04 内部速度指令 (rpm, 有符号16位)        */
#define LD2_MOTOR_REG_SPEED_ACCEL       0x0319u  /**< Pr3.12 加速时间 (ms/1000rpm)                */
#define LD2_MOTOR_REG_SPEED_DECEL       0x031Bu  /**< Pr3.13 减速时间 (ms/1000rpm)                */
#define LD2_MOTOR_REG_RUN_STATUS        0x0B05u  /**< PrB.05 运行状态: bit0=RDY bit1=RUN           */
#define LD2_MOTOR_REG_UNFILT_SPEED      0x0B06u  /**< PrB.06 未滤波实时速度 (rpm, 只读)            */
#define LD2_MOTOR_REG_ACTUAL_SPEED      0x0B09u  /**< PrB.09 滤波后实际速度 (rpm, 只读)            */
#define LD2_MOTOR_REG_ENCODER_POSITION_H 0x0B1Cu /**< PrB.24 电机位置反馈 (编码器单位, 高16位)      */
#define LD2_MOTOR_REG_ENCODER_POSITION_L 0x0B1Du /**< PrB.24 电机位置反馈 (编码器单位, 低16位)      */
#define LD2_MOTOR_REG_SPEED_KP          0x0103u  /**< Pr1.01 第1速度环增益 Kp              */
#define LD2_MOTOR_REG_SPEED_TI          0x0105u  /**< Pr1.02 第1速度环积分时间常数 Ti       */
#define LD2_MOTOR_REG_BAUD_RATE         0x053Du  /**< Pr5.30 RS485 通信速率                */
/** @} */

/** @name RS485 波特率设定值 (Pr5.30) */
/** @{ */
#define LD2_MOTOR_BAUD_2400              0u
#define LD2_MOTOR_BAUD_4800              1u
#define LD2_MOTOR_BAUD_9600              2u
#define LD2_MOTOR_BAUD_19200             3u
#define LD2_MOTOR_BAUD_38400             4u
#define LD2_MOTOR_BAUD_57600             5u
#define LD2_MOTOR_BAUD_115200            6u
/** @} */

/** @name 运行状态位掩码 (PrB.05) */
/** @{ */
#define LD2_MOTOR_STATUS_RDY_MASK       0x0001u  /**< 伺服就绪                          */
#define LD2_MOTOR_STATUS_RUN_MASK       0x0002u  /**< 电机运转中                        */
/** @} */

/** @name 驱动器报警相关位 */
#define LD2_MOTOR_STATUS_ALARM_MASK     0xFFF0u  /**< 报警位掩码 (bit2-bit15 中任一为1表示报警) */

/** @name 控制模式值 */
/** @{ */
#define LD2_MOTOR_MODE_POSITION         0u       /**< 位置模式                          */
#define LD2_MOTOR_MODE_SPEED            1u       /**< 速度模式                          */
#define LD2_MOTOR_MODE_TORQUE           2u       /**< 转矩模式                          */
/** @} */

/** @name 速度源选择 */
/** @{ */
#define LD2_MOTOR_SPEED_SRC_INTERNAL    1u       /**< 内部速度指令 (Pr3.04 作为速度源)    */
/** @} */

/** @name 转速范围限制 */
/** @{ */
#define LD2_MOTOR_SPEED_MIN_RPM        (-10000) /**< 最小转速 (rpm)                     */
#define LD2_MOTOR_SPEED_MAX_RPM         10000   /**< 最大转速 (rpm)                     */
/** @} */

/** @name 默认参数 */
/** @{ */
#define LD2_MOTOR_DEFAULT_TIMEOUT_MS    100u    /**< 默认通信超时 (ms)                   */
/** @} */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  LD2-RS 设备句柄
  */
typedef struct {
    bsp_rs485_handle_t *bus;      /**< RS485 总线句柄                      */
    uint8_t   slave_id;               /**< Modbus 从机站号 (1~247)              */
    uint32_t  timeout_ms;            /**< 通信超时 (ms)                        */

    /* 诊断信息 (调试用) */
    uint8_t   last_function_code;          /**< 上一次发送的功能码                     */
    uint8_t   last_exception_code;         /**< 上一次收到的异常码 (0=无异常)           */
    uint8_t   last_rx_buffer[32];             /**< 上一次接收帧的原始数据                  */
    uint16_t  last_rx_len;            /**< 上一次接收帧的长度                     */
    uint16_t  baud_rate_setting;             /**< 读回的 Pr5.30 波特率设定值            */
} ld2_motor_handle_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 LD2-RS 设备句柄
  * @param  dev         设备句柄指针
  * @param  bus         RS485 总线句柄
  * @param  slave_id    从机站号 (1~247)
  * @param  timeout_ms 通信超时 (ms), 0 则使用默认值 100ms
  */
void ld2_motor_init(ld2_motor_handle_t *dev,
                    bsp_rs485_handle_t *bus,
                    uint8_t slave_id,
                    uint32_t timeout_ms);

extern ld2_motor_handle_t g_ld2rs_dev_m1;
extern ld2_motor_handle_t g_ld2rs_dev_m2;

/**
  * @brief  读 LD2-RS 寄存器 (封装 Modbus 0x03)
  * @param  dev    设备句柄
  * @param  reg_addr 寄存器地址 (Modbus 485 地址)
  * @param  value_out  读取结果 (输出)
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ld2_motor_read_reg(ld2_motor_handle_t *dev,
                                 uint16_t reg_addr,
                                 uint16_t *value_out);

/**
  * @brief  写 LD2-RS 寄存器 (封装 Modbus 0x06)
  * @param  dev    设备句柄
  * @param  reg_addr 寄存器地址 (Modbus 485 地址)
  * @param  reg_value 写入值
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ld2_motor_write_reg(ld2_motor_handle_t *dev,
                                  uint16_t reg_addr,
                                  uint16_t reg_value);

/**
  * @brief  设置目标转速 (写 Pr3.04 第一段速寄存器)
  *
  * @details 将目标转速写入 Pr3.04 (Modbus 地址 0x0309)。
  *          包含参数范围校验：target_speed_rpm 必须在 [-10000, 10000] 范围内。
  *          写 0 等效于停止电机。
  *
  * @note   LD2-RS 有四段速寄存器 Pr3.04~Pr3.11，本函数仅写第一段速。
  *         需要搭配 Pr3.00=1 (内部速度源) 使用。
  *
  * @param  dev   设备句柄
  * @param  target_speed_rpm 目标转速 (rpm), -10000 ~ +10000, 正=正转, 负=反转
  * @retval MODBUS_RTU_OK            写入成功
  * @retval MODBUS_RTU_ERR_PARAM     转速越界
  * @retval 其他                 通信失败
  */
modbus_rtu_status_t ld2_motor_set_target_speed(ld2_motor_handle_t *dev, int16_t target_speed_rpm);

/**
  * @brief  设置速度源为内部指令 (写 Pr3.00 = 1)
  * @param  dev 设备句柄
  * @retval modbus_rtu_status_t
  */
modbus_rtu_status_t ld2_motor_set_speed_source_internal(ld2_motor_handle_t *dev);

/**
  * @brief  设置加减速时间 (写 Pr3.12 加速 + Pr3.13 减速)
  * @param  dev       设备句柄
  * @param  accel_time_ms 加速时间 (ms/1000rpm)
  * @param  decel_time_ms 减速时间 (ms/1000rpm)
  * @retval MODBUS_RTU_OK 两者均成功 / 其他 第一处或第二处写入失败
  */
modbus_rtu_status_t ld2_motor_set_accel_decel(ld2_motor_handle_t *dev,
                                        uint16_t accel_time_ms,
                                        uint16_t decel_time_ms);

modbus_rtu_status_t ld2_motor_set_baud_rate(ld2_motor_handle_t *dev, uint16_t baud_setting);

#ifdef __cplusplus
}
#endif

#endif /* LD2_MOTOR_H */


